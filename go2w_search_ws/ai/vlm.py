"""
Qwen2.5-VL 视觉语言模型引擎
============================
基于 Qwen2.5-VL-3B-Instruct，支持:
  1. 多轮对话 (chat) — 通用 VLM 推理
  2. 目标定位 (locate) — 自然语言目标在图像中的定位
  3. 目标跟踪 (track_target) — 计算跟踪控制量

平台兼容：transformers 原生加载，Jetson NX 可运行。
"""

import logging
import threading
import time

import numpy as np

from ai.config import VLM_MODEL_NAME, VLM_QUANT, DEVICE, CUDA_AVAILABLE, memory_summary

logger = logging.getLogger("go2w.vlm")


class VLMEngine:
    """Qwen2.5-VL 视觉定位引擎。

    职责：
      1. 接收图像 (numpy BGR) + 目标描述文本
      2. 在图像中定位目标，返回 bounding box
      3. 返回目标的图像中心偏移（供机器人跟踪用）

    用法：
        engine = VLMEngine()
        engine.load()
        bbox = engine.locate(frame, "穿蓝色衣服的人")
        # bbox = {"x1": 100, "y1": 50, "x2": 300, "y2": 400, "cx": 200, "cy": 225}
    """

    def __init__(self, model_name=VLM_MODEL_NAME, quant=VLM_QUANT):
        self._model_name = model_name
        self._quant = quant
        self._model = None
        self._processor = None
        self._loaded = False
        self._lock = threading.Lock()
        self._load_time = 0.0

    @property
    def loaded(self) -> bool:
        return self._loaded

    def load(self):
        """加载 Qwen2.5-VL 模型。"""
        if self._loaded:
            return True
        try:
            import torch
            from transformers import Qwen2_5_VLForConditionalGeneration, AutoProcessor

            t0 = time.time()
            logger.info(f"加载 VLM 模型: {self._model_name}")
            logger.info(f"设备: {DEVICE}, 量化: {self._quant}")
            logger.info(memory_summary())

            kwargs = {
                "torch_dtype": torch.float16 if CUDA_AVAILABLE else torch.float32,
            }
            if CUDA_AVAILABLE:
                kwargs["device_map"] = "auto"

            # Orin NX 不用 bitsandbytes
            if self._quant == "fp16":
                pass  # 已经设了 torch_dtype
            elif self._quant == "gptq":
                # GPTQ 量化版本，aarch64 兼容
                kwargs["revision"] = "gptq-4bit"

            self._model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
                self._model_name, **kwargs
            )
            self._processor = AutoProcessor.from_pretrained(self._model_name)
            self._model.eval()
            self._loaded = True
            self._load_time = time.time() - t0
            logger.info(f"VLM 模型加载完成 ({self._load_time:.1f}s)")
            logger.info(memory_summary())

            # warm-up
            self._warmup()
            return True

        except ImportError as e:
            logger.error(f"缺少依赖: {e}")
            logger.info("pip install transformers accelerate qwen-vl-utils")
            return False
        except Exception as e:
            logger.error(f"VLM 模型加载失败: {e}")
            self._model = None
            self._processor = None
            return False

    def _warmup(self):
        """用一张假图片做一次推理，预热模型。"""
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.locate(dummy, "test")
            logger.info("VLM warm-up 完成")
        except Exception as e:
            logger.debug(f"VLM warm-up 跳过: {e}")

    def chat(self, messages: list, max_new_tokens: int = 200) -> str:
        """多轮对话推理（核心方法）。

        Args:
            messages: Qwen 格式的消息列表，可含图像和文本
            max_new_tokens: 最大生成 token 数
        Returns:
            模型生成的文本
        """
        if not self._loaded:
            return "VLM 未加载"

        with self._lock:
            try:
                import torch
                from PIL import Image

                # 提取所有图像
                images = []
                for msg in messages:
                    if isinstance(msg.get("content"), list):
                        for item in msg["content"]:
                            if item.get("type") == "image" and "image" in item:
                                img = item["image"]
                                if not isinstance(img, Image.Image):
                                    img = Image.fromarray(img)
                                images.append(img)

                text_input = self._processor.apply_chat_template(
                    messages, tokenize=False, add_generation_prompt=True
                )
                inputs = self._processor(
                    text=[text_input],
                    images=images if images else None,
                    return_tensors="pt",
                ).to(self._model.device)

                with torch.no_grad():
                    output_ids = self._model.generate(
                        **inputs,
                        max_new_tokens=max_new_tokens,
                        do_sample=False,
                    )

                return self._processor.batch_decode(
                    output_ids[:, inputs["input_ids"].shape[1]:],
                    skip_special_tokens=True,
                )[0].strip()

            except Exception as e:
                logger.error(f"VLM chat 失败: {e}")
                return f"error: {e}"

    def unload(self):
        """卸载模型，释放显存。"""
        with self._lock:
            if self._model is not None:
                import torch
                del self._model
                del self._processor
                self._model = None
                self._processor = None
                self._loaded = False
                if CUDA_AVAILABLE:
                    torch.cuda.empty_cache()
                logger.info("VLM 模型已卸载")

    def locate(self, image: np.ndarray, target_description: str) -> dict:
        """在图像中定位目标。

        Args:
            image: BGR 图像 (numpy, uint8)
            target_description: 自然语言目标描述，如"穿蓝色衣服的人"
        Returns:
            {
                "found": bool,
                "bbox": [x1, y1, x2, y2],  # 像素坐标
                "cx": float, cy: float,     # 中心点归一化坐标 (0-1)
                "description": str,          # 模型描述
            }
            未找到时 found=False
        """
        if not self._loaded:
            return {"found": False, "bbox": None, "cx": 0, "cy": 0,
                    "description": "VLM 未加载"}

        with self._lock:
            try:
                import torch
                from PIL import Image

                # BGR → RGB → PIL
                rgb = image[:, :, ::-1] if image.ndim == 3 else image
                pil_img = Image.fromarray(rgb)
                w, h = pil_img.size

                # 构造 prompt
                prompt = (
                    f"在图片中找到\"{target_description}\"。"
                    f"如果找到了，用 JSON 回答："
                    f'{{"found": true, "bbox": [x1, y1, x2, y2], "description": "描述"}}\n'
                    f"其中 x1,y1 是左上角，x2,y2 是右下角的像素坐标。"
                    f"如果没找到，回答：{{\"found\": false}}"
                )

                messages = [
                    {
                        "role": "user",
                        "content": [
                            {"type": "image", "image": pil_img},
                            {"type": "text", "text": prompt},
                        ],
                    }
                ]

                # 处理输入
                text_input = self._processor.apply_chat_template(
                    messages, tokenize=False, add_generation_prompt=True
                )
                inputs = self._processor(
                    text=[text_input],
                    images=[pil_img],
                    return_tensors="pt",
                ).to(self._model.device)

                # 生成
                with torch.no_grad():
                    output_ids = self._model.generate(
                        **inputs,
                        max_new_tokens=200,
                        do_sample=False,
                    )

                # 解码
                response = self._processor.batch_decode(
                    output_ids[:, inputs["input_ids"].shape[1]:],
                    skip_special_tokens=True,
                )[0].strip()

                return self._parse_bbox(response, w, h)

            except Exception as e:
                logger.error(f"VLM 定位失败: {e}")
                return {"found": False, "bbox": None, "cx": 0, "cy": 0,
                        "description": f"error: {e}"}

    @staticmethod
    def _parse_bbox(response: str, img_w: int, img_h: int) -> dict:
        """解析模型输出的 bbox。"""
        import json
        import re

        result = {"found": False, "bbox": None, "cx": 0, "cy": 0,
                  "description": ""}

        # 提取 JSON
        json_match = re.search(r'\{[^}]+\}', response, re.DOTALL)
        if json_match:
            try:
                parsed = json.loads(json_match.group())
                if parsed.get("found", False) and "bbox" in parsed:
                    x1, y1, x2, y2 = parsed["bbox"]
                    # 裁剪到图像范围
                    x1 = max(0, min(int(x1), img_w))
                    y1 = max(0, min(int(y1), img_h))
                    x2 = max(0, min(int(x2), img_w))
                    y2 = max(0, min(int(y2), img_h))

                    result["found"] = True
                    result["bbox"] = [x1, y1, x2, y2]
                    result["cx"] = (x1 + x2) / 2 / img_w  # 归一化 0-1
                    result["cy"] = (y1 + y2) / 2 / img_h
                    result["description"] = parsed.get("description", "")
                    return result
                elif not parsed.get("found", True):
                    result["description"] = parsed.get("description", "未找到目标")
                    return result
            except (json.JSONDecodeError, ValueError, TypeError):
                pass

        # 非标准输出，尝试提取数字
        nums = re.findall(r'\d+', response)
        if len(nums) >= 4:
            try:
                x1, y1, x2, y2 = [int(n) for n in nums[:4]]
                x1 = max(0, min(x1, img_w))
                y1 = max(0, min(y1, img_h))
                x2 = max(0, min(x2, img_w))
                y2 = max(0, min(y2, img_h))
                result["found"] = True
                result["bbox"] = [x1, y1, x2, y2]
                result["cx"] = (x1 + x2) / 2 / img_w
                result["cy"] = (y1 + y2) / 2 / img_h
                result["description"] = response
            except ValueError:
                result["description"] = response
        else:
            result["description"] = response

        return result

    def track_target(self, image: np.ndarray, target_description: str,
                     img_w: int = 640, img_h: int = 480) -> dict:
        """跟踪目标，返回运动控制参数。

        在 locate 基础上额外计算机器人应该怎么动。

        Returns:
            {
                "found": bool,
                "vx": float,    # 前进速度 (-1 ~ 1)
                "vyaw": float,  # 旋转速度 (-1 ~ 1)
                "bbox": [...],
                "cx": float, "cy": float,
            }
        """
        loc = self.locate(image, target_description)
        if not loc["found"]:
            return {**loc, "vx": 0, "vyaw": 0}

        # 计算跟踪控制量
        cx = loc["cx"]  # 归一化中心 x (0=左, 0.5=中, 1=右)
        cy = loc["cy"]  # 归一化中心 y (0=上, 0.5=中, 1=下)

        # 偏离中心的程度 → 旋转速度
        # cx < 0.5 目标在左边，需要左转（vyaw > 0）
        # cx > 0.5 目标在右边，需要右转（vyaw < 0）
        offset_x = cx - 0.5
        vyaw = -offset_x * 2.0  # 映射到 ±1.0
        vyaw = max(-1.0, min(1.0, vyaw))

        # 目标大小 → 距离 → 前进速度
        bbox = loc["bbox"]
        if bbox:
            bbox_w = bbox[2] - bbox[0]
            bbox_h = bbox[3] - bbox[1]
            # bbox 占图像的比例
            ratio = (bbox_w * bbox_h) / (img_w * img_h)
            # 比例小 → 远 → 要前进; 比例大 → 近 → 减速/后退
            # 目标比例 ~0.05 (约占画面 5%) 时保持距离
            if ratio < 0.03:
                vx = 0.3
            elif ratio < 0.05:
                vx = 0.15
            elif ratio > 0.2:
                vx = -0.1  # 后退
            else:
                vx = 0.0
        else:
            vx = 0.0

        return {**loc, "vx": vx, "vyaw": vyaw}
