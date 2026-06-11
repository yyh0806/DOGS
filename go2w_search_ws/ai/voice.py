"""
Audio-Interaction 语音理解
===========================
基于 xzf-thu/Audio-Interaction-3B 模型，实现语音→指令理解。
支持流式音频输入，输出结构化的机器人指令。

平台兼容：使用 transformers 原生加载，不依赖 bitsandbytes。
Orin NX (aarch64) 上也能运行。
"""

import logging
import threading
import time
import numpy as np

from ai.config import (
    VOICE_MODEL_NAME, VOICE_QUANT, DEVICE, CUDA_AVAILABLE,
    AUDIO_SAMPLE_RATE, memory_summary
)

logger = logging.getLogger("go2w.voice")


class VoiceEngine:
    """Audio-Interaction 语音理解引擎。

    职责：
      1. 接收麦克风录音片段 (numpy int16)
      2. 通过 Audio-Interaction 模型理解语音内容
      3. 返回识别文本 + 结构化指令

    用法：
        engine = VoiceEngine()
        engine.load()
        audio = capture.get_utterance()  # numpy int16
        result = engine.process(audio)
        # result = {"text": "跟着那个蓝衣服的人", "intent": "follow", "target": "蓝衣服的人"}
    """

    def __init__(self, model_name=VOICE_MODEL_NAME, quant=VOICE_QUANT):
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
        """加载 Audio-Interaction 模型。"""
        if self._loaded:
            return True
        try:
            from transformers import AutoModelForCausalLM, AutoProcessor, AutoTokenizer
            import torch

            t0 = time.time()
            logger.info(f"加载语音模型: {self._model_name}")
            logger.info(f"设备: {DEVICE}, 量化: {self._quant}")
            logger.info(memory_summary())

            # 根据 Audio-Interaction 的实际接口加载
            # 该模型基于 Qwen2.5-Omni，使用 Qwen2_5OmniForConditionalGeneration
            try:
                from transformers import Qwen2_5OmniForConditionalGeneration
                model_cls = Qwen2_5OmniForConditionalGeneration
            except ImportError:
                # 旧版 transformers 可能没有这个类
                logger.warning("Qwen2_5OmniForConditionalGeneration 不可用，尝试 AutoModel")
                model_cls = AutoModelForCausalLM

            kwargs = {
                "torch_dtype": torch.float16 if CUDA_AVAILABLE else torch.float32,
                "device_map": "auto" if CUDA_AVAILABLE else None,
            }

            # Orin NX 不支持 bitsandbytes，不用 4-bit 量化
            # 如果显存足够 (< 16GB)，直接 fp16 即可
            if self._quant == "fp16" and not CUDA_AVAILABLE:
                kwargs["torch_dtype"] = torch.float32
                kwargs.pop("device_map", None)

            self._model = model_cls.from_pretrained(self._model_name, **kwargs)

            # 加载 processor（含 audio processor）
            try:
                self._processor = AutoProcessor.from_pretrained(self._model_name)
            except Exception:
                self._processor = AutoTokenizer.from_pretrained(self._model_name)

            self._model.eval()
            self._loaded = True
            self._load_time = time.time() - t0
            logger.info(f"语音模型加载完成 ({self._load_time:.1f}s)")
            logger.info(memory_summary())
            return True

        except Exception as e:
            logger.error(f"语音模型加载失败: {e}")
            logger.info("提示: pip install transformers accelerate")
            logger.info(f"模型需手动下载或通过 huggingface-cli: "
                        f"huggingface-cli download {self._model_name}")
            self._model = None
            self._processor = None
            return False

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
                logger.info("语音模型已卸载")

    def process(self, audio: np.ndarray, sample_rate: int = AUDIO_SAMPLE_RATE) -> dict:
        """处理一段语音，返回识别结果。

        Args:
            audio: numpy int16 数组，形状 (samples,) 或 (samples, 1)
            sample_rate: 采样率
        Returns:
            {"text": str, "intent": str, "target": str, "raw_response": str}
        """
        if not self._loaded:
            return self._fallback_process(audio, sample_rate)

        with self._lock:
            try:
                import torch

                # 预处理音频
                if audio.ndim == 2:
                    audio = audio.squeeze(axis=1)
                # int16 → float32, 归一化到 [-1, 1]
                audio_float = audio.astype(np.float32) / 32768.0

                # 构造输入
                conversation = [
                    {
                        "role": "system",
                        "content": (
                            "你是机器狗的语音助手。用户会通过语音下达指令。"
                            "你需要理解用户的意图，并用 JSON 格式回复。\n"
                            "支持的指令：\n"
                            "- follow: 跟着某人/某物（需要目标描述）\n"
                            "- search: 搜索一片区域\n"
                            "- stop: 停止当前动作\n"
                            "- come: 回到我身边\n"
                            "- look: 看向某处\n"
                            "- patrol: 巡逻\n"
                            "\n"
                            "回复格式：{\"intent\": \"<指令>\", \"target\": \"<目标描述，没有则为空>\", "
                            "\"text\": \"<用户说的话>\"}"
                        ),
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "audio", "audio": audio_float, "sampling_rate": sample_rate},
                        ],
                    },
                ]

                # 通过 processor 处理
                inputs = self._processor.apply_chat_template(
                    conversation,
                    add_generation_prompt=True,
                    tokenize=True,
                    return_tensors="pt",
                    return_dict=True,
                ).to(DEVICE)

                # 生成
                with torch.no_grad():
                    output_ids = self._model.generate(
                        **inputs,
                        max_new_tokens=256,
                        do_sample=False,
                        temperature=1.0,
                    )

                # 解码
                response = self._processor.batch_decode(
                    output_ids[:, inputs["input_ids"].shape[1]:],
                    skip_special_tokens=True,
                )[0].strip()

                return self._parse_response(response)

            except Exception as e:
                logger.error(f"语音处理失败: {e}")
                return {"text": "", "intent": "unknown", "target": "",
                        "raw_response": f"error: {e}"}

    def process_text(self, text: str) -> dict:
        """处理文本输入（用于前端文本指令或测试）。"""
        return self._parse_response(text)

    @staticmethod
    def _parse_response(response: str) -> dict:
        """解析模型输出为结构化指令。"""
        import json
        import re

        result = {
            "text": "",
            "intent": "unknown",
            "target": "",
            "raw_response": response,
        }

        # 尝试提取 JSON
        json_match = re.search(r'\{[^}]+\}', response)
        if json_match:
            try:
                parsed = json.loads(json_match.group())
                result["text"] = parsed.get("text", "")
                result["intent"] = parsed.get("intent", "unknown")
                result["target"] = parsed.get("target", "")
                return result
            except json.JSONDecodeError:
                pass

        # JSON 解析失败，用关键词匹配
        result["text"] = response
        text_lower = response.lower()

        # 中文关键词匹配
        if "跟着" in response or "跟上" in response or "跟随" in response:
            result["intent"] = "follow"
            # 提取目标描述
            for kw in ["跟着", "跟上", "跟随"]:
                if kw in response:
                    idx = response.index(kw) + len(kw)
                    target = response[idx:].strip().rstrip("。，！？")
                    if target:
                        result["target"] = target
                    break
        elif "搜索" in response or "找" in response or "寻找" in response:
            result["intent"] = "search"
            for kw in ["搜索", "找", "寻找"]:
                if kw in response:
                    idx = response.index(kw) + len(kw)
                    target = response[idx:].strip().rstrip("。，！？")
                    if target:
                        result["target"] = target
                    break
        elif "停" in response or "停止" in response:
            result["intent"] = "stop"
        elif "过来" in response or "回来" in response or "来" in response:
            result["intent"] = "come"
        elif "巡逻" in response:
            result["intent"] = "patrol"
        elif "看" in response or "面向" in response:
            result["intent"] = "look"

        return result

    @staticmethod
    def _fallback_process(audio: np.ndarray, sample_rate: int) -> dict:
        """模型未加载时的降级方案：用 Whisper 或简单能量检测。"""
        # 尝试 whisper
        try:
            import whisper
            model = whisper.load_model("tiny")
            audio_float = audio.astype(np.float32) / 32768.0
            result = model.transcribe(audio_float, language="zh")
            text = result.get("text", "")
            if text:
                return VoiceEngine._parse_response(text)
        except ImportError:
            pass
        except Exception as e:
            logger.debug(f"Whisper 降级也失败: {e}")

        return {"text": "", "intent": "unknown", "target": "",
                "raw_response": "模型未加载"}
