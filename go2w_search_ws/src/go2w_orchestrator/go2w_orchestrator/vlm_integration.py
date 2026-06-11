"""VLM 集成: Qwen2.5-VL-3B 多轮对话 + 任务分解 + 视觉问答。

单一 VLM 模型承担三个职责:
1. 语音/文本指令理解 → 任务分解 (JSON)
2. 视觉理解 + 交互 (场景描述、问答)
3. 目标定位 (复用 locate)
"""

import json
import re
import logging
import threading
from typing import Optional, List

import numpy as np

from ai.vlm import VLMEngine

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = """\
你是 Go2W 机器狗的智能助手。用户会通过语音或文本给你下达任务指令。
你需要理解用户的意图，并将复杂任务分解为可执行的子任务队列。

你可以调度的子任务类型:
- navigate: 导航到指定位置 {"x": float, "y": float, "yaw": float}
- search_area: 在区域内搜索 {"pattern": "lawnmower"|"spiral", "width": float, "height": float, "spacing": float, "target": "描述"}
- follow: 跟踪目标 {"target": "目标描述"}
- observe: 观察目标 {"target": "目标描述", "duration": float}
- return_home: 返回起始位置
- wait: 等待 {"duration": float}
- stop: 停止当前动作

回复格式 (JSON):
{
  "understanding": "你对用户指令的理解",
  "tasks": [
    {"type": "子任务类型", "priority": 1-10, "params": {任务参数}}
  ],
  "response": "用自然语言回复用户"
}

如果用户只是问问题而不是下达指令，直接在 response 中回答即可，tasks 为空数组。
"""

TASK_DECOMPOSE_PROMPT = """\
请将以下指令分解为子任务队列，用 JSON 格式回复。
"""

SCENE_DESCRIBE_PROMPT = """\
请描述你在图片中看到的场景。包括:
1. 环境（室内/室外，空间大小，主要物体）
2. 人物（如果有人，描述数量、位置、动作）
3. 潜在的目标或感兴趣的对象
用简洁的中文回答。
"""


class VLMIntegration:
    """VLM 统一接口，管理多轮对话和任务分解。"""

    def __init__(self):
        self._engine = VLMEngine()
        self._chat_history: List[dict] = []
        self._lock = threading.Lock()

    @property
    def loaded(self) -> bool:
        return self._engine.loaded

    def load(self) -> bool:
        """加载 VLM 模型。"""
        return self._engine.load()

    def unload(self):
        """卸载模型。"""
        self._engine.unload()
        self._chat_history.clear()

    def reset_conversation(self):
        """重置对话历史。"""
        self._chat_history.clear()

    def process_command(self, text: str) -> dict:
        """处理文本指令，返回任务列表。

        Returns:
            {"understanding": str, "tasks": [...], "response": str}
        """
        if not self._engine.loaded:
            return self._fallback_parse(text)

        messages = self._build_messages([
            {"role": "user", "content": [
                {"type": "text", "text": f"{TASK_DECOMPOSE_PROMPT}\n用户指令: {text}"}
            ]}
        ])

        response = self._engine.chat(messages)
        self._chat_history.append({"role": "user", "content": text})
        self._chat_history.append({"role": "assistant", "content": response})

        return self._parse_task_response(response)

    def process_command_with_image(self, text: str, image: np.ndarray) -> dict:
        """带图像的指令处理。"""
        if not self._engine.loaded:
            return self._fallback_parse(text)

        from PIL import Image
        if image.ndim == 3:
            rgb = image[:, :, ::-1] if image.shape[2] == 3 else image
        else:
            rgb = image
        pil_img = Image.fromarray(rgb)

        messages = self._build_messages([
            {"role": "user", "content": [
                {"type": "image", "image": pil_img},
                {"type": "text", "text": f"{TASK_DECOMPOSE_PROMPT}\n用户指令: {text}"}
            ]}
        ])

        response = self._engine.chat(messages, max_new_tokens=400)
        return self._parse_task_response(response)

    def describe_scene(self, image: np.ndarray) -> str:
        """描述当前场景。"""
        if not self._engine.loaded:
            return "VLM 未加载"

        from PIL import Image
        if image.ndim == 3 and image.shape[2] == 3:
            rgb = image[:, :, ::-1]
        else:
            rgb = image
        pil_img = Image.fromarray(rgb)

        messages = self._build_messages([
            {"role": "user", "content": [
                {"type": "image", "image": pil_img},
                {"type": "text", "text": SCENE_DESCRIBE_PROMPT}
            ]}
        ])

        return self._engine.chat(messages)

    def answer_question(self, question: str, image: Optional[np.ndarray] = None) -> str:
        """回答关于当前场景的问题。"""
        if not self._engine.loaded:
            return "VLM 未加载"

        content = [{"type": "text", "text": question}]

        if image is not None:
            from PIL import Image
            if image.ndim == 3 and image.shape[2] == 3:
                rgb = image[:, :, ::-1]
            else:
                rgb = image
            content.insert(0, {"type": "image", "image": Image.fromarray(rgb)})

        messages = self._build_messages([
            {"role": "user", "content": content}
        ])

        response = self._engine.chat(messages)
        self._chat_history.append({"role": "user", "content": question})
        self._chat_history.append({"role": "assistant", "content": response})
        return response

    def locate_target(self, image: np.ndarray, description: str) -> dict:
        """定位目标（复用 VLMEngine.locate）。"""
        return self._engine.locate(image, description)

    def _build_messages(self, new_messages: list) -> list:
        """构建含系统提示和历史的多轮消息。"""
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]

        # 最近 6 轮历史
        history = self._chat_history[-12:]
        for h in history:
            messages.append(h)

        messages.extend(new_messages)
        return messages

    @staticmethod
    def _parse_task_response(response: str) -> dict:
        """解析 VLM 输出为结构化任务列表。"""
        result = {
            "understanding": "",
            "tasks": [],
            "response": response,
        }

        # 尝试提取 JSON（匹配最外层的花括号对）
        depth = 0
        start = -1
        for i, ch in enumerate(response):
            if ch == '{':
                if depth == 0:
                    start = i
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0 and start >= 0:
                    candidate = response[start:i+1]
                    try:
                        parsed = json.loads(candidate)
                        result["understanding"] = parsed.get("understanding", "")
                        result["response"] = parsed.get("response", response)
                        result["tasks"] = parsed.get("tasks", [])
                        return result
                    except json.JSONDecodeError:
                        start = -1
                        continue

        return result

    @staticmethod
    def _fallback_parse(text: str) -> dict:
        """VLM 未加载时的关键词降级解析。"""
        result = {"understanding": text, "tasks": [], "response": ""}

        text_lower = text.lower()

        if "跟着" in text or "跟随" in text or "跟上" in text:
            target = ""
            for kw in ["跟着", "跟随", "跟上"]:
                if kw in text:
                    idx = text.index(kw) + len(kw)
                    target = text[idx:].strip().rstrip("。，！？")
                    break
            result["tasks"] = [{"type": "follow", "priority": 8, "params": {"target": target}}]
            result["response"] = f"好的，我来跟着{target}"

        elif "搜索" in text or "找" in text:
            result["tasks"] = [{"type": "search_area", "priority": 5,
                                "params": {"pattern": "lawnmower", "width": 10, "height": 10, "spacing": 2.5}}]
            result["response"] = "好的，开始搜索"

        elif "停" in text:
            result["tasks"] = [{"type": "stop", "priority": 10, "params": {}}]
            result["response"] = "好的，停止"

        elif "回来" in text or "回来" in text or "返回" in text:
            result["tasks"] = [{"type": "return_home", "priority": 7, "params": {}}]
            result["response"] = "好的，正在返回"

        elif "前进" in text:
            result["tasks"] = [{"type": "navigate", "priority": 6, "params": {"x": 2.0, "y": 0.0}}]
            result["response"] = "好的，前进"

        return result
