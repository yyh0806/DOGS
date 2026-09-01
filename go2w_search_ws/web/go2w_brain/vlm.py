"""vlm — DeepSeek 视觉模型客户端 (M7.3 语义锚定/分割, stdlib 零依赖)。

把 PIL 图像编码为 base64 data URI, 走 OpenAI 兼容的 /chat/completions。
key 复用 BrainConfig.llm_api_key (DEEPSEEK_API_KEY / ~/.dsh 凭据);
模型默认 deepseek-v4-flash-vision-exp (GO2W_VLM_MODEL 可覆盖)。
"""
from __future__ import annotations

import base64
import io
import json
import urllib.error
import urllib.request
from typing import Any

from PIL import Image

DEFAULT_VLM_MODEL = "deepseek-v4-flash-vision-exp"
_MAX_EDGE = 1024


class VLMUnavailable(Exception):
    pass


class VLMClient:
    def __init__(self, api_key: str = "", model: str = DEFAULT_VLM_MODEL,
                 base_url: str = "https://api.deepseek.com",
                 timeout: float = 60.0):
        self._key = (api_key or "").strip()
        self._model = model or DEFAULT_VLM_MODEL
        self._base = base_url.rstrip("/")
        self._timeout = timeout

    def available(self) -> bool:
        return bool(self._key)

    def vision(self, image: Image.Image, prompt: str,
               max_tokens: int = 1024) -> str:
        """图像 + 提示词 → 文本。不可用/失败抛 VLMUnavailable。

        max_tokens 默认 1024: 视觉模型多为推理模型, reasoning_content
        会先吃掉大量预算, 400 只够推理不够出答案 (实测 finish_reason=length
        且 content 为空) —— 预算不足时退而求其次从推理尾巴解析。
        """
        if not self.available():
            raise VLMUnavailable("no_api_key")
        encoded = _encode_image(image)
        body = {
            "model": self._model,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": encoded}},
                ],
            }],
            "max_tokens": max_tokens,
            "temperature": 0.1,
        }
        request = urllib.request.Request(
            self._base + "/chat/completions",
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            headers={"Authorization": f"Bearer {self._key}",
                     "Content-Type": "application/json"},
            method="POST")
        try:
            with urllib.request.urlopen(request,
                                        timeout=self._timeout) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise VLMUnavailable(f"{type(exc).__name__}: {exc}")
        message = (payload.get("choices") or [{}])[0].get("message") or {}
        content = message.get("content")
        if isinstance(content, list):  # 部分网关返回分段
            content = "".join(
                (c.get("text") or "") for c in content
                if isinstance(c, dict))
        content = str(content or "").strip()
        if not content:
            # 推理模型预算不足: 答案可能停在 reasoning 尾巴 (尽力兜底)
            content = str(message.get("reasoning_content") or "").strip()
        return content


def _encode_image(image: Image.Image) -> str:
    img = image.convert("RGB")
    w, h = img.size
    scale = min(1.0, _MAX_EDGE / max(w, h))
    if scale < 1.0:
        img = img.resize((int(w * scale), int(h * scale)))
    buffer = io.BytesIO()
    img.save(buffer, format="JPEG", quality=82)
    return ("data:image/jpeg;base64,"
            + base64.b64encode(buffer.getvalue()).decode("ascii"))


def parse_json_loose(text: str) -> dict[str, Any] | None:
    """宽容提取 VLM 输出中的 JSON 对象 (容忍围栏/前缀)。"""
    import re
    text = str(text or "").strip()
    match = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, re.S)
    if match:
        text = match.group(1)
    else:
        start = text.find("{")
        end = text.rfind("}")
        if start < 0 or end <= start:
            return None
        text = text[start:end + 1]
    try:
        payload = json.loads(text)
        return payload if isinstance(payload, dict) else None
    except json.JSONDecodeError:
        return None
