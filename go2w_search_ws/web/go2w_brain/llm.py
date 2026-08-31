"""DeepSeek function-calling 客户端 (stdlib urllib, 零第三方依赖)。

key 为空 → LLMClient.available() 为 False, 大脑自动退规则模式。
工具调用参数容忍 str 形态 (部分网关以字符串下发 JSON)。
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any

from .config import BrainConfig


class LLMUnavailable(Exception):
    pass


class LLMResponse:
    def __init__(self, content: str, tool_calls: list[dict[str, Any]],
                 finish_reason: str, reasoning: str = ""):
        self.content = content
        self.tool_calls = tool_calls
        self.finish_reason = finish_reason
        self.reasoning = reasoning


class LLMClient:
    def __init__(self, config: BrainConfig):
        self._config = config

    def available(self) -> bool:
        return bool(self._config.llm_api_key)

    def chat(self, messages: list[dict[str, Any]],
             tools: list[dict[str, Any]] | None = None) -> LLMResponse:
        if not self.available():
            raise LLMUnavailable("no_api_key")
        body: dict[str, Any] = {
            "model": self._config.llm_model,
            "messages": messages,
            "temperature": self._config.llm_temperature,
            "max_tokens": self._config.llm_max_tokens,
            "stream": False,
        }
        if tools:
            body["tools"] = tools
        request = urllib.request.Request(
            self._config.llm_base_url.rstrip("/") + "/chat/completions",
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self._config.llm_api_key}",
                "Content-Type": "application/json; charset=utf-8",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=30.0) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise LLMUnavailable(f"llm_http_error: {type(exc).__name__}: {exc}")
        message = (payload.get("choices") or [{}])[0].get("message") or {}
        return LLMResponse(
            content=message.get("content") or "",
            tool_calls=message.get("tool_calls") or [],
            finish_reason=((payload.get("choices") or [{}])[0]
                           .get("finish_reason") or ""),
            reasoning=message.get("reasoning_content") or "",
        )


def parse_tool_args(raw: Any) -> dict[str, Any]:
    """工具参数: dict 原样, str 按 JSON 解析, 失败返回空 dict。"""
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str):
        try:
            parsed = json.loads(raw)
            return parsed if isinstance(parsed, dict) else {}
        except json.JSONDecodeError:
            return {}
    return {}
