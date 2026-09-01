"""speak — 语音播报 (M7.2 起经 TTS 后端真实发声)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    text = str(args.get("text", ""))
    if not text.strip():
        return {"ok": False, "reason": "empty_text"}
    ctx.get("log", _NoLog()).append("event", event="speak", text=text)
    tts = ctx.get("tts")
    if tts is None:
        # 无后端 (单元测试直调): 保持诚实桩语义
        return {"ok": True, "text": text, "spoken": False,
                "note": "no_tts_backend"}
    result = tts.speak(text)
    return {"ok": result.get("ok", False), "text": text, **result}


class _NoLog:
    def append(self, *args, **kwargs):  # noqa: D401
        return None


TOOL = ToolRegistration(
    name="speak",
    description=(
        "语音播报一段文本 (经 TTS 后端发声: console/edge 由 GO2W_TTS 选择)。"
        "任务关键节点 (开始/发现 confirmed 告警/完成) 用它向操作员播报。"),
    parameters={"type": "object",
                "properties": {"text": {"type": "string"}},
                "required": ["text"]},
    execute=execute,
    risk="act",
    requires=(),
)
