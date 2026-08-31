"""speak — 语音播报 (M1 桩: 落盘留痕, M5 接入 TTS/狗喇叭)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    text = str(args.get("text", ""))
    if not text.strip():
        return {"ok": False, "reason": "empty_text"}
    ctx.get("log", _NoLog()).append("event", event="speak", text=text)
    # M1: 无 TTS 后端, 诚实报告未发声; M5 在此接入真机播音。
    return {"ok": True, "text": text, "spoken": False,
            "note": "M1 stub: TTS/狗喇叭待 M5 接入"}


class _NoLog:
    def append(self, *args, **kwargs):  # noqa: D401
        return None


TOOL = ToolRegistration(
    name="speak",
    description=("语音播报一段文本。M1 为桩实现 (spoken=false), "
                 "M5 起接入 TTS 与狗喇叭。"),
    parameters={"type": "object",
                "properties": {"text": {"type": "string"}},
                "required": ["text"]},
    execute=execute,
    risk="act",
    requires=(),
)
