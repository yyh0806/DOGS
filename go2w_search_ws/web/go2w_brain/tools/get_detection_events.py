"""get_detection_events — 读落水检测告警历史 (M4)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    detector = ctx.get("detector")
    if detector is None:
        return {"ok": False, "reason": "detector_unavailable"}
    tier = args.get("tier")
    events = detector.events(tier=tier if tier in ("suspect", "confirmed")
                             else None)
    return {"ok": True, "count": len(events),
            "stats": detector.stats(),
            "events": [{"tier": e["tier"], "lat": round(e["lat"], 6),
                        "lng": round(e["lng"], 6),
                        "bearing_deg": e["bearing_deg"],
                        "est_range_m": e["est_range_m"],
                        "confidence": e["confidence"]} for e in events]}


TOOL = ToolRegistration(
    name="get_detection_events",
    description=("读落水检测告警历史 (可按 tier=suspect|confirmed 过滤)。"
                 "任务结束汇报时用 confirmed 清单生成报告。"),
    parameters={"type": "object",
                "properties": {"tier": {"type": "string",
                                        "enum": ["suspect", "confirmed"]}},
                "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
