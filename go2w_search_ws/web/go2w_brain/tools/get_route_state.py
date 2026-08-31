"""get_route_state — 读当前 GPS 航线进度 (M2)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    state = ctx["platform"].gps_state() or {}
    summary = {
        "active": bool(state.get("active")),
        "status": state.get("status"),
        "waypoint_index": state.get("waypoint_index"),
        "waypoint_total": state.get("waypoint_total"),
        "reason": state.get("reason"),
    }
    return {"ok": True, "route": state, "summary": summary}


TOOL = ToolRegistration(
    name="get_route_state",
    description=("读取当前 GPS 航线状态: 是否 active、当前航点序号/总数、"
                 "停航原因。巡逻期间用它检查进度。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
