"""get_pose — 读取当前位姿 (localization/odometry 宽容提取)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    snapshot = ctx["platform"].snapshot()
    pose = snapshot.get("pose") or {}
    if not pose.get("available"):
        return {"ok": False, "reason": pose.get("reason", "no_pose")}
    return {"ok": True, "x": pose["x"], "y": pose["y"],
            "yaw_deg": pose.get("yaw_deg")}


TOOL = ToolRegistration(
    name="get_pose",
    description=("读取当前位姿 (导航系 x/y 米, yaw_deg 度)。"
                 "无位姿时返回 {\"ok\": false, \"reason\": ...}。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
