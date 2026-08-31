"""follow_route — 受理 GPS 航线 (M2, POST /api/gps/route)。

risk=act + 前置 mission_lock: 任务上下文存在才允许发起运动;
干跑模式 (config.dry_run) 只记录不下发。
"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    waypoints = args.get("waypoints")
    if args.get("from_plan"):
        plan_store = ctx.get("plan_store") or {}
        last = plan_store.get("last_route")
        if not last or not last.get("waypoints"):
            return {"ok": False, "reason": "no_plan_in_session",
                    "hint": "先调用 plan_lake_loop"}
        waypoints = last["waypoints"]
    if not isinstance(waypoints, list) or not waypoints:
        return {"ok": False, "reason": "invalid_waypoints",
                "hint": "传 waypoints 数组, 或 from_plan=true 引用最近规划"}
    normalized = []
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            return {"ok": False, "reason": f"invalid_waypoint:{i}"}
        lat, lon = wp.get("lat"), wp.get("lon")
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            return {"ok": False, "reason": f"invalid_waypoint:{i}",
                    "hint": "航点字段是 lat/lon (WGS-84)"}
        normalized.append({"lat": float(lat), "lon": float(lon),
                           **({"name": wp["name"]} if wp.get("name") else {})})
    config = ctx.get("config")
    if config is not None and getattr(config, "dry_run", False):
        ctx["log"].append("event", event="follow_route_dry",
                          waypoint_count=len(normalized))
        return {"ok": True, "dry": True, "waypoint_count": len(normalized),
                "note": "干跑模式: 已记录, 未下发到机器人"}
    result = ctx["platform"].submit_gps_route(normalized)
    return result


TOOL = ToolRegistration(
    name="follow_route",
    description=(
        "受理并开始沿 GPS 航线行走 (需先 calibrate_heading 北向标定)。"
        "两种给航线方式: ① from_plan=true 直接引用本会话最近一次 "
        "plan_lake_loop 的结果 (推荐, 航点不经上下文搬运); "
        "② 显式 waypoints 数组 (lat/lon)。"
        "服务器侧 fail-closed: 未标定/GPS 不健康/已有航线 → ok=false+reason。"),
    parameters={"type": "object",
                "properties": {
                    "waypoints": {"type": "array"},
                    "from_plan": {"type": "boolean"},
                    "mode": {"type": "string", "enum": ["patrol"]}},
                "required": []},
    execute=execute,
    risk="act",
    requires=("mission_lock",),
)
