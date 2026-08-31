"""plan_lake_loop — 绕湖航线规划 (M2, 调 lake_plan 确定性核心)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    lat = args.get("lat")
    lng = args.get("lng")
    if lat is None or lng is None:
        gps = ctx["platform"].snapshot().get("gps") or {}
        if not gps.get("available"):
            return {"ok": False, "reason": "no_gps_and_no_explicit_center",
                    "hint": "显式传 lat/lng, 或先等 GNSS 定位可用"}
        lat, lng = gps["lat"], gps["lng"]
    offset = args.get("offset_m", 15.0)
    offset = min(max(float(offset), 5.0), 500.0)
    try:
        from lake_plan import plan_route  # 姗姗导入: numpy/PIL 只在此需要
    except ImportError as exc:
        return {"ok": False, "reason": "lake_plan_import_failed",
                "detail": str(exc)}
    result = plan_route(float(lat), float(lng), offset_m=offset)
    result.setdefault("center", [lat, lng])
    if not result.get("ok"):
        return result
    # 引用传递 (上下文经济): 全量结果入 plan_store, LLM 只看摘要;
    # follow_route(from_plan=true) 直接取用, 不经模型上下文搬运。
    plan_store = ctx.get("plan_store")
    if plan_store is not None:
        plan_store["last_route"] = result
        ctx["log"].append("event", event="plan_result",
                          waypoint_count=len(result["waypoints"]),
                          length_m=result["stats"]["length_m"],
                          offset_m=result["stats"]["offset_m"],
                          lake=result["lake"])
    return {
        "ok": True,
        "waypoint_count": len(result["waypoints"]),
        "length_km": round(result["stats"]["length_m"] / 1000.0, 2),
        "closed": result["stats"]["closed"],
        "water_cross_ratio": result["stats"]["water_cross_ratio"],
        "lake": result["lake"],
        "first_waypoints": result["waypoints"][:3],
        "usage": "调用 follow_route 并传 from_plan=true 即可受理这条航线",
    }


TOOL = ToolRegistration(
    name="plan_lake_loop",
    description=(
        "规划绕湖巡查环线 (确定性几何引擎, 瓦片缓存优先)。以当前 GNSS 定位"
        "(或显式 lat/lng) 为中心, 自动选湖并生成离岸闭合环线。"
        "返回 waypoints (WGS-84, 可直接喂给 follow_route)、水域多边形与统计。"
        "离岸距离 offset_m 默认 15m (5-500)。"),
    parameters={"type": "object",
                "properties": {
                    "lat": {"type": "number"},
                    "lng": {"type": "number"},
                    "offset_m": {"type": "number"}},
                "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
