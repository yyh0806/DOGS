"""plan_campus_loop — 绕园区环线规划 (M2.1, Overpass landuse 聚类 + 凸包)。"""
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
        from lake_plan import plan_route
    except ImportError as exc:
        return {"ok": False, "reason": "lake_plan_import_failed",
                "detail": str(exc)}
    result = plan_route(float(lat), float(lng), kind="campus",
                        offset_m=offset)
    if not result.get("ok"):
        return result
    plan_store = ctx.get("plan_store")
    if plan_store is not None:
        plan_store["last_route"] = result
        # 注意: 不能用 kind= 作关键字 (SessionLog.append 首参即 kind)
        ctx["log"].append("event", event="plan_result",
                          plan_kind="campus",
                          waypoint_count=len(result["waypoints"]),
                          length_m=result["stats"]["length_m"],
                          offset_m=result["stats"]["offset_m"],
                          target=result["target"])
    # M5: 续航判决 (电量取自平台遥测)
    from ..patrol_math import endurance_check
    battery = ctx["platform"].snapshot().get("battery_soc")
    endurance = endurance_check(result["stats"]["length_m"], battery)
    return {
        "ok": True,
        "waypoint_count": len(result["waypoints"]),
        "scan_points": len(result.get("scan_points") or []),
        "length_km": round(result["stats"]["length_m"] / 1000.0, 2),
        "closed": result["stats"]["closed"],
        "target": result["target"],
        "endurance": endurance,
        "first_waypoints": result["waypoints"][:3],
        "usage": "调用 follow_route 并传 from_plan=true 即可受理这条航线"
                 + ("; 电量不足, 建议分段" if endurance.get("verdict")
            in ("SEGMENT", "REFUSE") else ""),
    }


TOOL = ToolRegistration(
    name="plan_campus_loop",
    description=(
        "规划绕园区(产业园/厂区等)一圈的巡查环线: 以当前 GNSS 定位(或显式 "
        "lat/lng) 为中心, 从 OSM landuse 地块聚类出园区轮廓, 沿其外侧"
        "offset_m 米(默认15)生成闭合环线。当前点位不在任何园区地块内时"
        "返回 not_inside_any_plot。与 plan_lake_loop (绕最近的湖) 对应, "
        "按任务语义选择。"),
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
