"""follow_route — 受理 GPS 航线 (M2, POST /api/gps/route; M3 加守卫前置)。

risk=act + 前置 mission_lock + water_guard_armed (M3):
任务上下文存在、离水守卫在岗, 才允许发起运动;
受理前做规划期禁区校验 —— 任一航点落在守卫禁区 (含 margin) 内 → 拒绝。
运行期漂移/改道的兜底由运动层守卫节点负责 (本工具是第一道闸)。
干跑模式 (config.dry_run) 只记录不下发, 但守卫校验照常执行。
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
    # M3 纵深防御: 工具自身也校验守卫在岗 (不只依赖 dispatcher 前置)
    guard = ctx.get("guard")
    if guard is not None and not guard.state().get("armed"):
        return {"ok": False, "dispatch_denied": "water_guard_not_armed"}
    # M3 规划期禁区校验: 恶意/错误航线在受理前拦截
    if guard is not None:
        state = guard.state()
        if state.get("armed"):
            offending = _first_offending(waypoints, guard, state)
            if offending is not None:
                ctx["log"].append("event",
                                  event="waypoint_in_keepout_rejected",
                                  waypoint_index=offending[0],
                                  min_dist_m=offending[1])
                return {"ok": False,
                        "reason": "waypoint_in_keepout",
                        "waypoint_index": offending[0],
                        "min_dist_m": offending[1],
                        "hint": ("航线第 %d 点距禁区 %.1fm (阈值 %.1fm), "
                                 "拒绝受理" % (offending[0], offending[1],
                                                state["veto_m"]))}
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


def _first_offending(waypoints, guard, state):
    """返回 (index, min_dist_m): 第一个落入禁区 veto 带内的航点; 无则 None。"""
    if guard.ring_copy() is None:
        return None
    veto_m = state.get("veto_m", 2.0)
    margin = state.get("margin_m", 0.0)
    for i, wp in enumerate(waypoints):
        try:
            dist = guard.distance_m(float(wp["lat"]), float(wp["lon"]))
        except (KeyError, TypeError, ValueError):
            continue
        if dist is not None and dist - margin < veto_m:
            return i, round(dist, 1)
    return None


TOOL = ToolRegistration(
    name="follow_route",
    description=(
        "受理并开始沿 GPS 航线行走。前置: calibrate_heading 北向标定 + "
        "arm_water_guard 守卫在岗 (M3 起强制)。"
        "两种给航线方式: ① from_plan=true 直接引用本会话最近一次规划"
        "(推荐, 航点不经上下文搬运); ② 显式 waypoints 数组 (lat/lon)。"
        "受理前校验: 任一航点落入守卫禁区 → 拒绝 (waypoint_in_keepout); "
        "服务器侧 fail-closed: 未标定/GPS 不健康/已有航线 → ok=false+reason。"),
    parameters={"type": "object",
                "properties": {
                    "waypoints": {"type": "array"},
                    "from_plan": {"type": "boolean"},
                    "mode": {"type": "string", "enum": ["patrol"]}},
                "required": []},
    execute=execute,
    risk="act",
    requires=("mission_lock", "water_guard_armed"),
)
