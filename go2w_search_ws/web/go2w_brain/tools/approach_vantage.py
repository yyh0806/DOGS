"""approach_vantage — 为 confirmed 告警选安全接近点 (M5)。

安全构造: 接近点取自本会话规划环线上距告警最近的航点 —— 环线经
规划-安全一致性保证 (距禁区 ≥5m), 接近点天然在 keepout 外;
再对守卫环做矢量校验兜底 (纵深防御)。
"""
from __future__ import annotations

from typing import Any

from ..patrol_math import nearest_route_point
from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    lat = args.get("lat")
    lng = args.get("lng")
    if not isinstance(lat, (int, float)) or not isinstance(lng, (int, float)):
        return {"ok": False, "reason": "invalid_alert_position"}
    plan_store = ctx.get("plan_store") or {}
    last = plan_store.get("last_route") or {}
    waypoints = last.get("waypoints")
    if not waypoints:
        return {"ok": False, "reason": "no_route_in_session",
                "hint": "接近点取自规划环线, 先 plan_lake_loop/plan_campus"}
    index, dist_m = nearest_route_point(waypoints, float(lat), float(lng))
    vantage = waypoints[index]
    # 矢量兜底: 守卫布防时校验 vantage 不在禁区 veto 带内
    guard = ctx.get("guard")
    guard_dist = None
    if guard is not None and guard.state().get("armed"):
        guard_dist = guard.distance_m(vantage["lat"], vantage["lon"])
        state = guard.state()
        if (guard_dist is not None
                and guard_dist - state.get("margin_m", 0.0)
                < state.get("veto_m", 2.0)):
            ctx["log"].append("event",
                              event="vantage_in_keepout_rejected",
                              route_index=index)
            return {"ok": False, "reason": "vantage_in_keepout",
                    "hint": "环线最近点落入守卫 veto 带, 放弃接近"}
    ctx["log"].append("event", event="vantage_selected",
                      route_index=index, dist_to_alert_m=dist_m)
    # M7 回写: 观察点经验入记忆 (下次任务直接检索可用观察点)
    memory = ctx.get("memory")
    if memory is not None:
        try:
            entry = memory.record(
                "vantage", {"lat": vantage["lat"], "lon": vantage["lon"]},
                data={"dist_to_alert_m": dist_m,
                      "guard_dist_m": round(guard_dist, 1)
                      if guard_dist is not None else None},
                confidence=0.7)
            ctx["log"].append("event", event="memory_recorded",
                              memory_id=entry["id"], mem_kind="vantage")
        except ValueError:
            pass
    return {
        "ok": True,
        "vantage": {"lat": vantage["lat"], "lon": vantage["lon"],
                    "route_index": index,
                    "route_name": vantage.get("name", f"wp{index:03d}")},
        "dist_to_alert_m": dist_m,
        "guard_dist_m": (round(guard_dist, 1)
                         if guard_dist is not None else None),
        "usage": ("干跑/演练: 已选出安全观察点; 实机 M6 接入 go-to。"
                  "接近后可再 scan_water 近距复查"),
    }


TOOL = ToolRegistration(
    name="approach_vantage",
    description=(
        "为一条 confirmed 落水告警选择安全接近观察点: 取本会话规划环线"
        "上距告警最近的航点 (环线本身保证在守卫禁区外), 并对守卫做矢量"
        "兜底校验。返回观察点坐标与距告警距离。前提: 已有规划 + 守卫布防。"),
    parameters={"type": "object",
                "properties": {"lat": {"type": "number"},
                               "lng": {"type": "number"}},
                "required": ["lat", "lng"]},
    execute=execute,
    risk="act",
    requires=("mission_lock", "water_guard_armed"),
)
