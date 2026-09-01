"""arm_water_guard — 装载禁区环 (水域/园区多边形, M3 安全层)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    guard = ctx.get("guard")
    if guard is None:
        return {"ok": False, "reason": "guard_unavailable"}
    ring = args.get("polygon")
    if args.get("from_plan") or ring is None:
        plan_store = ctx.get("plan_store") or {}
        last = plan_store.get("last_route") or {}
        ring = last.get("water_polygon") or last.get("campus_polygon")
        if not ring:
            return {"ok": False, "reason": "no_polygon_in_session",
                    "hint": "先 plan_lake_loop/plan_campus_loop, 或显式传 polygon"}
    margin = args.get("margin_m", 2.0)
    result = guard.arm([tuple(p) for p in ring], margin_m=margin)
    if not result.get("ok"):
        return result
    ctx["log"].append("event", event="water_guard_armed",
                      vertices=result["vertices"],
                      margin_m=result["margin_m"])
    # M7.1 安全层消费记忆: hazard 经验确定性地自动进守卫 (不经 LLM
    # 权衡) —— "记忆先让狗更安全"。取规划区域附近的 hazard 条目,
    # 每个以 15m 圆形禁区环叠加布防。
    hazard_count = 0
    memory = ctx.get("memory")
    if memory is not None and args.get("from_plan"):
        last = (ctx.get("plan_store") or {}).get("last_route") or {}
        centroid = (last.get("target") or {}).get("centroid")
        if centroid:
            radius = max(400.0, (last.get("stats") or {})
                         .get("length_m", 400.0) / 2.0)
            radius = min(radius, 5000.0)
            hazards = memory.query(centroid[0], centroid[1], radius,
                                   kinds=("hazard",), min_score=0.3)
            from ..memory import circle_ring
            for hazard in hazards:
                geo = hazard["geo"]
                if "points" in geo:
                    guard.arm([tuple(p) for p in geo["points"]],
                              margin_m=margin)
                else:
                    guard.arm(circle_ring(geo["lat"], geo["lng"], 15.0),
                              margin_m=margin)
                hazard_count += 1
                ctx["log"].append("event", event="hazard_auto_armed",
                                  memory_id=hazard["id"],
                                  dist_m=hazard["dist_m"])
    # 生产链路: 同步布防到 NX 侧守卫节点 (干跑时仅本会话生效,
    # 由返回字段标明 —— 本地守卫仍然拦截规划期违规)
    sync = None
    config = ctx.get("config")
    platform = ctx.get("platform")
    dry = config is not None and getattr(config, "dry_run", False)
    if not dry and hasattr(platform, "arm_water_guard"):
        try:
            sync = platform.arm_water_guard(ring, margin_m=margin)
        except Exception as exc:  # noqa: BLE001
            sync = {"ok": False, "reason": f"sync_failed:{type(exc).__name__}"}
    state = guard.state()
    return {"ok": True, "vertices": result["vertices"],
            "rings": state["rings"], "hazards_armed": hazard_count,
            "margin_m": result["margin_m"], "nx_sync": sync,
            "veto_m": state["veto_m"], "limit_m": state["limit_m"]}


TOOL = ToolRegistration(
    name="arm_water_guard",
    description=(
        "装载地理禁区环并启用离水守卫 (运动类任务的安全前置): 环可来自"
        "本会话最近规划的水域/园区多边形 (from_plan=true, 推荐), 或显式"
        "polygon=[[lat,lng]...]。装载后: 距禁区 <veto_m 禁止动车, <limit_m "
        "限速。follow_route 在守卫未布防时会被拒绝。解除须审批 "
        "(disarm_water_guard)。"),
    parameters={"type": "object",
                "properties": {
                    "from_plan": {"type": "boolean"},
                    "polygon": {"type": "array"},
                    "margin_m": {"type": "number"}},
                "required": []},
    execute=execute,
    risk="act",
    requires=("mission_lock",),
)
