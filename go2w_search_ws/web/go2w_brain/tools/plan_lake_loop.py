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
    # M7.1 L1 几何层: 先查记忆复用, 命中即免瓦片 (二次任务零网络)
    from ..plan_memory import persist_plan, try_reuse
    reused = try_reuse(ctx.get("memory"), "water", float(lat), float(lng))
    if reused is not None:
        result = reused
        ctx["log"].append("event", event="geometry_reused",
                          memory_id=result["memory_id"])
    else:
        try:
            from lake_plan import plan_route  # 姗姗导入: numpy/PIL 只在此需要
        except ImportError as exc:
            return {"ok": False, "reason": "lake_plan_import_failed",
                    "detail": str(exc)}
        result = plan_route(float(lat), float(lng), offset_m=offset)
        result.setdefault("center", [lat, lng])
        if result.get("ok"):
            # M7.3 语义锚定: 本体在哪 / 湖是哪个 (VLM 确认, 失败退化规则);
            # VLM 认定任务所指是另一候选水体 → 以其质心重规划 (至多一次)。
            task_text = str(args.get("_task") or ctx.get("task") or "")
            anchor = _anchor_plan(result, float(lat), float(lng),
                                  task_text, ctx)
            if (anchor.get("source") == "vlm"
                    and (anchor.get("target") or {}).get("idx", 0) != 0):
                prefer = (anchor.get("target") or {}).get("centroid")
                if prefer and len(prefer) == 2:
                    ctx["log"].append("event", event="anchor_replan",
                                      prefer=[float(prefer[0]),
                                              float(prefer[1])])
                    corrected = plan_route(float(lat), float(lng),
                                           offset_m=offset,
                                           prefer=(float(prefer[0]),
                                                   float(prefer[1])))
                    if corrected.get("ok"):
                        corrected.setdefault("center", [lat, lng])
                        result = corrected
                        anchor = _anchor_plan(result, float(lat),
                                              float(lng), task_text, ctx)
            result["anchor"] = anchor
            ctx["log"].append("event", event="semantic_anchor",
                              source=anchor["source"],
                              target=anchor["target"],
                              ambiguity=anchor.get("ambiguity"),
                              resolved_to_plan=(
                                  (anchor.get("target") or {})
                                  .get("idx", 0) == 0))
            mem_id = persist_plan(ctx.get("memory"), result)
            if mem_id:
                ctx["log"].append("event", event="geometry_persisted",
                                  memory_id=mem_id)
    if not result.get("ok"):
        return result
    # 引用传递 (上下文经济): 全量结果入 plan_store, LLM 只看摘要;
    # follow_route(from_plan=true) 直接取用, 不经模型上下文搬运。
    plan_store = ctx.get("plan_store")
    if plan_store is not None:
        plan_store["last_route"] = result
        # 注意: 不能用 kind= 作关键字 (SessionLog.append 首参即 kind)
        ctx["log"].append("event", event="plan_result",
                          plan_kind="water",
                          waypoint_count=len(result["waypoints"]),
                          length_m=result["stats"]["length_m"],
                          offset_m=result["stats"]["offset_m"],
                          target=result["target"])
    # M5: 续航判决 (电量取自平台遥测)
    from ..patrol_math import endurance_check
    battery = ctx["platform"].snapshot().get("battery_soc")
    endurance = endurance_check(result["stats"]["length_m"], battery)
    anchor = result.get("anchor") or {}
    return {
        "ok": True,
        "waypoint_count": len(result["waypoints"]),
        "scan_points": len(result.get("scan_points") or []),
        "length_km": round(result["stats"]["length_m"] / 1000.0, 2),
        "closed": result["stats"]["closed"],
        "water_cross_ratio": result["stats"]["water_cross_ratio"],
        "target": result["target"],
        "anchor": {"source": anchor.get("source"),
                   "self": anchor.get("self"),
                   "target": anchor.get("target"),
                   "ambiguity": anchor.get("ambiguity") or []},
        "endurance": endurance,
        "first_waypoints": result["waypoints"][:3],
        "usage": "调用 follow_route 并传 from_plan=true 即可受理这条航线"
                 + ("; 电量不足, 建议分段" if endurance.get("verdict")
            in ("SEGMENT", "REFUSE") else ""),
    }


def _anchor_plan(result, lat, lng, task, ctx):
    """M7.3: 语义锚定 (VLM 确认本体与目标水体; 不可用→规则退化)。"""
    from lake_plan import semantic_anchor
    vlm = ctx.get("vlm")
    center = (float(lat), float(lng))
    candidates = [{"centroid": result["target"]["centroid"],
                   "dist_km": 0.0}]
    candidates += result.get("nearby_candidates") or []
    return semantic_anchor.anchor_semantics(
        vlm, task or "绕湖巡查", {"lat": center[0], "lng": center[1]},
        candidates, center)


TOOL = ToolRegistration(
    name="plan_lake_loop",
    description=(
        "规划绕湖巡查环线 (确定性几何引擎, 瓦片缓存优先): 以当前 GNSS 定位"
        "(或显式 lat/lng) 为中心, 选择**距离最近的合格水体**(小到园区景观湖,"
        "大到天然湖泊), 沿其岸线外侧 offset_m 米(默认15)生成闭合环线。"
        "返回摘要; 调 follow_route(from_plan=true) 受理。"
        "若任务要绕的是园区/厂区轮廓而非湖, 用 plan_campus_loop。"),
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
