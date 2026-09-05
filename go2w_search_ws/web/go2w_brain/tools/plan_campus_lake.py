"""plan_campus_lake — 园区湖语义链 (2026-09-05): 先园区, 后园区里的湖。

用户架构要求: "绕着当前园区湖绕行一圈" 必须先识别园区 (本体所属园区),
再在园区内部找湖 —— 而不是"找最近的水体" (旧行为把园区外 0.6km 的河段
当湖, 2026-09-04 实测暴露)。

链路 (每步留痕):
  A. 园区识别: 任务文本/参数 与园区知识库 (CAMPUSES) 名称匹配 →
     园区中心 + 名称; 未匹配 → 诚实失败 (unknown_campus, 列出已知园区)。
  B. 园区内找湖: esri 卫星拼接 (z17 8×8, 以园区中心) → HSV 水体分割
     (water_mask_hsv) → 连通域候选 (园区半径内, 按面积取前 10) →
     VLM 锚定确认"哪块是园区里的湖" (失败→规则取最近)。
  C. 绕行规划: 目标水体多边形 → 离岸环线 (15m) + 5m 安全推出 → 扫描点。
事件: campus_identified / semantic_anchor / plan_result / geometry_persisted。
"""
from __future__ import annotations

import math
from typing import Any

from ..registry import ToolRegistration

# 园区知识库 (演示版: 部署时按园区扩展; 真机可换 POI 检索服务)。
# 中电海康无锡物联网产业园: 360地图 POI (清晏路32号/净慧东道78号, 新吴区新安)。
CAMPUSES = [
    {"name": "中电海康无锡物联网产业园",
     "aliases": ("海康", "中电海康", "hikvision"),
     "lat": 31.4848, "lng": 120.3747,
     "radius_m": 500.0, "source": "poi:360map"},
]

_SAT_Z, _SAT_NX, _SAT_NY = 17, 8, 8


def _hav_m(lat1, lng1, lat2, lng2):
    return 6378137 * math.acos(min(1.0, math.sin(math.radians(lat1))
        * math.sin(math.radians(lat2)) + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2)) * math.cos(math.radians(lng2 - lng1))))


def _match_campus(task_text: str, args: dict[str, Any]) -> dict | None:
    want = str(args.get("campus") or "")
    text = f"{task_text} {want}".lower()
    for c in CAMPUSES:
        names = [c["name"].lower()] + [a.lower() for a in c["aliases"]]
        if any(n in text for n in names):
            return c
    return None


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    task_text = str(args.get("task") or ctx.get("task") or "")

    # ---------- A. 园区识别 (名字匹配 → 本体所在园区兜底) ----------
    campus = _match_campus(task_text, args)
    if campus is None:
        gps = (ctx["platform"].snapshot().get("gps") or {})
        if gps.get("available"):
            glat, glng = float(gps["lat"]), float(gps["lng"])
            best, best_d = None, float("inf")
            for c in CAMPUSES:
                d = _hav_m(glat, glng, c["lat"], c["lng"])
                if d <= c["radius_m"] * 1.5 and d < best_d:
                    best, best_d = c, d
            campus = best
    if campus is None:
        known = "、".join(c["name"] for c in CAMPUSES)
        return {"ok": False, "reason": "unknown_campus",
                "hint": f"任务未指明园区且本体不在任何已知园区内; 已知园区: "
                        f"{known}。绕非园区水体请用 plan_lake_loop"}
    ctx["log"].append("event", event="campus_identified",
                      campus=campus["name"], source=campus["source"],
                      center=[campus["lat"], campus["lng"]])

    # ---------- B. 园区内找湖 (卫星分割 + 语义锚定) ----------
    try:
        from lake_plan import tiles, water
    except ImportError as exc:
        return {"ok": False, "reason": "lake_plan_import_failed",
                "detail": str(exc)}
    clat, clng = campus["lat"], campus["lng"]
    try:
        img, detail = tiles.stitch_centered(
            "esri", clat, clng, _SAT_Z, _SAT_NX, _SAT_NY)
    except tiles.TileError as exc:
        return {"ok": False, "reason": f"satellite_unavailable:{exc}"}
    georef = tiles.georef_from_detail(detail)
    mask = water.water_mask_hsv(img)
    comps = water.label_components(mask, down=4, min_area_px=150)

    near = []
    for c in comps:
        cy, cx = c["centroid_small"]
        lat, lng = georef.pixel_to_latlon(cx * c["down"], cy * c["down"])
        dist = _hav_m(clat, clng, lat, lng)
        if dist <= campus["radius_m"]:
            near.append((dist, c, [round(lat, 5), round(lng, 5)]))
    near.sort(key=lambda t: t[0])
    if not near:
        return {"ok": False, "reason": "no_water_in_campus",
                "campus": campus["name"],
                "hint": f"园区 {campus['radius_m']}m 半径内卫星图未检出"
                        f"候选水体 (蓝屋顶等误报已按尺寸过滤)"}
    cands = [{"centroid": c3, "dist_km": round(d / 1000.0, 3)}
             for (d, _, c3) in near[:10]]
    ctx["log"].append("event", event="campus_water_candidates",
                      campus=campus["name"], count=len(cands))

    from lake_plan import semantic_anchor
    anchor = semantic_anchor.anchor_semantics(
        ctx.get("vlm"), task_text or "绕着园区里的湖绕行一圈",
        {"lat": clat, "lng": clng}, cands, (clat, clng),
        provider="esri", z=_SAT_Z, nx=_SAT_NX, ny=_SAT_NY)
    ctx["log"].append("event", event="semantic_anchor",
                      source=anchor["source"], target=anchor.get("target"),
                      ambiguity=anchor.get("ambiguity"),
                      resolved_to_plan=(
                          (anchor.get("target") or {}).get("idx", 0) == 0))
    tgt = anchor.get("target") or {}
    tgt_centroid = tgt.get("centroid")
    if not tgt_centroid:
        return {"ok": False, "reason": "anchor_failed",
                "anchor": anchor}
    # 锚定质心 → 找回对应连通域
    pick = min(near, key=lambda t: (t[2][0] - tgt_centroid[0]) ** 2
               + (t[2][1] - tgt_centroid[1]) ** 2)
    _, comp, comp_ll = pick

    # ---------- C. 绕行规划 ----------
    poly = water.polygon_from_component(comp, mask.shape, allow_hull=True)
    if len(poly) < 4:
        return {"ok": False, "reason": "lake_polygon_degenerate"}
    poly_ll = [georef.pixel_to_latlon(x, y) for (x, y) in poly]
    perim_m, area_m2 = water.poly_stats_latlon(poly_ll, clat)
    from lake_plan import planner, route_api
    try:
        result = planner.plan_loop_around_polygon(
            poly, georef, offset_m=15.0, step_m=40.0)
    except ValueError as exc:
        return {"ok": False, "reason": f"plan_failed:{exc}"}
    result["route_latlon"] = route_api._snap_outside_ring_min(
        result["route_latlon"], poly_ll, min_dist_m=5.0)
    out = route_api._finish(result, poly_ll, "water", {
        "area_km2": round(area_m2 / 1e6, 3),
        "perim_km": round(perim_m / 1000.0, 2),
        "dist_km": round(pick[0] / 1000.0, 3),
        "campus": campus["name"]})
    out["anchor"] = anchor
    out["campus"] = campus["name"]

    # 引用传递 + 记忆持久化 (与 plan_lake_loop 同一契约)
    plan_store = ctx.get("plan_store")
    if plan_store is not None:
        plan_store["last_route"] = out
    ctx["log"].append("event", event="plan_result", plan_kind="campus_lake",
                      waypoint_count=len(out["waypoints"]),
                      length_m=result["stats"]["length_m"],
                      campus=campus["name"], target=out["target"])
    from ..plan_memory import persist_plan
    mem_id = persist_plan(ctx.get("memory"), out)
    if mem_id:
        ctx["log"].append("event", event="geometry_persisted",
                          memory_id=mem_id)

    from ..patrol_math import endurance_check
    battery = ctx["platform"].snapshot().get("battery_soc")
    endurance = endurance_check(result["stats"]["length_m"], battery)
    return {
        "ok": True,
        "campus": campus["name"],
        "waypoint_count": len(out["waypoints"]),
        "scan_points": len(out["scan_points"]),
        "length_km": round(result["stats"]["length_m"] / 1000.0, 2),
        "closed": result["stats"]["closed"],
        "water_cross_ratio": result["stats"]["water_cross_ratio"],
        "target": out["target"],
        "anchor": {"source": anchor.get("source"),
                   "self": anchor.get("self"),
                   "target": anchor.get("target"),
                   "ambiguity": anchor.get("ambiguity") or []},
        "endurance": endurance,
        "first_waypoints": out["waypoints"][:3],
        "usage": "调用 follow_route 并传 from_plan=true 即可受理这条航线",
    }


TOOL = ToolRegistration(
    name="plan_campus_lake",
    description=(
        "绕【园区里的湖】规划环线 (先识别园区, 再在园区内找湖): 任务说"
        "\"园区湖/当前园区湖/我们园区的湖\"时用它 —— 与 plan_lake_loop 不同,"
        "它不会选园区外的最近水体。链路: 园区知识库匹配园区中心 → esri 卫星"
        "分割出园区半径内候选水体 → 多模态大模型确认哪块是园区里的湖 → "
        "离岸环线规划。已知园区: 中电海康无锡物联网产业园。"
        "绕非园区水体 (不点名园区) 才用 plan_lake_loop。"),
    parameters={"type": "object",
                "properties": {
                    "campus": {"type": "string"},
                    "lat": {"type": "number"},
                    "lng": {"type": "number"}},
                "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
