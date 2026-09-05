"""plan_campus_lake — 园区湖语义链 (2026-09-05, VLM 直接 mask 版)。

用户要求: "绕着当前园区湖绕行一圈" 必须先识别园区, 再在园区内找湖;
且希望【卫星图上直接用 VLM 做 mask 圈出园区/湖】, 而不是画半径圆或
切街道图。

链路 (每步留痕):
  A. 园区识别: 任务文本/本体位置 → 园区知识库 (CAMPUSES) → 园区中心+名称;
  B. VLM 圈园区: esri 卫星 z17 拼接 → VLM 直接勾画园区边界多边形 (归一化
     坐标 → 经纬度 mask); VLM 不可用/退化 → 规则半径圆后备;
  C. VLM 圈湖: 按园区 mask bbox 裁剪放大 (z18) 卫星 → VLM 勾画湖岸线 mask
     (质心必须在园区 mask 内); 失败 → 规则后备链路 (OSM/HSV 候选+湖形
     过滤+锚定, 见 _plan_rule_fallback);
  D. 绕行规划: 沿湖 mask 多边形离岸环线 (15m) + 5m 安全推出 → 扫描点。
事件: campus_identified / campus_mask / lake_mask / semantic_anchor /
plan_result / geometry_persisted。
"""
from __future__ import annotations

import math
from typing import Any

from ..registry import ToolRegistration

# 园区知识库 (演示版: 部署时按园区扩展; 真机可换 POI 检索服务)。
# 中电海康无锡物联网产业园: 锚点 = 用户确认的机器人坐标 (2026-09-05)。
CAMPUSES = [
    {"name": "中电海康无锡物联网产业园",
     "aliases": ("海康", "中电海康", "hikvision"),
     "lat": 31.488192, "lng": 120.369486,
     "radius_m": 400.0, "source": "user-confirmed:2026-09-05"},
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


def _zoom_to_bbox(provider, ring_ll, z=18):
    """园区 mask bbox → 放大卫星窗口 (z 起步逐级降, 2×2~8×8 瓦片)。"""
    from lake_plan import tiles
    from lake_plan.geo import lat_to_global_px, lng_to_global_px
    lats = [p[0] for p in ring_ll]
    lngs = [p[1] for p in ring_ll]
    w, s, e, n = min(lngs), min(lats), max(lngs), max(lats)
    for zz in range(z, 15, -1):
        gx0 = int(lng_to_global_px(w, zz) // 256) - 1
        gx1 = int(lng_to_global_px(e, zz) // 256) + 1
        gy0 = int(lat_to_global_px(n, zz) // 256) - 1
        gy1 = int(lat_to_global_px(s, zz) // 256) + 1
        nx, ny = gx1 - gx0 + 1, gy1 - gy0 + 1
        if 2 <= nx <= 8 and 2 <= ny <= 8:
            img, detail = tiles.stitch_area(provider, zz, gx0, gy0, nx, ny)
            return img, tiles.georef_from_detail(detail)
    raise tiles.TileError("campus_bbox_too_large")


def _centroid_ll(ring_ll):
    return (sum(p[0] for p in ring_ll) / len(ring_ll),
            sum(p[1] for p in ring_ll) / len(ring_ll))


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    task_text = str(args.get("task") or ctx.get("task") or "")
    from lake_plan import tiles, water
    from lake_plan import vlm_mask

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
    clat, clng = campus["lat"], campus["lng"]
    vlm = ctx.get("vlm")

    # ---------- B. VLM 直接在卫星图上圈园区 (mask) ----------
    try:
        sat_img, sat_detail = tiles.stitch_centered(
            "esri", clat, clng, _SAT_Z, _SAT_NX, _SAT_NY)
    except tiles.TileError as exc:
        return {"ok": False, "reason": f"satellite_unavailable:{exc}"}
    sat_geo = tiles.georef_from_detail(sat_detail)
    campus_poly = vlm_mask.vlm_polygon(vlm, sat_img,
                                       vlm_mask.CAMPUS_MASK_PROMPT)
    if campus_poly is not None:
        campus_ll = vlm_mask.poly_to_latlon(campus_poly["polygon"], sat_geo)
        mask_source = "vlm"
    else:
        campus_ll = vlm_mask.circle_poly(clat, clng, campus["radius_m"])
        mask_source = "rule"
    ctx["log"].append("event", event="campus_mask", campus=campus["name"],
                      source=mask_source, vertices=len(campus_ll))

    # ---------- C. 园区 mask 内找湖: 放大卫星 → VLM 圈湖 ----------
    lake_ll, lake_geo, anchor = None, None, None
    try:
        lake_img, lake_geo = _zoom_to_bbox("esri", campus_ll, z=18)
        lake_poly = vlm_mask.vlm_polygon(vlm, lake_img,
                                         vlm_mask.LAKE_MASK_PROMPT)
        if lake_poly is not None:
            cand = vlm_mask.poly_to_latlon(lake_poly["polygon"], lake_geo)
            from lake_plan.osm_client import _point_in_ring
            c_lat, c_lng = _centroid_ll(cand)
            if _point_in_ring((c_lat, c_lng), campus_ll):
                lake_ll = cand
                anchor = {"source": "vlm",
                          "self": {"lat": clat, "lng": clng,
                                   "confirmed": True},
                          "target": {"idx": 0,
                                     "centroid": [round(c_lat, 5),
                                                  round(c_lng, 5)],
                                     "why": str(lake_poly.get("why") or "")},
                          "ambiguity": [],
                          "why": str(lake_poly.get("why") or "")}
    except tiles.TileError:
        pass
    if lake_ll is not None:
        ctx["log"].append("event", event="lake_mask", source="vlm",
                          vertices=len(lake_ll))
        ctx["log"].append("event", event="semantic_anchor",
                          source=anchor["source"], target=anchor["target"],
                          ambiguity=anchor.get("ambiguity"),
                          resolved_to_plan=True)
    else:
        # 规则后备: OSM/HSV 候选 + 湖形过滤 + 锚定 (老链路)
        fallback = _plan_rule_fallback(ctx, task_text, campus, clat, clng,
                                       vlm, tiles, water)
        if isinstance(fallback, dict) and not fallback.get("ok", True):
            return fallback
        lake_ll, lake_geo, anchor = fallback

    # ---------- D. 沿湖 mask 规划环线 (离岸 15m + 5m 安全推出) ----------
    from lake_plan import planner, route_api
    poly_px = [lake_geo.latlon_to_pixel(a, b) for a, b in lake_ll]
    perim_m, area_m2 = water.poly_stats_latlon(lake_ll, clat)
    try:
        result = planner.plan_loop_around_polygon(
            poly_px, lake_geo, offset_m=15.0, step_m=40.0)
    except ValueError as exc:
        return {"ok": False, "reason": f"plan_failed:{exc}"}
    result["route_latlon"] = route_api._snap_outside_ring_min(
        result["route_latlon"], lake_ll, min_dist_m=5.0)
    tgt_c = anchor.get("target") or {}
    tgt_c = tgt_c.get("centroid") or list(_centroid_ll(lake_ll))
    out = route_api._finish(result, lake_ll, "water", {
        "area_km2": round(area_m2 / 1e6, 3),
        "perim_km": round(perim_m / 1000.0, 2),
        "dist_km": round(_hav_m(clat, clng, tgt_c[0], tgt_c[1]) / 1000.0, 3),
        "campus": campus["name"]})
    out["anchor"] = anchor
    out["campus"] = campus["name"]

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


def _plan_rule_fallback(ctx, task_text, campus, clat, clng, vlm,
                        tiles, water):
    """VLM mask 失败时的规则后备 (老链路): OSM/HSV 候选 + 湖形过滤 + 锚定。

    返回 (lake_ll, lake_geo, anchor) 或 {"ok": False, "reason": ...}。
    """
    near, georef, mask = [], None, None
    for source, provider, mask_fn, min_area in (
            ("osm", "osm", lambda im: water.water_mask(
                im, ref_rgb=(170, 211, 223)), 200),
            ("satellite", "esri", water.water_mask_hsv, 150)):
        try:
            img, detail = tiles.stitch_centered(
                provider, clat, clng, _SAT_Z, _SAT_NX, _SAT_NY)
        except tiles.TileError:
            continue
        georef = tiles.georef_from_detail(detail)
        mask = mask_fn(img)
        comps = water.label_components(mask, down=4, min_area_px=min_area)
        for c in comps:
            cy, cx = c["centroid_small"]
            lat, lng = georef.pixel_to_latlon(cx * c["down"], cy * c["down"])
            dist = _hav_m(clat, clng, lat, lng)
            if dist <= campus["radius_m"]:
                near.append((dist, c, [round(lat, 5), round(lng, 5)]))
        if near:
            break
    lake_like = []
    for dist, c, c3 in near:
        poly = water.polygon_from_component(c, mask.shape, allow_hull=True)
        if len(poly) < 4:
            continue
        poly_ll = [georef.pixel_to_latlon(x, y) for (x, y) in poly]
        perim_m, area_m2 = water.poly_stats_latlon(poly_ll, clat)
        compact = (4 * math.pi * area_m2 / (perim_m ** 2)) if perim_m else 0.0
        if area_m2 >= 300.0 and compact >= 0.12:
            lake_like.append((dist, c, c3, poly, poly_ll,
                              perim_m, area_m2, compact))
    lake_like.sort(key=lambda t: t[0])
    if not lake_like:
        if not near:
            return {"ok": False, "reason": "no_water_in_campus",
                    "campus": campus["name"],
                    "hint": f"园区 {campus['radius_m']}m 半径内 OSM 渲染与"
                            f"卫星图均未检出候选水体"}
        return {"ok": False, "reason": "no_lake_like_water_in_campus",
                "campus": campus["name"],
                "hint": "园区半径内只有细长河道/碎斑, 无湖形水体"}
    cands = [{"centroid": c3, "dist_km": round(d / 1000.0, 3)}
             for (d, _, c3, *_rest) in lake_like[:10]]
    ctx["log"].append("event", event="campus_water_candidates",
                      campus=campus["name"], count=len(cands),
                      source="rule")
    from lake_plan import semantic_anchor
    anchor = semantic_anchor.anchor_semantics(
        vlm, task_text or "绕着园区里的湖绕行一圈",
        {"lat": clat, "lng": clng}, cands, (clat, clng),
        provider="esri", z=_SAT_Z, nx=_SAT_NX, ny=_SAT_NY)
    ctx["log"].append("event", event="semantic_anchor",
                      source=anchor["source"], target=anchor.get("target"),
                      ambiguity=anchor.get("ambiguity"),
                      resolved_to_plan=(
                          (anchor.get("target") or {}).get("idx", 0) == 0))
    tgt_centroid = (anchor.get("target") or {}).get("centroid")
    if not tgt_centroid:
        return {"ok": False, "reason": "anchor_failed", "anchor": anchor}
    pick_i = min(range(len(lake_like)), key=lambda i:
                 (lake_like[i][2][0] - tgt_centroid[0]) ** 2
                 + (lake_like[i][2][1] - tgt_centroid[1]) ** 2)
    ladder = [lake_like[pick_i]] + [t for i, t in enumerate(lake_like)
                                    if i != pick_i]
    for (dist, comp, comp_ll, poly, poly_ll, perim_m, area_m2,
         compact) in ladder[:4]:
        p_poly, p_geo, refined = poly, georef, False
        try:
            from lake_plan import route_api
            target = {"_poly_ll": poly_ll, "_poly_px": poly,
                      "_georef": georef, "area_km2": area_m2 / 1e6}
            fine = route_api._refine(target, "osm")
            if fine is not None:
                p_poly, p_geo, refined = fine
                poly_ll = [p_geo.pixel_to_latlon(x, y)
                           for (x, y) in p_poly]
        except Exception:  # noqa: BLE001
            pass
        from lake_plan import planner
        try:
            planner.plan_loop_around_polygon(
                p_poly, p_geo, offset_m=15.0, step_m=40.0)
        except ValueError:
            continue
        ctx["log"].append("event", event="lake_mask", source="rule",
                          vertices=len(poly_ll))
        return poly_ll, p_geo, anchor
    return {"ok": False, "reason": "plan_failed:no_candidate_planable"}


TOOL = ToolRegistration(
    name="plan_campus_lake",
    description=(
        "绕【园区里的湖】规划环线 (先识别园区, 再在园区内找湖): 任务说"
        "\"园区湖/当前园区湖/我们园区的湖\"时用它。链路: 园区知识库识别园区"
        " → esri 卫星图上【VLM 直接勾画园区边界 mask】→ 按园区 mask 放大卫星"
        " → VLM 直接勾画园区里的湖岸线 mask → 沿湖 mask 离岸环线规划;"
        " VLM 不可用时规则后备 (候选+湖形过滤+锚定)。已知园区: "
        "中电海康无锡物联网产业园。绕非园区水体才用 plan_lake_loop。"),
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
