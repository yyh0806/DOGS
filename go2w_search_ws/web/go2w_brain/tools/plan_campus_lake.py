"""plan_campus_lake — 园区湖语义链 (2026-09-05, VLM 直接 mask 版)。

用户要求: "绕着当前园区湖绕行一圈" 必须先识别园区, 再在园区内找湖;
且主张【卫星图上直接用 VLM 做 mask 圈出园区/湖】。

链路 (每步留痕):
  A. 园区识别: 任务文本/本体位置 → 园区知识库 (CAMPUSES) → 园区中心+名称;
  B. VLM 圈园区: esri 卫星 z17 拼接 → VLM 直接勾画园区边界多边形 (归一化
     坐标 → 经纬度 mask), 质量闸门: 必须包含本体 + 面积占比 2%~60%;
     VLM 不可用/假形状 → 规则半径圆后备;
  C. VLM 圈湖: 按园区 mask bbox 放大 (z18) 卫星 → VLM 勾画湖岸线 mask;
  D. 最终湖界裁决 (2026-09-05, VLM 湖 mask 曾圈出 6.7km 周长假形状):
     OSM 渲染水体 + 湖形过滤 (面积/紧凑度) + 园区 mask 内 + 距 VLM 择向最近
     → z19..z16 聚焦精修 → 精确岸线; OSM 链拿不到时才用 VLM mask 兜底。
事件: campus_identified / campus_mask / lake_mask / plan_result / \
geometry_persisted。
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

_SAT_Z, _SAT_NX, _SAT_NY = 18, 8, 8  # 园区 mask 层级: z18 ±520m, 园区充满画面
_LAKE_Z, _LAKE_NX, _LAKE_NY = 19, 4, 4  # 湖 mask 层级: z19 4×4 湖心特写


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


def _centroid_ll(ring_ll):
    return (sum(p[0] for p in ring_ll) / len(ring_ll),
            sum(p[1] for p in ring_ll) / len(ring_ll))


def _zoom_to_bbox(provider, ring_ll, z=18):
    """mask bbox → 放大卫星窗口 (z 起步逐级降, 2×2~8×8 瓦片)。"""
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

    # ---------- B0. 确定性园区边界: 地块聚类 (M2.1 已验证) ----------
    from lake_plan import planner, route_api
    from lake_plan.osm_client import _point_in_ring, campus_polygon
    from lake_plan.campus_deline import _hull as _convex_hull
    from ..plan_memory import persist_plan
    from ..patrol_math import endurance_check

    parcel_ll = None
    try:
        parcel = campus_polygon(clat, clng, radius_m=1200.0)
        if "ring" in parcel:
            parcel_ll = [(p[0], p[1]) for p in parcel["ring"]]
    except Exception:  # noqa: BLE001 — Overpass 不可达 → 规则圆
        pass
    # C0. OSM 湖形候选 (园区半径内, 用于园区 mask 并集 + 后续裁决)
    circle_ll = vlm_mask.circle_poly(clat, clng, campus["radius_m"])
    osm_cands = _osm_lake_candidates(campus, clat, clng, circle_ll,
                                     tiles, water)
    # 园区 mask = 地块聚类 ∪ 园区湖 (语义: 园区含湖)
    union_pts = list(parcel_ll or circle_ll)
    for ll, _, _, _ in osm_cands:
        union_pts.extend(ll)
    base_ll = _convex_hull(union_pts)
    base_ll = base_ll + [base_ll[0]] if len(base_ll) > 2 else circle_ll
    ctx["log"].append("event", event="campus_parcel",
                      source="overpass" if parcel_ll else "rule",
                      vertices=len(base_ll))

    # ---------- B. VLM 圈园区 (IoU 真值闸门; 不可信 → 用地块聚类) ----------
    try:
        sat_img, sat_detail = tiles.stitch_centered(
            "esri", clat, clng, _SAT_Z, _SAT_NX, _SAT_NY)
        sat_geo = tiles.georef_from_detail(sat_detail)
        campus_poly = vlm_mask.vlm_polygon(vlm, sat_img,
                                           vlm_mask.CAMPUS_MASK_PROMPT)
        if campus_poly is not None:
            pts = campus_poly["polygon"]
            frac = vlm_mask.poly_area_frac(pts)
            rx, ry = sat_geo.latlon_to_pixel(clat, clng)
            px, py = rx / sat_geo.w, ry / sat_geo.h
            if not (0.02 <= frac <= 0.60 and
                    vlm_mask.point_in_poly01(px, py, pts)):
                campus_poly = None
    except tiles.TileError:
        sat_geo = None
        campus_poly = None
    vlm_campus_iou = None
    if campus_poly is not None:
        vlm_campus_ll = vlm_mask.poly_to_latlon(campus_poly["polygon"],
                                                sat_geo)
        vlm_campus_iou = _mc_iou(vlm_campus_ll, base_ll)
        if vlm_campus_iou >= 0.3:
            campus_ll = vlm_campus_ll
            mask_source = "vlm"
        else:
            campus_ll = base_ll
            mask_source = "osm_parcel"
    else:
        campus_ll = base_ll
        mask_source = "osm_parcel"
    ctx["log"].append("event", event="campus_mask", campus=campus["name"],
                      source=mask_source, vertices=len(campus_ll),
                      vlm_iou=vlm_campus_iou)

    # ---------- C. VLM 在 z19 湖心特写窗上圈湖 ----------
    nudge = _centroid_ll(osm_cands[0][0]) if osm_cands \
        else (clat, clng)
    lake_vlm_ll = None
    lake_vlm_why = ""
    lake_vlm_iou = None
    try:
        if osm_cands:
            # 湖心 z19 特写窗 (4×4 ≈ ±130m, 湖充满画面)
            lake_img, ld = tiles.stitch_centered(
                "esri", nudge[0], nudge[1], _LAKE_Z, _LAKE_NX, _LAKE_NY)
            lake_geo = tiles.georef_from_detail(ld)
        else:
            lake_img, lake_geo = _zoom_to_bbox("esri", campus_ll, z=18)
        lake_poly = vlm_mask.vlm_polygon(vlm, lake_img,
                                         vlm_mask.LAKE_MASK_PROMPT)
        if lake_poly is not None:
            cand = vlm_mask.poly_to_latlon(lake_poly["polygon"], lake_geo)
            c_lat, c_lng = _centroid_ll(cand)
            frac = vlm_mask.poly_area_frac(lake_poly["polygon"])
            if _point_in_ring((c_lat, c_lng), campus_ll) \
                    and 0.002 <= frac <= 0.30:
                lake_vlm_ll = cand
                lake_vlm_why = str(lake_poly.get("why") or "")
    except tiles.TileError:
        pass
    if lake_vlm_ll is not None:
        # 真值闸门 (2026-09-05): 有 OSM 候选时 VLM mask 必须 IoU≥0.3
        # (GLM 免费模型实测湖 IoU=0.00, 圈出 406m 街区当湖)
        if osm_cands:
            lake_vlm_iou = _mc_iou(lake_vlm_ll, osm_cands[0][0])
        ctx["log"].append("event", event="lake_mask", source="vlm",
                          vertices=len(lake_vlm_ll), iou=lake_vlm_iou,
                          why=lake_vlm_why)

    # ---------- D. 最终湖界裁决 + 沿湖环线 ----------
    final = None
    final_source = "osm_rule"
    if osm_cands:
        sort_key_nudge = lake_vlm_ll if (lake_vlm_iou or 0) >= 0.3 \
            else None
        final = _osm_lake_final(ctx, osm_cands, clat, clng, campus_ll,
                                sort_key_nudge or nudge, route_api)
    if final is not None:
        lake_ll, lake_geo = final
        ctx["log"].append("event", event="lake_mask", source="osm_rule",
                          vertices=len(lake_ll))
    elif lake_vlm_ll is not None:
        lake_ll = lake_vlm_ll
        final_source = "vlm_fallback"
        ctx["log"].append("event", event="lake_mask", source="vlm_fallback",
                          vertices=len(lake_ll))
    else:
        return {"ok": False, "reason": "no_lake_like_water_in_campus",
                "campus": campus["name"],
                "hint": "园区 mask 内无湖形水体 (OSM + satellite 均未检出)"}

    poly_px = [lake_geo.latlon_to_pixel(a, b) for a, b in lake_ll]
    perim_m, area_m2 = water.poly_stats_latlon(lake_ll, clat)
    try:
        result = planner.plan_loop_around_polygon(
            poly_px, lake_geo, offset_m=15.0, step_m=40.0)
    except ValueError as exc:
        return {"ok": False, "reason": f"plan_failed:{exc}"}
    result["route_latlon"] = route_api._snap_outside_ring_min(
        result["route_latlon"], lake_ll, min_dist_m=5.0)
    tgt_c = _centroid_ll(lake_ll)
    out = route_api._finish(result, lake_ll, "water", {
        "area_km2": round(area_m2 / 1e6, 3),
        "perim_km": round(perim_m / 1000.0, 2),
        "dist_km": round(_hav_m(clat, clng, tgt_c[0], tgt_c[1]) / 1000.0, 3),
        "campus": campus["name"]})
    out["anchor"] = {"source": final_source,
                     "self": {"lat": clat, "lng": clng, "confirmed": True},
                     "target": {"idx": 0, "centroid": [round(tgt_c[0], 5),
                                                       round(tgt_c[1], 5)],
                                "why": lake_vlm_why or "osm_shape_filtered",
                                "vlm_mask_iou": lake_vlm_iou},
                     "ambiguity": []}
    out["campus"] = campus["name"]

    plan_store = ctx.get("plan_store")
    if plan_store is not None:
        plan_store["last_route"] = out
    ctx["log"].append("event", event="plan_result", plan_kind="campus_lake",
                      waypoint_count=len(out["waypoints"]),
                      length_m=result["stats"]["length_m"],
                      campus=campus["name"], target=out["target"])
    mem_id = persist_plan(ctx.get("memory"), out)
    if mem_id:
        ctx["log"].append("event", event="geometry_persisted",
                          memory_id=mem_id)

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
        "anchor": out["anchor"],
        "endurance": endurance,
        "first_waypoints": out["waypoints"][:3],
        "usage": "调用 follow_route 并传 from_plan=true 即可受理这条航线",
    }


def _mc_iou(a, b, n=140, seed: int = 7) -> float:
    """蒙特卡洛 IoU (两多边形 bbox 并集内采样)。"""
    import random
    from lake_plan.osm_client import _point_in_ring

    def bbox(ring):
        lats = [p[0] for p in ring]
        lngs = [p[1] for p in ring]
        return (min(lats), min(lngs), max(lats), max(lngs))

    b1, b2 = bbox(a), bbox(b)
    rng = random.Random(seed)
    inter = uni = 0
    for _ in range(n):
        la = rng.uniform(min(b1[0], b2[0]), max(b1[2], b2[2]))
        ln = rng.uniform(min(b1[1], b2[1]), max(b1[3], b2[3]))
        ina = _point_in_ring((la, ln), a)
        inb = _point_in_ring((la, ln), b)
        if ina or inb:
            uni += 1
            if ina and inb:
                inter += 1
    return inter / uni if uni else 0.0


def _osm_lake_candidates(campus, clat, clng, campus_ll, tiles, water):
    """OSM 渲染水体 + 湖形过滤 + 园区 mask 内 → [(ll, geo, poly, area)] 按距
    园区中心排序; 拿不到返回 []。"""
    from lake_plan.osm_client import _point_in_ring
    try:
        img, detail = tiles.stitch_centered("osm", clat, clng, 16, 8, 8)
    except tiles.TileError:
        return []
    geo = tiles.georef_from_detail(detail)
    mask = water.water_mask(img, ref_rgb=(170, 211, 223))
    comps = water.label_components(mask, down=4, min_area_px=200)
    cands = []
    for comp in comps:
        cy, cx = comp["centroid_small"]
        lat, lng = geo.pixel_to_latlon(cx * 4, cy * 4)
        if not _point_in_ring((lat, lng), campus_ll):
            continue
        poly = water.polygon_from_component(comp, mask.shape, allow_hull=True)
        if len(poly) < 4:
            continue
        ll = [geo.pixel_to_latlon(x, y) for (x, y) in poly]
        perim, area = water.poly_stats_latlon(ll, clat)
        compact = 4 * math.pi * area / (perim ** 2) if perim else 0.0
        if 300.0 <= area <= 80000.0 and compact >= 0.12:
            cands.append((ll, geo, poly, area))
    cands.sort(key=lambda t: _hav_m(clat, clng, *_centroid_ll(t[0])))
    return cands


def _osm_lake_final(ctx, cands, clat, clng, campus_ll, nudge, route_api):
    """湖形候选 → z19..z16 聚焦精修 → 精确湖界; 拿不到返回 None。"""
    ordered = sorted(cands, key=lambda t: _hav_m(nudge[0], nudge[1],
                                                 *_centroid_ll(t[0])))
    for ll, geo, poly, area in ordered[:4]:
        try:
            target = {"_poly_ll": ll, "_poly_px": poly, "_georef": geo,
                      "area_km2": area / 1e6}
            fine = route_api._refine(target, "osm")
        except Exception:  # noqa: BLE001
            fine = None
        if fine is not None:
            fpoly, fgeo, _ = fine
            fll = [fgeo.pixel_to_latlon(x, y) for (x, y) in fpoly]
            ctx["log"].append("event", event="lake_refined",
                              vertices=len(fll))
            return fll, fgeo
    # 精修全失败 → 用形状合格的粗边界 (hull, 含桥等略胀) 兜底
    return ordered[0][0], ordered[0][1]


TOOL = ToolRegistration(
    name="plan_campus_lake",
    description=(
        "绕【园区里的湖】规划环线 (先识别园区, 再在园区内找湖): 任务说"
        "\"园区湖/当前园区湖/我们园区的湖\"时用它。链路: 园区知识库识别园区"
        " → esri 卫星图上 VLM 直接勾画园区边界 mask → 按园区 mask 放大卫星"
        " → VLM 勾画湖 mask (语义择向) → OSM 湖形过滤+z16 聚焦精修出精确"
        "岸线 → 离岸环线规划; VLM 不可用时规则后备。已知园区: 中电海康无锡"
        "物联网产业园。绕非园区水体才用 plan_lake_loop。"),
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
