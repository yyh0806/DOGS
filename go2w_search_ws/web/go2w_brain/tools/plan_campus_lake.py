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

_SAT_Z, _SAT_NX, _SAT_NY = 19, 12, 12  # 园区识别层级: z19 高清 (±390m)
_LAKE_Z, _LAKE_NX, _LAKE_NY = 19, 4, 4  # 湖 mask 特写层级


def _task_scope_radius(task_text: str, default: float) -> float:
    """任务范围理解 (2026-09-05 用户要求): '当前园区湖' 的范围 = 本体周围。
    不依赖园区多边形成功 —— 附近/周边词放宽, 缺省 = 园区知识库半径。"""
    if any(k in task_text for k in ("附近", "周边", "周围")):
        return max(default, 800.0)
    return default


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


def _georef_for_ring(ring_ll, z=19):
    """标定多边形 → 纯虚拟地理参照 (不抓瓦片, 供 planner 像素换算)。"""
    from lake_plan.geo import (StitchGeoref, lat_to_global_px,
                               lng_to_global_px)
    lats = [p[0] for p in ring_ll]
    lngs = [p[1] for p in ring_ll]
    w, s, e, n = min(lngs), min(lats), max(lngs), max(lats)
    gx0 = int(lng_to_global_px(w, z) // 256) - 1
    gx1 = int(lng_to_global_px(e, z) // 256) + 1
    gy0 = int(lat_to_global_px(n, z) // 256) - 1
    gy1 = int(lat_to_global_px(s, z) // 256) + 1
    return StitchGeoref(z, gx0, gy0, (gx1 - gx0 + 1) * 256,
                        (gy1 - gy0 + 1) * 256, "wgs84")


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

    # ---------- B0. 任务范围 + 确定性园区边界 (标定真值 > 地块聚类 > 范围圆) ----------
    from lake_plan import planner, route_api
    from lake_plan.osm_client import _point_in_ring, campus_polygon
    from lake_plan.campus_deline import _hull as _convex_hull
    from .. import campus_kb
    from ..plan_memory import persist_plan
    from ..patrol_math import endurance_check

    scope_radius = _task_scope_radius(task_text, campus["radius_m"])
    ctx["log"].append("event", event="task_scope", center=[clat, clng],
                      radius_m=scope_radius, campus=campus["name"])
    # 标定真值: 记忆库优先 (永久记录, 指令×记忆), 文件后备
    cal_boundary = None
    cal_lakes: list = []          # 湖面可多块 (不连通水体, 2026-09-05)
    memory = ctx.get("memory")
    if memory is not None:
        try:
            entries = memory.query(clat, clng, 1500.0,
                                   kinds=("geometry",), min_score=0.05)
            latest: dict = {}   # (campus, calibrated_kind) → ts 最新条目
            for e in entries:
                d = e.get("data") or {}
                if d.get("campus") != campus["name"]:
                    continue
                ck = d.get("calibrated_kind")
                if ck not in ("campus_boundary", "lake_shore"):
                    continue
                key = ck
                if key not in latest or e.get("ts", 0) > latest[key].get("ts", 0):
                    latest[key] = e
            for ck, e in latest.items():
                d = e.get("data") or {}
                parts = d.get("parts")
                if parts:
                    rings = [[tuple(p) for p in ring] for ring in parts
                             if len(ring) >= 3]
                else:
                    ring = (e.get("geo") or {}).get("points")
                    rings = [[tuple(p) for p in ring] if ring and
                             len(ring) >= 3 else []]
                rings = [r for r in rings if len(r) >= 3]
                if not rings:
                    continue
                if ck == "campus_boundary":
                    cal_boundary = rings[0]
                else:
                    cal_lakes = rings
        except Exception:  # noqa: BLE001
            pass
    kb = campus_kb.get(campus["name"]) or {}
    cal_boundary = cal_boundary or kb.get("boundary")
    if not cal_lakes:
        cal_lakes = [[tuple(p) for p in r]
                     for r in campus_kb.lake_rings(kb)]
    cal_lake = cal_lakes[0] if cal_lakes else None
    if cal_boundary:
        ctx["log"].append("event", event="calibration_loaded",
                          source="memory", campus=campus["name"])
    osm_cands: list = []
    lake_vlm_why = ""
    lake_vlm_iou = None
    if cal_boundary and len(cal_boundary) >= 3:
        campus_ll = [tuple(p) for p in cal_boundary]
        if campus_ll[0] != campus_ll[-1]:
            campus_ll = campus_ll + [campus_ll[0]]
        ctx["log"].append("event", event="campus_mask",
                          campus=campus["name"], source="calibrated",
                          vertices=len(campus_ll), vlm_iou=None)
    else:
        parcel_ll = None
        try:
            parcel = campus_polygon(clat, clng, radius_m=1200.0)
            if "ring" in parcel:
                parcel_ll = [(p[0], p[1]) for p in parcel["ring"]]
        except Exception:  # noqa: BLE001 — Overpass 不可达 → 任务范围圆
            pass
        circle_ll = vlm_mask.circle_poly(clat, clng, scope_radius)
        osm_cands = _osm_lake_candidates(campus, clat, clng, circle_ll,
                                         tiles, water)
        union_pts = list(parcel_ll or [])
        for ll, _, _, _ in osm_cands:
            union_pts.extend(ll)
        if union_pts:
            base_ll = _convex_hull(union_pts)
            base_ll = base_ll + [base_ll[0]] if len(base_ll) > 2 else circle_ll
            mask_base_source = "osm_parcel" if parcel_ll else "scope_lake"
        else:
            base_ll = circle_ll
            mask_base_source = "scope"
        ctx["log"].append("event", event="campus_parcel",
                          source="overpass" if parcel_ll else "scope",
                          vertices=len(base_ll))

        # ---------- B. VLM 在 z19 高清图上圈园区 (IoU 真值闸门) ----------
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
            vlm_campus_ll = vlm_mask.poly_to_latlon(
                campus_poly["polygon"], sat_geo)
            vlm_campus_iou = _mc_iou(vlm_campus_ll, base_ll)
            if vlm_campus_iou >= 0.3:
                campus_ll = vlm_campus_ll
                mask_source = "vlm"
            else:
                campus_ll = base_ll
                mask_source = mask_base_source
        else:
            campus_ll = base_ll
            mask_source = mask_base_source
        ctx["log"].append("event", event="campus_mask",
                          campus=campus["name"], source=mask_source,
                          vertices=len(campus_ll), vlm_iou=vlm_campus_iou)

    # ---------- C. 湖界: 标定真值(可多块) > OSM 精修 > VLM 兜底 ----------
    cal_lakes = [r for r in cal_lakes if len(r) >= 3]
    if cal_lakes:
        # 多块湖: 每块单独出环线, 顺路串联成一条巡逻路线 (2026-09-05)
        lake_rings = [list(r) for r in cal_lakes]
        lake_geo = None
        final_source = "calibrated"
        for r in lake_rings:
            if r[0] != r[-1]:
                r.append(r[0])
        ctx["log"].append("event", event="lake_mask", source="calibrated",
                          vertices=sum(len(r) - 1 for r in lake_rings),
                          parts=len(lake_rings))
        osm_cands = []
    else:
        lake_rings = None
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

        # ---------- D. 最终湖界裁决 (OSM 精修 > VLM 兜底) ----------
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
            ctx["log"].append("event", event="lake_mask",
                              source="vlm_fallback",
                              vertices=len(lake_ll))
        else:
            return {"ok": False, "reason": "no_lake_like_water_in_campus",
                    "campus": campus["name"],
                    "hint": "园区 mask 内无湖形水体 (OSM + satellite "
                            "均未检出)"}

    # ---------- E. 沿湖环线 (多块湖: 每块一环, 顺路串联) ----------
    rings = lake_rings if lake_rings else [lake_ll]
    # 按距本体排序 (先近后远, 巡逻路线自然)
    rings.sort(key=lambda r: _hav_m(clat, clng, *_centroid_ll(r)))
    combined: list[tuple[float, float]] = []
    total_len = 0.0
    per_ring_closed = True
    ring_stats = []
    for ring in rings:
        ring = [tuple(p) for p in ring]
        if ring[0] != ring[-1]:
            ring.append(ring[0])
        geo = _georef_for_ring(ring)
        poly_px = [geo.latlon_to_pixel(a, b) for a, b in ring]
        try:
            r_result = planner.plan_loop_around_polygon(
                poly_px, geo, offset_m=15.0, step_m=40.0)
        except ValueError as exc:
            return {"ok": False, "reason": f"plan_failed:{exc}"}
        r_route = route_api._snap_outside_ring_min(
            r_result["route_latlon"], ring, min_dist_m=5.0)
        ring_stats.append({"waypoints": len(r_route),
                           "length_m": round(r_result["stats"]["length_m"], 1),
                           "closed": r_result["stats"]["closed"]})
        per_ring_closed = per_ring_closed and r_result["stats"]["closed"]
        if combined:
            # 串联过渡: 上一环终点 → 本环起点, 每 ~40m 采样, 推出所有环外
            prev = combined[-1]
            nxt = r_route[0]
            seg_len = _hav_m(prev[0], prev[1], nxt[0], nxt[1])
            steps = max(1, int(seg_len / 40.0))
            for i in range(1, steps):
                t = i / steps
                mid = (prev[0] + (nxt[0] - prev[0]) * t,
                       prev[1] + (nxt[1] - prev[1]) * t)
                for rg in rings:  # 过渡点不得入任何湖块
                    mid = route_api._snap_outside_ring_min(
                        [mid], [tuple(p) for p in rg],
                        min_dist_m=5.0)[0]
                combined.append(mid)
            total_len += seg_len
        combined.extend(r_route)
        total_len += r_result["stats"]["length_m"]

    all_pts = [p for ring in rings for p in ring]
    perim_m, area_m2 = water.poly_stats_latlon(all_pts, clat)
    tgt_c = _centroid_ll(all_pts)
    result = {"route_latlon": combined,
              "stats": {"length_m": total_len,
                        "closed": per_ring_closed and len(rings) == 1,
                        "water_cross_ratio": 0.0}}
    out = route_api._finish(result, rings[0], "water", {
        "area_km2": round(area_m2 / 1e6, 3),
        "perim_km": round(perim_m / 1000.0, 2),
        "dist_km": round(_hav_m(clat, clng, tgt_c[0], tgt_c[1]) / 1000.0, 3),
        "campus": campus["name"]})
    out["water_polygons"] = [[list(p) for p in ring] for ring in rings]
    if len(rings) > 1:
        out["lake_parts"] = len(rings)
        out["ring_stats"] = ring_stats
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
