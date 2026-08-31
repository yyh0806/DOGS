"""route_api — 绕湖航线规划主入口 (确定性, 无 LLM)。

自 lake_loop/agent.py 的规则路径移植: 粗扫感知 → 选湖(规则) →
高分辨率细化 → 离岸环线规划 → waypoints 输出。

输出契约 (与 nx_web_server POST /api/gps/route 对齐, 注意是 "lon"):
    {"ok": true,
     "waypoints": [{"lat": .., "lon": .., "name": "wp000"}, ...],  # 闭合环
     "water_polygon": [[lat, lng], ...],                           # WGS-84
     "lake": {"centroid": [lat, lng], "area_km2": .., "perim_km": ..},
     "stats": {..planner stats + 细化上下文..}}

失败一律 {"ok": false, "reason": .., "candidates": [..摘要..]}, 绝不抛出。
"""
from __future__ import annotations

import math
from typing import Any

from . import config, planner, tiles, water
from .config import (DEFAULT_LOOP_OFFSET_M, DEFAULT_MAX_PERIM_KM,
                     DEFAULT_MIN_PERIM_KM, DEFAULT_STEP_M)
from .geo import haversine_m, lat_to_global_px, lng_to_global_px

# 粗扫视野计划: (zoom, nx, ny), 逐级放宽 (自 agent.node_perceive)
_PERCEIVE_PLANS = ((13, 8, 8), (12, 10, 10), (11, 10, 10))


def plan_route(lat: float, lng: float, provider: str = "osm",
               offset_m: float = DEFAULT_LOOP_OFFSET_M,
               step_m: float = DEFAULT_STEP_M,
               max_perim_km: float = DEFAULT_MAX_PERIM_KM,
               min_perim_km: float = DEFAULT_MIN_PERIM_KM,
               compact_min: float = config.COMPACT_MIN) -> dict[str, Any]:
    """以 (lat,lng) 为中心规划绕湖环线。任何失败返回 ok=False + reason。"""
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lng) <= 180.0):
        return {"ok": False, "reason": "invalid_center"}

    # ---------- 1. 粗扫感知 (逐级放宽视野) ----------
    usable: list[dict[str, Any]] = []
    georef_coarse = None
    last_candidates: list[dict[str, Any]] = []
    for z, nx, ny in _PERCEIVE_PLANS:
        try:
            img, detail = tiles.stitch_centered(provider, lat, lng, z, nx, ny)
        except tiles.TileError as exc:
            return {"ok": False, "reason": f"tiles_error:{exc}"}
        georef = tiles.georef_from_detail(detail)
        mask = water.water_mask(
            img, ref_rgb=config.PROVIDERS[provider]["water_rgb"])
        comps = water.label_components(mask, down=4, min_area_px=200)
        cands = _candidates(comps, georef, mask.shape, lat, lng)
        last_candidates = cands
        usable = [c for c in cands
                  if not c["clipped"]
                  and min_perim_km <= c["perim_km"] <= max_perim_km
                  and c["compact"] >= compact_min]
        if usable:
            georef_coarse = georef
            break
    if not usable:
        return {"ok": False, "reason": "no_suitable_lake",
                "candidates": _slim(last_candidates)}

    # ---------- 2. 选湖 (规则: 距离最近) ----------
    target = min(usable, key=lambda c: c["dist_km"])

    # ---------- 3. 高分辨率细化 (自 agent.node_refine, 去 LLM) ----------
    poly_px = target["_poly_px"]
    georef_plan = georef_coarse
    refined = False
    try:
        fine = _refine(target, provider)
    except tiles.TileError:
        fine = None
    if fine is not None:
        poly_px, georef_plan, refined = fine

    # ---------- 4. 离岸环线规划 ----------
    try:
        result = planner.plan_loop_around_polygon(
            poly_px, georef_plan, offset_m=offset_m, step_m=step_m)
    except ValueError as exc:
        if refined:  # 细化多边形退化 → 回退粗扫多边形再试一次
            try:
                result = planner.plan_loop_around_polygon(
                    target["_poly_px"], georef_coarse,
                    offset_m=offset_m, step_m=step_m)
                poly_px = target["_poly_px"]
                georef_plan = georef_coarse
                refined = False
            except ValueError:
                return {"ok": False, "reason": f"plan_failed:{exc}"}
        else:
            return {"ok": False, "reason": f"plan_failed:{exc}"}

    stats = dict(result["stats"])
    stats["refined"] = refined
    stats["provider"] = provider
    waypoints = [{"lat": round(ll[0], 6), "lon": round(ll[1], 6),
                  "name": f"wp{i:03d}"}
                 for i, ll in enumerate(result["route_latlon"])]
    water_poly_ll = [georef_plan.pixel_to_latlon(x, y)
                     for (x, y) in poly_px]
    cent_lat = sum(p[0] for p in water_poly_ll) / len(water_poly_ll)
    cent_lng = sum(p[1] for p in water_poly_ll) / len(water_poly_ll)
    return {
        "ok": True,
        "waypoints": waypoints,
        "water_polygon": [[round(p[0], 6), round(p[1], 6)]
                          for p in water_poly_ll],
        "lake": {"centroid": [round(cent_lat, 6), round(cent_lng, 6)],
                 "area_km2": target["area_km2"],
                 "perim_km": target["perim_km"],
                 "dist_km": target["dist_km"]},
        "stats": stats,
    }


# ---------- 内部 ------------------------------------------------------------

def _candidates(comps, georef, mask_shape, ref_lat, ref_lng):
    """连通域 → 候选摘要 (自 agent.node_perceive 的循环体)。"""
    cands = []
    for c in comps[:8]:
        poly = water.polygon_from_component(c, mask_shape)
        if len(poly) < 4:
            continue
        poly_ll = [georef.pixel_to_latlon(x, y) for (x, y) in poly]
        perim_m, area_m2 = water.poly_stats_latlon(poly_ll, ref_lat)
        bb = c["bbox_small"]
        clipped = (bb[0] <= 1 or bb[1] <= 1
                   or bb[2] >= c["mask"].shape[1] - 2
                   or bb[3] >= c["mask"].shape[0] - 2)
        cy, cx = c["centroid_small"]
        clat, clng = georef.pixel_to_latlon(cx * c["down"], cy * c["down"])
        compact = 0.0
        if perim_m > 0:
            compact = 4 * math.pi * area_m2 / (perim_m ** 2)
        cands.append({
            "area_km2": round(area_m2 / 1e6, 3),
            "perim_km": round(perim_m / 1000.0, 2),
            "compact": round(compact, 3),
            "clipped": bool(clipped),
            "centroid": [round(clat, 5), round(clng, 5)],
            "dist_km": round(haversine_m(ref_lat, ref_lng, clat, clng)
                             / 1000.0, 2),
            "_poly_px": poly,
            "_poly_ll": poly_ll,
            "_georef": georef,
        })
    return cands


def _refine(target, provider):
    """湖面 bbox 高分辨率重扫 (自 agent.node_refine)。失败返回 None。

    返回 (fine_poly_px, fine_georef, True)。
    """
    poly_ll = target["_poly_ll"]
    lats = [p[0] for p in poly_ll]
    lngs = [p[1] for p in poly_ll]
    w, s, e, n = min(lngs), min(lats), max(lngs), max(lats)
    best = None
    for z in range(16, 9, -1):
        gx0 = int(lng_to_global_px(w, z) // 256)
        gx1 = int(lng_to_global_px(e, z) // 256)
        gy0 = int(lat_to_global_px(n, z) // 256)
        gy1 = int(lat_to_global_px(s, z) // 256)
        nx, ny = gx1 - gx0 + 1, gy1 - gy0 + 1
        if nx * ny <= config.MAX_TILES_PER_STITCH and nx >= 2:
            best = (z, gx0, gy0, nx, ny)
            break
    if best is None:
        return None
    z, x0, y0, nx, ny = best
    if (nx + 2) * (ny + 2) <= config.MAX_TILES_PER_STITCH:
        x0, y0, nx, ny = x0 - 1, y0 - 1, nx + 2, ny + 2  # 留边给外扩
    img, detail = tiles.stitch_area(provider, z, x0, y0, nx, ny)
    fine_georef = tiles.georef_from_detail(detail)
    mask = water.water_mask(
        img, ref_rgb=config.PROVIDERS[provider]["water_rgb"])
    comps = water.label_components(mask, down=2, min_area_px=400)
    if not comps:
        return None
    cc_lat = sum(lats) / len(lats)
    cc_lng = sum(lngs) / len(lngs)
    ref_px = fine_georef.latlon_to_pixel(cc_lat, cc_lng)

    def _ccomp(c):
        cy, cx = c["centroid_small"]
        return math.hypot(cx * c["down"] - ref_px[0],
                          cy * c["down"] - ref_px[1])

    comp = min(comps[:8], key=_ccomp)  # 质心就近匹配, 防拿到别的水塘
    fine_poly = water.polygon_from_component(comp, mask.shape)
    if len(fine_poly) < 4:
        return None
    fine_ll = [fine_georef.pixel_to_latlon(x, y) for (x, y) in fine_poly]
    _, fine_area = water.poly_stats_latlon(fine_ll, fine_ll[0][0])
    if fine_area < 0.4 * (target["area_km2"] * 1e6):
        return None  # 面积骤降 = 匹配到别的小水体
    return fine_poly, fine_georef, True


def _slim(cands):
    return [{k: c[k] for k in ("area_km2", "perim_km", "compact",
                               "clipped", "centroid", "dist_km")}
            for c in cands[:6]]
