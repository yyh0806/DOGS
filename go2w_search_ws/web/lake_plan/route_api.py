"""route_api — 环线规划主入口 (确定性, 无 LLM)。

两种目标 (kind):
- "water"  绕湖: 瓦片水域分割 (自 lake_loop/agent.py 规则路径移植);
- "campus" 绕园区: Overpass landuse 地块聚类 + 凸包 (osm_client),
  纯向量, 不需要瓦片。

两者共用离岸外扩管线 (planner.plan_loop_around_polygon)。

输出契约 (与 nx_web_server POST /api/gps/route 对齐, 注意是 "lon"):
    {"ok": true, "kind": "water"|"campus",
     "waypoints": [{"lat": .., "lon": .., "name": "wp000"}, ...],  # 闭合环
     "water_polygon" | "campus_polygon": [[lat, lng], ...],
     "target": {"kind", "centroid", "area_km2", "perim_km", "dist_km", ..},
     "stats": {..}}

失败一律 {"ok": false, "reason": ..}, 绝不抛出。
"""
from __future__ import annotations

import math
from typing import Any

from . import config, osm_client, planner, tiles, water
from .config import (DEFAULT_LOOP_OFFSET_M, DEFAULT_MAX_PERIM_KM,
                     DEFAULT_MIN_PERIM_KM, DEFAULT_SCAN_SPACING_M,
                     DEFAULT_STEP_M)
from .geo import (StitchGeoref, haversine_m, lat_to_global_px,
                  lng_to_global_px)

# 感知窗口 (2026-09-04 重设计): 命令通常针对周边 → 只扫两级本地窗口,
# 不再逐层外扩到 z14/z12 城区尺度。z17 8×8 ≈ ±1.0km (最细可用层级:
# z18/z19 上 OSM 把小湖与邻近水渠渲染成一体, 候选被紧凑度/周长淘汰),
# 找不到再放宽到 z16 8×8 ≈ ±2.4km。找到即停, 选距离最近者。
# (更细的聚焦重扫由 _refine 对选定湖面做, 感知只负责"哪片水在附近"。)
_PERCEIVE_PLANS = ((17, 8, 8), (16, 8, 8))
_CANDIDATE_SCAN = 24  # 连通域扫描深度 (按面积排序取前 N)


def plan_route(lat: float, lng: float, provider: str = "osm",
               kind: str = "water",
               offset_m: float = DEFAULT_LOOP_OFFSET_M,
               step_m: float = DEFAULT_STEP_M,
               max_perim_km: float = DEFAULT_MAX_PERIM_KM,
               min_perim_km: float = DEFAULT_MIN_PERIM_KM,
               compact_min: float = config.COMPACT_MIN,
               campus_radius_m: float = 1200.0,
               prefer: tuple[float, float] | None = None) -> dict[str, Any]:
    """以 (lat,lng) 为中心规划环线。任何失败返回 ok=False + reason。

    prefer: M7.3 语义锚定纠正 —— 语义层认定任务所指水体的质心
    (lat, lng), 选湖改为"距该质心最近的合格水体"而非距本体的。
    """
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lng) <= 180.0):
        return {"ok": False, "reason": "invalid_center"}
    if kind not in ("water", "campus"):
        return {"ok": False, "reason": "invalid_kind"}
    if kind == "campus":
        return _plan_campus(lat, lng, offset_m, step_m, campus_radius_m)
    return _plan_water(lat, lng, provider, offset_m, step_m,
                       max_perim_km, min_perim_km, compact_min, prefer)


# ---------- 绕湖 ------------------------------------------------------------

def _plan_water(lat, lng, provider, offset_m, step_m,
                max_perim_km, min_perim_km, compact_min, prefer=None):
    # ---------- 1. 粗扫感知 (先近后远) ----------
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

    # ---------- 2. 选湖 (规则: 距离最近; M7.3 prefer → 距语义锚点最近) ----
    if prefer is not None:
        target = min(usable, key=lambda c: haversine_m(
            float(prefer[0]), float(prefer[1]),
            c["centroid"][0], c["centroid"][1]))
    else:
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
    poly_ll = [georef_plan.pixel_to_latlon(x, y) for (x, y) in poly_px]
    # 规划-安全一致性 (M3): 航点距输出水域多边形 ≥ 5m。覆盖链路:
    # 守卫 veto 2.0m + 布防默认 margin 2.0m + 1m 余量 —— 圆角平滑在
    # 岸线细节处可贴水到 <2m, 与其让守卫受理时拒收, 规划期就推出。
    result["route_latlon"] = _snap_outside_ring_min(
        result["route_latlon"], poly_ll, min_dist_m=5.0)
    out = _finish(result, poly_ll, "water", {
        "area_km2": target["area_km2"], "perim_km": target["perim_km"],
        "dist_km": target["dist_km"]})
    out["stats"] = stats
    # M7.3 语义锚定输入: 视野内其他合格水体 (供"哪个是湖"的歧义清单)
    out["nearby_candidates"] = _slim(
        [c for c in usable if c is not target])
    return out


# ---------- 绕园区 ----------------------------------------------------------

def _plan_campus(lat, lng, offset_m, step_m, campus_radius_m):
    campus = osm_client.campus_polygon(lat, lng,
                                       radius_m=campus_radius_m)
    if "ring" not in campus:
        extra = {k: v for k, v in campus.items() if k != "reason"}
        return {"ok": False,
                "reason": campus.get("reason", "campus_failed"), **extra}
    ring = campus["ring"]
    # 虚拟地理参照: 纯向量规划, 不抓任何瓦片 → 缩放自由选择。
    # 条件: 凸包像素跨度 300~3600px (分辨率够 + 内存有界), 从 z19 向下
    # 找第一个满足者; 到 z10 仍超上限 (>~140km) 才算异常大。
    lats = [p[0] for p in ring]
    lngs = [p[1] for p in ring]
    west, south = min(lngs), min(lats)
    east, north = max(lngs), max(lats)
    chosen_z = None
    for z in range(19, 9, -1):
        px_w = lng_to_global_px(east, z) - lng_to_global_px(west, z)
        px_h = lat_to_global_px(south, z) - lat_to_global_px(north, z)
        span = max(px_w, px_h)
        if span <= 3600.0:
            chosen_z = z
            if span >= 300.0:
                break
            # span < 300: 多边形极小, 但 z 不能再高 (z19 封顶), 就用它
            break
    if chosen_z is None:
        return {"ok": False, "reason": "campus_too_large"}
    z = chosen_z
    gx0 = int(lng_to_global_px(west, z) // 256)
    gx1 = int(lng_to_global_px(east, z) // 256)
    gy0 = int(lat_to_global_px(north, z) // 256)
    gy1 = int(lat_to_global_px(south, z) // 256)
    nx, ny = gx1 - gx0 + 1, gy1 - gy0 + 1
    georef = StitchGeoref(z, gx0, gy0, nx * 256, ny * 256, "wgs84")
    poly_px = [georef.latlon_to_pixel(p[0], p[1]) for p in ring]
    try:
        result = planner.plan_loop_around_polygon(
            poly_px, georef, offset_m=offset_m, step_m=step_m)
    except ValueError as exc:
        return {"ok": False, "reason": f"plan_failed:{exc}"}
    # 矢量级保证 + 规划-安全一致性 (M3/M7): 环线任何点不得落在园区凸包
    # 内, 且距凸包 ≥5m (守卫 veto 2.0 + margin 2.0 + 1m 余量) —— 与水
    # 分支同一纪律, 圆角切入与贴边航点都会被推出。
    result["route_latlon"] = _snap_outside_ring_min(
        result["route_latlon"], ring, min_dist_m=5.0)
    stats = dict(result["stats"])
    stats["refined"] = False
    stats["cluster_plots"] = campus["cluster_plots"]
    target = {"area_km2": campus["area_km2"],
              "perim_km": round(result["stats"]["length_m"] / 1000.0, 2),
              "dist_km": 0.0,
              "landuse": campus["kind"],
              "containing": campus.get("containing", {})}
    out = _finish(result, [list(p) for p in ring], "campus", target)
    out["stats"] = stats
    return out


# ---------- 公共收尾 --------------------------------------------------------

def _finish(result, poly_ll, kind, target,
            scan_spacing_m: float = DEFAULT_SCAN_SPACING_M):
    waypoints = [{"lat": round(ll[0], 6), "lon": round(ll[1], 6),
                  "name": f"wp{i:03d}"}
                 for i, ll in enumerate(result["route_latlon"])]
    cent_lat = sum(p[0] for p in poly_ll) / len(poly_ll)
    cent_lng = sum(p[1] for p in poly_ll) / len(poly_ll)
    target = dict(target)
    target["kind"] = kind
    target["centroid"] = target.get(
        "centroid", [round(cent_lat, 6), round(cent_lng, 6)])
    # M5 扫描点: 沿环线每 scan_spacing_m 米一个, 朝向目标质心
    # (湖法线方向) —— 云台扫视的朝向基准。
    scan_points = _scan_points(result["route_latlon"],
                               (cent_lat, cent_lng), scan_spacing_m)
    out = {
        "ok": True,
        "kind": kind,
        "waypoints": waypoints,
        "scan_points": scan_points,
        "target": target,
        "stats": result["stats"],
    }
    out["water_polygon" if kind == "water" else "campus_polygon"] = [
        [round(p[0], 6), round(p[1], 6)] for p in poly_ll]
    return out


def _scan_points(route_ll, centroid, spacing_m):
    """沿闭合环线每 spacing_m 取扫描点, 附朝向质心的方位角 (真北)。"""
    if spacing_m <= 0 or len(route_ll) < 2:
        return []
    clat, clng = centroid
    kx = 111320.0 * math.cos(math.radians(clat))
    ky = 110540.0
    out = []
    acc = 0.0
    nxt = spacing_m
    n = len(route_ll)
    for i in range(1, n + 1):
        lat0, lng0 = route_ll[i - 1]
        lat1, lng1 = route_ll[i % n]
        seg = math.hypot((lng1 - lng0) * kx, (lat1 - lat0) * ky)
        while acc + seg >= nxt and seg > 1e-9:
            t = (nxt - acc) / seg
            plat = lat0 + t * (lat1 - lat0)
            plng = lng0 + t * (lng1 - lng0)
            bearing = (math.degrees(math.atan2((clng - plng) * kx,
                                               (clat - plat) * ky))
                       % 360.0)
            out.append({"lat": round(plat, 6), "lon": round(plng, 6),
                        "look_bearing_deg": round(bearing, 1)})
            nxt += spacing_m
        acc += seg
    return out


# ---------- 内部 ------------------------------------------------------------

def _snap_outside_ring(route_ll, ring):
    """把落入 ring (矢量多边形) 内的点从环质心方向推出, 直到在环外。"""
    return _snap_outside_ring_min(route_ll, ring, min_dist_m=0.0)


def _snap_outside_ring_min(route_ll, ring, min_dist_m=0.0):
    """推出到环外且至少距环边 min_dist_m 米 (规划-安全一致性)。

    逐点沿 (点→环质心) 反方向尝试递增步长; 包内独立实现基础几何
    (lake_plan 不依赖 web/ 平铺模块)。
    """
    from .osm_client import _point_in_ring
    clat = sum(p[0] for p in ring) / len(ring)
    clng = sum(p[1] for p in ring) / len(ring)
    kx = 111320.0 * math.cos(math.radians(clat))
    ky = 110540.0
    xs = [p[1] * kx for p in ring]
    ys = [p[0] * ky for p in ring]

    def _edge_dist_m(lat, lng):
        x, y = lng * kx, lat * ky
        best = float("inf")
        n = len(ring)
        for i in range(n):
            dx, dy = xs[(i + 1) % n] - xs[i], ys[(i + 1) % n] - ys[i]
            seg2 = dx * dx + dy * dy or 1e-9
            t = max(0.0, min(1.0, ((x - xs[i]) * dx + (y - ys[i]) * dy) / seg2))
            best = min(best, math.hypot(x - (xs[i] + t * dx),
                                        y - (ys[i] + t * dy)))
        return best

    out = []
    for lat, lng in route_ll:
        if (not _point_in_ring((lat, lng), ring)
                and _edge_dist_m(lat, lng) >= min_dist_m):
            out.append((lat, lng))
            continue
        dx, dy = lng - clng, lat - clat
        norm = math.hypot(dx * kx, dy * ky) or 1.0
        ux, uy = dx * kx / norm, dy * ky / norm
        pushed = (lat, lng)
        for step_m in (1.0, 3.0, 6.0, 12.0, 25.0, 50.0, 100.0, 200.0):
            cand = (lat + uy * step_m / ky, lng + ux * step_m / kx)
            if (not _point_in_ring(cand, ring)
                    and _edge_dist_m(cand[0], cand[1]) >= min_dist_m):
                pushed = cand
                break
        out.append(pushed)
    return out


def _candidates(comps, georef, mask_shape, ref_lat, ref_lng):
    """连通域 → 候选摘要 (自 agent.node_perceive 的循环体)。"""
    cands = []
    for c in comps[:_CANDIDATE_SCAN]:
        # allow_hull: 感知阶段允许退化多边形凸包兜底 (小水体在高倍级
        # 星形自交); 精确边界由 _refine 聚焦重扫给出。
        poly = water.polygon_from_component(c, mask_shape, allow_hull=True)
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
    """湖面 bbox 聚焦重扫 (自 agent.node_refine, 去 LLM)。失败返回 None。

    M7.3 修复 (聚焦附近区域): 原实现从 z16 向下找层级、且要求 bbox 自身
    跨 ≥2 张瓦片 —— 几十米级小水体 (园区景观湖) 在任何层级都 1×1,
    "细化"从未生效。改为:
    - z19→z16 逐级尝试, bbox 四周各留 1 张瓦片边距 (2×2~8×8 封顶);
    - 每级重扫后按面积校验: 细扫面积须落在粗扫的 0.4×~2.5× 窗口内
      (高倍级上 OSM 常把湖与邻近水渠渲染成一体, 面积骤变即弃用并降级;
      2026-09-01 园区湖实测: z19~z17 均粘连水渠, z16 聚焦重扫面积 ×1.03);
    - 命中即返回, 全级失败 → 沿用粗扫多边形 (诚实降级)。
    返回 (fine_poly_px, fine_georef, True)。
    """
    poly_ll = target["_poly_ll"]
    lats = [p[0] for p in poly_ll]
    lngs = [p[1] for p in poly_ll]
    w, s, e, n = min(lngs), min(lats), max(lngs), max(lats)
    cc_lat, cc_lng = sum(lats) / len(lats), sum(lngs) / len(lngs)
    coarse_area = target["area_km2"] * 1e6
    for z in range(19, 15, -1):
        gx0 = int(lng_to_global_px(w, z) // 256) - 1
        gx1 = int(lng_to_global_px(e, z) // 256) + 1
        gy0 = int(lat_to_global_px(n, z) // 256) - 1
        gy1 = int(lat_to_global_px(s, z) // 256) + 1
        nx, ny = gx1 - gx0 + 1, gy1 - gy0 + 1
        if not (2 <= nx <= 8 and 2 <= ny <= 8):
            continue
        img, detail = tiles.stitch_area(provider, z, gx0, gy0, nx, ny)
        fine_georef = tiles.georef_from_detail(detail)
        mask = water.water_mask(
            img, ref_rgb=config.PROVIDERS[provider]["water_rgb"])
        comps = water.label_components(mask, down=2, min_area_px=400)
        if not comps:
            continue
        ref_px = fine_georef.latlon_to_pixel(cc_lat, cc_lng)
        comp = min(comps[:8], key=lambda c: math.hypot(
            c["centroid_small"][1] * c["down"] - ref_px[0],
            c["centroid_small"][0] * c["down"] - ref_px[1]))
        fine_poly = water.polygon_from_component(comp, mask.shape)
        if len(fine_poly) < 4:
            continue
        fine_ll = [fine_georef.pixel_to_latlon(x, y)
                   for (x, y) in fine_poly]
        _, fine_area = water.poly_stats_latlon(fine_ll, fine_ll[0][0])
        # 面积窗口: 上限 2.5× 防粘连水渠 (细扫面积暴涨 = 匹配到合并水体);
        # 下限 0.05× 只防退化 —— 粗扫多边形可能来自凸包兜底 (小水体在
        # z17 的自交星形修复, 面积被放大 ~17×), 真细化面积反而小得多,
        # 而"匹配到别的小水体"已由质心就近匹配拦截。
        if not (0.05 * coarse_area <= fine_area <= 2.5 * coarse_area):
            continue
        return fine_poly, fine_georef, True
    return None


def _slim(cands):
    return [{k: c[k] for k in ("area_km2", "perim_km", "compact",
                               "clipped", "centroid", "dist_km")}
            for c in cands[:6]]
