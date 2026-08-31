"""osm_client — Overpass 向量查询 (园区多边形获取)。

设计:
- 镜像轮换 (kumi → overpass.de → mail.ru), 全失败才算失败;
- 响应按查询内容哈希缓存到 cache_dir()/overpass/, 与瓦片缓存同策略:
  GO2W_LAKE_OFFLINE=1 时只读缓存, 未命中即错 (测试封闭);
- 园区判定: OSM 中园区 = 碎片化的同类 landuse 地块集合。
  算法 = 中心点所在地块 → 同类地块按邻近 (bbox 间距 < cluster_gap_m)
  泛洪聚类 → 聚类顶点凸包 = 园区外轮廓。
"""
from __future__ import annotations

import hashlib
import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

from . import config

_ENDPOINTS = (
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass-api.de/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
)
_TIMEOUT = 40.0

# 园区类别优先级 (越小越"园区"): 产业园/教育园优先, 住宅区兜底
_KIND_RANK = {"industrial": 0, "education": 0, "commercial": 1,
              "retail": 1, "construction": 1, "park": 1,
              "residential": 2}


class OsmError(Exception):
    pass


def _cache_path(query: str):
    digest = hashlib.sha1(query.encode("utf-8")).hexdigest()[:16]
    return config.cache_dir() / "overpass" / f"{digest}.json"


def overpass_query(query: str) -> dict[str, Any]:
    """带缓存的 Overpass 查询。离线模式只读缓存。"""
    cache = _cache_path(query)
    if cache.exists():
        try:
            return json.loads(cache.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass
    if config.offline():
        raise OsmError("offline_cache_miss")
    data = urllib.parse.urlencode({"data": query}).encode()
    last: Exception | None = None
    for endpoint in _ENDPOINTS:
        try:
            request = urllib.request.Request(
                endpoint, data=data,
                headers={"User-Agent": config.USER_AGENT})
            with urllib.request.urlopen(request, timeout=_TIMEOUT) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
            if not isinstance(payload, dict) or "elements" not in payload:
                raise OsmError("malformed_overpass_response")
            cache.parent.mkdir(parents=True, exist_ok=True)
            cache.write_text(json.dumps(payload, ensure_ascii=False),
                             encoding="utf-8")
            return payload
        except (urllib.error.URLError, OSError, json.JSONDecodeError,
                OsmError) as exc:
            last = exc
            time.sleep(0.5)
    raise OsmError(f"overpass_unreachable: {last}")


def campus_polygon(lat: float, lng: float,
                   radius_m: float = 1200.0,
                   cluster_gap_m: float = 80.0) -> dict[str, Any]:
    """取包含 (lat,lng) 的园区外轮廓 (同类地块聚类 + 凸包)。

    返回 {"ring": [[lat,lng]...], "kind": .., "area_km2": ..,
    "cluster_plots": N, "containing": {...}} ; 找不到返回
    {"reason": ..} (无 ok 字段即失败, 与 plan_route 的失败风格一致)。
    """
    radius = max(300.0, min(float(radius_m), 3000.0))
    query = (
        "[out:json][timeout:25];\n"
        "( way[\"landuse\"](around:%d,%.6f,%.6f);\n"
        "  relation[\"landuse\"](around:%d,%.6f,%.6f);\n"
        "  way[\"leisure\"=\"park\"](around:%d,%.6f,%.6f);\n"
        "  relation[\"leisure\"=\"park\"](around:%d,%.6f,%.6f); );\n"
        "out geom qt;"
        % (int(radius), lat, lng, int(radius), lat, lng,
           int(radius), lat, lng, int(radius), lat, lng))
    try:
        payload = overpass_query(query)
    except OsmError as exc:
        return {"reason": f"overpass_error:{exc}"}

    plots: list[dict[str, Any]] = []
    for el in payload.get("elements", []):
        ring = _ring_of(el)
        if len(ring) < 4:
            continue
        tags = el.get("tags", {})
        kind = tags.get("landuse") or (
            "park" if tags.get("leisure") == "park" else None)
        if kind not in _KIND_RANK:
            continue
        if _ring_closed_len_km(ring) > 40.0:  # 异常大环 (数据脏)
            continue
        plots.append({"ring": ring, "kind": kind,
                      "bbox": _bbox(ring),
                      "name": tags.get("name", "")})
    if not plots:
        return {"reason": "no_landuse_plots"}

    # 1) 中心所在地块 (含点判定)
    containing = [p for p in plots
                  if _point_in_ring((lat, lng), p["ring"])]
    if not containing:
        return {"reason": "not_inside_any_plot",
                "hint": "该点不在任何 landuse 多边形内, 园区目标无法界定"}
    # 多个包含时取最具体的 (面积最小)
    seed = min(containing, key=lambda p: _ring_area_km2(p["ring"]))

    # 2) 同类邻近聚类 (bbox 间距 < cluster_gap_m 泛洪)
    kind_rank = _KIND_RANK[seed["kind"]]
    same_class = [p for p in plots
                  if _KIND_RANK[p["kind"]] == kind_rank]
    cluster = _cluster_from_seed(seed, same_class, cluster_gap_m)

    # 3) 凸包 = 园区外轮廓
    points = [pt for p in cluster for pt in p["ring"]]
    hull = _convex_hull(points)
    if len(hull) < 4:
        return {"reason": "degenerate_hull"}
    area = _ring_area_km2(hull)
    if not (0.02 <= area <= 60.0):
        return {"reason": "campus_area_out_of_range",
                "area_km2": round(area, 3)}
    return {"ring": hull, "kind": seed["kind"],
            "area_km2": round(area, 3),
            "cluster_plots": len(cluster),
            "containing": {"kind": seed["kind"], "name": seed["name"],
                           "area_km2": round(_ring_area_km2(seed["ring"]), 3)}}


# ---------- 内部几何 (纯 stdlib) -------------------------------------------

def _ring_of(el: dict[str, Any]) -> list[tuple[float, float]]:
    if el.get("type") == "way":
        return [(p["lat"], p["lon"]) for p in el.get("geometry", [])
                if "lat" in p]
    pts: list[tuple[float, float]] = []
    for m in el.get("members", []):
        if m.get("role") == "outer" and m.get("geometry"):
            pts += [(p["lat"], p["lon"]) for p in m["geometry"]
                    if "lat" in p]
    return pts


def _bbox(ring):
    lats = [p[0] for p in ring]
    lngs = [p[1] for p in ring]
    return (min(lats), min(lngs), max(lats), max(lngs))


def _bbox_gap_m(a, b):
    # bbox = (min_lat, min_lng, max_lat, max_lng); 轴分离间距取 max
    kx = 111320.0 * math.cos(math.radians((a[0] + a[2]) / 2))
    ky = 110540.0
    dx = max(a[1] - b[3], b[1] - a[3], 0.0)  # 经度方向间隙
    dy = max(a[0] - b[2], b[0] - a[2], 0.0)  # 纬度方向间隙
    return math.hypot(dx * kx, dy * ky)


def _cluster_from_seed(seed, candidates, gap_m):
    cluster = [seed]
    frontier = [seed]
    seen = {id(seed)}
    while frontier:
        cur = frontier.pop()
        for cand in candidates:
            if id(cand) in seen:
                continue
            if _bbox_gap_m(cur["bbox"], cand["bbox"]) <= gap_m:
                seen.add(id(cand))
                cluster.append(cand)
                frontier.append(cand)
    return cluster


def _point_in_ring(pt, ring) -> bool:
    x, y = pt
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _ring_area_km2(ring) -> float:
    if not ring:
        return 0.0
    ref = ring[0][0]
    kx = 111320.0 * math.cos(math.radians(ref))
    ky = 110540.0
    xs = [p[1] * kx for p in ring]
    ys = [p[0] * ky for p in ring]
    a2 = sum(xs[i] * ys[i - 1] - xs[i - 1] * ys[i]
             for i in range(len(xs)))
    return abs(a2) / 2.0 / 1e6


def _ring_closed_len_km(ring) -> float:
    ref = ring[0][0]
    kx = 111320.0 * math.cos(math.radians(ref))
    ky = 110540.0
    xs = [p[1] * kx for p in ring]
    ys = [p[0] * ky for p in ring]
    return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
               for i in range(len(xs))) / 1000.0


def _convex_hull(points):
    """Andrew 单调链凸包, 返回闭合环 [(lat,lng)...]。"""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]
