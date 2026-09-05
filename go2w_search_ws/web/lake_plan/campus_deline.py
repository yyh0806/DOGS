"""campus_deline — 园区边界刻画 (矢量方法, 2026-09-05)。

用户问题: 卫星图 VLM 圈园区 mask 不准 (免费模型 IoU 0.14)。工程答案:
园区本质是建筑群 —— 用 OSM 建筑足迹做网格密度聚类, 以本体为中心的
密集连通域边界即园区 mask。确定性、免费、可离线缓存, 与模型无关。

方法:
  1. Overpass 取半径内建筑足迹 (way["building"] out geom);
  2. 40m 网格统计建筑覆盖密度 → 密度阈值二值化;
  3. 取包含本体的密集连通域;
  4. 域内建筑质心的凸包 + 30m 缓冲 = 园区边界多边形。
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any, Optional

_GRID_M = 40.0
_DENSITY_MIN = 0.08   # 单元格内建筑覆盖面积占比阈值
_BUFFER_M = 30.0      # 凸包外扩 (围墙/绿化带)


def overpass_buildings(lat: float, lng: float, radius_m: float,
                       timeout: int = 90) -> list[dict[str, Any]]:
    """Overpass: 半径内建筑足迹 (geometry)。网络失败抛 OSError。"""
    import json
    import urllib.parse
    import urllib.request
    query = f"""
[out:json][timeout:{timeout}];
way["building"](around:{int(radius_m)},{lat},{lng});
out geom;
"""
    url = ("https://overpass-api.de/api/interpreter?data="
           + urllib.parse.quote(query))
    req = urllib.request.Request(url, headers={"User-Agent":
                                               "go2w-lake-plan/1.0"})
    with urllib.request.urlopen(req, timeout=timeout + 30) as resp:
        payload = json.loads(resp.read().decode("utf-8"))
    return payload.get("elements", [])


def buildings_to_centroids(els: list[dict[str, Any]]) -> list[tuple[float, float]]:
    out = []
    for e in els:
        geom = e.get("geometry") or []
        if len(geom) < 3:
            continue
        out.append((sum(g["lat"] for g in geom) / len(geom),
                    sum(g["lon"] for g in geom) / len(geom)))
    return out


def _cell(lat: float, lng: float, clat: float, clng: float):
    ky = _GRID_M / 110540.0
    kx = _GRID_M / (111320.0 * math.cos(math.radians(clat)))
    return int((lat - clat) / ky), int((lng - clng) / kx)


def _hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
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


def _buffer_hull(hull, clat, dist_m, n_per_edge=4):
    """凸包向外缓冲: 顶点沿外法线推 dist_m (近似, 足够做园区 mask)。"""
    ky = 110540.0
    kx = 111320.0 * math.cos(math.radians(clat))
    out = []
    m = len(hull)
    for i in range(m):
        x0, y0 = hull[i - 1][1] * kx, hull[i - 1][0] * ky
        x1, y1 = hull[i][1] * kx, hull[i][0] * ky
        x2, y2 = hull[(i + 1) % m][1] * kx, hull[(i + 1) % m][0] * ky
        for dx, dy in ((x1 - x0, y1 - y0), (x2 - x1, y2 - y1)):
            n = math.hypot(dx, dy) or 1.0
            # 外法线 (左侧)
            nx, ny = -dy / n, dx / n
            out.append(((y1 + ny * dist_m) / ky, (x1 + nx * dist_m) / kx))
    return _hull(out)


def building_density_campus(lat: float, lng: float, radius_m: float = 1000.0,
                            grid_m: float = _GRID_M,
                            density_min: float = _DENSITY_MIN,
                            buffer_m: float = _BUFFER_M,
                            els: Optional[list[dict]] = None
                            ) -> Optional[list[tuple[float, float]]]:
    """建筑密度聚类 → 园区边界多边形 (含本体); 失败返回 None。"""
    global _GRID_M, _DENSITY_MIN, _BUFFER_M  # noqa: PLW0603 (参数化网格)
    _GRID_M, _DENSITY_MIN, _BUFFER_M = grid_m, density_min, buffer_m
    if els is None:
        try:
            els = overpass_buildings(lat, lng, radius_m)
        except OSError:
            return None
    centroids = buildings_to_centroids(els)
    if not centroids:
        return None
    # 密度网格 (覆盖面积近似: 每建筑按 200m² 计 → 面积占比)
    cells: dict[tuple[int, int], float] = {}
    for clat_c, clng_c in centroids:
        cells[_cell(clat_c, clng_c, lat, lng)] = \
            cells.get(_cell(clat_c, clng_c, lat, lng), 0.0) + 200.0
    area_cell = grid_m * grid_m
    dense = {c for c, a in cells.items() if a / area_cell >= density_min}
    if not dense:
        return None
    # 取包含本体的连通域 (4 邻域)
    home = _cell(lat, lng, lat, lng)
    if home not in dense:
        # 本体格不密集 → 找最近密集格
        home = min(dense, key=lambda c: (c[0] - home[0]) ** 2
                   + (c[1] - home[1]) ** 2)
    comp = set()
    q = deque([home])
    while q:
        c = q.popleft()
        if c in comp or c not in dense:
            continue
        comp.add(c)
        for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            q.append((c[0] + d[0], c[1] + d[1]))
    # 域内建筑质心 → 凸包 → 缓冲
    ky = grid_m / 110540.0
    kx = grid_m / (111320.0 * math.cos(math.radians(lat)))
    inside = [(clat_c, clng_c) for clat_c, clng_c in centroids
              if _cell(clat_c, clng_c, lat, lng) in comp]
    hull = _hull(inside)
    if len(hull) < 3:
        return None
    ring = _buffer_hull(hull, lat, buffer_m)
    ring.append(ring[0])
    return ring
