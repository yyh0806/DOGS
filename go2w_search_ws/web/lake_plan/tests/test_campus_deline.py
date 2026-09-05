"""test_campus_deline.py — 建筑密度园区刻画测试 (合成建筑, 封闭)。"""
from __future__ import annotations

import os

os.environ.setdefault("GO2W_LAKE_OFFLINE", "1")

from lake_plan import campus_deline
from lake_plan.osm_client import _point_in_ring


def _buildings(n=5, d=0.0004):
    """中心 (n×n) 密集建筑 + 远处 2 栋孤立建筑。"""
    els = []
    for i in range(-n // 2, n // 2 + 1):
        for j in range(-n // 2, n // 2 + 1):
            lat = 31.488192 + i * d
            lng = 120.369486 + j * d
            e = 0.00005
            els.append({"geometry": [{"lat": lat, "lon": lng},
                                     {"lat": lat + e, "lon": lng},
                                     {"lat": lat + e, "lon": lng + e},
                                     {"lat": lat, "lon": lng + e}]})
    els.append({"geometry": [{"lat": 31.4982, "lon": 120.3695},
                             {"lat": 31.4983, "lon": 120.3695},
                             {"lat": 31.4983, "lon": 120.3696},
                             {"lat": 31.4982, "lon": 120.3696}]})
    return els


def test_building_density_campus_contains_robot_not_outlier():
    ring = campus_deline.building_density_campus(
        31.488192, 120.369486, 1000.0, els=_buildings())
    assert ring is not None and len(ring) >= 4
    assert ring[0] == ring[-1]
    assert _point_in_ring((31.488192, 120.369486), ring)  # 含本体
    # 孤立建筑 (~1.1km 外) 不应并入密集域 → 边界不含它
    assert not _point_in_ring((31.4982, 120.3695), ring)


def test_building_density_none_when_sparse():
    # 只有 1 栋 (中心) + 1 栋孤立 → 无密集域
    els = [{"geometry": [{"lat": 31.488192, "lon": 120.369486},
                         {"lat": 31.488242, "lon": 120.369486},
                         {"lat": 31.488242, "lon": 120.369536},
                         {"lat": 31.488192, "lon": 120.369536}]},
           {"geometry": [{"lat": 31.4982, "lon": 120.3695},
                         {"lat": 31.4983, "lon": 120.3695},
                         {"lat": 31.4983, "lon": 120.3696},
                         {"lat": 31.4982, "lon": 120.3696}]}]
    ring = campus_deline.building_density_campus(
        31.488192, 120.369486, 1000.0, els=els)
    assert ring is None


def test_hull_basic():
    pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (5.0, 5.0), (0.0, 10.0)]
    hull = campus_deline._hull(pts)
    assert len(hull) == 4  # 内部点剔除
