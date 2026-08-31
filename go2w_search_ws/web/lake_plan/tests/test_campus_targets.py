"""园区基准点双目标回归: 绕湖 + 绕园区, 逐航点 golden 比对。

基准再生: LAKE_PLAN_REGEN_GOLDEN=1 python -m pytest ...
(仅有意变更规划行为时使用, commit message 必须说明原因)。
"""
from __future__ import annotations

import json
import os
from pathlib import Path

from lake_plan import plan_route
from lake_plan.geo import haversine_m

# 本文件自含基准常量 (不从 conftest 导入: 多测试目录并存时
# `import conftest` 会歧义解析到 web/tests/conftest.py)
FIXTURES = Path(__file__).resolve().parent / "fixtures"
CAMPUS_CENTER = (31.488192, 120.369486)  # 太科园园区 (M2.1 基准点)


# ---------- 绕湖 (kind=water) ----------

def test_water_selects_nearest_lake(water_route):
    """先近后远: 必须选园区内/旁的近水体, 而非 10km 外的大湖。"""
    centroid = water_route["target"]["centroid"]
    dist = haversine_m(CAMPUS_CENTER[0], CAMPUS_CENTER[1],
                       centroid[0], centroid[1])
    assert dist < 3000, f"选了远处水体: 质心距中心 {dist:.0f}m"


def test_water_quality(water_route):
    stats = water_route["stats"]
    assert water_route["kind"] == "water"
    assert stats["closed"] is True
    assert stats["water_cross_ratio"] < 0.05
    assert len(water_route["waypoints"]) >= 8
    assert water_route["water_polygon"]
    wps = water_route["waypoints"]
    assert abs(wps[0]["lat"] - wps[-1]["lat"]) < 1e-5
    assert abs(wps[0]["lon"] - wps[-1]["lon"]) < 1e-5
    for wp in wps[:5]:
        assert set(("lat", "lon", "name")) <= set(wp)


def test_water_golden_match(water_route, water_golden):
    got, want = water_route["waypoints"], water_golden["waypoints"]
    assert len(got) == len(want), (
        f"航点数变化: {len(got)} != 基准 {len(want)}")
    max_d = max(max(abs(g["lat"] - w["lat"]),
                    abs(g["lon"] - w["lon"]))
                for g, w in zip(got, want))
    _maybe_regen("water_campus_golden", water_route)
    assert max_d < 1e-5, f"绕湖航线漂移 {max_d:.2e} 度"
    got_len = water_route["stats"]["length_m"]
    want_len = water_golden["stats"]["length_m"]
    assert abs(got_len - want_len) / want_len < 0.005


# ---------- 绕园区 (kind=campus) ----------

def test_campus_quality(campus_route):
    stats = campus_route["stats"]
    assert campus_route["kind"] == "campus"
    assert stats["closed"] is True
    assert stats["water_cross_ratio"] < 0.05
    assert len(campus_route["waypoints"]) >= 8
    assert campus_route["campus_polygon"]
    target = campus_route["target"]
    assert target["kind"] == "campus"
    assert target["landuse"] in ("industrial", "education", "residential",
                                 "retail", "commercial", "construction",
                                 "park")
    assert 0.02 <= target["area_km2"] <= 60.0


def test_campus_ring_outside_hull(campus_route):
    """环线必须全程在园区凸包外侧 (kind=campus 的语义: 绕着园区走)。"""
    hull = campus_route["campus_polygon"]
    from lake_plan.osm_client import _point_in_ring
    inside = sum(1 for wp in campus_route["waypoints"]
                 if _point_in_ring((wp["lat"], wp["lon"]), hull))
    assert inside == 0, f"{inside} 个航点落入园区内部"


def test_campus_golden_match(campus_route, campus_golden):
    got, want = campus_route["waypoints"], campus_golden["waypoints"]
    assert len(got) == len(want), (
        f"航点数变化: {len(got)} != 基准 {len(want)}")
    max_d = max(max(abs(g["lat"] - w["lat"]),
                    abs(g["lon"] - w["lon"]))
                for g, w in zip(got, want))
    _maybe_regen("campus_golden", campus_route)
    assert max_d < 1e-5, f"绕园区航线漂移 {max_d:.2e} 度"


def test_invalid_kind_and_center():
    assert plan_route(95.0, 300.0)["reason"] == "invalid_center"
    assert plan_route(*CAMPUS_CENTER,
                      kind="moon")["reason"] == "invalid_kind"


# ---------- 失败路径 (合成, 零网络) ----------

def test_no_suitable_lake_synthetic(monkeypatch):
    """合成纯陆地拼接图 → 全部视野无候选 → no_suitable_lake。"""
    from PIL import Image

    import lake_plan.tiles as tiles_mod

    def fake_stitch(provider, lat, lng, z, nx, ny, use_cache=True):
        img = Image.new("RGB", (nx * 256, ny * 256), (240, 238, 232))
        detail = {"provider": provider, "z": z, "x0": 0, "y0": 0,
                  "nx": nx, "ny": ny, "w": nx * 256, "h": ny * 256,
                  "tiles_ok": nx * ny, "tiles_miss": [], "crs": "wgs84"}
        return img, detail

    monkeypatch.setattr(tiles_mod, "stitch_centered", fake_stitch)
    result = plan_route(*CAMPUS_CENTER, kind="water")
    assert result["ok"] is False
    assert result["reason"] == "no_suitable_lake"
    assert result["candidates"] == []


def test_campus_not_inside_any_plot(monkeypatch):
    """空 Overpass 响应 → 无地块 → not_inside_any_plot / no_landuse_plots。"""
    import lake_plan.osm_client as osm_mod

    monkeypatch.setattr(osm_mod, "overpass_query",
                        lambda q: {"elements": []})
    result = plan_route(*CAMPUS_CENTER, kind="campus")
    assert result["ok"] is False
    assert result["reason"] in ("no_landuse_plots", "not_inside_any_plot")


def _maybe_regen(name: str, result: dict) -> None:
    if os.environ.get("LAKE_PLAN_REGEN_GOLDEN") != "1":
        return
    out = {"center": list(CAMPUS_CENTER), "kind": result["kind"],
           "result": result}
    (FIXTURES / f"{name}.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
