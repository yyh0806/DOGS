"""golden 航线回归: 固定瓦片 → 规划输出必须与基准一致 (偏差 < 1m / 0.5%)。

这是 M2 的核心回归件: 任何 planner/water/geo 的改动若悄悄劣化航线,
这里立刻红。基准再生: LAKE_PLAN_REGEN_GOLDEN=1 python -m pytest ...
(仅在有意变更规划行为时使用, 并在 commit message 里说明原因)。
"""
from __future__ import annotations

import json
import os
from pathlib import Path

from conftest import FIXTURES, LIHU_CENTER

from lake_plan import plan_route


def test_lihu_selected(lihu_route):
    """选中的必须是蠡湖本身 (质心距中心 < 2km), 不是远处小水体。"""
    centroid = lihu_route["lake"]["centroid"]
    from lake_plan.geo import haversine_m
    dist = haversine_m(LIHU_CENTER[0], LIHU_CENTER[1],
                       centroid[0], centroid[1])
    assert dist < 2000, f"选错湖: 质心 {centroid} 距中心 {dist:.0f}m"


def test_route_quality(lihu_route):
    stats = lihu_route["stats"]
    assert stats["closed"] is True
    assert stats["water_cross_ratio"] < 0.05
    assert len(lihu_route["waypoints"]) >= 8
    # 首尾闭合 (POST /api/gps/route 契约: 环线)
    wps = lihu_route["waypoints"]
    assert abs(wps[0]["lat"] - wps[-1]["lat"]) < 1e-5
    assert abs(wps[0]["lon"] - wps[-1]["lon"]) < 1e-5
    # 航点字段契约: lat/lon/name (注意是 lon, 对齐 nx_web_server)
    for wp in wps[:5]:
        assert set(("lat", "lon", "name")) <= set(wp)


def test_golden_waypoints_match(lihu_route, golden):
    """逐航点与基准比对: 最大偏差 < 1e-5 度 (约 1.1m)。"""
    regen = os.environ.get("LAKE_PLAN_REGEN_GOLDEN") == "1"
    got, want = lihu_route["waypoints"], golden["waypoints"]
    assert len(got) == len(want), (
        f"航点数变化: {len(got)} != 基准 {len(want)}")
    max_d = 0.0
    for g, w in zip(got, want):
        max_d = max(max_d, abs(g["lat"] - w["lat"]),
                    abs(g["lon"] - w["lon"]))
    if regen:
        out = {"center": list(LIHU_CENTER), "result": lihu_route}
        (FIXTURES / "lake_lihu_golden.json").write_text(
            json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    assert max_d < 1e-5, f"航线漂移 {max_d:.2e} 度 (>1e-5), 见基准再生流程"


def test_golden_length_within_tolerance(lihu_route, golden):
    got = lihu_route["stats"]["length_m"]
    want = golden["stats"]["length_m"]
    assert abs(got - want) / want < 0.005, (
        f"环线长度变化 {got:.1f}m vs 基准 {want:.1f}m (>0.5%)")


def test_invalid_center_fail_closed():
    result = plan_route(95.0, 300.0)
    assert result["ok"] is False
    assert result["reason"] == "invalid_center"


def test_no_suitable_lake_synthetic(monkeypatch):
    """合成纯陆地拼接图 → 三级视野都无候选 → no_suitable_lake。

    零瓦片依赖 (monkeypatch stitch), 确定性验证拒绝路径的结构化返回。
    """
    from PIL import Image

    import lake_plan.tiles as tiles_mod

    def fake_stitch(provider, lat, lng, z, nx, ny, use_cache=True):
        # 纯"陆地"色 (非水): HSV/参考色都不命中 → 空掩码
        img = Image.new("RGB", (nx * 256, ny * 256), (240, 238, 232))
        detail = {"provider": provider, "z": z, "x0": 0, "y0": 0,
                  "nx": nx, "ny": ny, "w": nx * 256, "h": ny * 256,
                  "tiles_ok": nx * ny, "tiles_miss": [], "crs": "wgs84"}
        return img, detail

    monkeypatch.setattr(tiles_mod, "stitch_centered", fake_stitch)
    result = plan_route(*LIHU_CENTER)
    assert result["ok"] is False
    assert result["reason"] == "no_suitable_lake"
    assert result["candidates"] == []
