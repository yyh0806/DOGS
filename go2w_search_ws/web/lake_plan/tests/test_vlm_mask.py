"""test_vlm_mask.py — VLM 直接 mask 的解析/校验/降级测试 (2026-09-05)。"""
from __future__ import annotations

import os

os.environ.setdefault("GO2W_LAKE_OFFLINE", "1")

from PIL import Image

from lake_plan import vlm_mask


class _FakeVlm:
    def __init__(self, text="", available=True):
        self._text = text
        self._avail = available

    def available(self):
        return self._avail

    def vision(self, image, prompt, max_tokens=1024):
        return self._text


IMG = Image.new("RGB", (100, 80))


def test_vlm_polygon_parses_and_closes():
    vlm = _FakeVlm('```json {"polygon": [[0.1,0.1],[0.9,0.1],[0.9,0.9],'
                   '[0.1,0.9]], "why": "ok"}```')
    out = vlm_mask.vlm_polygon(vlm, IMG, "圈园区")
    assert out is not None
    assert len(out["polygon"]) == 5  # 自动闭合
    assert out["polygon"][-1] == out["polygon"][0]


def test_vlm_polygon_rejects_bad():
    assert vlm_mask.vlm_polygon(_FakeVlm("{}"), IMG, "x") is None
    assert vlm_mask.vlm_polygon(
        _FakeVlm('{"polygon": [[0.1,0.1],[0.9,0.1]]}'), IMG, "x") is None
    # 越界坐标
    assert vlm_mask.vlm_polygon(
        _FakeVlm('{"polygon": [[0.1,0.1],[0.9,0.1],[0.9,2.0],[0.1,0.9]]}'),
        IMG, "x") is None
    # 面积过小 (归一化 < 0.5%)
    assert vlm_mask.vlm_polygon(
        _FakeVlm('{"polygon": [[0.1,0.1],[0.1002,0.1],[0.1002,0.1002],'
                 '[0.1,0.1002]]}'), IMG, "x") is None
    assert vlm_mask.vlm_polygon(_FakeVlm("抱歉, 看不清"), IMG, "x") is None


def test_vlm_polygon_unavailable():
    assert vlm_mask.vlm_polygon(None, IMG, "x") is None
    assert vlm_mask.vlm_polygon(_FakeVlm("{}", available=False),
                                IMG, "x") is None


def test_poly_to_latlon_bounds():
    from lake_plan.geo import StitchGeoref
    geo = StitchGeoref(17, 100, 100, 256, 256, "wgs84")
    ll = vlm_mask.poly_to_latlon([[0.0, 0.0], [1.0, 1.0]], geo)
    assert len(ll) == 3  # 自动闭合
    assert ll[0] == ll[-1]
    assert ll[0] == geo.pixel_to_latlon(0, 0)
    assert ll[1] == geo.pixel_to_latlon(256, 256)


def test_circle_poly_radius():
    ring = vlm_mask.circle_poly(31.488192, 120.369486, 400.0)
    assert len(ring) == 25
    assert ring[0] == ring[-1]
    for lat, lng in ring[:-1]:
        dlat = (lat - 31.488192) * 110540
        dlng = (lng - 120.369486) * 111320 * 0.852
        assert 395 < (dlat ** 2 + dlng ** 2) ** 0.5 < 405


def test_poly_area_frac():
    square = [[0.1, 0.1], [0.9, 0.1], [0.9, 0.9], [0.1, 0.9], [0.1, 0.1]]
    assert abs(vlm_mask.poly_area_frac(square) - 0.64) < 1e-9
    # 未闭合也能算 (自动按列表)
    open_ = square[:-1]
    assert abs(vlm_mask.poly_area_frac(open_) - 0.64) < 1e-9
    assert vlm_mask.poly_area_frac([[0.1, 0.1], [0.2, 0.2]]) == 0.0


def test_point_in_poly01():
    square = [[0.1, 0.1], [0.9, 0.1], [0.9, 0.9], [0.1, 0.9], [0.1, 0.1]]
    assert vlm_mask.point_in_poly01(0.5, 0.5, square) is True
    assert vlm_mask.point_in_poly01(0.05, 0.5, square) is False
    # GLM 免费模型实测的对角线假形状: 机器人 (0.5,0.5) 不在带内
    band = [[0.2, 0.3], [0.4, 0.5], [0.6, 0.7], [0.8, 0.9], [0.9, 0.95],
            [0.85, 0.98], [0.75, 0.99], [0.55, 0.95], [0.35, 0.91],
            [0.15, 0.87], [0.01, 0.83], [0.2, 0.3]]
    assert vlm_mask.point_in_poly01(0.5, 0.5, band) is False
    assert vlm_mask.point_in_poly01(0.5, 0.9, band) is True
