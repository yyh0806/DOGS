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
