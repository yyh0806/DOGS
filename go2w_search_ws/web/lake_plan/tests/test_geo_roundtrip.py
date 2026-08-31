"""geo 数学回归: Web-Mercator 往返 + Georef 像素往返 + GCJ-02 逼近。"""
from __future__ import annotations

from lake_plan.geo import (StitchGeoref, gcj02_to_wgs84, global_px_to_lat,
                           global_px_to_lng, haversine_m, lat_to_global_px,
                           lng_to_global_px, wgs84_to_gcj02)


def test_mercator_roundtrip():
    for lat, lng in ((31.5163, 120.2673), (0.0, 0.0), (-45.0, 30.0)):
        px, py = lng_to_global_px(lng, 15), lat_to_global_px(lat, 15)
        assert abs(global_px_to_lng(px, 15) - lng) < 1e-9
        assert abs(global_px_to_lat(py, 15) - lat) < 1e-6


def test_georef_pixel_roundtrip():
    georef = StitchGeoref(15, 27338, 13361, 256, 256, "wgs84")
    lat, lng = georef.pixel_to_latlon(100.0, 200.0)
    px, py = georef.latlon_to_pixel(lat, lng)
    assert abs(px - 100.0) < 1e-6 and abs(py - 200.0) < 1e-6


def test_haversine_known_distance():
    # 无锡到上海 约 120km 量级 (粗校验)
    d = haversine_m(31.49, 120.31, 31.23, 121.47)
    assert 100_000 < d < 150_000


def test_gcj02_wgs84_roundtrip():
    lat, lng = 31.5163, 120.2673
    glat, glng = wgs84_to_gcj02(lat, lng)
    wlat, wlng = gcj02_to_wgs84(glat, glng)
    assert abs(wlat - lat) < 1e-6 and abs(wlng - lng) < 1e-6
