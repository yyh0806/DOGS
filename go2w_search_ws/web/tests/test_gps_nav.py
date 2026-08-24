"""室外 GPS 航线导航 —— 纯逻辑核心行为测试 (无 ROS 依赖)。

覆盖三条主线:
  1. 数据入口可信: NMEA 解析 (严格校验) / NavSatFix 转换 / 大地坐标数学;
  2. GpsFixGate fail-closed 门禁: 过期/无定位/卫星数/HDOP 全部拒绝;
  3. GpsRouteController 航线状态机: 受理预检、逐航点推进、
     每一条停车路径 (GPS 降级/超时/所有权被抢/提交失败) 都必须 park 或拒绝。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

WEB = Path(__file__).resolve().parent.parent  # go2w_search_ws/web
if str(WEB) not in sys.path:
    sys.path.insert(0, str(WEB))

import nx_gps_nav as gps_nav_module  # noqa: E402
from nx_gps_nav import (  # noqa: E402
    GpsFix,
    GpsFixGate,
    GpsRouteController,
    GpsWaypoint,
    derive_heading_from_track,
    enu_from_latlon,
    latlon_from_enu,
    map_goal_for_waypoint,
    nav_sat_fix_to_gps_fix,
    parse_nmea_sentence,
)


GGA_VALID = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
RMC_VALID = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"


def _rebuild(body: str) -> str:
    """按内容重算校验和, 拼出完整 NMEA 语句 (避免手写校验和出错)。"""
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return "$%s*%02X" % (body, checksum)


GGA_SOUTH = _rebuild("GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,")


class FakeClock:
    """可拨动的单调时钟 (测试全程不真睡)。"""

    def __init__(self, start=1000.0):
        self.now = float(start)

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += float(seconds)


class FakePointPort:
    """PointNavigationController 的鸭子类型替身。"""

    def __init__(self):
        self.state = {
            "status": "idle",
            "generation": 0,
            "drained": True,
            "healthy": True,
            "quarantined": False,
            "stopped": False,
        }
        self.submissions = []
        self.cancels = []
        self.submit_raises = None
        self.submit_result = None
        self.get_state_raises = False

    def get_state(self):
        if self.get_state_raises:
            raise RuntimeError("executor gone")
        return dict(self.state)

    def submit(self, x, y, yaw=0.0):
        if self.submit_raises is not None:
            raise self.submit_raises
        self.submissions.append((x, y, yaw))
        if self.submit_result is not None:
            return dict(self.submit_result)
        self.state["generation"] += 1
        self.state.update({"status": "pending", "drained": False})
        return {"ok": True, "generation": self.state["generation"]}

    def cancel(self, reason="canceled"):
        self.cancels.append(reason)
        self.state.update({"status": "canceled", "drained": True})
        return True

    def succeed(self):
        self.state.update({"status": "succeeded", "drained": True})

    def fail(self, status="aborted"):
        self.state.update({"status": status, "drained": True})


def make_controller(port, clock, parks=None, states=None, **kwargs):
    controller = GpsRouteController(
        port,
        fix_gate=kwargs.pop("fix_gate", None) or GpsFixGate(max_age_sec=2.0),
        park_hook=(parks.append if parks is not None else None),
        state_callback=(states.append if states is not None else None),
        monotonic=clock,
        sleep=lambda _s: None,
        **kwargs,
    )
    controller.set_heading_calibration(0.0)  # 测试默认北向 = map +x
    return controller


def fresh_fix(clock, lat=30.0, lon=120.0, **overrides):
    payload = dict(
        lat=lat, lon=lon, stamp=clock.now, altitude=10.0,
        fix_status=0, satellites=10, hdop=1.0,
    )
    payload.update(overrides)
    return GpsFix(**payload)


def anchor_pose():
    return {"x": 5.0, "y": -3.0}


# ---------------------------------------------------------------------------
# NMEA 解析
# ---------------------------------------------------------------------------

class TestNmeaParsing:
    def test_valid_gga_parses_position_quality_and_hdop(self):
        fix = parse_nmea_sentence(GGA_VALID)
        assert fix is not None
        assert fix.lat == pytest.approx(48.0 + 7.038 / 60.0)
        assert fix.lon == pytest.approx(11.0 + 31.0 / 60.0)
        assert fix.fix_status == 0
        assert fix.satellites == 8
        assert fix.hdop == pytest.approx(0.9)
        assert fix.altitude == pytest.approx(545.4)

    def test_south_west_hemispheres_go_negative(self):
        fix = parse_nmea_sentence(GGA_SOUTH)
        assert fix is not None
        assert fix.lat < 0.0 and fix.lon < 0.0

    def test_gga_zero_quality_is_no_fix(self):
        body = _rebuild("GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,")
        fix = parse_nmea_sentence(body)
        assert fix is not None
        assert fix.fix_status == -1

    def test_bad_checksum_rejected(self):
        assert parse_nmea_sentence(GGA_VALID[:-2] + "48") is None

    def test_minutes_overflow_rejected(self):
        # 48°70.0' 的"分"字段 ≥60, 必属坏数据
        assert parse_nmea_sentence(_rebuild(
            "GPGGA,123519,4870.000,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,")) is None

    def test_valid_rmc_maps_active_status(self):
        fix = parse_nmea_sentence(RMC_VALID)
        assert fix is not None
        assert fix.fix_status == 0
        assert fix.lat == pytest.approx(48.1173, abs=1e-4)

    def test_rmc_void_status_is_no_fix(self):
        fix = parse_nmea_sentence(_rebuild(
            "GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"))
        assert fix is not None
        assert fix.fix_status == -1

    def test_unrelated_sentence_and_garbage_rejected(self):
        assert parse_nmea_sentence(_rebuild(
            "GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1")) is None
        assert parse_nmea_sentence("garbage") is None
        assert parse_nmea_sentence("") is None
        assert parse_nmea_sentence(None) is None


# ---------------------------------------------------------------------------
# 大地坐标 → ENU
# ---------------------------------------------------------------------------

class TestGeodesy:
    def test_zero_offset(self):
        assert enu_from_latlon(30.0, 120.0, 30.0, 120.0) == (0.0, 0.0)

    def test_one_degree_latitude_is_about_110km(self):
        east, north = enu_from_latlon(30.0, 120.0, 31.0, 120.0)
        assert east == pytest.approx(0.0, abs=1e-6)
        assert north == pytest.approx(110850.0, rel=5e-4)

    def test_one_degree_longitude_at_equator_is_about_111km(self):
        east, north = enu_from_latlon(0.0, 100.0, 0.0, 101.0)
        assert east == pytest.approx(111319.0, rel=5e-4)
        assert north == pytest.approx(0.0, abs=1e-6)

    def test_roundtrip_is_exact(self):
        for lat0, lon0, e, n in [
            (0.0, 0.0, 10.0, 20.0),
            (30.0, 120.0, -123.4, 567.8),
            (-45.0, -70.0, 1000.0, -2000.0),
        ]:
            lat, lon = latlon_from_enu(lat0, lon0, e, n)
            e2, n2 = enu_from_latlon(lat0, lon0, lat, lon)
            assert e2 == pytest.approx(e, abs=1e-6)
            assert n2 == pytest.approx(n, abs=1e-6)

    def test_map_goal_rotates_by_north_heading(self):
        anchor = GpsFix(lat=30.0, lon=120.0, stamp=1.0)
        wp_lat, wp_lon = latlon_from_enu(30.0, 120.0, 0.0, 10.0)  # 正北 10m
        waypoint = GpsWaypoint(lat=wp_lat, lon=wp_lon)
        # 北向 = map +x: 北 10m → +x
        x, y = map_goal_for_waypoint(anchor, (5.0, -3.0), 0.0, waypoint)
        assert x == pytest.approx(15.0, abs=1e-3)
        assert y == pytest.approx(-3.0, abs=1e-3)
        # 北向 = map +y: 北 10m → +y
        x, y = map_goal_for_waypoint(anchor, (5.0, -3.0), 90.0, waypoint)
        assert x == pytest.approx(5.0, abs=1e-3)
        assert y == pytest.approx(7.0, abs=1e-3)
        # 东 10m 在北向=map+y (H=90, R=I) 时 → +x
        e_lat, e_lon = latlon_from_enu(30.0, 120.0, 10.0, 0.0)
        x, y = map_goal_for_waypoint(
            anchor, (5.0, -3.0), 90.0, GpsWaypoint(lat=e_lat, lon=e_lon))
        assert x == pytest.approx(15.0, abs=1e-3)
        assert y == pytest.approx(-3.0, abs=1e-3)


# ---------------------------------------------------------------------------
# GPS 质量门禁 (fail-closed)
# ---------------------------------------------------------------------------

class TestFixGate:
    def test_fresh_valid_fix_passes(self):
        clock = FakeClock()
        gate = GpsFixGate(max_age_sec=2.0)
        assert gate.evaluate(fresh_fix(clock), clock.now).ok

    def test_missing_fix_rejected(self):
        assert not GpsFixGate().evaluate(None, 100.0).ok

    def test_stale_fix_rejected(self):
        clock = FakeClock()
        gate = GpsFixGate(max_age_sec=2.0)
        clock.advance(2.5)
        health = gate.evaluate(fresh_fix(FakeClock(1000.0)), clock.now)
        assert not health.ok
        assert health.reason == "stale_fix"

    def test_clock_skew_rejected(self):
        gate = GpsFixGate(max_age_sec=2.0, future_tolerance_sec=0.25)
        fix = GpsFix(lat=30.0, lon=120.0, stamp=1100.0)
        health = gate.evaluate(fix, 1000.0)
        assert not health.ok
        assert health.reason == "clock_skew"

    def test_small_future_stamp_tolerated(self):
        gate = GpsFixGate(max_age_sec=2.0, future_tolerance_sec=0.25)
        fix = GpsFix(lat=30.0, lon=120.0, stamp=1000.1)
        assert gate.evaluate(fix, 1000.0).ok

    def test_no_fix_status_rejected(self):
        clock = FakeClock()
        gate = GpsFixGate()
        fix = fresh_fix(clock, fix_status=-1)
        assert gate.evaluate(fix, clock.now).reason == "no_fix"

    @pytest.mark.parametrize("lat,lon", [(91.0, 0.0), (0.0, 181.0), (float("nan"), 0.0)])
    def test_invalid_coordinates_rejected(self, lat, lon):
        clock = FakeClock()
        gate = GpsFixGate()
        fix = GpsFix(lat=lat, lon=lon, stamp=clock.now)
        assert gate.evaluate(fix, clock.now).reason == "invalid_coordinates"

    def test_min_satellites_enforced_when_configured(self):
        clock = FakeClock()
        gate = GpsFixGate(min_satellites=6)
        assert gate.evaluate(fresh_fix(clock, satellites=4), clock.now).reason ==             "insufficient_satellites"
        assert gate.evaluate(fresh_fix(clock, satellites=8), clock.now).ok

    def test_min_satellites_with_unknown_count_rejected_in_strict_mode(self):
        # 显式开启严格模式后, "不知道卫星数" 也不放行 (fail-closed)。
        clock = FakeClock()
        gate = GpsFixGate(min_satellites=6)
        fix = fresh_fix(clock, satellites=None)
        assert gate.evaluate(fix, clock.now).reason == "insufficient_satellites"

    def test_default_gate_ignores_unknown_satellite_count(self):
        # 默认 (未配置 min_satellites): NavSatFix 无卫星数字段, 仍可放行。
        clock = FakeClock()
        gate = GpsFixGate()
        assert gate.evaluate(fresh_fix(clock, satellites=None), clock.now).ok

    def test_hdop_exceeded_rejected(self):
        clock = FakeClock()
        gate = GpsFixGate(max_hdop=2.0)
        assert gate.evaluate(fresh_fix(clock, hdop=2.5), clock.now).reason ==             "hdop_exceeded"
        assert gate.evaluate(fresh_fix(clock, hdop=None), clock.now).reason ==             "hdop_exceeded"
        assert gate.evaluate(fresh_fix(clock, hdop=1.5), clock.now).ok


# ---------------------------------------------------------------------------
# NavSatFix 鸭子类型转换
# ---------------------------------------------------------------------------

class TestNavSatFixConverter:
    def _msg(self, lat=30.0, lon=120.0, alt=10.0, status=0):
        return SimpleNamespace(
            latitude=lat, longitude=lon, altitude=alt,
            status=SimpleNamespace(status=status),
        )

    def test_maps_fields_and_status(self):
        fix = nav_sat_fix_to_gps_fix(self._msg(), 1234.5)
        assert fix == GpsFix(
            lat=30.0, lon=120.0, stamp=1234.5,
            altitude=10.0, fix_status=0)

    def test_no_fix_status_preserved(self):
        fix = nav_sat_fix_to_gps_fix(self._msg(status=-1), 1234.5)
        assert fix is not None and fix.fix_status == -1

    def test_non_finite_coordinates_return_none(self):
        assert nav_sat_fix_to_gps_fix(self._msg(lat=float("nan")), 1.0) is None
        assert nav_sat_fix_to_gps_fix(self._msg(lon=float("inf")), 1.0) is None

    def test_broken_message_returns_none(self):
        assert nav_sat_fix_to_gps_fix(None, 1.0) is None
        assert nav_sat_fix_to_gps_fix(object(), 1.0) is None


# ---------------------------------------------------------------------------
# 北向标定辅助
# ---------------------------------------------------------------------------

class TestHeadingCalibration:
    def test_pure_north_track_derives_90_degrees(self):
        a_fix = GpsFix(lat=30.0, lon=120.0, stamp=1.0)
        t_lat, t_lon = latlon_from_enu(30.0, 120.0, 0.0, 25.0)
        t_fix = GpsFix(lat=t_lat, lon=t_lon, stamp=2.0)
        heading = derive_heading_from_track(
            a_fix, (0.0, 0.0), t_fix, (0.0, 25.0))
        assert heading == pytest.approx(90.0, abs=1e-6)

    def test_pure_east_track_also_derives_90_degrees(self):
        # 东 = map +x 时北必为 map +y (H=90): 正交关系是标定自洽性检查。
        a_fix = GpsFix(lat=30.0, lon=120.0, stamp=1.0)
        t_lat, t_lon = latlon_from_enu(30.0, 120.0, 25.0, 0.0)
        t_fix = GpsFix(lat=t_lat, lon=t_lon, stamp=2.0)
        heading = derive_heading_from_track(
            a_fix, (0.0, 0.0), t_fix, (25.0, 0.0))
        assert heading == pytest.approx(90.0, abs=1e-6)

    def test_north_track_along_map_x_derives_0_degrees(self):
        a_fix = GpsFix(lat=30.0, lon=120.0, stamp=1.0)
        t_lat, t_lon = latlon_from_enu(30.0, 120.0, 0.0, 25.0)
        t_fix = GpsFix(lat=t_lat, lon=t_lon, stamp=2.0)
        heading = derive_heading_from_track(
            a_fix, (0.0, 0.0), t_fix, (25.0, 0.0))
        assert heading == pytest.approx(0.0, abs=1e-6)

    def test_degenerate_track_rejected(self):
        a_fix = GpsFix(lat=30.0, lon=120.0, stamp=1.0)
        heading = derive_heading_from_track(
            a_fix, (0.0, 0.0), a_fix, (0.05, 0.0))
        assert heading is None

    def test_controller_requires_calibration_before_first_route(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = GpsRouteController(port, monotonic=clock)
        controller.update_fix(fresh_fix(clock))
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["reason"] == "heading_not_calibrated"
        assert port.submissions == []

    def test_calibration_unblocks_admission(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = GpsRouteController(port, monotonic=clock)
        controller.update_fix(fresh_fix(clock))
        assert controller.set_heading_calibration(0.0)["ok"]
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["ok"]

    def test_invalid_calibration_rejected(self):
        controller = GpsRouteController(FakePointPort())
        assert controller.set_heading_calibration(float("nan"))["ok"] is False


# ---------------------------------------------------------------------------
# 航线受理 (fail-closed 预检)
# ---------------------------------------------------------------------------

class TestRouteAdmission:
    def test_route_requires_fresh_fix(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["reason"] == "gps_no_fix"
        assert port.submissions == []

    def test_route_requires_fresh_enough_fix(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        controller.update_fix(fresh_fix(FakeClock(1000.0)))
        clock.advance(3.0)
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["reason"] == "gps_stale_fix"

    def test_empty_route_rejected(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        controller.update_fix(fresh_fix(clock))
        assert controller.submit_route([], map_pose=anchor_pose())["reason"] ==             "empty_route"

    @pytest.mark.parametrize("waypoint", [
        {"lat": 91.0, "lon": 120.0},
        {"lat": 30.0, "lon": 200.0},
        {"lat": float("nan"), "lon": 120.0},
        {"lat": 30.0},
        "not-a-dict",
    ])
    def test_malformed_waypoint_rejected(self, waypoint):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        controller.update_fix(fresh_fix(clock))
        assert controller.submit_route(
            [waypoint], map_pose=anchor_pose())["reason"] == "invalid_waypoint"

    @pytest.mark.parametrize("flag,reason", [
        ("quarantined", "point_nav_quarantined"),
        ("stopped", "point_nav_stopped"),
        ("healthy", "point_nav_unhealthy"),
    ])
    def test_unhealthy_port_rejected(self, flag, reason):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        controller.update_fix(fresh_fix(clock))
        port.state[flag] = True if flag != "healthy" else False
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["reason"] == reason
        assert port.submissions == []

    def test_waypoint_out_of_range_rejected(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock, max_waypoint_range_m=100.0)
        controller.update_fix(fresh_fix(clock))
        # 0.01° ≈ 1.1km, 远超 100m 上限
        result = controller.submit_route(
            [{"lat": 30.01, "lon": 120.0}], map_pose=anchor_pose())
        assert result["reason"] == "waypoint_out_of_range"
        assert port.submissions == []

    def test_second_route_rejected_while_active(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        controller.update_fix(fresh_fix(clock))
        assert controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())["ok"]
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.002}], map_pose=anchor_pose())
        assert result["reason"] == "route_active"

    def test_invalid_map_pose_rejected(self):
        port = FakePointPort()
        clock = FakeClock()
        controller = make_controller(port, clock)
        controller.update_fix(fresh_fix(clock))
        assert controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}],
            map_pose={"x": float("nan"), "y": 0.0})["reason"] == "invalid_map_pose"

    def test_accepted_route_anchors_goal_in_map_frame(self):
        port = FakePointPort()
        clock = FakeClock()
        parks = []
        controller = make_controller(port, clock, parks=parks)
        controller.update_fix(fresh_fix(clock))
        wp_lat, wp_lon = latlon_from_enu(30.0, 120.0, 0.0, 10.0)
        result = controller.submit_route(
            [{"lat": wp_lat, "lon": wp_lon, "yaw": 1.5}],
            map_pose=anchor_pose())
        assert result["ok"]
        assert port.submissions, "首个航点必须立刻提交给 point_port"
        x, y, yaw = port.submissions[0]
        assert x == pytest.approx(15.0, abs=1e-3)  # 北 10m → +x
        assert y == pytest.approx(-3.0, abs=1e-3)
        assert yaw == pytest.approx(1.5)
        assert parks == []  # 受理本身不该停车


# ---------------------------------------------------------------------------
# 航线生命周期与停车路径
# ---------------------------------------------------------------------------

class TestRouteLifecycle:
    def _accepted(self, waypoint_count=1, **kwargs):
        port = FakePointPort()
        clock = FakeClock()
        parks = []
        states = []
        controller = make_controller(port, clock, parks=parks, states=states, **kwargs)
        controller.update_fix(fresh_fix(clock))
        waypoints = []
        for i in range(waypoint_count):
            lat, lon = latlon_from_enu(30.0, 120.0, 0.0, 10.0 * (i + 1))
            waypoints.append({"lat": lat, "lon": lon})
        result = controller.submit_route(waypoints, map_pose=anchor_pose())
        assert result["ok"]
        return controller, port, clock, parks, states

    def _refresh_fix(self, controller, clock):
        # tick 用 gate(max_age=2s) 复查新鲜度, 每次推进前续上新鲜定位。
        controller.update_fix(fresh_fix(clock))

    def test_first_waypoint_submitted_and_state_navigating(self):
        controller, port, _, _, _ = self._accepted()
        state = controller.get_state()
        assert state["active"] is True
        assert state["status"] == "navigating"
        assert state["waypoint_index"] == 0
        assert state["waypoint_total"] == 1

    def test_success_advances_to_next_waypoint(self):
        controller, port, clock, _, _ = self._accepted(waypoint_count=2)
        first_gen = port.state["generation"]
        port.succeed()
        self._refresh_fix(controller, clock)
        controller.tick()
        assert len(port.submissions) == 2
        assert port.state["generation"] > first_gen
        state = controller.get_state()
        assert state["waypoint_index"] == 1
        assert state["status"] == "navigating"

    def test_final_success_completes_and_parks(self):
        controller, port, clock, parks, _ = self._accepted(waypoint_count=2)
        port.succeed(); self._refresh_fix(controller, clock); controller.tick()
        port.succeed(); self._refresh_fix(controller, clock); controller.tick()
        state = controller.get_state()
        assert state["status"] == "completed"
        assert state["active"] is False
        assert parks == ["gps_route_completed"]

    def test_waypoint_aborted_stops_whole_route(self):
        controller, port, clock, parks, _ = self._accepted(waypoint_count=3)
        port.fail("aborted")
        self._refresh_fix(controller, clock)
        controller.tick()
        state = controller.get_state()
        assert state["status"] == "aborted"
        assert state["reason"] == "waypoint_aborted"
        assert len(port.submissions) == 1  # 后续航点绝不再提交
        assert parks == ["waypoint_aborted"]

    def test_gps_staleness_mid_route_cancels_and_parks(self):
        controller, port, clock, parks, _ = self._accepted()
        clock.advance(3.0)  # 超过 max_age 2.0s, 且不续新定位
        controller.tick()
        state = controller.get_state()
        assert state["status"] == "aborted"
        assert state["reason"] == "gps_stale_fix"
        assert port.cancels == ["gps_stale_fix"]
        assert parks == ["gps_stale_fix"]

    def test_gps_quality_loss_mid_route_parks(self):
        controller, port, clock, parks, _ = self._accepted()
        controller.update_fix(fresh_fix(clock, fix_status=-1))
        controller.tick()
        assert controller.get_state()["reason"] == "gps_no_fix"
        assert parks == ["gps_no_fix"]

    def test_waypoint_ownership_lost_neither_cancels_nor_parks(self):
        # 航点被其它生产者替换 (面板点选): 停自己, 不抢别人的目标/底盘。
        controller, port, clock, parks, _ = self._accepted()
        port.state["generation"] += 5  # 外部提交推进了 generation
        self._refresh_fix(controller, clock)
        controller.tick()
        state = controller.get_state()
        assert state["status"] == "aborted"
        assert state["reason"] == "waypoint_ownership_lost"
        assert port.cancels == []
        assert parks == []

    def test_waypoint_timeout_cancels_and_parks(self):
        controller, port, clock, parks, _ = self._accepted(arrive_timeout=30.0)
        self._refresh_fix(controller, clock)
        clock.advance(31.0)
        controller.update_fix(fresh_fix(clock))
        controller.tick()
        state = controller.get_state()
        assert state["reason"] == "waypoint_timeout"
        assert port.cancels == ["waypoint_timeout"]
        assert parks == ["waypoint_timeout"]

    def test_operator_cancel_cancels_port_without_park(self):
        controller, port, clock, parks, _ = self._accepted()
        result = controller.cancel("operator_stop")
        assert result["ok"]
        assert port.cancels == ["operator_stop"]
        assert parks == []  # 操作员接管路径不 park (由 arbiter 停车路径负责)
        assert controller.get_state()["status"] == "canceled"

    def test_operator_cancel_when_idle_is_noop(self):
        controller = make_controller(FakePointPort(), FakeClock())
        assert controller.cancel()["ignored"] is True

    def test_submit_exception_aborts_and_parks(self):
        # 受理后 submit 抛异常 (如底层控制器被隔离)
        port = FakePointPort()
        port.submit_raises = RuntimeError("controller quarantined")
        clock = FakeClock()
        parks = []
        controller = make_controller(port, clock, parks=parks)
        controller.update_fix(fresh_fix(clock))
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["ok"] is False
        assert result["reason"] == "waypoint_submit_error"
        assert parks == ["waypoint_submit_error"]
        assert controller.get_state()["status"] == "aborted"

    def test_submit_rejection_aborts_and_parks(self):
        port = FakePointPort()
        port.submit_result = {"ok": False, "reason": "planner_failed"}
        clock = FakeClock()
        parks = []
        controller = make_controller(port, clock, parks=parks)
        controller.update_fix(fresh_fix(clock))
        result = controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}], map_pose=anchor_pose())
        assert result["ok"] is False
        assert parks == ["planner_failed"]
        assert controller.get_state()["status"] == "aborted"

    def test_port_state_read_failure_aborts_and_parks(self):
        controller, port, clock, parks, _ = self._accepted()
        port.get_state_raises = True
        controller.update_fix(fresh_fix(clock))
        controller.tick()
        assert controller.get_state()["reason"] == "point_port_unavailable"
        assert parks == ["point_port_unavailable"]

    def test_state_callback_receives_transitions(self):
        controller, port, clock, _, states = self._accepted()
        statuses = [s["status"] for s in states]
        assert "navigating" in statuses
        port.succeed()
        self._refresh_fix(controller, clock)
        controller.tick()
        statuses = [s["status"] for s in states]
        assert "completed" in statuses

    def test_state_callback_exception_does_not_break_safety(self):
        # 坏的 WS 消费者不允许影响状态机 (对齐 point_nav 约定)。
        port = FakePointPort()
        clock = FakeClock()

        def broken_callback(_state):
            raise RuntimeError("ws consumer gone")

        controller = GpsRouteController(
            port, state_callback=broken_callback, monotonic=clock)
        controller.set_heading_calibration(0.0)
        controller.update_fix(fresh_fix(clock))
        assert controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}],
            map_pose=anchor_pose())["ok"]
        assert port.submissions  # 主路径未被打断

    def test_park_hook_exception_does_not_break_abort(self):
        port = FakePointPort()
        clock = FakeClock()

        def broken_park(_reason):
            raise RuntimeError("arbiter gone")

        controller = GpsRouteController(
            port, park_hook=broken_park, monotonic=clock)
        controller.set_heading_calibration(0.0)
        controller.update_fix(fresh_fix(clock))
        assert controller.submit_route(
            [{"lat": 30.0, "lon": 120.001}],
            map_pose=anchor_pose())["ok"]
        clock.advance(3.0)  # GPS 过期触发 abort (含 park 调用)
        state = controller.tick()
        assert state["status"] == "aborted"

    def test_wait_until_settled_returns_terminal_state(self):
        controller, port, clock, _, _ = self._accepted()
        port.succeed()
        # wait_until_settled 内部会 tick, GPS 仍新鲜
        state = controller.wait_until_settled(timeout=1.0)
        assert state["status"] == "completed"

    def test_tick_without_route_is_idle_noop(self):
        controller = make_controller(FakePointPort(), FakeClock())
        assert controller.tick()["status"] == "idle"


# ---------------------------------------------------------------------------
# 文件契约 (分层铁律)
# ---------------------------------------------------------------------------

class TestFileContracts:
    CORE = WEB / "nx_gps_nav.py"
    SHELL = WEB / "nx_gps_nav_ros.py"

    def test_core_has_no_rclpy_import(self):
        # 纯逻辑核心零 rclpy —— 无 ROS 机器上必须可 import 可测。
        source = self.CORE.read_text(encoding="utf-8")
        assert "import rclpy" not in source
        assert "from rclpy" not in source

    def test_shell_imports_core_not_reimplements(self):
        # 壳必须复用核心 (依赖方向: 壳 → 核心), 且自己 import rclpy。
        shell = self.SHELL.read_text(encoding="utf-8")
        assert "import rclpy" in shell
        assert "from nx_gps_nav import" in shell

    def test_core_declares_fail_closed_stop_conditions(self):
        # 停车条件必须成文在核心模块头注释里 (审计锚点)。
        source = self.CORE.read_text(encoding="utf-8")
        assert "停车条件" in source
        assert "heading_not_calibrated" in source

    def test_shell_freshness_uses_monotonic_receipt_time(self):
        # 新鲜度以"到达本机时刻"为基准, 壳层必须用 time.monotonic()。
        shell = self.SHELL.read_text(encoding="utf-8")
        assert "time.monotonic()" in shell
