"""WaterGuardClient (motion_safety M3) 单测: 否决/限速/过龄 fail-closed。

对齐 test_motion_safety.py 的纯逻辑风格, 零 ROS 依赖。
"""
from __future__ import annotations

from motion_safety import WaterGuardClient


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def test_no_status_passthrough():
    client = WaterGuardClient(clock=FakeClock())
    assert client.filter_nav_velocity((1.0, 0.2, 0.3)) == (1.0, 0.2, 0.3)
    assert client.guard_reason() is None


def test_veto_zeroes_velocity():
    clock = FakeClock()
    client = WaterGuardClient(clock=clock)
    client.observe_status({"armed": True, "verdict": "veto", "speed_cap": 0.0})
    assert client.filter_nav_velocity((1.0, 0.2, 0.3)) == (0.0, 0.0, 0.0)
    assert client.guard_reason() == "water_guard_veto"


def test_limit_scales_linear_only():
    clock = FakeClock()
    client = WaterGuardClient(clock=clock)
    client.observe_status({"armed": True, "verdict": "limit",
                           "speed_cap": 0.3})
    result = client.filter_nav_velocity((1.0, 0.0, 0.4))
    assert result[0] == 0.3 and result[1] == 0.0 and result[2] == 0.4
    assert client.guard_reason() == "water_guard_limit"
    # 已低于 cap → 原样
    assert client.filter_nav_velocity((0.1, 0.0, 0.4)) == (0.1, 0.0, 0.4)


def test_stale_and_armed_fails_closed():
    clock = FakeClock()
    client = WaterGuardClient(stale_after=1.5, clock=clock)
    client.observe_status({"armed": True, "verdict": "allow",
                           "speed_cap": None})
    clock.now += 10.0  # 心跳过龄
    assert client.filter_nav_velocity((1.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert client.guard_reason() == "water_guard_veto"


def test_stale_but_disarmed_still_allows():
    clock = FakeClock()
    client = WaterGuardClient(stale_after=1.5, clock=clock)
    client.observe_status({"armed": False, "verdict": None, "speed_cap": None})
    clock.now += 10.0
    assert client.filter_nav_velocity((1.0, 0.0, 0.0)) == (1.0, 0.0, 0.0)


def test_disarm_restores_passthrough():
    clock = FakeClock()
    client = WaterGuardClient(clock=clock)
    client.observe_status({"armed": True, "verdict": "veto", "speed_cap": 0.0})
    assert client.filter_nav_velocity((1.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    client.observe_status({"armed": False, "verdict": None, "speed_cap": None})
    assert client.filter_nav_velocity((1.0, 0.0, 0.0)) == (1.0, 0.0, 0.0)
    assert client.guard_reason() is None


def test_malformed_status_ignored():
    clock = FakeClock()
    client = WaterGuardClient(clock=clock)
    client.observe_status("garbage")  # type: ignore[arg-type]
    assert client.filter_nav_velocity((1.0, 0.0, 0.0)) == (1.0, 0.0, 0.0)


def test_snapshot_shape():
    clock = FakeClock()
    client = WaterGuardClient(clock=clock)
    client.observe_status({"armed": True, "verdict": "allow"})
    snap = client.snapshot()
    assert snap["last_status"]["armed"] is True
    assert snap["status_age_s"] == 0.0
