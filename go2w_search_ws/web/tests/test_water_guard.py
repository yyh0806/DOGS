"""nx_water_guard 纯逻辑测试: 三档裁决 / 有符号距离 / 审批解除 / fail-closed。"""
from __future__ import annotations

import math

import pytest

from nx_water_guard import WaterGuard, new_approval_token

# 太科园附近的近似方块环 (~200m 见方), 中心 (31.4900, 120.3700)
_RING = [
    (31.4910, 120.3690),
    (31.4910, 120.3710),
    (31.4890, 120.3710),
    (31.4890, 120.3690),
]
_CENTER = (31.4900, 120.3700)
_OUTSIDE_FAR = (31.4940, 120.3700)   # ~445m 外
_OUTSIDE_NEAR = (31.491004, 120.3700)  # 环外 ~0.4m


def _guard(**kwargs) -> WaterGuard:
    return WaterGuard(**kwargs)


def test_not_armed_evaluates_not_armed():
    guard = _guard()
    result = guard.evaluate(*_CENTER)
    assert result["verdict"] == "not_armed"
    assert guard.state()["armed"] is False


def test_arm_rejects_invalid_rings():
    guard = _guard()
    assert not guard.arm([[0, 0]] * 3)["ok"]           # 点太少
    assert not guard.arm([(95.0, 0.0)] * 4)["ok"]      # 坐标非法
    assert not guard.arm("nope")["ok"]                 # type: ignore[arg-type]
    assert guard.state()["armed"] is False


def test_inside_is_veto_with_negative_distance():
    guard = _guard()
    guard.arm(_RING)
    result = guard.evaluate(*_CENTER)
    assert result["verdict"] == "veto"
    assert result["min_dist_m"] < 0          # 环内为负 (进入深度)
    assert result["speed_cap"] == 0.0
    assert guard.state()["violation_count"] == 1


def test_outside_far_allows():
    guard = _guard()
    guard.arm(_RING)
    result = guard.evaluate(*_OUTSIDE_FAR)
    assert result["verdict"] == "allow"
    assert result["speed_cap"] is None


def test_thresholds_limit_then_veto():
    guard = _guard(veto_m=5.0, limit_m=50.0)
    guard.arm(_RING)
    far = guard.evaluate(*_OUTSIDE_FAR)          # ~445m → allow
    assert far["verdict"] == "allow"
    # ~30m 外 (limit 带)
    near = guard.evaluate(31.49127, 120.3700)
    assert near["verdict"] == "limit"
    assert near["speed_cap"] == pytest.approx(0.4)
    assert guard.evaluate(*_OUTSIDE_NEAR)["verdict"] == "veto"


def test_margin_tightens():
    guard = _guard(veto_m=2.0, limit_m=8.0)
    guard.arm(_RING, margin_m=10.0)
    # 北缘外 ~12.2m: 12.2 - margin 10 = 2.2 → limit 带
    result = guard.evaluate(31.49112, 120.3700)
    assert result["verdict"] == "limit"
    # 北缘外 ~5.5m: 5.5 - 10 < 0 → veto (margin 内收后贴得更紧)
    assert guard.evaluate(31.49105, 120.3700)["verdict"] == "veto"


def test_stale_fix_fails_closed():
    guard = _guard()
    guard.arm(_RING)
    result = guard.evaluate(*_OUTSIDE_FAR, fix_age_s=2.0)
    assert result["verdict"] == "veto"
    assert result["reason"] == "fix_stale"


def test_disarm_requires_matching_approval_token():
    guard = _guard(approval_token="tok-42")
    guard.arm(_RING)
    assert guard.disarm("wrong")["reason"] == "approval_token_mismatch"
    assert guard.state()["armed"] is True
    assert guard.disarm("tok-42")["ok"] is True
    assert guard.state()["armed"] is False


def test_disarm_without_configured_token_fails_closed():
    guard = _guard(approval_token="")
    guard.arm(_RING)
    result = guard.disarm("anything")
    assert not result["ok"]
    assert result["reason"] == "approval_token_not_configured"


def test_distance_and_ring_copy():
    guard = _guard()
    assert guard.ring_copy() is None
    assert guard.distance_m(*_CENTER) is None
    guard.arm(_RING)
    assert len(guard.ring_copy()) == 4
    inside = guard.distance_m(*_CENTER)
    assert inside < 0
    outside = guard.distance_m(*_OUTSIDE_FAR)
    assert outside > 300  # ~332m (0.003° 纬度 × 110540 m/°)


def test_invalid_thresholds_rejected():
    with pytest.raises(ValueError):
        WaterGuard(veto_m=10.0, limit_m=5.0)


def test_new_approval_token_format():
    token = new_approval_token()
    assert len(token) == 12 and token.isalnum()
    assert new_approval_token() != token
