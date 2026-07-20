"""Pure-function tests for nx_move_executor (no ROS, no hardware)."""

import math
import sys
from pathlib import Path

import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_move_executor import (
    compute_linear_target, compute_angular_target_yaw, yaw_error,
    angular_turn_complete, run_angular_turn,
)


def test_linear_forward_adds_distance_along_yaw():
    tx, ty, tyaw = compute_linear_target(1.0, 0.0, 0.0, "forward", 2.0)
    assert (tx, ty, tyaw) == pytest.approx((3.0, 0.0, 0.0))


def test_linear_backward_subtracts_distance():
    tx, ty, tyaw = compute_linear_target(3.0, 4.0, 0.0, "backward", 1.0)
    assert (tx, ty, tyaw) == pytest.approx((2.0, 4.0, 0.0))


def test_linear_forward_45deg():
    yaw = math.radians(45)
    tx, ty, _ = compute_linear_target(0.0, 0.0, yaw, "forward", math.sqrt(2))
    assert (tx, ty) == pytest.approx((1.0, 1.0), abs=1e-6)


def test_angular_left_adds_angle():
    assert compute_angular_target_yaw(0.0, "left", 90.0) == pytest.approx(math.radians(90))


def test_angular_right_subtracts_angle():
    assert compute_angular_target_yaw(math.radians(10), "right", 45.0) == pytest.approx(math.radians(-35))


def test_yaw_error_wraps_to_neg_pi_pi():
    assert yaw_error(0.0, math.radians(350)) == pytest.approx(math.radians(-10), abs=1e-6)
    assert yaw_error(0.0, math.radians(10)) == pytest.approx(math.radians(10), abs=1e-6)


def test_angular_turn_complete_within_tolerance():
    assert angular_turn_complete(0.0, math.radians(2), math.radians(3)) is True
    assert angular_turn_complete(0.0, math.radians(10), math.radians(3)) is False


def test_run_angular_turn_succeeds_when_yaw_reaches_target():
    target = math.radians(90)
    yaw_readings = [0.0, math.radians(30), math.radians(60), math.radians(89)] + [target] * 20
    idx = {"i": 0}
    read_yaw = lambda: yaw_readings[min(idx["i"], len(yaw_readings) - 1)]
    sent = []
    send_cmd = lambda vx, vy, vyaw: sent.append((vx, vy, vyaw))

    def sleep(_s):
        idx["i"] += 1

    result = run_angular_turn(read_yaw, send_cmd, sleep, lambda: 0.0,
                              target, "left", vyaw=0.5,
                              tolerance_rad=math.radians(3), max_duration=10.0)
    assert result == "succeeded"
    assert sent[0][2] == 0.5           # 起步: 左转正 vyaw
    assert sent[-1] == (0.0, 0.0, 0.0)  # 结束: 停


def test_run_angular_turn_times_out_when_yaw_never_reaches():
    read_yaw = lambda: 0.0  # 永不动
    sent = []
    send_cmd = lambda vx, vy, vyaw: sent.append((vx, vy, vyaw))
    clock = iter(range(0, 100, 2))
    monotonic = lambda: next(clock)
    result = run_angular_turn(read_yaw, send_cmd, lambda _s: None, monotonic,
                              math.radians(90), "right", vyaw=0.5,
                              tolerance_rad=math.radians(3), max_duration=1.0)
    assert result == "timed_out"
    assert sent[-1] == (0.0, 0.0, 0.0)  # 超时也停
