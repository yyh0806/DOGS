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
    angular_turn_complete, directional_clearance_from_scan, run_angular_turn,
    run_linear_translation, sanitize_clearance_margin,
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


def test_run_angular_turn_refreshes_velocity_heartbeat_until_complete():
    target = math.radians(90)
    yaw_readings = [0.0, math.radians(10), math.radians(25),
                    math.radians(50), math.radians(89)]
    index = {"value": 0}
    sent = []

    def read_yaw():
        return yaw_readings[min(index["value"], len(yaw_readings) - 1)]

    def sleep(_seconds):
        index["value"] += 1

    result = run_angular_turn(
        read_yaw,
        lambda vx, vy, vyaw: sent.append((vx, vy, vyaw)),
        sleep,
        lambda: 0.0,
        target,
        "left",
        vyaw=0.5,
        tolerance_rad=math.radians(3),
        max_duration=10.0,
    )

    assert result == "succeeded"
    nonzero = [command for command in sent if command != (0.0, 0.0, 0.0)]
    assert len(nonzero) >= 3
    assert all(command == (0.0, 0.0, 0.5) for command in nonzero)
    assert sent[-1] == (0.0, 0.0, 0.0)


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


class LinearHarness:
    def __init__(self, positions, clearances=(2.0,)):
        self.positions = list(positions)
        self.clearances = list(clearances)
        self.position_index = 0
        self.clearance_index = 0
        self.now = 0.0
        self.sent = []

    def read_xy(self):
        return self.positions[min(self.position_index, len(self.positions) - 1)]

    def read_clearance(self, direction):
        assert direction in {"forward", "backward"}
        value = self.clearances[min(self.clearance_index,
                                    len(self.clearances) - 1)]
        self.clearance_index += 1
        return value

    def send_cmd(self, vx, vy, vyaw):
        self.sent.append((vx, vy, vyaw))

    def sleep(self, seconds):
        self.now += seconds
        self.position_index += 1

    def monotonic(self):
        return self.now

    def run(self, **kwargs):
        return run_linear_translation(
            self.read_xy,
            self.read_clearance,
            self.send_cmd,
            self.sleep,
            self.monotonic,
            start_xy=(0.0, 0.0),
            direction="backward",
            distance_m=0.6,
            **kwargs,
        )


def test_reverse_translation_succeeds_at_requested_displacement_and_slows():
    harness = LinearHarness([(0.0, 0.0), (-0.20, 0.0),
                             (-0.45, 0.0), (-0.58, 0.0)])

    result = harness.run(max_duration=5.0)

    assert result == "succeeded"
    moving = [cmd for cmd in harness.sent if cmd != (0.0, 0.0, 0.0)]
    assert moving
    assert all(-0.3 <= vx < 0.0 and vy == vyaw == 0.0
               for vx, vy, vyaw in moving)
    assert abs(moving[-1][0]) < abs(moving[0][0])
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_forward_translation_uses_the_same_displacement_math():
    harness = LinearHarness([(0.0, 0.0), (0.30, 0.0), (0.58, 0.0)])

    result = run_linear_translation(
        harness.read_xy, harness.read_clearance, harness.send_cmd,
        harness.sleep, harness.monotonic, (0.0, 0.0), "forward", 0.6,
        max_duration=5.0,
    )

    assert result == "succeeded"
    assert harness.sent[0][0] > 0.0
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_does_not_count_pure_lateral_drift_as_progress():
    harness = LinearHarness([(0.0, 0.0), (0.0, 0.10), (0.0, 0.20)])

    assert harness.run(max_duration=5.0) == "localization_lost"
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_rejects_excess_accumulated_arc_path():
    harness = LinearHarness([
        (0.0, 0.0), (-0.12, 0.10), (-0.24, -0.10),
        (-0.36, 0.10), (-0.48, -0.10),
    ])

    assert harness.run(max_duration=5.0) == "localization_lost"
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_rejects_motion_in_wrong_direction():
    harness = LinearHarness([(0.0, 0.0), (0.05, 0.0)])

    assert harness.run(max_duration=5.0) == "localization_lost"
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_rejects_implausible_localization_jump():
    harness = LinearHarness([(0.0, 0.0), (-0.55, 0.0)])

    assert harness.run(max_duration=5.0) == "localization_lost"
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_projects_progress_on_start_yaw_axis():
    harness = LinearHarness([
        (1.0, 2.0), (1.0, 1.75), (1.0, 1.50), (1.0, 1.42),
    ])

    result = run_linear_translation(
        harness.read_xy, harness.read_clearance, harness.send_cmd,
        harness.sleep, harness.monotonic, (1.0, 2.0), "backward", 0.6,
        start_yaw=math.pi / 2.0, max_duration=5.0,
    )

    assert result == "succeeded"
    assert harness.sent[0][0] < 0.0
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_rejects_initial_rear_obstacle_without_motion():
    harness = LinearHarness([(0.0, 0.0)], clearances=(0.55,))

    assert harness.run(max_duration=5.0) == "obstacle"
    assert harness.sent == [(0.0, 0.0, 0.0)]


def test_reverse_translation_stops_when_obstacle_appears_mid_run():
    harness = LinearHarness([(0.0, 0.0), (-0.1, 0.0)],
                            clearances=(2.0, 0.4))

    assert harness.run(max_duration=5.0) == "obstacle"
    assert harness.sent[0][0] < 0.0
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("clearance", [None, math.nan, math.inf, -math.inf])
def test_reverse_translation_fails_closed_on_missing_or_nonfinite_clearance(clearance):
    harness = LinearHarness([(0.0, 0.0)], clearances=(clearance,))

    assert harness.run(max_duration=5.0) == "obstacle"
    assert harness.sent == [(0.0, 0.0, 0.0)]


def test_reverse_translation_reports_localization_loss_after_start():
    harness = LinearHarness([(0.0, 0.0), None])

    assert harness.run(max_duration=5.0) == "localization_lost"
    assert harness.sent[0][0] < 0.0
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_times_out_and_stops():
    harness = LinearHarness([(0.0, 0.0)])

    assert harness.run(max_duration=0.1) == "timed_out"
    assert harness.sent[-1] == (0.0, 0.0, 0.0)


def test_reverse_translation_honors_cancellation_before_motion():
    harness = LinearHarness([(0.0, 0.0)])

    assert harness.run(max_duration=5.0, is_cancelled=lambda: True) == "cancelled"
    assert harness.sent == [(0.0, 0.0, 0.0)]


def test_reverse_translation_callback_exception_fails_closed_and_stops():
    harness = LinearHarness([(0.0, 0.0)])

    def broken_clearance(_direction):
        raise RuntimeError("scan unavailable")

    result = run_linear_translation(
        harness.read_xy, broken_clearance, harness.send_cmd, harness.sleep,
        harness.monotonic, (0.0, 0.0), "backward", 0.6,
        max_duration=5.0,
    )

    assert result == "obstacle"
    assert harness.sent == [(0.0, 0.0, 0.0)]


@pytest.mark.parametrize("max_duration", [0.0, -1.0, math.inf, math.nan])
def test_reverse_translation_rejects_invalid_max_duration(max_duration):
    harness = LinearHarness([(0.0, 0.0)])

    with pytest.raises(ValueError, match="max_duration"):
        harness.run(max_duration=max_duration)

    assert harness.sent == []


def test_directional_clearance_selects_front_and_rear_by_angle():
    ranges = [0.8, 4.0, 1.7, 3.0]
    args = (-math.pi, math.pi / 2.0, 0.05, 10.0, ranges)

    assert directional_clearance_from_scan(*args, center_deg=0.0,
                                           half_fov_deg=20.0) == 1.7
    assert directional_clearance_from_scan(*args, center_deg=180.0,
                                           half_fov_deg=20.0) == 0.8


def test_directional_clearance_wraps_at_pi():
    result = directional_clearance_from_scan(
        math.radians(170.0), math.radians(10.0), 0.05, 10.0,
        [2.0, 1.0, 0.4], center_deg=-180.0, half_fov_deg=15.0,
    )
    assert result == 0.4


def test_directional_clearance_supports_negative_angle_increment():
    result = directional_clearance_from_scan(
        math.pi, -math.pi / 2.0, 0.05, 10.0,
        [0.8, 4.0, 1.7, 3.0], center_deg=0.0, half_fov_deg=20.0,
    )
    assert result == 1.7


def test_directional_clearance_ignores_invalid_sensor_ranges():
    result = directional_clearance_from_scan(
        -0.2, 0.1, 0.1, 5.0,
        [math.nan, 0.05, 2.0, 8.0, math.inf],
        center_deg=0.0, half_fov_deg=30.0,
    )
    assert result == 2.0


def test_directional_clearance_returns_none_for_partial_scan_without_direction():
    result = directional_clearance_from_scan(
        math.radians(-45.0), math.radians(10.0), 0.05, 10.0,
        [1.0] * 5, center_deg=180.0, half_fov_deg=20.0,
    )
    assert result is None


@pytest.mark.parametrize("increment", [0.0, math.nan, math.inf])
def test_directional_clearance_returns_none_for_invalid_metadata(increment):
    assert directional_clearance_from_scan(
        0.0, increment, 0.05, 10.0, [1.0], center_deg=0.0,
    ) is None


@pytest.mark.parametrize("configured", [0.54, 0.55])
def test_reverse_clearance_margin_never_drops_below_hard_minimum(configured):
    assert sanitize_clearance_margin(configured) == 0.55


@pytest.mark.parametrize("configured", [None, "invalid", math.nan,
                                         math.inf, -math.inf])
def test_reverse_clearance_margin_invalid_values_use_hard_minimum(configured):
    assert sanitize_clearance_margin(configured) == 0.55


def test_reverse_clearance_margin_preserves_larger_configured_value():
    assert sanitize_clearance_margin(0.72) == 0.72
