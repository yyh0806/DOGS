import math
import types


class Clock:
    def __init__(self, value=100.0):
        self.value = value

    def __call__(self):
        return self.value


def scan(stamp=(10, 20), ranges=None):
    values = ([5.0] * 16 + [float("inf")] * 344) if ranges is None else ranges
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            frame_id="base_link",
            stamp=types.SimpleNamespace(sec=stamp[0], nanosec=stamp[1]),
        ),
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        range_min=0.15,
        range_max=10.0,
        ranges=values,
    )


def test_scan_watchdog_fails_closed_and_accepts_only_new_legal_scans():
    from go2w_bridge.motion_safety import ScanFreshnessWatchdog

    clock = Clock()
    watchdog = ScanFreshnessWatchdog(timeout=0.3, clock=clock)
    assert watchdog.filter_nav_velocity((0.2, 0.0, 0.0)) == watchdog.ZERO
    assert watchdog.observe_scan(scan()) is True
    assert watchdog.observe_scan(scan()) is False
    assert watchdog.filter_nav_velocity((0.2, 0.0, 0.0)) == (0.2, 0.0, 0.0)
    clock.value += 0.31
    assert watchdog.nav_must_stop() is True


def test_drive_watchdog_requires_measured_wheel_response():
    from go2w_bridge.motion_safety import DriveExecutionWatchdog

    clock = Clock()
    watchdog = DriveExecutionWatchdog(
        timeout=0.2, min_wheel_speed=0.1, clock=clock)
    assert watchdog.observe_feedback(
        [0.0] * 4, 80, 0, sport_mode=1, sport_progress=0.0) is True
    assert watchdog.evaluate((0.2, 0.0, 0.0)) is None
    clock.value += 0.21
    assert watchdog.observe_feedback(
        [0.0] * 4, 80, 0, sport_mode=1, sport_progress=0.0) is True
    assert watchdog.evaluate((0.2, 0.0, 0.0)) == "wheel_no_response"


def test_drive_watchdog_uses_longer_wheel_response_grace_than_feedback_timeout():
    from go2w_bridge.motion_safety import DriveExecutionWatchdog

    clock = Clock()
    watchdog = DriveExecutionWatchdog(
        timeout=0.2,
        response_grace=0.7,
        min_wheel_speed=0.1,
        clock=clock,
    )
    assert watchdog.observe_feedback(
        [0.0] * 4, 80, 0, sport_mode=1, sport_progress=0.0) is True
    assert watchdog.evaluate((0.0, 0.0, 0.15)) is None

    clock.value += 0.64
    assert watchdog.observe_feedback(
        [0.0] * 4, 80, 0, sport_mode=1, sport_progress=0.0) is True
    assert watchdog.evaluate((0.0, 0.0, 0.15)) is None

    clock.value += 0.07
    assert watchdog.observe_feedback(
        [0.0] * 4, 80, 0, sport_mode=1, sport_progress=0.0) is True
    assert watchdog.evaluate((0.0, 0.0, 0.15)) == "wheel_no_response"


def test_drive_watchdog_stale_feedback_keeps_short_confirmation_timeout():
    from go2w_bridge.motion_safety import DriveExecutionWatchdog

    clock = Clock()
    watchdog = DriveExecutionWatchdog(
        timeout=0.2,
        response_grace=0.7,
        min_wheel_speed=0.1,
        clock=clock,
    )
    assert watchdog.observe_feedback(
        [0.0] * 4, 80, 0, sport_mode=1, sport_progress=0.0) is True
    assert watchdog.evaluate((0.2, 0.0, 0.0)) is None

    clock.value += 0.21
    assert watchdog.evaluate((0.2, 0.0, 0.0)) is None
    clock.value += 0.21
    assert watchdog.evaluate((0.2, 0.0, 0.0)) == "wheel_feedback_stale"


def test_drive_watchdog_rejects_non_wheel_gait_before_velocity_continues():
    from go2w_bridge.motion_safety import DriveExecutionWatchdog

    clock = Clock()
    watchdog = DriveExecutionWatchdog(
        timeout=0.2, min_wheel_speed=0.1, clock=clock)
    assert watchdog.observe_feedback(
        [2.0, -2.0, 2.0, -2.0],
        80,
        0,
        sport_mode=3,
        sport_progress=0.0,
        gait_type=1,
    ) is True

    assert watchdog.evaluate((0.2, 0.0, 0.0)) == "unexpected_gait"


def test_velocity_helpers_fail_closed_without_creating_reverse_motion():
    from go2w_bridge.motion_safety import (
        compensate_pure_turn_creep,
        motion_command_timed_out,
    )

    assert compensate_pure_turn_creep(
        (0.0, 0.0, 0.15), gain=1.0, maximum=0.15,
        linear_epsilon=0.02, angular_threshold=0.05,
    ) == (0.0, 0.0, 0.15)
    assert motion_command_timed_out(
        True, "nav", 10.0, 10.31, 0.5, 0.3) is True
