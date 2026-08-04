"""Fail-closed contracts for the Go2-W wheel/IMU odometry source."""

import ast
import math
from pathlib import Path

import pytest


SENSOR = (
    Path(__file__).resolve().parents[1]
    / "src/go2w_bridge/go2w_bridge/nx_sensor_node.py"
)


def _load_helper():
    tree = ast.parse(SENSOR.read_text(encoding="utf-8"))
    definition = next(
        item for item in tree.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "_select_sport_odom_velocity"
    )
    namespace = {"math": math}
    exec(
        compile(ast.Module(body=[definition], type_ignores=[]), str(SENSOR), "exec"),
        namespace,
    )
    return namespace["_select_sport_odom_velocity"]


@pytest.mark.parametrize("mode", [None, 0, 2, 6, 7, 8, 255])
def test_non_wheel_modes_never_integrate_body_translation(mode):
    select = _load_helper()

    assert select([0.4, -0.2, 0.0], mode, 0, 0.01, 0.25, 1.5) == (
        0.0,
        0.0,
        "mode_locked",
    )


def test_fresh_sdk_body_velocity_is_used_in_balance_or_locomotion():
    select = _load_helper()

    vx, vy, source = select([0.35, -0.08, 0.0], 1, 0, 0.02, 0.25, 1.5)
    assert (vx, vy) == pytest.approx((0.35, -0.08))
    assert source == "sport_velocity"
    vx, vy, source = select([-0.2, 0.04, 0.0], 3, 0, 0.02, 0.25, 1.5)
    assert (vx, vy) == pytest.approx((-0.2, 0.04))
    assert source == "sport_velocity"


def test_nonzero_sdk_sport_error_fails_closed():
    select = _load_helper()

    assert select([0.2, 0.0, 0.0], 3, 4, 0.01, 0.25, 1.5) == (
        0.0,
        0.0,
        "sport_error",
    )


@pytest.mark.parametrize(
    ("velocity", "age", "reason"),
    [
        ([0.2, 0.0, 0.0], 0.26, "sport_stale"),
        ([0.2, 0.0, 0.0], -0.01, "sport_stale"),
        ([float("nan"), 0.0, 0.0], 0.01, "sport_invalid"),
        ([float("inf"), 0.0, 0.0], 0.01, "sport_invalid"),
        ([0.2], 0.01, "sport_invalid"),
        ([1.51, 0.0, 0.0], 0.01, "sport_implausible"),
        ([1.1, 1.1, 0.0], 0.01, "sport_implausible"),
    ],
)
def test_stale_invalid_or_implausible_sport_velocity_fails_closed(
        velocity, age, reason):
    select = _load_helper()

    assert select(velocity, 3, 0, age, 0.25, 1.5) == (0.0, 0.0, reason)


def test_odometry_no_longer_integrates_raw_motor_speed_as_body_motion():
    source = SENSOR.read_text(encoding="utf-8")
    tree = ast.parse(source)
    publish = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef) and item.name == "_publish_odom_imu"
    )
    publish_source = ast.unparse(publish)

    assert "sum(wheel_dq)" not in publish_source
    assert "_select_sport_odom_velocity" in publish_source
    assert "time.monotonic()" in publish_source


def test_drive_feedback_exposes_sdk_motion_estimate_for_live_diagnostics():
    source = SENSOR.read_text(encoding="utf-8")

    for field in (
        "sport_velocity",
        "sport_position",
        "sport_yaw_speed",
        "odom_velocity_source",
    ):
        assert repr(field) in source
    assert "sport_error_code=sport_error_code" in source
    assert "build_wheel_feedback_payload(" in source
