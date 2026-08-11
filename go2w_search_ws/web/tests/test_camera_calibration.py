import math
import sys
from pathlib import Path

import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_camera_calibration import resolve_camera_calibration


def test_c13_visible_uses_documented_nominal_hfov(monkeypatch):
    for name in (
        "GO2W_CAMERA_HFOV_C13_VIS_DEG",
        "GO2W_CAMERA_HFOV_DEG",
        "GO2W_CAMERA_HFOV",
        "GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG",
        "GO2W_CAMERA_YAW_OFFSET_DEG",
        "GO2W_CAMERA_CALIBRATION_C13_VIS",
    ):
        monkeypatch.delenv(name, raising=False)

    calibration = resolve_camera_calibration("c13_vis")

    assert calibration["hfov_deg"] == pytest.approx(77.4)
    assert calibration["yaw_offset_deg"] == pytest.approx(0.0)
    assert calibration["profile"] == "nominal_centered"
    assert calibration["measured"] is False
    assert calibration["gimbal_yaw_feedback_available"] is False


def test_source_specific_measured_calibration_and_gimbal_feedback(monkeypatch):
    monkeypatch.setenv("GO2W_CAMERA_HFOV_C13_VIS_DEG", "76.2")
    monkeypatch.setenv("GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG", "-3.5")
    monkeypatch.setenv("GO2W_CAMERA_CALIBRATION_C13_VIS", "measured")

    calibration = resolve_camera_calibration(
        "c13_vis", gimbal_yaw_rad=math.radians(8.0))

    assert calibration["hfov_deg"] == pytest.approx(76.2)
    assert calibration["yaw_offset_deg"] == pytest.approx(-3.5)
    assert calibration["gimbal_yaw_deg"] == pytest.approx(8.0)
    assert calibration["effective_yaw_offset_deg"] == pytest.approx(4.5)
    assert calibration["profile"] == "measured"
    assert calibration["measured"] is True
    assert calibration["gimbal_yaw_feedback_available"] is True


def test_invalid_environment_values_fall_back_safely(monkeypatch):
    monkeypatch.setenv("GO2W_CAMERA_HFOV_C13_VIS_DEG", "nan")
    monkeypatch.setenv("GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG", "inf")

    calibration = resolve_camera_calibration("c13_vis")

    assert calibration["hfov_deg"] == pytest.approx(77.4)
    assert calibration["yaw_offset_deg"] == pytest.approx(0.0)
