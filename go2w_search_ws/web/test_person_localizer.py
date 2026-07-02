import math
import sys
from pathlib import Path

import pytest


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_person_localizer import (  # noqa: E402
    DetectionFrame,
    LaserScanSnapshot,
    localize_person_detection,
    range_at_bearing,
)


def test_center_bbox_uses_forward_lidar_range():
    det = {"class": "person", "confidence": 0.9, "bbox": [590, 100, 690, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[180] = 2.5

    result = localize_person_detection(det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0)

    assert result is not det
    assert result["position_quality"] == "range_lidar"
    assert result["range_source"] == "lidar"
    assert result["bearing_base"] == pytest.approx(0.0)
    assert result["bearing_map"] == pytest.approx(0.0)
    assert result["range_m"] == pytest.approx(2.5)
    assert result["world_x"] == pytest.approx(3.5)
    assert result["world_y"] == pytest.approx(2.0)


def test_invalid_scan_returns_bearing_only():
    det = {"class": "person", "confidence": 0.9, "bbox": [590, 100, 690, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)

    result = localize_person_detection(det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0)

    assert result["position_quality"] == "bearing_only"
    assert result["range_source"] == "unresolved"
    assert result["range_m"] is None
    assert result["world_x"] is None
    assert result["world_y"] is None


def test_bad_bbox_returns_bearing_only_without_map_position():
    det = {"class": "person", "confidence": 0.9, "bbox": [590, "bad", 690, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[180] = 2.5

    result = localize_person_detection(det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0)

    assert result["position_quality"] == "bearing_only"
    assert result["range_source"] == "unresolved"
    assert result["bearing_base"] is None
    assert result["bearing_map"] is None
    assert result["world_x"] is None
    assert result["world_y"] is None


def test_right_side_bbox_rotates_with_robot_yaw_and_uses_matching_scan_index():
    det = {"class": "person", "confidence": 0.8, "bbox": [960, 100, 1060, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[200] = 3.0

    result = localize_person_detection(det, frame, scan, robot_x=0.0, robot_y=0.0, robot_yaw=math.pi / 2)

    expected_base = math.radians(20.234375)
    expected_map = math.pi / 2 + expected_base
    assert result["position_quality"] == "range_lidar"
    assert result["bearing_base"] == pytest.approx(expected_base)
    assert result["bearing_map"] == pytest.approx(expected_map)
    assert result["range_m"] == pytest.approx(3.0)
    assert result["world_x"] == pytest.approx(3.0 * math.cos(expected_map))
    assert result["world_y"] == pytest.approx(3.0 * math.sin(expected_map))


def test_range_at_bearing_uses_median_valid_ranges_and_wraparound_angle_diff():
    scan = LaserScanSnapshot(
        angle_min=math.radians(170.0),
        angle_increment=math.radians(5.0),
        ranges=[1.0, float("nan"), 2.0, 2.4, 15.0],
        range_min=0.5,
        range_max=10.0,
    )

    result = range_at_bearing(scan, math.radians(-180.0), window_rad=math.radians(10.0))

    assert result == pytest.approx(2.0)
