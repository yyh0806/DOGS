import math
import sys
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_person_localizer import (  # noqa: E402
    DetectionFrame,
    LaserScanSnapshot,
    PointCloudSnapshot,
    decode_pointcloud_xyz,
    height_at_bearing,
    localize_person_detection,
    localize_target_detection,
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


def test_generic_table_detection_uses_same_lidar_geometry_and_preserves_class():
    det = {
        "class": "dining table",
        "confidence": 0.88,
        "bbox": [590, 100, 690, 500],
    }
    frame = DetectionFrame(
        width=1280,
        height=720,
        camera_hfov_rad=math.radians(70.0),
    )
    scan = LaserScanSnapshot(
        angle_min=-math.pi,
        angle_increment=math.pi / 180.0,
        ranges=[0.0] * 360,
    )
    scan.ranges[180] = 2.0

    result = localize_target_detection(
        det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0)

    assert result["class"] == "dining table"
    assert result["position_quality"] == "range_lidar"
    assert result["range_m"] == pytest.approx(2.0)
    assert result["world_x"] == pytest.approx(3.0)
    assert result["world_y"] == pytest.approx(2.0)


def test_pointcloud_association_adds_evidence_backed_world_z():
    det = {"class": "person", "confidence": 0.9,
           "bbox": [590, 100, 690, 500]}
    frame = DetectionFrame(
        width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(
        angle_min=-math.pi,
        angle_increment=math.pi / 180.0,
        ranges=[0.0] * 360,
    )
    scan.ranges[180] = 2.5
    cloud = PointCloudSnapshot(points=[
        (2.48, -0.02, -0.2),
        (2.50, 0.00, 0.6),
        (2.52, 0.03, 1.2),
        (1.00, 0.00, 0.1),  # wrong range: must not influence height
        (2.50, 1.00, 1.4),  # wrong bearing: must not influence height
    ])

    result = localize_person_detection(
        det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0,
        robot_z=0.15, pointcloud=cloud)

    assert result["world_z"] == pytest.approx(0.75)
    assert result["position_dimension"] == 3
    assert result["height_source"] == "mid360_pointcloud"
    assert result["height_point_count"] == 3


def test_missing_pointcloud_match_keeps_explicit_2d_quality():
    det = {"class": "person", "confidence": 0.9,
           "bbox": [590, 100, 690, 500]}
    frame = DetectionFrame(
        width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(
        angle_min=-math.pi,
        angle_increment=math.pi / 180.0,
        ranges=[0.0] * 360,
    )
    scan.ranges[180] = 2.5
    cloud = PointCloudSnapshot(points=[(4.0, 0.0, 1.0)])

    result = localize_person_detection(
        det, frame, scan, robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
        pointcloud=cloud)

    assert result["world_z"] is None
    assert result["position_dimension"] == 2
    assert result["height_source"] == "unresolved"


def test_height_at_bearing_rejects_malformed_points():
    cloud = PointCloudSnapshot(points=[
        (2.0, 0.0, 0.4),
        (float("nan"), 0.0, 1.0),
        (2.0, "bad", 1.0),
        (2.0, 0.0),
    ])

    result = height_at_bearing(
        cloud, bearing_rad=0.0, range_m=2.0,
        window_rad=math.radians(5.0), range_tolerance_m=0.2)

    assert result == {"height_m": pytest.approx(0.4), "point_count": 1}


def test_decode_pointcloud_xyz_reads_float_fields_and_bounds_size():
    values = np.asarray([
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    ], dtype="<f4")
    message = SimpleNamespace(
        is_bigendian=False,
        point_step=12,
        width=3,
        height=1,
        data=values.tobytes(),
        fields=[
            SimpleNamespace(name="x", offset=0, datatype=7),
            SimpleNamespace(name="y", offset=4, datatype=7),
            SimpleNamespace(name="z", offset=8, datatype=7),
        ],
    )

    points = decode_pointcloud_xyz(message, max_points=2)

    assert points.shape == (2, 3)
    assert points[0].tolist() == pytest.approx([1.0, 2.0, 3.0])
    assert points[1].tolist() == pytest.approx([7.0, 8.0, 9.0])


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


def test_right_side_bbox_uses_negative_scan_angle_and_rotates_with_robot_yaw():
    det = {"class": "person", "confidence": 0.8, "bbox": [960, 100, 1060, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[160] = 3.0

    result_forward = localize_person_detection(det, frame, scan, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)

    expected_base = math.radians(-20.234375)
    assert result_forward["position_quality"] == "range_lidar"
    assert result_forward["bearing_base"] == pytest.approx(expected_base)
    assert result_forward["bearing_map"] == pytest.approx(expected_base)
    assert result_forward["range_m"] == pytest.approx(3.0)
    assert result_forward["world_x"] == pytest.approx(3.0 * math.cos(expected_base))
    assert result_forward["world_y"] == pytest.approx(3.0 * math.sin(expected_base))
    assert result_forward["world_y"] < 0.0

    result_rotated = localize_person_detection(det, frame, scan, robot_x=0.0, robot_y=0.0, robot_yaw=math.pi / 2)

    expected_map = math.pi / 2 + expected_base
    assert result_rotated["position_quality"] == "range_lidar"
    assert result_rotated["bearing_base"] == pytest.approx(expected_base)
    assert result_rotated["bearing_map"] == pytest.approx(expected_map)
    assert result_rotated["range_m"] == pytest.approx(3.0)
    assert result_rotated["world_x"] == pytest.approx(3.0 * math.cos(expected_map))
    assert result_rotated["world_y"] == pytest.approx(3.0 * math.sin(expected_map))


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


def test_range_at_bearing_rejects_malformed_scan_metadata():
    valid_scan = LaserScanSnapshot(
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        ranges=[2.0],
        range_min=0.1,
        range_max=10.0,
    )
    cases = [
        (LaserScanSnapshot(angle_min=float("nan"), angle_increment=math.pi / 180.0, ranges=[2.0]), 0.0, 0.1),
        (LaserScanSnapshot(angle_min="bad", angle_increment=math.pi / 180.0, ranges=[2.0]), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=float("nan"), ranges=[2.0]), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=0.0, ranges=[2.0]), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=-0.1, ranges=[2.0]), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=math.pi / 180.0, ranges=[2.0], range_min=0.0), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=math.pi / 180.0, ranges=[2.0], range_min=float("nan")), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=math.pi / 180.0, ranges=[2.0], range_max=float("inf")), 0.0, 0.1),
        (LaserScanSnapshot(angle_min=0.0, angle_increment=math.pi / 180.0, ranges=[2.0], range_min=5.0, range_max=1.0), 0.0, 0.1),
        (valid_scan, float("nan"), 0.1),
        (valid_scan, "bad", 0.1),
        (valid_scan, 0.0, float("nan")),
        (valid_scan, 0.0, 0.0),
        (valid_scan, 0.0, -0.1),
        (valid_scan, 0.0, "bad"),
    ]

    for scan, bearing, window in cases:
        assert range_at_bearing(scan, bearing, window_rad=window) is None


def test_localize_person_detection_rejects_malformed_frame_and_bbox_values():
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[180] = 2.5
    valid_frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    valid_bbox = [590, 100, 690, 500]
    cases = [
        (DetectionFrame(width="bad", height=720, camera_hfov_rad=math.radians(70.0)), valid_bbox),
        (DetectionFrame(width=float("nan"), height=720, camera_hfov_rad=math.radians(70.0)), valid_bbox),
        (DetectionFrame(width=1280, height="bad", camera_hfov_rad=math.radians(70.0)), valid_bbox),
        (DetectionFrame(width=1280, height=float("nan"), camera_hfov_rad=math.radians(70.0)), valid_bbox),
        (DetectionFrame(width=1280, height=720, camera_hfov_rad="bad"), valid_bbox),
        (DetectionFrame(width=1280, height=720, camera_hfov_rad=float("nan")), valid_bbox),
        (valid_frame, [590, 100, float("nan"), 500]),
        (valid_frame, [590, 100, 690, float("inf")]),
        (valid_frame, [590, "bad", 690, 500]),
    ]

    for frame, bbox in cases:
        result = localize_person_detection(
            {"class": "person", "confidence": 0.9, "bbox": bbox},
            frame,
            scan,
            robot_x=1.0,
            robot_y=2.0,
            robot_yaw=0.0,
        )

        assert result["position_quality"] == "bearing_only"
        assert result["range_source"] == "unresolved"
        assert result["bearing_base"] is None
        assert result["bearing_map"] is None
        assert result["range_m"] is None
        assert result["world_x"] is None
        assert result["world_y"] is None
