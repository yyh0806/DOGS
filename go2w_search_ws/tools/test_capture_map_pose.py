import math
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from capture_map_pose import build_pose_receipt, quaternion_to_yaw


def test_quaternion_to_yaw_uses_ros_left_positive_convention():
    half = math.pi / 4.0

    yaw = quaternion_to_yaw(0.0, 0.0, math.sin(half), math.cos(half))

    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-9)


def test_pose_receipt_contains_map_coordinates_and_yaml_snippet():
    receipt = build_pose_receipt(
        map_frame="map",
        base_frame="base_link",
        x=1.23456,
        y=-2.34567,
        quaternion=(0.0, 0.0, 0.0, 1.0),
        captured_at=123.0,
        room="实验室",
    )

    assert receipt["read_only"] is True
    assert receipt["frame_id"] == "map"
    assert receipt["child_frame_id"] == "base_link"
    assert receipt["room"] == "实验室"
    assert receipt["pose"] == {"x": 1.23456, "y": -2.34567, "yaw": 0.0}
    assert receipt["yaml_nav_pose"] == (
        "nav_pose: {x: 1.23456, y: -2.34567, yaw: 0.00000}")


def test_pose_receipt_rejects_non_finite_transform():
    try:
        build_pose_receipt(
            map_frame="map",
            base_frame="base_link",
            x=float("nan"),
            y=0.0,
            quaternion=(0.0, 0.0, 0.0, 1.0),
            captured_at=1.0,
        )
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("non-finite transform was accepted")


def test_capture_tool_has_no_ros_publisher_or_motion_client():
    source = (TOOLS_DIR / "capture_map_pose.py").read_text(encoding="utf-8")

    assert "create_publisher" not in source
    assert "SportClient" not in source
    assert "NavigateToPose" not in source

