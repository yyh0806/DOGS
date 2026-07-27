"""YOLO bbox bearing plus LaserScan range localization."""

import math
from dataclasses import dataclass
from statistics import median
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class DetectionFrame:
    width: int
    height: int
    camera_hfov_rad: float
    camera_yaw_offset_rad: float = 0.0
    gimbal_yaw_rad: float = 0.0


@dataclass(frozen=True)
class LaserScanSnapshot:
    angle_min: float
    angle_increment: float
    ranges: Sequence[float]
    range_min: float = 0.15
    range_max: float = 10.0


def coerce_laser_scan_snapshot(value: Any) -> LaserScanSnapshot | None:
    """Convert a ROS-free scan mapping/object into the localizer contract."""

    if value is None:
        return None

    def field(name, default=None):
        if isinstance(value, dict):
            return value.get(name, default)
        return getattr(value, name, default)

    try:
        angle_min = float(field("angle_min"))
        angle_increment = float(field("angle_increment"))
        range_min = float(field("range_min", 0.15))
        range_max = float(field("range_max", 10.0))
        ranges = list(field("ranges", []) or [])
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        not math.isfinite(angle_min)
        or not math.isfinite(angle_increment)
        or angle_increment <= 0.0
        or not math.isfinite(range_min)
        or range_min <= 0.0
        or not math.isfinite(range_max)
        or range_max <= range_min
        or not ranges
    ):
        return None
    return LaserScanSnapshot(
        angle_min=angle_min,
        angle_increment=angle_increment,
        ranges=ranges,
        range_min=range_min,
        range_max=range_max,
    )


@dataclass(frozen=True)
class PointCloudSnapshot:
    """Recent MID360 points expressed in ``base_link`` coordinates."""

    points: Sequence[Sequence[float]]
    frame_id: str = "base_link"


def decode_pointcloud_xyz(message, max_points: int = 50000) -> np.ndarray:
    """Decode x/y/z float32 fields from PointCloud2 without ROS helpers."""
    try:
        point_step = int(message.point_step)
        width = int(message.width)
        height = int(message.height)
        fields = {field.name: field for field in message.fields}
        limit = max(1, int(max_points))
    except (AttributeError, TypeError, ValueError):
        return np.empty((0, 3), dtype=np.float32)
    if point_step <= 0 or width <= 0 or height <= 0 or not message.data:
        return np.empty((0, 3), dtype=np.float32)
    if any(name not in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    # sensor_msgs/PointField.FLOAT32 == 7.  Keep the decoder ROS-independent
    # so its geometry is covered by workstation unit tests.
    if any(int(fields[name].datatype) != 7 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    offsets = [int(fields[name].offset) for name in ("x", "y", "z")]
    if any(offset < 0 or offset + 4 > point_step for offset in offsets):
        return np.empty((0, 3), dtype=np.float32)
    count = min(width * height, len(message.data) // point_step)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float32)
    endian = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
    try:
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [f"{endian}f4"] * 3,
            "offsets": offsets,
            "itemsize": point_step,
        })
        records = np.ndarray(
            shape=(count,), dtype=dtype, buffer=memoryview(message.data))
        points = np.column_stack(
            (records["x"], records["y"], records["z"])).astype(
                np.float32, copy=False)
    except Exception:
        return np.empty((0, 3), dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > limit:
        indices = np.linspace(0, len(points) - 1, limit, dtype=np.int64)
        points = points[indices]
    return np.ascontiguousarray(points)


def localize_target_detection(
    detection: dict,
    frame: DetectionFrame,
    scan: LaserScanSnapshot,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    window_rad: float = math.radians(5.0),
    robot_z: float = 0.0,
    pointcloud: PointCloudSnapshot | None = None,
    height_range_tolerance_m: float = 0.35,
) -> dict:
    bbox = detection.get("bbox") or []
    bearing_base = _bearing_from_bbox(bbox, frame)
    if bearing_base is None:
        return _bearing_only(detection, None, None)

    bearing_map = float(robot_yaw) + bearing_base
    lidar_range = range_at_bearing(scan, bearing_base, window_rad=window_rad)
    if lidar_range is None:
        return _bearing_only(detection, bearing_base, bearing_map)

    height = height_at_bearing(
        pointcloud,
        bearing_rad=bearing_base,
        range_m=lidar_range,
        window_rad=window_rad,
        range_tolerance_m=height_range_tolerance_m,
    ) if pointcloud is not None else None
    base_z = _finite_float(robot_z)
    world_z = (
        base_z + height["height_m"]
        if base_z is not None and height is not None
        else None
    )

    result = dict(detection)
    result.update(
        {
            "bearing_base": bearing_base,
            "bearing_map": bearing_map,
            "range_m": lidar_range,
            "range_source": "lidar",
            "position_quality": "range_lidar",
            "world_x": float(robot_x) + lidar_range * math.cos(bearing_map),
            "world_y": float(robot_y) + lidar_range * math.sin(bearing_map),
            "world_z": world_z,
            "position_dimension": 3 if world_z is not None else 2,
            "height_source": (
                "mid360_pointcloud" if world_z is not None else "unresolved"),
            "height_point_count": (
                int(height["point_count"]) if height is not None else 0),
        }
    )
    return result


def localize_person_detection(
    detection: dict,
    frame: DetectionFrame,
    scan: LaserScanSnapshot,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    window_rad: float = math.radians(5.0),
    robot_z: float = 0.0,
    pointcloud: PointCloudSnapshot | None = None,
    height_range_tolerance_m: float = 0.35,
) -> dict:
    """Backward-compatible person-specific name for generic localization."""
    return localize_target_detection(
        detection,
        frame,
        scan,
        robot_x,
        robot_y,
        robot_yaw,
        window_rad=window_rad,
        robot_z=robot_z,
        pointcloud=pointcloud,
        height_range_tolerance_m=height_range_tolerance_m,
    )


def height_at_bearing(
    cloud: PointCloudSnapshot,
    bearing_rad: float,
    range_m: float,
    window_rad: float,
    range_tolerance_m: float = 0.35,
) -> dict | None:
    """Associate a horizontal camera/LaserScan ray with 3D MID360 returns.

    Range gating is essential: bearing alone can include a nearer chair and a
    farther wall.  The returned height is the robust median base-frame Z of
    points consistent with both the detection bearing and its LaserScan range.
    """
    if cloud is None or str(getattr(cloud, "frame_id", "")) != "base_link":
        return None
    bearing = _finite_float(bearing_rad)
    expected_range = _finite_float(range_m)
    window = _finite_float(window_rad)
    tolerance = _finite_float(range_tolerance_m)
    if (
        bearing is None or expected_range is None or window is None
        or tolerance is None or expected_range <= 0.0 or window <= 0.0
        or tolerance <= 0.0
    ):
        return None

    points = getattr(cloud, "points", None)
    if points is None:
        return None
    heights = []
    try:
        iterator = iter(points)
    except TypeError:
        return None
    for point in iterator:
        try:
            if len(point) < 3:
                continue
            x, y, z = (float(point[index]) for index in range(3))
        except (TypeError, ValueError, IndexError):
            continue
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        horizontal_range = math.hypot(x, y)
        if abs(horizontal_range - expected_range) > tolerance:
            continue
        if abs(_angle_diff(math.atan2(y, x), bearing)) > window + 1e-12:
            continue
        heights.append(z)
    if not heights:
        return None
    return {"height_m": float(median(heights)), "point_count": len(heights)}


def range_at_bearing(scan: LaserScanSnapshot, bearing_rad: float, window_rad: float) -> float | None:
    angle_min = _finite_float(scan.angle_min)
    angle_increment = _finite_float(scan.angle_increment)
    range_min = _finite_float(scan.range_min)
    range_max = _finite_float(scan.range_max)
    bearing = _finite_float(bearing_rad)
    window = _finite_float(window_rad)
    if (
        angle_min is None
        or angle_increment is None
        or range_min is None
        or range_max is None
        or bearing is None
        or window is None
        or angle_increment <= 0.0
        or range_min <= 0.0
        or range_max <= 0.0
        or range_min >= range_max
        or window <= 0.0
    ):
        return None

    ranges = scan.ranges
    if not ranges:
        return None

    valid_ranges = []
    for index, raw_range in enumerate(scan.ranges):
        try:
            range_m = float(raw_range)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(range_m) or range_m < range_min or range_m > range_max:
            continue

        ray_angle = angle_min + index * angle_increment
        if abs(_angle_diff(ray_angle, bearing)) <= window + 1e-12:
            valid_ranges.append(range_m)

    return float(median(valid_ranges)) if valid_ranges else None


def _bearing_from_bbox(bbox: Sequence[float], frame: DetectionFrame) -> float | None:
    if isinstance(bbox, (str, bytes)):
        return None
    try:
        bbox_values = bbox[:4]
    except (TypeError, KeyError):
        return None
    if len(bbox_values) < 4:
        return None

    try:
        x1, y1, x2, y2 = (float(value) for value in bbox_values)
    except (TypeError, ValueError):
        return None

    width = _finite_float(frame.width)
    height = _finite_float(frame.height)
    camera_hfov_rad = _finite_float(frame.camera_hfov_rad)
    camera_yaw_offset_rad = _finite_float(frame.camera_yaw_offset_rad)
    gimbal_yaw_rad = _finite_float(frame.gimbal_yaw_rad)
    if (
        width is None
        or height is None
        or camera_hfov_rad is None
        or camera_yaw_offset_rad is None
        or gimbal_yaw_rad is None
        or width <= 0.0
        or height <= 0.0
        or camera_hfov_rad <= 0.0
    ):
        return None

    values = (x1, y1, x2, y2)
    if not all(math.isfinite(value) for value in values) or x2 < x1 or y2 < y1:
        return None

    cx_norm = ((x1 + x2) / 2.0) / width
    # Image x grows right; base_link/LaserScan positive yaw points left.
    camera_angle = (0.5 - cx_norm) * camera_hfov_rad
    return camera_yaw_offset_rad + gimbal_yaw_rad + camera_angle


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _finite_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _bearing_only(detection: dict, bearing_base: float | None, bearing_map: float | None) -> dict:
    result = dict(detection)
    result.update(
        {
            "bearing_base": bearing_base,
            "bearing_map": bearing_map,
            "range_m": None,
            "range_source": "unresolved",
            "position_quality": "bearing_only",
            "world_x": None,
            "world_y": None,
            "world_z": None,
            "position_dimension": 0,
            "height_source": "unresolved",
            "height_point_count": 0,
        }
    )
    return result
