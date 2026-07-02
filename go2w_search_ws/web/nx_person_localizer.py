"""YOLO bbox bearing plus LaserScan range localization."""

import math
from dataclasses import dataclass
from statistics import median
from typing import Sequence


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


def localize_person_detection(
    detection: dict,
    frame: DetectionFrame,
    scan: LaserScanSnapshot,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    window_rad: float = math.radians(5.0),
) -> dict:
    bbox = detection.get("bbox") or []
    bearing_base = _bearing_from_bbox(bbox, frame)
    if bearing_base is None:
        return _bearing_only(detection, None, None)

    bearing_map = float(robot_yaw) + bearing_base
    lidar_range = range_at_bearing(scan, bearing_base, window_rad=window_rad)
    if lidar_range is None:
        return _bearing_only(detection, bearing_base, bearing_map)

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
        }
    )
    return result


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
        }
    )
    return result
