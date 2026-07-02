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
    if not scan.ranges or scan.angle_increment == 0:
        return None

    valid_ranges = []
    for index, raw_range in enumerate(scan.ranges):
        try:
            range_m = float(raw_range)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(range_m) or range_m < scan.range_min or range_m > scan.range_max:
            continue

        ray_angle = scan.angle_min + index * scan.angle_increment
        if abs(_angle_diff(ray_angle, bearing_rad)) <= window_rad + 1e-12:
            valid_ranges.append(range_m)

    return float(median(valid_ranges)) if valid_ranges else None


def _bearing_from_bbox(bbox: Sequence[float], frame: DetectionFrame) -> float | None:
    if len(bbox) < 4 or frame.width <= 0 or frame.height <= 0:
        return None
    try:
        x1, y1, x2, y2 = (float(value) for value in bbox[:4])
        camera_hfov_rad = float(frame.camera_hfov_rad)
        camera_yaw_offset_rad = float(frame.camera_yaw_offset_rad)
        gimbal_yaw_rad = float(frame.gimbal_yaw_rad)
    except (TypeError, ValueError):
        return None

    values = (x1, y1, x2, y2, camera_hfov_rad, camera_yaw_offset_rad, gimbal_yaw_rad)
    if not all(math.isfinite(value) for value in values) or x2 < x1 or y2 < y1:
        return None

    cx_norm = ((x1 + x2) / 2.0) / float(frame.width)
    camera_angle = (cx_norm - 0.5) * camera_hfov_rad
    return camera_yaw_offset_rad + gimbal_yaw_rad + camera_angle


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


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
