"""Shared camera-to-map annotation calibration resolution."""

from __future__ import annotations

import math
import os


C13_VISIBLE_NOMINAL_HFOV_DEG = 77.4
_GENERIC_NOMINAL_HFOV_DEG = 70.0


def camera_source_key(source) -> str:
    return "".join(
        character if character.isalnum() else "_"
        for character in str(source or "").upper()
    ).strip("_")


def _finite_environment(names, *, minimum=None, maximum=None):
    for name in names:
        raw = os.environ.get(name)
        if raw is None or not str(raw).strip():
            continue
        try:
            value = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(value):
            continue
        if minimum is not None and value <= minimum:
            continue
        if maximum is not None and value >= maximum:
            continue
        return value, name
    return None, None


def resolve_camera_calibration(source=None, *, gimbal_yaw_rad=None) -> dict:
    """Resolve source-specific optics and expose whether values were measured."""

    source_name = str(source or "")
    source_key = camera_source_key(source_name)
    c13_visible = source_key == "C13_VIS"

    hfov_names = []
    if source_key:
        hfov_names.append(f"GO2W_CAMERA_HFOV_{source_key}_DEG")
    hfov_names.extend(("GO2W_CAMERA_HFOV_DEG", "GO2W_CAMERA_HFOV"))
    hfov_deg, hfov_environment = _finite_environment(
        hfov_names, minimum=0.0, maximum=180.0)
    if hfov_deg is None:
        hfov_deg = (
            C13_VISIBLE_NOMINAL_HFOV_DEG
            if c13_visible else _GENERIC_NOMINAL_HFOV_DEG)

    yaw_names = []
    if source_key:
        yaw_names.append(f"GO2W_CAMERA_YAW_OFFSET_{source_key}_DEG")
    yaw_names.append("GO2W_CAMERA_YAW_OFFSET_DEG")
    yaw_offset_deg, yaw_environment = _finite_environment(yaw_names)
    if yaw_offset_deg is None:
        yaw_offset_deg = 0.0

    profile_names = []
    if source_key:
        profile_names.append(f"GO2W_CAMERA_CALIBRATION_{source_key}")
    profile_names.append("GO2W_CAMERA_CALIBRATION")
    profile = next((
        str(os.environ.get(name) or "").strip().lower()
        for name in profile_names
        if str(os.environ.get(name) or "").strip()
    ), "nominal_centered" if c13_visible else "uncalibrated")

    try:
        gimbal_yaw = float(gimbal_yaw_rad)
    except (TypeError, ValueError):
        gimbal_yaw = None
    if gimbal_yaw is not None and not math.isfinite(gimbal_yaw):
        gimbal_yaw = None
    gimbal_yaw_deg = (
        None if gimbal_yaw is None else math.degrees(gimbal_yaw))
    effective_yaw_offset_deg = yaw_offset_deg + (gimbal_yaw_deg or 0.0)
    measured = profile == "measured"

    return {
        "source": source_name,
        "hfov_deg": float(hfov_deg),
        "yaw_offset_deg": float(yaw_offset_deg),
        "gimbal_yaw_deg": gimbal_yaw_deg,
        "effective_yaw_offset_deg": float(effective_yaw_offset_deg),
        "profile": profile,
        "measured": measured,
        "hfov_environment": hfov_environment,
        "yaw_environment": yaw_environment,
        "gimbal_yaw_feedback_available": gimbal_yaw is not None,
        "requires_gimbal_centered": gimbal_yaw is None,
        "annotation_ready": profile in {"measured", "nominal_centered"},
    }


__all__ = [
    "C13_VISIBLE_NOMINAL_HFOV_DEG",
    "camera_source_key",
    "resolve_camera_calibration",
]
