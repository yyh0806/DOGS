"""Pure-function move execution primitives for move_relative tasks.

No ROS imports here — callers inject read_yaw/send_cmd_vel/sleep/monotonic so
the closed-loop turn logic is unit-testable without nav2 or hardware.
"""

from __future__ import annotations

import math
from typing import Callable


_REVERSE_CLEARANCE_HARD_MIN_M = 0.55


def sanitize_clearance_margin(value) -> float:
    """Return a finite reverse margin no lower than the hard 0.55 m floor."""
    try:
        configured = float(value)
    except (TypeError, ValueError, OverflowError):
        return _REVERSE_CLEARANCE_HARD_MIN_M
    if not math.isfinite(configured):
        return _REVERSE_CLEARANCE_HARD_MIN_M
    return max(_REVERSE_CLEARANCE_HARD_MIN_M, configured)


def directional_clearance_from_scan(
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    ranges,
    *,
    center_deg: float,
    half_fov_deg: float = 30.0,
) -> float | None:
    """Return the closest valid LaserScan sample around a body-frame angle."""
    try:
        angle_min = float(angle_min)
        angle_increment = float(angle_increment)
        range_min = float(range_min)
        range_max = float(range_max)
        center = math.radians(float(center_deg))
        half_fov = math.radians(float(half_fov_deg))
        samples = tuple(ranges)
    except (TypeError, ValueError, OverflowError):
        return None
    metadata = (angle_min, angle_increment, range_min, range_max,
                center, half_fov)
    if (not all(math.isfinite(value) for value in metadata)
            or angle_increment == 0.0
            or range_min < 0.0
            or range_max <= range_min
            or half_fov < 0.0
            or half_fov > math.pi
            or not samples):
        return None

    selected = []
    for index, raw_range in enumerate(samples):
        try:
            distance = float(raw_range)
        except (TypeError, ValueError, OverflowError):
            continue
        if (not math.isfinite(distance)
                or distance < range_min
                or distance > range_max):
            continue
        angle = angle_min + index * angle_increment
        difference = math.atan2(math.sin(angle - center),
                                math.cos(angle - center))
        if abs(difference) <= half_fov + 1e-12:
            selected.append(distance)
    return min(selected) if selected else None


def compute_linear_target(
    x: float, y: float, yaw: float, direction: str, distance_m: float
) -> tuple[float, float, float]:
    """Forward/backward target pose in the same frame, keeping heading.

    Used for linear move_relative (前进/后退): the caller submits this target
    to PointNavigationController so nav2 handles path planning + 避障绕行.
    """
    sign = 1.0 if direction == "forward" else -1.0
    tx = x + sign * distance_m * math.cos(yaw)
    ty = y + sign * distance_m * math.sin(yaw)
    return (tx, ty, yaw)


def compute_angular_target_yaw(
    current_yaw: float, direction: str, angle_deg: float
) -> float:
    """Target yaw after turning in place. Left = +, right = -."""
    delta = math.radians(angle_deg)
    return current_yaw + delta if direction == "left" else current_yaw - delta


def yaw_error(current_yaw: float, target_yaw: float) -> float:
    """Smallest signed difference, wrapped to [-pi, pi]."""
    err = (target_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return err


def angular_turn_complete(
    current_yaw: float, target_yaw: float, tolerance_rad: float
) -> bool:
    return abs(yaw_error(current_yaw, target_yaw)) <= tolerance_rad


def run_angular_turn(
    read_yaw: Callable[[], float],
    send_cmd_vel: Callable[[float, float, float], None],
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    target_yaw: float,
    direction: str,
    *,
    vyaw: float = 0.5,
    tolerance_rad: float = math.radians(3.0),
    max_duration: float | None = None,
) -> str:
    """Closed-loop in-place turn. Returns 'succeeded' or 'timed_out'.

    Always publishes a zero-velocity stop before returning (via finally) so the
    robot never keeps spinning on timeout/exception. Caller still should call
    robot.stop_move() for belt-and-suspenders.
    """
    sign = 1.0 if direction == "left" else -1.0
    if max_duration is None:
        remaining = abs(yaw_error(read_yaw(), target_yaw))
        max_duration = (remaining / max(vyaw, 1e-6)) * 2.0 + 1.0
    deadline = monotonic() + max_duration
    try:
        while monotonic() < deadline:
            # MotionController intentionally expires nav commands after 0.3s.
            # Refresh at the 50ms control cadence so a closed-loop turn keeps
            # authority until yaw reaches the target instead of moving only
            # for one watchdog window and then silently timing out.
            send_cmd_vel(0.0, 0.0, sign * vyaw)
            sleep(0.05)
            if angular_turn_complete(read_yaw(), target_yaw, tolerance_rad):
                return "succeeded"
        return "timed_out"
    finally:
        send_cmd_vel(0.0, 0.0, 0.0)


def run_linear_translation(
    read_xy: Callable[[], tuple[float, float] | None],
    read_clearance: Callable[[str], float | None],
    send_cmd_vel: Callable[[float, float, float], None],
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    start_xy: tuple[float, float],
    direction: str,
    distance_m: float,
    *,
    start_yaw: float = 0.0,
    is_cancelled: Callable[[], bool] | None = None,
    max_speed: float = 0.3,
    slow_speed: float = 0.12,
    slow_distance_m: float = 0.25,
    tolerance_m: float = 0.03,
    clearance_margin_m: float = 0.55,
    control_period: float = 0.05,
    max_cross_track_m: float = 0.15,
    max_localization_step_m: float = 0.50,
    max_path_overrun_m: float = 0.20,
    max_duration: float | None = None,
) -> str:
    """Run a bounded, clearance-gated translation from fresh XY data.

    Progress is the signed projection onto the commanded start-yaw axis, not
    radial displacement.  Geometry fails closed if cross-track drift exceeds
    0.15 m, a localization update jumps over 0.50 m, or accumulated travel
    exceeds the requested axial distance by more than 0.20 m.
    """
    callbacks = {
        "read_xy": read_xy,
        "read_clearance": read_clearance,
        "send_cmd_vel": send_cmd_vel,
        "sleep": sleep,
        "monotonic": monotonic,
    }
    for name, callback in callbacks.items():
        if not callable(callback):
            raise ValueError(f"{name} must be callable")
    if is_cancelled is not None and not callable(is_cancelled):
        raise ValueError("is_cancelled must be callable")
    if direction not in {"forward", "backward"}:
        raise ValueError("direction must be 'forward' or 'backward'")
    try:
        start_x, start_y = (float(start_xy[0]), float(start_xy[1]))
        start_yaw = float(start_yaw)
        distance_m = float(distance_m)
        max_speed = float(max_speed)
        slow_speed = float(slow_speed)
        slow_distance_m = float(slow_distance_m)
        tolerance_m = float(tolerance_m)
        clearance_margin_m = float(clearance_margin_m)
        control_period = float(control_period)
        max_cross_track_m = float(max_cross_track_m)
        max_localization_step_m = float(max_localization_step_m)
        max_path_overrun_m = float(max_path_overrun_m)
    except (TypeError, ValueError, IndexError, OverflowError) as exc:
        raise ValueError("linear translation arguments must be finite numbers") from exc
    numeric = (start_x, start_y, start_yaw, distance_m, max_speed, slow_speed,
               slow_distance_m, tolerance_m, clearance_margin_m,
               control_period, max_cross_track_m,
               max_localization_step_m, max_path_overrun_m)
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("linear translation arguments must be finite numbers")
    if distance_m <= 0.0:
        raise ValueError("distance_m must be positive")
    if max_speed <= 0.0 or max_speed > 0.3:
        raise ValueError("max_speed must be in (0, 0.3]")
    if slow_speed <= 0.0 or slow_speed > max_speed:
        raise ValueError("slow_speed must be in (0, max_speed]")
    if slow_distance_m <= 0.0:
        raise ValueError("slow_distance_m must be positive")
    if tolerance_m <= 0.0 or tolerance_m >= distance_m:
        raise ValueError("tolerance_m must be in (0, distance_m)")
    if clearance_margin_m < 0.0:
        raise ValueError("clearance_margin_m must be nonnegative")
    if control_period <= 0.0:
        raise ValueError("control_period must be positive")
    if max_cross_track_m <= 0.0:
        raise ValueError("max_cross_track_m must be positive")
    if max_localization_step_m <= 0.0:
        raise ValueError("max_localization_step_m must be positive")
    if max_path_overrun_m <= 0.0:
        raise ValueError("max_path_overrun_m must be positive")
    if max_duration is None:
        max_duration = distance_m / max_speed * 2.5 + 1.0
    try:
        max_duration = float(max_duration)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("max_duration must be a finite positive number") from exc
    if not math.isfinite(max_duration) or max_duration <= 0.0:
        raise ValueError("max_duration must be a finite positive number")

    def current_xy() -> tuple[float, float] | None:
        try:
            xy = read_xy()
            x, y = float(xy[0]), float(xy[1])
        except Exception:
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return (x, y)

    sign = 1.0 if direction == "forward" else -1.0
    axis_x = math.cos(start_yaw)
    axis_y = math.sin(start_yaw)
    previous_x, previous_y = start_x, start_y
    path_length = 0.0
    try:
        try:
            started_at = float(monotonic())
        except Exception:
            return "timed_out"
        if not math.isfinite(started_at):
            return "timed_out"
        deadline = started_at + max_duration
        while True:
            if is_cancelled is not None:
                try:
                    if is_cancelled():
                        return "cancelled"
                except Exception:
                    return "cancelled"
            try:
                now = float(monotonic())
            except Exception:
                return "timed_out"
            if not math.isfinite(now) or now >= deadline:
                return "timed_out"

            xy = current_xy()
            if xy is None:
                return "localization_lost"
            x, y = xy
            step = math.hypot(x - previous_x, y - previous_y)
            if step > max_localization_step_m:
                return "localization_lost"
            path_length += step
            previous_x, previous_y = x, y

            dx, dy = x - start_x, y - start_y
            axial_progress = sign * (dx * axis_x + dy * axis_y)
            cross_track = abs(-dx * axis_y + dy * axis_x)
            if cross_track > max_cross_track_m:
                return "localization_lost"
            if axial_progress < -tolerance_m:
                return "localization_lost"

            remaining = distance_m - axial_progress
            if path_length > distance_m + max_path_overrun_m:
                return "localization_lost"
            if abs(remaining) <= tolerance_m:
                return "succeeded"
            if remaining < -tolerance_m:
                return "localization_lost"

            try:
                clearance = float(read_clearance(direction))
            except Exception:
                return "obstacle"
            if (not math.isfinite(clearance)
                    or clearance <= clearance_margin_m):
                return "obstacle"

            speed = slow_speed if remaining <= slow_distance_m else max_speed
            try:
                send_cmd_vel(sign * speed, 0.0, 0.0)
            except Exception:
                return "timed_out"
            try:
                sleep(control_period)
            except Exception:
                return "timed_out"
    finally:
        try:
            send_cmd_vel(0.0, 0.0, 0.0)
        except Exception:
            pass


__all__ = [
    "compute_linear_target", "compute_angular_target_yaw", "yaw_error",
    "angular_turn_complete", "directional_clearance_from_scan",
    "run_angular_turn", "run_linear_translation", "sanitize_clearance_margin",
]
