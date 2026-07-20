"""Pure-function move execution primitives for move_relative tasks.

No ROS imports here — callers inject read_yaw/send_cmd_vel/sleep/monotonic so
the closed-loop turn logic is unit-testable without nav2 or hardware.
"""

from __future__ import annotations

import math
from typing import Callable


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
        send_cmd_vel(0.0, 0.0, sign * vyaw)
        while monotonic() < deadline:
            sleep(0.05)
            if angular_turn_complete(read_yaw(), target_yaw, tolerance_rad):
                return "succeeded"
        return "timed_out"
    finally:
        send_cmd_vel(0.0, 0.0, 0.0)


__all__ = [
    "compute_linear_target", "compute_angular_target_yaw", "yaw_error",
    "angular_turn_complete", "run_angular_turn",
]
