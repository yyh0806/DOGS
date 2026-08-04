"""Pure, SDK-aligned motion domain values for Go2W.

This module deliberately imports neither ROS nor Unitree SDK2.  It is the
single vocabulary shared by the reducer, adapters, protocol, and tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Optional, Tuple


class PhysicalMode(Enum):
    UNKNOWN = "unknown"
    IDLE = "idle"
    WHEEL_BALANCE = "wheel_balance"
    POSE = "pose"
    WHEEL_LOCOMOTION = "wheel_locomotion"
    LIE_DOWN = "lie_down"
    JOINT_LOCK = "joint_lock"
    DAMPING = "damping"
    RECOVERY = "recovery"


class ActualMotionState(Enum):
    UNKNOWN = "unknown"
    STOPPED = "stopped"
    MOVING = "moving"


class SessionState(Enum):
    BOOT_HOLD = "boot_hold"
    PARKED = "parked"
    ACTIVATING = "activating"
    MANUAL_ACTIVE = "manual_active"
    NAV_ACTIVE = "nav_active"
    STOPPING = "stopping"
    PARKING = "parking"
    ESTOP = "estop"
    FAULT = "fault"


class MotionIntent(Enum):
    START_MANUAL = "start_manual"
    START_NAV = "start_nav"
    PARK = "park"
    ESTOP = "estop"
    CLEAR_ESTOP = "clear_estop"


class StopProfile(Enum):
    MOVE_ZERO_ONLY = "move_zero_only"
    MOVE_ZERO_THEN_STOP_MOVE = "move_zero_then_stop_move"


@dataclass(frozen=True)
class Effect:
    operation: str
    sequence: int
    arguments: Tuple[float, ...] = ()
    transition_id: Optional[int] = None
    reason: Optional[str] = None


@dataclass(frozen=True)
class Telemetry:
    sample_id: int
    source_stamp: float
    received_at: float
    raw_mode: int
    wheel_dq: Tuple[float, float, float, float]
    battery_soc: float
    error_code: int
    roll: float
    pitch: float
    motion_service: Optional[str]
    motor_fault: bool

    def __post_init__(self) -> None:
        if not isinstance(self.sample_id, int) or self.sample_id < 0:
            raise ValueError("sample_id must be a non-negative integer")
        self._require_finite("source_stamp", self.source_stamp)
        self._require_finite("received_at", self.received_at)
        if not isinstance(self.raw_mode, int) or not 0 <= self.raw_mode <= 255:
            raise ValueError("raw_mode must be an integer in [0, 255]")
        if len(self.wheel_dq) != 4 or not all(
                math.isfinite(float(value)) for value in self.wheel_dq):
            raise ValueError("wheel_dq must contain four finite values")
        self._require_finite("battery_soc", self.battery_soc)
        if not 0.0 <= float(self.battery_soc) <= 100.0:
            raise ValueError("battery_soc must be in [0, 100]")
        if not isinstance(self.error_code, int) or self.error_code < 0:
            raise ValueError("error_code must be a non-negative integer")
        self._require_finite("roll", self.roll)
        self._require_finite("pitch", self.pitch)
        if self.motion_service is not None and not str(self.motion_service).strip():
            raise ValueError("motion_service must be non-empty or None")
        if not isinstance(self.motor_fault, bool):
            raise ValueError("motor_fault must be boolean")

    @staticmethod
    def _require_finite(field: str, value: float) -> None:
        try:
            finite = math.isfinite(float(value))
        except (TypeError, ValueError, OverflowError):
            finite = False
        if not finite:
            raise ValueError(f"{field} must be finite")

    def is_fresh(self, now: float, max_age: float) -> bool:
        try:
            age = float(now) - float(self.received_at)
            limit = float(max_age)
        except (TypeError, ValueError, OverflowError):
            return False
        return math.isfinite(age) and math.isfinite(limit) and 0.0 <= age <= limit

    def wheels_stopped(self, threshold: float) -> bool:
        limit = max(0.0, float(threshold))
        return max(abs(float(value)) for value in self.wheel_dq) <= limit


@dataclass(frozen=True)
class CommandReceipt:
    operation: str
    code: int
    sequence: int
    physical_confirmed: bool = False

    @property
    def transport_ok(self) -> bool:
        return self.code == 0


@dataclass(frozen=True)
class InitializationResult:
    """Result of acquiring or probing the sport motion service.

    Returned by motion-service adapters (SportGatewayClient on real hardware,
    SimSportGateway under GO2W_SIM) so the motion controller can decide whether
    lease acquisition succeeded and which service mode is currently active.
    Relocated here from unitree_sport_adapter.py when the dead
    UnitreeSportAdapter class was removed; the dataclass stays the single
    initialization vocabulary shared by every adapter implementation.
    """

    code: int
    motion_service: Optional[str]
    raw_mode: Any = None


class Go2WModeProfile:
    """Decode the deployed Go2W sport-mode profile and fail closed."""

    _MAP = {
        0: PhysicalMode.IDLE,
        1: PhysicalMode.WHEEL_BALANCE,
        2: PhysicalMode.POSE,
        3: PhysicalMode.WHEEL_LOCOMOTION,
        5: PhysicalMode.LIE_DOWN,
        6: PhysicalMode.JOINT_LOCK,
        7: PhysicalMode.DAMPING,
        8: PhysicalMode.RECOVERY,
    }

    def decode(self, raw: object) -> PhysicalMode:
        try:
            key = int(raw)
        except (TypeError, ValueError, OverflowError):
            return PhysicalMode.UNKNOWN
        return self._MAP.get(key, PhysicalMode.UNKNOWN)
