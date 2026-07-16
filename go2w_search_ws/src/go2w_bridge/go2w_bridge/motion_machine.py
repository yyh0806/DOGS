"""Deterministic, feedback-confirmed Go2W motion policy.

The machine is intentionally pure: it owns state and emits immutable SDK
effects, but performs no ROS, threading, sleeping, or Unitree calls.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, List, Optional

try:
    from .motion_types import (
        ActualMotionState,
        CommandReceipt,
        Effect,
        Go2WModeProfile,
        MotionIntent,
        PhysicalMode,
        SessionState,
        StopProfile,
        Telemetry,
    )
except ImportError:  # Direct-file compatibility deployment on the NX.
    from motion_types import (
        ActualMotionState,
        CommandReceipt,
        Effect,
        Go2WModeProfile,
        MotionIntent,
        PhysicalMode,
        SessionState,
        StopProfile,
        Telemetry,
    )


@dataclass(frozen=True)
class MotionSnapshot:
    session: SessionState
    physical_mode: PhysicalMode
    actual_motion: ActualMotionState
    velocity_authorized: bool
    telemetry_fresh: bool
    error_code: int
    fault: Optional[str]
    motion_service: Optional[str]
    transition_id: Optional[int]
    transition_operation: Optional[str]
    owner: Optional[str]
    last_sample_id: Optional[int]
    rejected_samples: int


class Go2WMotionMachine:
    """Reduce SDK readiness, telemetry, intents, and time into SDK effects."""

    WHEEL_MODES = frozenset({
        PhysicalMode.WHEEL_BALANCE,
        PhysicalMode.WHEEL_LOCOMOTION,
    })

    def __init__(
        self,
        *,
        now: Callable[[], float] = time.monotonic,
        mode_profile: Optional[Go2WModeProfile] = None,
        stop_profile: StopProfile = StopProfile.MOVE_ZERO_ONLY,
        expected_motion_service: str = "ai-w",
        telemetry_max_age: float = 0.5,
        # Go2W joint-lock encoder noise measured up to 0.085 rad/s on the
        # powered robot.  0.12 rad/s is ~7.8 mm/s at the 65 mm wheel radius,
        # while real creep remains well above this boundary.
        wheel_stop_threshold: float = 0.20,   # 2026-07-16 放宽: 原0.12太近编码器噪声0.085, 导航停车微动误判parked_state_lost→EMERGENCY
        moving_fault_samples: int = 15,       # 2026-07-16: PARKED态Go2W停车后轮速余动/平衡微动误报parked_state_lost, 要求15帧(~1.5s)持续运动才fault
        max_attitude_rad: float = 0.70,
        minimum_battery_soc: float = 5.0,
        transition_timeout: float = 5.0,
        park_zero_settle: float = 0.50,       # 2026-07-16 放宽: 原0.30s轮子没停稳就检测, 给0.50s沉淀
    ) -> None:
        self._now = now
        self._profile = mode_profile or Go2WModeProfile()
        self._stop_profile = StopProfile(stop_profile)
        self._expected_motion_service = str(expected_motion_service)
        self._telemetry_max_age = float(telemetry_max_age)
        self._wheel_stop_threshold = float(wheel_stop_threshold)
        self._moving_fault_samples = max(1, int(moving_fault_samples))
        self._max_attitude_rad = float(max_attitude_rad)
        self._minimum_battery_soc = float(minimum_battery_soc)
        self._transition_timeout = float(transition_timeout)
        self._park_zero_settle = max(0.0, float(park_zero_settle))

        self._session = SessionState.BOOT_HOLD
        self._physical_mode = PhysicalMode.UNKNOWN
        self._actual_motion = ActualMotionState.UNKNOWN
        self._owner: Optional[str] = None
        self._fault: Optional[str] = None
        self._motion_service: Optional[str] = None
        self._sdk_ready = False
        self._last_telemetry: Optional[Telemetry] = None
        self._pending_telemetry: Optional[Telemetry] = None
        self._last_sample_id: Optional[int] = None
        self._rejected_samples = 0
        self._sequence = 0
        self._transition_counter = 0
        self._transition_id: Optional[int] = None
        self._transition_operation: Optional[str] = None
        self._transition_started_at: Optional[float] = None
        self._transition_deadline: Optional[float] = None
        self._pending_zero_on_tick = False
        self._moving_sample_count = 0
        self._parked_mode_unstable_count = 0  # P1 (2026-07-16): PARKED 态 mode≠JOINT_LOCK 连续帧计数(去抖)

    def sdk_ready(self, motion_service: Optional[str]) -> List[Effect]:
        normalized = str(motion_service).strip() if motion_service else None
        self._motion_service = normalized
        if normalized != self._expected_motion_service:
            self._sdk_ready = False
            self._enter_fault("wrong_motion_service")
            return []

        self._sdk_ready = True
        self._fault = None
        pending = self._pending_telemetry
        self._pending_telemetry = None
        return self._accept_telemetry(pending) if pending is not None else []

    def observe(self, telemetry: Telemetry) -> List[Effect]:
        if not self._sdk_ready:
            if (self._pending_telemetry is None
                    or self._telemetry_follows(
                        self._pending_telemetry, telemetry)):
                self._pending_telemetry = telemetry
            else:
                self._rejected_samples += 1
            return []
        return self._accept_telemetry(telemetry)

    def request(
        self,
        intent: MotionIntent,
        *,
        scan_fresh: bool = False,
    ) -> List[Effect]:
        intent = MotionIntent(intent)
        if intent is MotionIntent.ESTOP:
            if self._session is SessionState.ESTOP:
                return []
            self._session = SessionState.ESTOP
            self._owner = "safety"
            self._clear_transition()
            self._fault = "estop_latched"
            return [self._effect("MoveZero", reason="estop")]

        if intent is MotionIntent.CLEAR_ESTOP:
            # P0 (2026-07-16): reset_drive_fault 映射到此。原本只清 ESTOP, 导致 FAULT
            # (如 parked_state_lost) 锁死后只能重启进程。现扩展: ESTOP 直接清, FAULT 走
            # 受保护恢复——仅当宇树底盘健康(_health_fault=None: error_code=0/姿态/电池/
            # telemetry 正常) + 已停靠(mode=JOINT_LOCK + 轮停) 时才 FAULT→PARKED。
            if self._session is SessionState.ESTOP:
                if (self._physical_mode is PhysicalMode.JOINT_LOCK
                        and self._actual_motion is ActualMotionState.STOPPED
                        and self._telemetry_is_fresh()):
                    self._session = SessionState.PARKED
                    self._owner = None
                    self._fault = None
                return []
            if self._session is SessionState.FAULT:
                if self._last_telemetry is None:
                    return []
                blocking = self._health_fault(self._last_telemetry)
                if (blocking is None
                        and self._physical_mode is PhysicalMode.JOINT_LOCK
                        and self._actual_motion is ActualMotionState.STOPPED):
                    self._session = SessionState.PARKED
                    self._owner = None
                    self._fault = None
                else:
                    self._fault = blocking or "recovery_unsafe"
                return []
            return []

        if intent in (MotionIntent.START_MANUAL, MotionIntent.START_NAV):
            if self._session is not SessionState.PARKED:
                self._fault = "session_busy"
                return []
            if not self._telemetry_is_fresh():
                self._fault = "telemetry_stale"
                return []
            if self._physical_mode is not PhysicalMode.JOINT_LOCK:
                self._fault = "not_parked_mode"
                return []
            if intent is MotionIntent.START_NAV and not scan_fresh:
                self._fault = "nav_scan_stale"
                return []
            owner = "manual" if intent is MotionIntent.START_MANUAL else "nav"
            self._fault = None
            self._owner = owner
            self._session = SessionState.ACTIVATING
            transition_id = self._begin_transition("BalanceStand")
            return [self._effect(
                "BalanceStand",
                transition_id=transition_id,
                reason=f"start_{owner}",
            )]

        if intent is MotionIntent.PARK:
            if self._session is SessionState.PARKED:
                self._fault = None
                return []
            if self._session not in {
                    SessionState.BOOT_HOLD,
                    SessionState.ACTIVATING,
                    SessionState.MANUAL_ACTIVE,
                    SessionState.NAV_ACTIVE,
                    SessionState.STOPPING}:
                self._fault = "park_not_allowed"
                return []
            if self._session is SessionState.STOPPING:
                return []
            self._session = SessionState.STOPPING
            self._fault = None
            transition_id = self._begin_transition("Park")
            effects = [self._effect(
                "MoveZero", transition_id=transition_id, reason="park")]
            if self._stop_profile is StopProfile.MOVE_ZERO_THEN_STOP_MOVE:
                effects.append(self._effect(
                    "StopMove", transition_id=transition_id, reason="park"))
            return effects

        raise ValueError(f"unsupported motion intent: {intent.value}")

    def tick(self, now: Optional[float] = None) -> List[Effect]:
        timestamp = self._now() if now is None else float(now)
        if self._pending_zero_on_tick:
            self._pending_zero_on_tick = False
            return [self._effect("MoveZero", reason="sdk_transport_error")]
        if self._session is SessionState.ESTOP:
            return []
        if self._session is SessionState.FAULT:
            # P2 (2026-07-16): 应用故障自愈。parked_state_lost/physical_mode_lost 等
            # 瞬时误报(底盘实际健康)不再锁死导航——仅当宇树底盘 _health_fault=None
            # (error_code=0/姿态/电池/telemetry 正常) + 已停靠(mode=JOINT_LOCK+轮停) 时
            # 自动 FAULT→PARKED。ESTOP/robot_error 等真底盘故障(_health_fault!=None)不恢复。
            if self._last_telemetry is not None:
                blocking = self._health_fault(self._last_telemetry)
                if (blocking is None
                        and self._physical_mode is PhysicalMode.JOINT_LOCK
                        and self._actual_motion is ActualMotionState.STOPPED):
                    self._session = SessionState.PARKED
                    self._owner = None
                    self._fault = None
            return []
        # BOOT_HOLD is observation-only. A process restart must never promote
        # a wheel mode into StandUp merely because wheel feedback reached zero;
        # only an explicit PARK intent may enter STOPPING/PARKING.
        settling_to_park = self._session is SessionState.STOPPING
        if settling_to_park and self._transition_started_at is not None:
            if timestamp >= self._transition_started_at + self._park_zero_settle:
                self._session = SessionState.PARKING
                if self._owner is None:
                    self._owner = "startup"
                transition_id = self._begin_transition("StandUp")
                return [self._effect(
                    "StandUp", transition_id=transition_id,
                    reason="park_after_zero_settle")]
            return [self._effect("MoveZero", reason="parking_zero_settle")]
        if (self._transition_deadline is not None
                and timestamp > self._transition_deadline):
            self._enter_fault("transition_timeout")
            return [self._effect("MoveZero", reason="transition_timeout")]
        if (self._last_telemetry is not None
                and not self._last_telemetry.is_fresh(
                    timestamp, self._telemetry_max_age)):
            was_active = self._session in {
                SessionState.ACTIVATING,
                SessionState.MANUAL_ACTIVE,
                SessionState.NAV_ACTIVE,
                SessionState.STOPPING,
                SessionState.PARKING,
            }
            if was_active:
                self._enter_fault("telemetry_stale")
                return [self._effect("MoveZero", reason="telemetry_stale")]
            # PARKED/BOOT_HOLD already prohibit velocity.  A feedback-publisher
            # restart can create a short, harmless gap during Nav2 bringup, so
            # surface the inhibition without converting it into a permanent
            # fault that would require an operator reset after data recovers.
            self._fault = "telemetry_stale"
            return []
        return []

    def command_velocity(
        self,
        owner: str,
        velocity: object,
        *,
        scan_fresh: bool = False,
    ) -> Effect:
        try:
            values = tuple(float(value) for value in velocity)
        except (TypeError, ValueError, OverflowError):
            values = ()
        valid = len(values) == 3 and all(math.isfinite(value) for value in values)
        if not valid or values == (0.0, 0.0, 0.0):
            return self._effect("MoveZero", reason="zero_or_invalid_velocity")
        snapshot = self.snapshot()
        owner = str(owner).strip().lower()
        owner_matches = owner == self._owner
        scan_allows = owner != "nav" or bool(scan_fresh)
        if not snapshot.velocity_authorized or not owner_matches or not scan_allows:
            reason = (
                "owner_mismatch" if not owner_matches
                else "nav_scan_stale" if not scan_allows
                else "velocity_not_authorized")
            return self._effect("MoveZero", reason=reason)
        return Effect(
            operation="Move",
            sequence=self._next_sequence(),
            arguments=values,
            reason=f"{owner}_velocity",
        )

    def report_fault(self, reason: str) -> List[Effect]:
        was_active = self._session in {
            SessionState.ACTIVATING,
            SessionState.MANUAL_ACTIVE,
            SessionState.NAV_ACTIVE,
            SessionState.STOPPING,
            SessionState.PARKING,
        }
        self._enter_fault(str(reason))
        return ([self._effect("MoveZero", reason=str(reason))]
                if was_active else [])

    def record_receipt(self, receipt: CommandReceipt) -> None:
        if receipt.transport_ok:
            return
        was_active = self._session in {
            SessionState.ACTIVATING,
            SessionState.MANUAL_ACTIVE,
            SessionState.NAV_ACTIVE,
            SessionState.STOPPING,
            SessionState.PARKING,
        }
        self._enter_fault(f"{receipt.operation}_transport_error")
        self._pending_zero_on_tick = was_active

    def snapshot(self) -> MotionSnapshot:
        telemetry_fresh = self._telemetry_is_fresh()
        velocity_authorized = (
            self._session in {
                SessionState.MANUAL_ACTIVE,
                SessionState.NAV_ACTIVE,
            }
            and self._physical_mode in self.WHEEL_MODES
            and telemetry_fresh
            and self._last_telemetry is not None
            and self._last_telemetry.error_code == 0
            and not self._last_telemetry.motor_fault
        )
        return MotionSnapshot(
            session=self._session,
            physical_mode=self._physical_mode,
            actual_motion=self._actual_motion,
            velocity_authorized=velocity_authorized,
            telemetry_fresh=telemetry_fresh,
            error_code=(self._last_telemetry.error_code
                        if self._last_telemetry is not None else 0),
            fault=self._fault,
            motion_service=self._motion_service,
            transition_id=self._transition_id,
            transition_operation=self._transition_operation,
            owner=self._owner,
            last_sample_id=self._last_sample_id,
            rejected_samples=self._rejected_samples,
        )

    def _accept_telemetry(self, telemetry: Telemetry) -> List[Effect]:
        if (self._last_telemetry is not None
                and not self._telemetry_follows(
                    self._last_telemetry, telemetry)):
            self._rejected_samples += 1
            return []
        self._last_sample_id = telemetry.sample_id
        self._last_telemetry = telemetry
        self._physical_mode = self._profile.decode(telemetry.raw_mode)
        self._actual_motion = (
            ActualMotionState.STOPPED
            if telemetry.wheels_stopped(self._wheel_stop_threshold)
            else ActualMotionState.MOVING)
        if self._actual_motion is ActualMotionState.MOVING:
            self._moving_sample_count += 1
        else:
            self._moving_sample_count = 0
        self._parked_mode_unstable_count = 0  # P1 (2026-07-16): PARKED 态 mode≠JOINT_LOCK 连续帧计数(去抖)

        if (telemetry.motion_service is not None
                and str(telemetry.motion_service).strip()
                != self._expected_motion_service):
            self._enter_fault("wrong_motion_service")
            return []
        unhealthy = self._health_fault(telemetry)
        if unhealthy is not None:
            was_active = self._session in {
                SessionState.ACTIVATING,
                SessionState.MANUAL_ACTIVE,
                SessionState.NAV_ACTIVE,
                SessionState.STOPPING,
                SessionState.PARKING,
            }
            self._enter_fault(unhealthy)
            return ([self._effect("MoveZero", reason=unhealthy)]
                    if was_active else [])

        if self._session is SessionState.BOOT_HOLD:
            if self._physical_mode is PhysicalMode.JOINT_LOCK:
                if self._actual_motion is ActualMotionState.STOPPED:
                    self._session = SessionState.PARKED
                    self._owner = None
                    self._fault = None
                    self._clear_transition()
                    return []
                return [self._effect(
                    "MoveZero", reason="boot_joint_lock_moving")]
            if self._physical_mode in self.WHEEL_MODES:
                if self._actual_motion is ActualMotionState.MOVING:
                    return [self._effect(
                        "MoveZero",
                        reason="boot_wheels_moving",
                    )]
                return []
            self._enter_fault("unexpected_boot_mode")
            return []

        if self._session is SessionState.ACTIVATING:
            if self._physical_mode in self.WHEEL_MODES:
                self._session = (
                    SessionState.NAV_ACTIVE if self._owner == "nav"
                    else SessionState.MANUAL_ACTIVE)
                self._fault = None
                self._clear_transition()
            elif self._physical_mode is not PhysicalMode.JOINT_LOCK:
                self._enter_fault("activation_mode_invalid")
                return [self._effect("MoveZero", reason="activation_mode_invalid")]
            return []

        if self._session in {
                SessionState.MANUAL_ACTIVE, SessionState.NAV_ACTIVE}:
            if self._physical_mode not in self.WHEEL_MODES:
                self._enter_fault("physical_mode_lost")
                return [self._effect("MoveZero", reason="physical_mode_lost")]
            return []

        if self._session is SessionState.STOPPING:
            # Do not let one transient zero-wheel sample bypass the bounded
            # MoveZero settling window.  BalanceStand can briefly report zero
            # before its control reaction has decayed; StandUp at that instant
            # leaves a post-transition wheel burst that looks like a lost
            # parked state.  tick() is the single time-qualified path to
            # PARKING/StandUp.
            return [self._effect(
                "MoveZero", transition_id=self._transition_id, reason="stopping")]

        if self._session is SessionState.PARKING:
            if (self._physical_mode is PhysicalMode.JOINT_LOCK
                    and self._actual_motion is ActualMotionState.STOPPED):
                self._session = SessionState.PARKED
                self._owner = None
                self._fault = None
                self._clear_transition()
            elif self._actual_motion is ActualMotionState.MOVING:
                return [self._effect(
                    "MoveZero", transition_id=self._transition_id,
                    reason="parking_wheels_moving")]
            return []

        if self._session is SessionState.PARKED:
            if self._physical_mode is not PhysicalMode.JOINT_LOCK:
                # P1 (2026-07-16): mode 去抖。原一帧 mode≠6 即 fault, 停车切换
                # MoveZero→StandUp→jointLock 中间帧瞬时回 1/3 会误报 parked_state_lost。
                # 连续 5 帧 mode≠6 才 fault; mode=6 立即清零。
                self._parked_mode_unstable_count += 1
                if self._parked_mode_unstable_count >= 5:
                    self._enter_fault("parked_state_lost")
                    return []
                return []
            self._parked_mode_unstable_count = 0
            if self._actual_motion is ActualMotionState.MOVING:
                if self._moving_sample_count >= self._moving_fault_samples:
                    self._enter_fault("parked_state_lost")
                    return [self._effect(
                        "MoveZero", reason="parked_state_lost")]
                return [self._effect(
                    "MoveZero", reason="parked_motion_unconfirmed")]
            if self._fault == "telemetry_stale":
                self._fault = None
        return []

    @staticmethod
    def _telemetry_follows(previous: Telemetry, current: Telemetry) -> bool:
        """Order samples across both packets and publisher process epochs.

        ``nx_sensor_node`` derives ``source_stamp`` from wall-clock receipt of
        LowState, which remains monotonic when that publisher restarts, while
        its sample counter restarts at zero.  A lower counter with a strictly
        newer timestamp therefore begins a new valid epoch.  An older source
        timestamp, or an identical counter, remains a duplicate/out-of-order
        packet and is rejected.  The timestamp guard also prevents a delayed
        high-counter packet from the old epoch reclaiming the stream.
        """
        if current.source_stamp < previous.source_stamp:
            return False
        if current.sample_id == previous.sample_id:
            return False
        if current.sample_id > previous.sample_id:
            return True
        return current.source_stamp > previous.source_stamp

    def _health_fault(self, telemetry: Telemetry) -> Optional[str]:
        if not telemetry.is_fresh(self._now(), self._telemetry_max_age):
            return "telemetry_stale"
        if telemetry.error_code != 0:
            return "robot_error"
        if telemetry.motor_fault:
            return "motor_fault"
        if (abs(telemetry.roll) > self._max_attitude_rad
                or abs(telemetry.pitch) > self._max_attitude_rad):
            return "attitude_unsafe"
        if telemetry.battery_soc < self._minimum_battery_soc:
            return "battery_low"
        if self._physical_mode is PhysicalMode.UNKNOWN:
            return "unknown_physical_mode"
        return None

    def _telemetry_is_fresh(self) -> bool:
        return (
            self._last_telemetry is not None
            and self._last_telemetry.is_fresh(
                self._now(), self._telemetry_max_age)
        )

    def _begin_transition(self, operation: str) -> int:
        self._transition_counter += 1
        self._transition_id = self._transition_counter
        self._transition_operation = str(operation)
        self._transition_started_at = self._now()
        self._transition_deadline = (
            self._transition_started_at + self._transition_timeout)
        return self._transition_id

    def _clear_transition(self) -> None:
        self._transition_id = None
        self._transition_operation = None
        self._transition_started_at = None
        self._transition_deadline = None

    def _enter_fault(self, reason: str) -> None:
        self._session = SessionState.FAULT
        self._owner = None
        self._fault = str(reason)
        self._clear_transition()

    def _effect(
        self,
        operation: str,
        *,
        transition_id: Optional[int] = None,
        reason: Optional[str] = None,
    ) -> Effect:
        sequence = self._next_sequence()
        return Effect(
            operation=str(operation),
            sequence=sequence,
            transition_id=transition_id,
            reason=reason,
        )

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence
