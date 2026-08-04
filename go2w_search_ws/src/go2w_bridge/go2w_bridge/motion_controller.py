"""Single-thread motion orchestration between the reducer and SDK adapter."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Any, Dict, Optional, Tuple

try:
    from .motion_machine import Go2WMotionMachine
    from .motion_protocol import MotionIntentEnvelope, decode_wheel_feedback
    from .motion_safety import (
        DriveExecutionWatchdog,
        ScanFreshnessWatchdog,
        compensate_pure_turn_creep,
        motion_command_timed_out,
    )
    from .motion_types import Effect, SessionState
except ImportError:  # Direct-file compatibility deployment on the NX.
    from motion_machine import Go2WMotionMachine
    from motion_protocol import MotionIntentEnvelope, decode_wheel_feedback
    from motion_safety import (
        DriveExecutionWatchdog,
        ScanFreshnessWatchdog,
        compensate_pure_turn_creep,
        motion_command_timed_out,
    )
    from motion_types import Effect, SessionState


@dataclass(frozen=True)
class VelocitySample:
    owner: str
    velocity: Tuple[float, float, float]
    received_at: float


class MotionController:
    """Execute all SDK effects from one actor thread."""

    def __init__(
        self,
        *,
        machine: Go2WMotionMachine,
        scan_watchdog: ScanFreshnessWatchdog,
        drive_watchdog: DriveExecutionWatchdog,
        clock,
        manual_timeout: float,
        nav_timeout: float,
        max_vx: float,
        max_vy: float,
        max_vyaw: float,
        turn_creep_gain: float = 1.0,
        turn_creep_maximum: float = 0.15,
        turn_linear_epsilon: float = 0.02,
        turn_angular_threshold: float = 0.05,
    ) -> None:
        self.machine = machine
        self.scan_watchdog = scan_watchdog
        self.drive_watchdog = drive_watchdog
        self._clock = clock
        self._manual_timeout = float(manual_timeout)
        self._nav_timeout = float(nav_timeout)
        self._limits = (
            abs(float(max_vx)), abs(float(max_vy)), abs(float(max_vyaw)))
        self._turn_creep = {
            "gain": float(turn_creep_gain),
            "maximum": float(turn_creep_maximum),
            "linear_epsilon": float(turn_linear_epsilon),
            "angular_threshold": float(turn_angular_threshold),
        }
        self._adapter = None
        self._motion_service: Optional[str] = None
        self._actor_thread_id: Optional[int] = None
        self._commands: Dict[str, VelocitySample] = {}
        self._legacy_deprecation_count = 0
        self._last_feedback_payload: Dict[str, Any] = {}
        self._last_receipt = None

    @property
    def sdk_ready(self) -> bool:
        return self._adapter is not None and self._motion_service == "ai-w"

    @property
    def legacy_deprecation_count(self) -> int:
        return self._legacy_deprecation_count

    @property
    def last_feedback_payload(self) -> Dict[str, Any]:
        return dict(self._last_feedback_payload)

    @property
    def last_receipt(self):
        return self._last_receipt

    def attach_adapter(self, adapter, motion_service: Optional[str]):
        if self._adapter is not None and adapter is not self._adapter:
            raise RuntimeError("motion controller already owns an SDK adapter")
        self._adapter = adapter
        self._motion_service = (
            str(motion_service).strip() if motion_service else None)
        self._actor_thread_id = threading.get_ident()
        effects = self.machine.sdk_ready(self._motion_service)
        self._execute(effects)

    def observe_feedback(self, payload: object) -> None:
        if isinstance(payload, str):
            import json
            decoded = json.loads(payload)
        else:
            decoded = dict(payload)
        telemetry = decode_wheel_feedback(
            decoded,
            received_at=float(self._clock()),
            motion_service=self._motion_service,
        )
        self._last_feedback_payload = decoded
        self.drive_watchdog.observe_feedback(
            telemetry.wheel_dq,
            telemetry.battery_soc,
            int(decoded.get("bms_status", 0)),
            sport_mode=telemetry.raw_mode,
            sport_progress=decoded.get("sport_progress"),
            gait_type=decoded.get("gait_type"),
        )
        self._execute(self.machine.observe(telemetry))

    def observe_scan(self, message: object) -> bool:
        return self.scan_watchdog.observe_scan(message)

    def handle_intent(self, envelope: object) -> None:
        if not isinstance(envelope, MotionIntentEnvelope):
            envelope = MotionIntentEnvelope.parse(envelope)
        if envelope.legacy:
            self._legacy_deprecation_count += 1
        if envelope.intent.value == "start_nav":
            self.scan_watchdog.reset_nav_guard()
        effects = self.machine.request(
            envelope.intent,
            scan_fresh=self.scan_watchdog.is_fresh(),
        )
        self._execute(effects)

    def update_velocity(self, owner: str, velocity: object) -> None:
        owner = str(owner).strip().lower()
        if owner not in {"manual", "nav"}:
            return
        try:
            values = tuple(float(value) for value in velocity)
        except (TypeError, ValueError, OverflowError):
            values = (0.0, 0.0, 0.0)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            values = (0.0, 0.0, 0.0)
        self._commands[owner] = VelocitySample(
            owner=owner,
            velocity=values,
            received_at=float(self._clock()),
        )

    def tick(self) -> None:
        now = float(self._clock())
        self._execute(self.machine.tick(now))
        snapshot = self.machine.snapshot()
        if snapshot.session not in {
                SessionState.MANUAL_ACTIVE, SessionState.NAV_ACTIVE}:
            return
        owner = snapshot.owner or ""
        sample = self._commands.get(owner)
        timed_out = (
            sample is None
            or motion_command_timed_out(
                True,
                owner,
                sample.received_at if sample is not None else -1.0,
                now,
                self._manual_timeout,
                self._nav_timeout,
            )
        )
        velocity = (0.0, 0.0, 0.0) if timed_out else sample.velocity
        velocity = self._clamp_velocity(velocity)
        if owner == "nav":
            # Autonomous goals behind the robot must rotate first.  Keep
            # explicit manual reverse available, but never let an unexpected
            # planner/recovery command drive the robot backwards.
            velocity = (max(0.0, velocity[0]), velocity[1], velocity[2])
            velocity = self.scan_watchdog.filter_nav_velocity(velocity)
        velocity = compensate_pure_turn_creep(velocity, **self._turn_creep)
        effect = self.machine.command_velocity(
            owner,
            velocity,
            scan_fresh=(self.scan_watchdog.is_fresh() if owner == "nav" else True),
        )
        fault = self.drive_watchdog.evaluate(
            effect.arguments if effect.operation == "Move"
            else DriveExecutionWatchdog.ZERO)
        if fault:
            self._execute(self.machine.report_fault(fault))
            return
        self._execute([effect])

    def shutdown(self) -> None:
        if self._adapter is None:
            return
        zero = self.machine.command_velocity(
            self.machine.snapshot().owner or "safety",
            (0.0, 0.0, 0.0),
            scan_fresh=False,
        )
        self._execute([zero, Effect("MoveZero", zero.sequence + 1)])

    def _execute(self, effects) -> None:
        if not effects or self._adapter is None:
            return
        if (self._actor_thread_id is not None
                and threading.get_ident() != self._actor_thread_id):
            raise RuntimeError("SDK effect executed outside the motion actor thread")
        for effect in effects:
            receipt = self._adapter.execute(effect)
            self._last_receipt = receipt
            self.machine.record_receipt(receipt)

    def _clamp_velocity(self, velocity):
        values = tuple(float(value) for value in velocity)
        clamped = []
        for value, limit in zip(values, self._limits):
            if limit <= 0.0:
                clamped.append(0.0)
            else:
                clamped.append(max(-limit, min(limit, value)))
        return tuple(clamped)
