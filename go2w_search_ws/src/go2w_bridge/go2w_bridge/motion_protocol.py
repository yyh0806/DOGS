"""Versioned ROS string-message protocol for motion intent and status."""

from __future__ import annotations

from dataclasses import dataclass
import json
import re
from typing import Any, Dict, Mapping, Optional
from uuid import uuid4

try:
    from .motion_machine import MotionSnapshot
    from .motion_types import MotionIntent, Telemetry
except ImportError:  # Direct-file compatibility deployment on the NX.
    from motion_machine import MotionSnapshot
    from motion_types import MotionIntent, Telemetry


INTENT_SCHEMA_VERSION = 1
STATUS_SCHEMA_VERSION = 4
FEEDBACK_SCHEMA_VERSION = 2
_IDENTITY_RE = re.compile(r"^[A-Za-z0-9_.:@/-]+$")
_LEGACY_INTENTS = {
    "manual_start": MotionIntent.START_MANUAL,
    "nav_start": MotionIntent.START_NAV,
    "manual_stop": MotionIntent.PARK,
    "nav_stop": MotionIntent.PARK,
    "park": MotionIntent.PARK,
    "estop": MotionIntent.ESTOP,
}


class MotionProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class MotionIntentEnvelope:
    request_id: str
    intent: MotionIntent
    source: str
    legacy: bool = False

    @classmethod
    def parse(cls, payload: object) -> "MotionIntentEnvelope":
        if isinstance(payload, Mapping):
            return cls._from_mapping(payload)
        if not isinstance(payload, str):
            raise MotionProtocolError("motion intent must be JSON text or mapping")
        text = payload.strip()
        if text in _LEGACY_INTENTS:
            return cls(
                request_id=f"legacy-{uuid4()}",
                intent=_LEGACY_INTENTS[text],
                source="legacy",
                legacy=True,
            )
        if not text:
            raise MotionProtocolError("motion intent is empty")
        try:
            decoded = json.loads(text)
        except json.JSONDecodeError as exc:
            raise MotionProtocolError("unknown legacy motion intent") from exc
        if not isinstance(decoded, Mapping):
            raise MotionProtocolError("motion intent JSON must be an object")
        return cls._from_mapping(decoded)

    @classmethod
    def _from_mapping(cls, payload: Mapping[str, object]) -> "MotionIntentEnvelope":
        if payload.get("schema_version") != INTENT_SCHEMA_VERSION:
            raise MotionProtocolError("unsupported motion intent schema_version")
        request_id = cls._identity(payload.get("request_id"), "request_id", 128)
        source = cls._identity(payload.get("source"), "source", 64)
        try:
            intent = MotionIntent(str(payload.get("intent", "")))
        except ValueError as exc:
            raise MotionProtocolError("unknown motion intent") from exc
        return cls(request_id=request_id, intent=intent, source=source)

    @staticmethod
    def _identity(value: object, field: str, limit: int) -> str:
        text = str(value).strip() if value is not None else ""
        if not text or len(text) > limit or not _IDENTITY_RE.fullmatch(text):
            raise MotionProtocolError(f"invalid {field}")
        return text

    def to_dict(self) -> Dict[str, object]:
        return {
            "schema_version": INTENT_SCHEMA_VERSION,
            "request_id": self.request_id,
            "intent": self.intent.value,
            "source": self.source,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"), sort_keys=True)


def motion_status_dict(
    snapshot: MotionSnapshot,
    *,
    release_id: str,
    raw: Optional[Mapping[str, Any]] = None,
    legacy_deprecation_count: int = 0,
) -> Dict[str, object]:
    transition = None
    if snapshot.transition_id is not None:
        transition = {
            "id": snapshot.transition_id,
            "operation": snapshot.transition_operation,
        }
    return {
        "schema_version": STATUS_SCHEMA_VERSION,
        "release_id": str(release_id),
        "session": snapshot.session.value,
        "physical_mode": snapshot.physical_mode.value,
        "actual_motion": snapshot.actual_motion.value,
        "velocity_authorized": snapshot.velocity_authorized,
        "telemetry_fresh": snapshot.telemetry_fresh,
        "motion_service": snapshot.motion_service,
        "owner": snapshot.owner,
        "transition": transition,
        "fault": snapshot.fault,
        "last_sample_id": snapshot.last_sample_id,
        "rejected_samples": snapshot.rejected_samples,
        "legacy_deprecation_count": int(legacy_deprecation_count),
        "raw": dict(raw or {}),
    }


def decode_wheel_feedback(
    payload: object,
    *,
    received_at: float,
    motion_service: Optional[str],
) -> Telemetry:
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("wheel feedback is not valid JSON") from exc
    if not isinstance(payload, Mapping):
        raise ValueError("wheel feedback must be an object")
    if payload.get("schema_version") != FEEDBACK_SCHEMA_VERSION:
        raise ValueError("schema_version must be 2")
    required = (
        "sample_id",
        "source_stamp",
        "wheel_dq",
        "battery_soc",
        "sport_mode",
        "sport_error_code",
        "roll",
        "pitch",
        "motor_lost",
    )
    for field in required:
        if field not in payload:
            raise ValueError(f"missing {field}")
    motor_lost = payload["motor_lost"]
    if (not isinstance(motor_lost, (list, tuple))
            or len(motor_lost) != 4):
        raise ValueError("motor_lost must contain four values")
    try:
        lost_values = tuple(int(value) for value in motor_lost)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("motor_lost must contain four integers") from exc

    return Telemetry(
        sample_id=int(payload["sample_id"]),
        source_stamp=float(payload["source_stamp"]),
        received_at=float(received_at),
        raw_mode=int(payload["sport_mode"]),
        wheel_dq=tuple(float(value) for value in payload["wheel_dq"]),
        battery_soc=float(payload["battery_soc"]),
        error_code=int(payload["sport_error_code"]),
        roll=float(payload["roll"]),
        pitch=float(payload["pitch"]),
        motion_service=motion_service,
        # Unitree exposes SportModeState.error_code as the robot error signal.
        # MotorState.lost is a cumulative uint32 communication-loss counter;
        # a historical non-zero value is diagnostic data, not a live motor
        # fault.  Freshness and sport_error_code remain the fail-closed health
        # gates while the original counters stay in the feedback payload.
        motor_fault=False,
    )


def build_wheel_feedback_payload(
    *,
    sample_id: int,
    source_stamp: float,
    wheel_dq: object,
    battery_soc: float,
    bms_status: int,
    sport_mode: int,
    sport_error_code: int,
    roll: float,
    pitch: float,
    motor_lost: object,
    extras: Optional[Mapping[str, Any]] = None,
) -> Dict[str, object]:
    payload: Dict[str, object] = {
        "schema_version": FEEDBACK_SCHEMA_VERSION,
        "sample_id": int(sample_id),
        "source_stamp": float(source_stamp),
        "wheel_dq": [float(value) for value in wheel_dq],
        "battery_soc": float(battery_soc),
        "bms_status": int(bms_status),
        "sport_mode": int(sport_mode),
        "sport_error_code": int(sport_error_code),
        "roll": float(roll),
        "pitch": float(pitch),
        "motor_lost": [int(value) for value in motor_lost],
    }
    if extras:
        for key, value in extras.items():
            if key not in payload:
                payload[str(key)] = value
    return payload
