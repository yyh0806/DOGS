"""Bounded durable evidence for Go2W motion and firmware safety incidents."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Optional
import uuid


class SafetyEventRecorder:
    """Write state changes and safety-critical events to rotating JSONL."""

    _STATE_FIELDS = (
        "mode",
        "error_code",
        "gait_type",
        "progress",
        "velocity",
        "yaw_speed",
        "position",
        "wheel_dq",
        "roll",
        "pitch",
        "motor_lost",
        "motor_mode",
        "motor_temp",
        "foot_force",
        "battery_soc",
        "bms_status",
        "power_v",
        "power_a",
        "level_flag",
        "bit_flag",
    )

    def __init__(
        self,
        path: object,
        *,
        max_bytes: int = 4 * 1024 * 1024,
        backups: int = 4,
        process_epoch: Optional[str] = None,
        monotonic=time.monotonic,
        normal_interval: float = 1.0,
    ) -> None:
        self._path = Path(path)
        self._max_bytes = max(256, int(max_bytes))
        self._backups = max(1, int(backups))
        self._process_epoch = process_epoch or str(uuid.uuid4())
        self._monotonic = monotonic
        self._normal_interval = max(0.0, float(normal_interval))
        self._lock = threading.RLock()
        self._last_state_fingerprint: Optional[str] = None
        self._last_state_critical_fingerprint: Optional[str] = None
        self._last_state_written_at: Optional[float] = None
        self._last_command_fingerprint: Optional[str] = None

    @property
    def process_epoch(self) -> str:
        return self._process_epoch

    def record_state(self, snapshot: Mapping[str, Any]) -> None:
        state = {
            field: self._json_value(snapshot[field])
            for field in self._STATE_FIELDS if field in snapshot
        }
        fingerprint = self._fingerprint(state)
        critical_fingerprint = self._fingerprint(
            self._critical_state(state))
        safety = self._is_safety_state(state)
        now = float(self._monotonic())
        with self._lock:
            if fingerprint == self._last_state_fingerprint:
                return
            critical_changed = (
                critical_fingerprint
                != self._last_state_critical_fingerprint
            )
            if (not critical_changed
                    and self._last_state_written_at is not None
                    and now - self._last_state_written_at
                    < self._normal_interval):
                return
            self._last_state_fingerprint = fingerprint
            self._last_state_critical_fingerprint = critical_fingerprint
            self._last_state_written_at = now
            self._write({
                "kind": "state",
                "safety": safety,
                "state": state,
            }, durable=safety)

    def record_command(
        self,
        operation: str,
        code: int,
        *,
        arguments: object = (),
        reason: Optional[str] = None,
        error: Optional[str] = None,
    ) -> None:
        normalized_arguments = tuple(
            self._json_value(value) for value in tuple(arguments))
        row = {
            "kind": "command",
            "operation": str(operation),
            "code": int(code),
            "arguments": list(normalized_arguments),
            "reason": str(reason) if reason is not None else None,
            "error": str(error)[:512] if error is not None else None,
        }
        fingerprint = self._fingerprint(row)
        moving = any(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            and abs(float(value)) > 1e-9
            for value in normalized_arguments
        )
        durable = (
            int(code) != 0
            or moving
            or str(operation) not in {"Move", "MoveZero"}
        )
        with self._lock:
            if fingerprint == self._last_command_fingerprint and not durable:
                return
            self._last_command_fingerprint = fingerprint
            self._write(row, durable=durable)

    def record_event(
        self,
        event: str,
        details: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self._write({
            "kind": "event",
            "event": str(event),
            "details": self._json_value(dict(details or {})),
        }, durable=True)

    def _write(self, row: Mapping[str, Any], *, durable: bool) -> None:
        envelope = {
            "wall_time": datetime.now(timezone.utc).isoformat(),
            "monotonic": round(float(self._monotonic()), 6),
            "process_epoch": self._process_epoch,
            **dict(row),
        }
        data = (json.dumps(
            envelope,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ) + "\n").encode("utf-8")
        with self._lock:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            current_size = self._path.stat().st_size if self._path.exists() else 0
            if current_size and current_size + len(data) > self._max_bytes:
                self._rotate()
            with self._path.open("ab", buffering=0) as stream:
                stream.write(data)
                if durable:
                    stream.flush()
                    os.fsync(stream.fileno())

    def _rotate(self) -> None:
        oldest = self._backup_path(self._backups)
        oldest.unlink(missing_ok=True)
        for index in range(self._backups - 1, 0, -1):
            source = self._backup_path(index)
            if source.exists():
                source.replace(self._backup_path(index + 1))
        if self._path.exists():
            self._path.replace(self._backup_path(1))

    def _backup_path(self, index: int) -> Path:
        return self._path.with_name(f"{self._path.name}.{index}")

    @staticmethod
    def _fingerprint(value: object) -> str:
        return json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )

    @staticmethod
    def _is_safety_state(state: Mapping[str, Any]) -> bool:
        try:
            error_code = int(state.get("error_code", 0))
        except (TypeError, ValueError, OverflowError):
            error_code = 1
        try:
            roll = abs(float(state.get("roll", 0.0)))
            pitch = abs(float(state.get("pitch", 0.0)))
        except (TypeError, ValueError, OverflowError):
            roll, pitch = math.inf, math.inf
        try:
            mode = int(state.get("mode", -1))
        except (TypeError, ValueError, OverflowError):
            mode = -1
        return (
            error_code != 0
            or roll > 0.70
            or pitch > 0.70
            or mode in {5, 7}
        )

    @staticmethod
    def _critical_state(state: Mapping[str, Any]) -> Mapping[str, Any]:
        def unsafe_attitude(field: str) -> bool:
            try:
                return abs(float(state.get(field, 0.0))) > 0.70
            except (TypeError, ValueError, OverflowError):
                return True

        return {
            "mode": state.get("mode"),
            "error_code": state.get("error_code"),
            "roll_unsafe": unsafe_attitude("roll"),
            "pitch_unsafe": unsafe_attitude("pitch"),
            "bms_status": state.get("bms_status"),
            "level_flag": state.get("level_flag"),
            "bit_flag": state.get("bit_flag"),
        }

    @classmethod
    def _json_value(cls, value: Any) -> Any:
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else str(value)
        if isinstance(value, Mapping):
            return {str(key): cls._json_value(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_value(item) for item in value]
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError):
            return str(value)
        return numeric if math.isfinite(numeric) else str(numeric)
