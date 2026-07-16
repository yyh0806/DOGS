"""Serialized boundary around the Unitree high-level sport clients."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable, Mapping, Optional

try:
    from .motion_types import CommandReceipt, Effect
except ImportError:  # Direct-file compatibility deployment on the NX.
    from motion_types import CommandReceipt, Effect


@dataclass(frozen=True)
class InitializationResult:
    code: int
    motion_service: Optional[str]
    raw_mode: Any = None


class UnitreeSportAdapter:
    """Own one sport client and serialize every SDK operation."""

    _NO_ARG_OPERATIONS = frozenset({
        "StandUp",
        "BalanceStand",
        "StopMove",
    })

    def __init__(
        self,
        sport_client: Any,
        motion_switcher: Any,
        *,
        sleep: Callable[[float], None] = time.sleep,
        lease_settle_seconds: float = 0.20,
    ) -> None:
        self._sport = sport_client
        self._switcher = motion_switcher
        self._sleep = sleep
        self._lease_settle_seconds = max(0.0, float(lease_settle_seconds))
        self._lock = threading.RLock()
        self._initialized = False

    @property
    def sport_client(self) -> Any:
        return self._sport

    def initialize(self) -> InitializationResult:
        """Acquire the client, zero retained velocity, then read service mode."""

        with self._lock:
            if self._initialized:
                return self.check_motion_service()
            self._sport.Init()
            self._move_zero_twice()
            self._sleep(self._lease_settle_seconds)
            self._move_zero_twice()
            self._switcher.Init()
            result = self.check_motion_service()
            self._initialized = result.code == 0
            return result

    def check_motion_service(self) -> InitializationResult:
        """Read MotionSwitcher state; this adapter never selects a mode."""

        with self._lock:
            code, raw = self._switcher.CheckMode()
            normalized_code = self._normalize_code(code)
            service = self._extract_service_name(raw) if normalized_code == 0 else None
            return InitializationResult(normalized_code, service, raw)

    def execute(self, effect: Effect) -> CommandReceipt:
        with self._lock:
            operation = effect.operation
            if operation == "MoveZero":
                code = self._sport.Move(0.0, 0.0, 0.0)
            elif operation == "Move":
                if len(effect.arguments) != 3:
                    raise ValueError("Move effect requires exactly three arguments")
                code = self._sport.Move(*effect.arguments)
            elif operation in self._NO_ARG_OPERATIONS:
                method = getattr(self._sport, operation, None)
                if not callable(method):
                    raise RuntimeError(f"SportClient does not provide {operation}")
                code = method()
            else:
                raise ValueError(f"unsupported SDK effect: {operation}")
            return CommandReceipt(
                operation=operation,
                code=self._normalize_code(code),
                sequence=effect.sequence,
                physical_confirmed=False,
            )

    def _move_zero_twice(self) -> None:
        self._sport.Move(0.0, 0.0, 0.0)
        self._sport.Move(0.0, 0.0, 0.0)

    @staticmethod
    def _normalize_code(value: Any) -> int:
        if value is None:
            return 0
        try:
            return int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeError(f"invalid Unitree SDK return code: {value!r}") from exc

    @staticmethod
    def _extract_service_name(raw: Any) -> Optional[str]:
        if not isinstance(raw, Mapping):
            return None
        for key in ("name", "alias"):
            value = raw.get(key)
            if value is not None and str(value).strip():
                return str(value).strip()
        mode = raw.get("mode")
        if isinstance(mode, Mapping):
            return UnitreeSportAdapter._extract_service_name(mode)
        return None
