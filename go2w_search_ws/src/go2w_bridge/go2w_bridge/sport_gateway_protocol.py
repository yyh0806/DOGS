"""Strict local protocol between the motion policy and Sport lease gateway."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Optional, Tuple


PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 4096

_ARGUMENT_COUNTS = {
    "CheckMode": 0,
    "MoveZero": 0,
    "Move": 3,
    "StandUp": 0,
    "BalanceStand": 0,
    "StopMove": 0,
}
_RESPONSE_OPERATIONS = frozenset({*_ARGUMENT_COUNTS, "Error"})


class ProtocolError(ValueError):
    """A local gateway message violated the closed protocol schema."""


@dataclass(frozen=True)
class GatewayRequest:
    version: int
    request_id: str
    operation: str
    arguments: Tuple[float, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "request_id": self.request_id,
            "operation": self.operation,
            "arguments": list(self.arguments),
        }


@dataclass(frozen=True)
class GatewayResponse:
    version: int
    request_id: str
    operation: str
    code: int
    motion_service: Optional[str] = None
    error: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "version": self.version,
            "request_id": self.request_id,
            "operation": self.operation,
            "code": self.code,
            "motion_service": self.motion_service,
        }
        if self.error is not None:
            payload["error"] = self.error
        return payload


def _version(value: object) -> int:
    if isinstance(value, bool) or value != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    return PROTOCOL_VERSION


def _request_id(value: object) -> str:
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ProtocolError("request_id must be a non-empty string")
    if any(character in value for character in ("\r", "\n", "\x00")):
        raise ProtocolError("request_id contains forbidden characters")
    return value


def _operation(value: object, *, response: bool = False) -> str:
    allowed = _RESPONSE_OPERATIONS if response else _ARGUMENT_COUNTS
    if not isinstance(value, str) or value not in allowed:
        raise ProtocolError(f"unsupported operation: {value!r}")
    return value


def decode_request(payload: object) -> GatewayRequest:
    if not isinstance(payload, Mapping):
        raise ProtocolError("request must be a JSON object")
    version = _version(payload.get("version"))
    request_id = _request_id(payload.get("request_id"))
    operation = _operation(payload.get("operation"))
    arguments = payload.get("arguments")
    if not isinstance(arguments, (list, tuple)):
        raise ProtocolError("arguments must be an array")
    required = _ARGUMENT_COUNTS[operation]
    if len(arguments) != required:
        raise ProtocolError(f"{operation} requires {required} arguments")
    normalized = []
    for value in arguments:
        if isinstance(value, bool):
            raise ProtocolError("arguments must contain finite numbers")
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ProtocolError(
                "arguments must contain finite numbers") from exc
        if not math.isfinite(number):
            raise ProtocolError("arguments must contain finite numbers")
        normalized.append(number)
    return GatewayRequest(
        version=version,
        request_id=request_id,
        operation=operation,
        arguments=tuple(normalized),
    )


def decode_response(
    payload: object,
    *,
    expected_request_id: Optional[str] = None,
) -> GatewayResponse:
    if not isinstance(payload, Mapping):
        raise ProtocolError("response must be a JSON object")
    version = _version(payload.get("version"))
    request_id = _request_id(payload.get("request_id"))
    if expected_request_id is not None and request_id != expected_request_id:
        raise ProtocolError("request_id mismatch")
    operation = _operation(payload.get("operation"), response=True)
    code = payload.get("code")
    if isinstance(code, bool) or not isinstance(code, int):
        raise ProtocolError("response requires an integer code")
    motion_service = payload.get("motion_service")
    if motion_service is not None:
        if not isinstance(motion_service, str) or not motion_service.strip():
            raise ProtocolError("motion_service must be non-empty or null")
        motion_service = motion_service.strip()
    error = payload.get("error")
    if error is not None and not isinstance(error, str):
        raise ProtocolError("error must be a string or null")
    return GatewayResponse(
        version=version,
        request_id=request_id,
        operation=operation,
        code=code,
        motion_service=motion_service,
        error=error,
    )


def encode_frame(payload: Mapping[str, Any]) -> bytes:
    try:
        frame = json.dumps(
            dict(payload),
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8") + b"\n"
    except (TypeError, ValueError, OverflowError) as exc:
        raise ProtocolError("payload is not canonical JSON") from exc
    if len(frame) > MAX_FRAME_BYTES:
        raise ProtocolError("frame exceeds maximum size")
    return frame


def decode_frame(frame: bytes) -> Mapping[str, Any]:
    if not isinstance(frame, bytes):
        raise ProtocolError("frame must be bytes")
    if len(frame) > MAX_FRAME_BYTES:
        raise ProtocolError("frame exceeds maximum size")
    if not frame.endswith(b"\n"):
        raise ProtocolError("frame must end with newline")
    try:
        payload = json.loads(frame[:-1].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("frame is not valid UTF-8 JSON") from exc
    if not isinstance(payload, Mapping):
        raise ProtocolError("frame payload must be a JSON object")
    return payload
