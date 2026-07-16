"""Synchronous motion-controller adapter for the stable local Sport gateway."""

from __future__ import annotations

import os
import socket
import threading
from typing import Optional

try:
    from .motion_types import CommandReceipt, Effect
    from .sport_gateway_protocol import (
        GatewayRequest,
        MAX_FRAME_BYTES,
        PROTOCOL_VERSION,
        ProtocolError,
        decode_frame,
        decode_request,
        decode_response,
        encode_frame,
    )
    from .unitree_sport_adapter import InitializationResult
except ImportError:  # Direct-file compatibility deployment on the NX.
    from motion_types import CommandReceipt, Effect
    from sport_gateway_protocol import (
        GatewayRequest,
        MAX_FRAME_BYTES,
        PROTOCOL_VERSION,
        ProtocolError,
        decode_frame,
        decode_request,
        decode_response,
        encode_frame,
    )
    from unitree_sport_adapter import InitializationResult


class SportGatewayClient:
    """Preserve the old adapter contract without owning a Unitree lease."""

    _TRANSPORT_ERROR_CODE = 599
    _EFFECT_OPERATIONS = frozenset({
        "MoveZero", "Move", "StandUp", "BalanceStand", "StopMove",
    })

    def __init__(
        self,
        socket_path: object = "/run/go2w-sport-gateway/sport.sock",
        *,
        socket_family: Optional[int] = None,
        timeout: float = 0.8,
    ) -> None:
        if socket_family is None:
            if not hasattr(socket, "AF_UNIX"):
                raise RuntimeError("AF_UNIX is required for the production client")
            socket_family = socket.AF_UNIX
        self._address = (
            str(socket_path)
            if int(socket_family) == getattr(socket, "AF_UNIX", -1)
            else socket_path
        )
        self._socket_family = int(socket_family)
        self._timeout = max(0.05, float(timeout))
        self._socket: Optional[socket.socket] = None
        self._lock = threading.RLock()
        self._sequence = 0
        self._initialized = False

    def initialize(self) -> InitializationResult:
        with self._lock:
            result = self.check_motion_service()
            self._initialized = result.code == 0
            return result

    def check_motion_service(self) -> InitializationResult:
        response = self._exchange("CheckMode", (), reconnect_read_only=True)
        return InitializationResult(
            code=response.code,
            motion_service=response.motion_service,
            raw_mode=response.to_dict(),
        )

    def execute(self, effect: Effect) -> CommandReceipt:
        operation = str(effect.operation)
        if operation not in self._EFFECT_OPERATIONS:
            raise ValueError(f"unsupported SDK effect: {operation}")
        arguments = () if operation == "MoveZero" else effect.arguments
        try:
            response = self._exchange(
                operation, arguments, reconnect_read_only=False)
            code = response.code
        except RuntimeError:
            # A motion request is never replayed: the gateway may have applied
            # it before the response was lost.  Return a failed receipt so the
            # reducer revokes motion authority while the gateway watchdog
            # independently holds zero velocity.
            code = self._TRANSPORT_ERROR_CODE
        return CommandReceipt(
            operation=operation,
            code=code,
            sequence=effect.sequence,
            physical_confirmed=False,
        )

    def close(self) -> None:
        with self._lock:
            self._close_socket()
            self._initialized = False

    def _exchange(
        self,
        operation: str,
        arguments: object,
        *,
        reconnect_read_only: bool,
    ):
        with self._lock:
            request_id = self._next_request_id()
            request = decode_request({
                "version": PROTOCOL_VERSION,
                "request_id": request_id,
                "operation": operation,
                "arguments": list(arguments),
            })
            attempts = 2 if reconnect_read_only else 1
            last_error: Optional[Exception] = None
            for _ in range(attempts):
                try:
                    connection = self._ensure_socket()
                    connection.sendall(encode_frame(request.to_dict()))
                    payload = decode_frame(self._read_frame(connection))
                    response = decode_response(
                        payload, expected_request_id=request_id)
                    if response.operation not in {operation, "Error"}:
                        raise ProtocolError("response operation mismatch")
                    return response
                except (EOFError, OSError, ProtocolError) as exc:
                    last_error = exc
                    self._close_socket()
            raise RuntimeError(
                f"gateway transport error during {operation}: {last_error}")

    def _ensure_socket(self) -> socket.socket:
        if self._socket is not None:
            return self._socket
        connection = socket.socket(self._socket_family, socket.SOCK_STREAM)
        connection.settimeout(self._timeout)
        try:
            connection.connect(self._address)
        except Exception:
            connection.close()
            raise
        self._socket = connection
        return connection

    def _read_frame(self, connection: socket.socket) -> bytes:
        frame = bytearray()
        while True:
            chunk = connection.recv(min(1024, MAX_FRAME_BYTES + 1))
            if not chunk:
                raise EOFError("gateway disconnected")
            frame.extend(chunk)
            newline = frame.find(b"\n")
            if newline >= 0:
                framed = bytes(frame[:newline + 1])
                if len(framed) > MAX_FRAME_BYTES:
                    raise ProtocolError("frame exceeds maximum size")
                return framed
            if len(frame) > MAX_FRAME_BYTES:
                raise ProtocolError("frame exceeds maximum size")

    def _close_socket(self) -> None:
        connection = self._socket
        self._socket = None
        if connection is None:
            return
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            connection.close()
        except OSError:
            pass

    def _next_request_id(self) -> str:
        self._sequence += 1
        return f"{os.getpid()}-{self._sequence}"
