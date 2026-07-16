"""Stable Unix-socket owner for one leased Unitree SportClient."""

from __future__ import annotations

import json
from pathlib import Path
import queue
import socket
import threading
import time
from typing import Any, Optional

try:
    from .sport_gateway_protocol import (
        GatewayRequest,
        GatewayResponse,
        MAX_FRAME_BYTES,
        PROTOCOL_VERSION,
        ProtocolError,
        decode_frame,
        decode_request,
        encode_frame,
    )
except ImportError:  # Direct-file compatibility deployment on the NX.
    from sport_gateway_protocol import (
        GatewayRequest,
        GatewayResponse,
        MAX_FRAME_BYTES,
        PROTOCOL_VERSION,
        ProtocolError,
        decode_frame,
        decode_request,
        encode_frame,
    )


class SportGatewayServer:
    """Keep the lease owner alive while policy clients are replaced."""

    def __init__(
        self,
        sport_client: Any,
        motion_switcher: Any,
        *,
        socket_path: object,
        socket_family: Optional[int] = None,
        command_timeout: float = 0.25,
        zero_period: float = 0.05,
        clock=time.monotonic,
        recorder: Optional[Any] = None,
    ) -> None:
        self._sport = sport_client
        self._switcher = motion_switcher
        if socket_family is None:
            if not hasattr(socket, "AF_UNIX"):
                raise RuntimeError("AF_UNIX is required for the production gateway")
            socket_family = socket.AF_UNIX
        self._socket_family = int(socket_family)
        self._socket_path = (
            Path(socket_path)
            if self._socket_family == getattr(socket, "AF_UNIX", -1)
            else None
        )
        self._bind_address = (
            str(self._socket_path)
            if self._socket_path is not None else socket_path
        )
        self._bound_address = self._bind_address
        self._command_timeout = max(0.01, float(command_timeout))
        self._zero_period = max(0.01, float(zero_period))
        self._clock = clock
        self._recorder = recorder
        self._requests: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._listen_socket: Optional[socket.socket] = None
        self._connections: set[socket.socket] = set()
        self._connection_lock = threading.Lock()
        self._active_connection: Optional[socket.socket] = None
        self._actor_thread: Optional[threading.Thread] = None
        self._accept_thread: Optional[threading.Thread] = None
        self._actor_ready = threading.Event()
        self._actor_boot_error: Optional[Exception] = None
        self._motion_service: Optional[str] = None
        self._zero_healthy = False

    @property
    def socket_path(self) -> Path:
        if self._socket_path is None:
            raise RuntimeError("the test transport does not use a Unix path")
        return self._socket_path

    @property
    def socket_family(self) -> int:
        return self._socket_family

    @property
    def address(self):
        return self._bound_address

    def start(self) -> None:
        if self._listen_socket is not None:
            return
        if self._socket_path is not None:
            self._socket_path.parent.mkdir(parents=True, exist_ok=True)
            self._socket_path.unlink(missing_ok=True)
        listener = socket.socket(self._socket_family, socket.SOCK_STREAM)
        listener.bind(self._bind_address)
        self._bound_address = listener.getsockname()
        listener.listen(8)
        listener.settimeout(0.1)
        self._listen_socket = listener
        self._actor_thread = threading.Thread(
            target=self._actor_loop,
            name="go2w-sport-gateway-actor",
            daemon=True,
        )
        self._accept_thread = threading.Thread(
            target=self._accept_loop,
            name="go2w-sport-gateway-accept",
            daemon=True,
        )
        self._actor_thread.start()
        if not self._actor_ready.wait(timeout=3.0):
            self.close()
            raise RuntimeError("Sport gateway actor bootstrap timed out")
        if self._actor_boot_error is not None:
            error = self._actor_boot_error
            self.close()
            raise RuntimeError(f"Sport gateway actor bootstrap failed: {error}")
        self._accept_thread.start()
        self._record_event("gateway_start")

    def close(self) -> None:
        if self._stop.is_set():
            return
        actor = self._actor_thread
        if (
            actor is not None
            and actor.is_alive()
            and self._actor_ready.is_set()
            and self._actor_boot_error is None
        ):
            completed: queue.Queue = queue.Queue(maxsize=1)
            self._requests.put(("shutdown", None, completed))
            try:
                completed.get(timeout=1.0)
            except queue.Empty:
                self._record_event("gateway_final_zero_timeout")
        self._stop.set()
        listener = self._listen_socket
        self._listen_socket = None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
        with self._connection_lock:
            connections = tuple(self._connections)
        for connection in connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass
        self._requests.put(("stop", None, None))
        for thread in (self._accept_thread, self._actor_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        if self._socket_path is not None:
            self._socket_path.unlink(missing_ok=True)
        self._record_event("gateway_stop")

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            listener = self._listen_socket
            if listener is None:
                return
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            connection.settimeout(0.1)
            with self._connection_lock:
                self._connections.add(connection)
                owner = self._active_connection is None
                if owner:
                    self._active_connection = connection
            threading.Thread(
                target=self._connection_loop,
                args=(connection, owner),
                name="go2w-sport-gateway-client",
                daemon=True,
            ).start()

    def _connection_loop(
        self, connection: socket.socket, owner: bool,
    ) -> None:
        self._record_event("client_connect" if owner else "client_rejected")
        try:
            if not owner:
                self._reject_secondary(connection)
                return
            while not self._stop.is_set():
                try:
                    frame = self._read_frame(connection)
                except socket.timeout:
                    continue
                except EOFError:
                    return
                except ProtocolError as exc:
                    self._send_error(connection, "protocol-error", 400, exc)
                    return
                try:
                    request = decode_request(decode_frame(frame))
                except ProtocolError as exc:
                    request_id = self._best_effort_request_id(frame)
                    self._send_error(connection, request_id, 400, exc)
                    return
                response_queue: queue.Queue = queue.Queue(maxsize=1)
                self._requests.put(("request", request, response_queue))
                try:
                    response = response_queue.get(timeout=2.0)
                except queue.Empty:
                    self._send_error(
                        connection, request.request_id, 504,
                        RuntimeError("gateway actor timeout"),
                    )
                    return
                connection.sendall(encode_frame(response.to_dict()))
        except (BrokenPipeError, ConnectionError, OSError):
            return
        finally:
            with self._connection_lock:
                self._connections.discard(connection)
                if self._active_connection is connection:
                    self._active_connection = None
            try:
                connection.close()
            except OSError:
                pass
            if owner:
                self._requests.put(("disconnect", None, None))
                self._record_event("client_disconnect")

    def _reject_secondary(self, connection: socket.socket) -> None:
        try:
            frame = self._read_frame(connection)
            request = decode_request(decode_frame(frame))
            response = GatewayResponse(
                version=PROTOCOL_VERSION,
                request_id=request.request_id,
                operation=request.operation,
                code=409,
                motion_service=self._motion_service,
                error="a motion policy client already owns the gateway",
            )
            connection.sendall(encode_frame(response.to_dict()))
        except (EOFError, OSError, ProtocolError) as exc:
            try:
                self._send_error(connection, "protocol-error", 409, exc)
            except OSError:
                pass

    def _read_frame(self, connection: socket.socket) -> bytes:
        frame = bytearray()
        while not self._stop.is_set():
            chunk = connection.recv(min(1024, MAX_FRAME_BYTES + 1))
            if not chunk:
                raise EOFError("client disconnected")
            frame.extend(chunk)
            newline = frame.find(b"\n")
            if newline >= 0:
                framed = bytes(frame[:newline + 1])
                if len(framed) > MAX_FRAME_BYTES:
                    raise ProtocolError("frame exceeds maximum size")
                return framed
            if len(frame) > MAX_FRAME_BYTES:
                raise ProtocolError("frame exceeds maximum size")
        raise EOFError("gateway stopping")

    def _actor_loop(self) -> None:
        try:
            code, raw = self._switcher.CheckMode()
            code = self._normalize_code(code)
            service = self._extract_service_name(raw) if code == 0 else None
            self._motion_service = service
        except Exception as exc:
            self._motion_service = None
            self._record_command(
                "CheckMode", 500, (), reason="gateway_start",
                error=str(exc),
            )
        for _ in range(2):
            self._execute_watchdog_zero(reason="gateway_start")
        self._actor_ready.set()
        last_valid_request = self._clock()
        last_zero = float("-inf")
        while not self._stop.is_set():
            try:
                kind, request, response_queue = self._requests.get(timeout=0.01)
            except queue.Empty:
                kind, request, response_queue = None, None, None
            if kind == "stop":
                return
            if kind == "shutdown":
                self._execute_watchdog_zero(reason="gateway_stop")
                response_queue.put(True)
                return
            if kind == "disconnect":
                last_valid_request = float("-inf")
            elif kind == "request":
                response = self._execute_request(request)
                last_valid_request = self._clock()
                response_queue.put(response)
            now = self._clock()
            if (now - last_valid_request >= self._command_timeout
                    and now - last_zero >= self._zero_period):
                self._execute_watchdog_zero()
                last_zero = now

    def _execute_request(self, request: GatewayRequest) -> GatewayResponse:
        try:
            if request.operation == "CheckMode":
                code, raw = self._switcher.CheckMode()
                code = self._normalize_code(code)
                service = self._extract_service_name(raw) if code == 0 else None
                self._motion_service = service
                error = None
                if code == 0 and service == "ai-w" and not self._zero_healthy:
                    code = 503
                    error = "gateway zero hold is not healthy"
                return GatewayResponse(
                    version=PROTOCOL_VERSION,
                    request_id=request.request_id,
                    operation=request.operation,
                    code=code,
                    motion_service=service,
                    error=error,
                )
            if self._motion_service != "ai-w":
                return GatewayResponse(
                    version=PROTOCOL_VERSION,
                    request_id=request.request_id,
                    operation=request.operation,
                    code=423,
                    motion_service=self._motion_service,
                    error="motion service is not ai-w",
                )
            if request.operation != "MoveZero" and not self._zero_healthy:
                return GatewayResponse(
                    version=PROTOCOL_VERSION,
                    request_id=request.request_id,
                    operation=request.operation,
                    code=503,
                    motion_service=self._motion_service,
                    error="gateway zero hold is not healthy",
                )
            if request.operation == "MoveZero":
                code = self._execute_watchdog_zero(reason="policy")
                service = self._motion_service
            elif request.operation == "Move":
                arguments = (
                    request.arguments
                )
                code = self._normalize_code(self._sport.Move(*arguments))
                service = self._motion_service
                self._record_command(request.operation, code, arguments)
            else:
                method = getattr(self._sport, request.operation)
                code = self._normalize_code(method())
                service = self._motion_service
                self._record_command(request.operation, code, ())
            return GatewayResponse(
                version=PROTOCOL_VERSION,
                request_id=request.request_id,
                operation=request.operation,
                code=code,
                motion_service=service,
            )
        except Exception as exc:
            self._record_command(request.operation, 500, request.arguments,
                                 error=str(exc))
            return GatewayResponse(
                version=PROTOCOL_VERSION,
                request_id=request.request_id,
                operation=request.operation,
                code=500,
                motion_service=self._motion_service,
                error=str(exc)[:256],
            )

    def _execute_watchdog_zero(self, *, reason: str = "watchdog") -> int:
        try:
            code = self._normalize_code(self._sport.Move(0.0, 0.0, 0.0))
            self._zero_healthy = code == 0
            self._record_command("MoveZero", code, (), reason=reason)
            return code
        except Exception as exc:
            self._zero_healthy = False
            self._record_command(
                "MoveZero", 500, (), reason=reason, error=str(exc))
            return 500

    def _send_error(
        self,
        connection: socket.socket,
        request_id: str,
        code: int,
        error: Exception,
    ) -> None:
        response = GatewayResponse(
            version=PROTOCOL_VERSION,
            request_id=request_id,
            operation="Error",
            code=int(code),
            motion_service=self._motion_service,
            error=str(error)[:256],
        )
        connection.sendall(encode_frame(response.to_dict()))

    @staticmethod
    def _best_effort_request_id(frame: bytes) -> str:
        try:
            payload = json.loads(frame.decode("utf-8"))
            value = payload.get("request_id")
            if (isinstance(value, str) and value
                    and len(value) <= 128
                    and not any(char in value for char in "\r\n\x00")):
                return value
        except Exception:
            pass
        return "protocol-error"

    @staticmethod
    def _normalize_code(value: Any) -> int:
        if value is None:
            return 0
        if isinstance(value, bool):
            raise RuntimeError("invalid Unitree SDK return code")
        return int(value)

    @staticmethod
    def _extract_service_name(raw: Any) -> Optional[str]:
        if not isinstance(raw, dict):
            return None
        for key in ("name", "alias"):
            value = raw.get(key)
            if value is not None and str(value).strip():
                return str(value).strip()
        nested = raw.get("mode")
        return SportGatewayServer._extract_service_name(nested)

    def _record_command(
        self,
        operation: str,
        code: int,
        arguments: object,
        *,
        reason: Optional[str] = None,
        error: Optional[str] = None,
    ) -> None:
        recorder = self._recorder
        method = getattr(recorder, "record_command", None)
        if callable(method):
            method(
                operation, code, arguments=tuple(arguments),
                reason=reason, error=error,
            )

    def _record_event(self, event: str) -> None:
        recorder = self._recorder
        method = getattr(recorder, "record_event", None)
        if callable(method):
            method(event)
