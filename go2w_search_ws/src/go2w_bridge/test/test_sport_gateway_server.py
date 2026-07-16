import json
import socket
import time

import pytest

from go2w_bridge.sport_gateway_protocol import MAX_FRAME_BYTES
from go2w_bridge.sport_gateway_server import SportGatewayServer


class FakeSport:
    def __init__(self):
        self.calls = []
        self.fail_operation = None

    def _call(self, operation, arguments=()):
        self.calls.append((operation, tuple(arguments)))
        if self.fail_operation == operation:
            raise RuntimeError(f"{operation} failed")
        return 0

    def Move(self, vx, vy, vyaw):
        return self._call("Move", (vx, vy, vyaw))

    def BalanceStand(self):
        return self._call("BalanceStand")

    def StandUp(self):
        return self._call("StandUp")

    def StopMove(self):
        return self._call("StopMove")


class FakeSwitcher:
    def __init__(self, service="ai-w", code=0):
        self.service = service
        self.code = code

    def CheckMode(self):
        return self.code, {"name": self.service}


def _request(client, request_id, operation, arguments=()):
    client.sendall((json.dumps({
        "version": 1,
        "request_id": request_id,
        "operation": operation,
        "arguments": list(arguments),
    }) + "\n").encode())
    return json.loads(client.recv(4096))


def _make_server(sport, switcher, tmp_path, **kwargs):
    if hasattr(socket, "AF_UNIX"):
        server = SportGatewayServer(
            sport, switcher, socket_path=tmp_path / "sport.sock", **kwargs)
    else:
        server = SportGatewayServer(
            sport, switcher,
            socket_path=("127.0.0.1", 0), socket_family=socket.AF_INET,
            **kwargs,
        )
    server.start()
    return server


def _connect(server):
    client = socket.socket(server.socket_family, socket.SOCK_STREAM)
    client.settimeout(0.5)
    client.connect(server.address)
    return client


def _wait_until(predicate, timeout=0.8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def test_client_disconnect_keeps_server_alive_and_repeats_zero(tmp_path):
    sport = FakeSport()
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=0.04, zero_period=0.02,
    )
    try:
        with _connect(server) as client:
            response = _request(client, "move", "Move", (0.1, 0.0, 0.0))
            assert response["code"] == 0
        assert _wait_until(
            lambda: sum(
                call == ("Move", (0.0, 0.0, 0.0))
                for call in sport.calls
            ) >= 2
        )
        with _connect(server) as replacement:
            assert _request(replacement, "check", "CheckMode")["code"] == 0
    finally:
        server.close()


def test_actor_checks_mode_and_zeros_twice_before_client_is_accepted(tmp_path):
    sport = FakeSport()
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=0.5, zero_period=0.05,
    )
    try:
        assert sport.calls[:2] == [
            ("Move", (0.0, 0.0, 0.0)),
            ("Move", (0.0, 0.0, 0.0)),
        ]
        with _connect(server) as client:
            assert _request(client, "check", "CheckMode")["code"] == 0
    finally:
        server.close()


def test_second_policy_client_cannot_execute_concurrently(tmp_path):
    sport = FakeSport()
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=0.2, zero_period=0.05,
    )
    first = _connect(server)
    second = _connect(server)
    try:
        assert _request(first, "owner", "CheckMode")["code"] == 0
        response = _request(second, "intruder", "Move", (0.2, 0.0, 0.0))
        assert response["code"] == 409
        assert ("Move", (0.2, 0.0, 0.0)) not in sport.calls
    finally:
        first.close()
        second.close()
        server.close()


@pytest.mark.parametrize(
    "payload",
    [
        b"not-json\n",
        (b"x" * (MAX_FRAME_BYTES + 1)) + b"\n",
    ],
)
def test_bad_frame_does_not_kill_gateway_or_block_reconnect(tmp_path, payload):
    sport = FakeSport()
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=0.04, zero_period=0.02,
    )
    try:
        with _connect(server) as client:
            client.sendall(payload)
            response = json.loads(client.recv(4096))
            assert response["code"] == 400
        with _connect(server) as replacement:
            assert _request(replacement, "check", "CheckMode")["code"] == 0
    finally:
        server.close()


def test_sdk_exception_is_bounded_and_watchdog_continues_zero(tmp_path):
    sport = FakeSport()
    sport.fail_operation = "BalanceStand"
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=0.04, zero_period=0.02,
    )
    try:
        with _connect(server) as client:
            response = _request(client, "balance", "BalanceStand")
            assert response["code"] == 500
            assert "BalanceStand failed" in response["error"]
        assert _wait_until(
            lambda: ("Move", (0.0, 0.0, 0.0)) in sport.calls)
    finally:
        server.close()


def test_orderly_close_serializes_one_final_zero_on_the_sdk_actor(tmp_path):
    sport = FakeSport()
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=5.0, zero_period=1.0,
    )
    before_close = len(sport.calls)

    server.close()

    assert len(sport.calls) == before_close + 1
    assert sport.calls[-1] == ("Move", (0.0, 0.0, 0.0))


def test_wrong_motion_service_holds_lease_and_rejects_motion(tmp_path):
    sport = FakeSport()
    server = _make_server(
        sport, FakeSwitcher(service="sport"), tmp_path,
        command_timeout=0.5, zero_period=0.05,
    )
    try:
        with _connect(server) as client:
            check = _request(client, "check-wrong", "CheckMode")
            move = _request(client, "move-wrong", "Move", (0.2, 0.0, 0.0))
        assert check["code"] == 0
        assert check["motion_service"] == "sport"
        assert move["code"] == 423
        assert ("Move", (0.2, 0.0, 0.0)) not in sport.calls
    finally:
        server.close()


def test_failed_startup_zero_keeps_gateway_alive_but_not_ready(tmp_path):
    sport = FakeSport()
    sport.fail_operation = "Move"
    server = _make_server(
        sport, FakeSwitcher(), tmp_path,
        command_timeout=0.5, zero_period=0.05,
    )
    try:
        with _connect(server) as client:
            check = _request(client, "check-zero-failed", "CheckMode")
        assert check["code"] == 503
        assert check["motion_service"] == "ai-w"
        assert "zero hold" in check["error"]
    finally:
        server.close()
