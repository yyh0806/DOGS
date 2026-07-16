import socket

import pytest

from go2w_bridge.motion_types import Effect
from go2w_bridge.sport_gateway_client import SportGatewayClient
from go2w_bridge.sport_gateway_server import SportGatewayServer


class FakeSport:
    def __init__(self):
        self.calls = []

    def Move(self, vx, vy, vyaw):
        self.calls.append(("Move", (vx, vy, vyaw)))
        return 0

    def StandUp(self):
        self.calls.append(("StandUp", ()))
        return 0

    def BalanceStand(self):
        self.calls.append(("BalanceStand", ()))
        return 0

    def StopMove(self):
        self.calls.append(("StopMove", ()))
        return 0


class FakeSwitcher:
    def CheckMode(self):
        return 0, {"name": "ai-w"}


@pytest.fixture
def fake_gateway(tmp_path):
    sport = FakeSport()
    if hasattr(socket, "AF_UNIX"):
        server = SportGatewayServer(
            sport, FakeSwitcher(), socket_path=tmp_path / "sport.sock",
            command_timeout=0.5,
        )
    else:
        server = SportGatewayServer(
            sport, FakeSwitcher(), socket_path=("127.0.0.1", 0),
            socket_family=socket.AF_INET, command_timeout=0.5,
        )
    server.start()
    yield server, sport
    server.close()


def _client(server):
    return SportGatewayClient(
        server.address,
        socket_family=server.socket_family,
        timeout=0.2,
    )


def test_gateway_client_maps_effect_and_receipt(fake_gateway):
    server, sport = fake_gateway
    client = _client(server)
    try:
        initialized = client.initialize()
        assert initialized.code == 0
        assert initialized.motion_service == "ai-w"
        receipt = client.execute(Effect("MoveZero", sequence=7))
        assert receipt.operation == "MoveZero"
        assert receipt.sequence == 7
        assert receipt.code == 0
        assert ("Move", (0.0, 0.0, 0.0)) in sport.calls
    finally:
        client.close()


def test_gateway_client_forwards_exact_move_arguments(fake_gateway):
    server, sport = fake_gateway
    client = _client(server)
    try:
        client.initialize()
        receipt = client.execute(Effect(
            "Move", sequence=8, arguments=(0.12, 0.0, -0.08)))
        assert receipt.code == 0
        assert ("Move", (0.12, 0.0, -0.08)) in sport.calls
    finally:
        client.close()


def test_gateway_client_rejects_unload_effect_before_transport(fake_gateway):
    server, sport = fake_gateway
    client = _client(server)
    try:
        client.initialize()
        with pytest.raises(ValueError, match="unsupported SDK effect"):
            client.execute(Effect("Damp", sequence=9))
        assert all(operation != "Damp" for operation, _ in sport.calls)
    finally:
        client.close()


def test_gateway_client_reports_transport_loss_without_replaying_motion(
    fake_gateway,
):
    server, sport = fake_gateway
    client = _client(server)
    client.initialize()
    server.close()

    receipt = client.execute(
        Effect("Move", sequence=10, arguments=(0.2, 0.0, 0.0)))

    assert receipt.code != 0
    assert receipt.transport_ok is False
    assert receipt.sequence == 10
    assert ("Move", (0.2, 0.0, 0.0)) not in sport.calls
