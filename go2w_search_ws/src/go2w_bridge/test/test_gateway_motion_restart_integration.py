import socket
import threading
import time

from go2w_bridge.motion_types import Effect
from go2w_bridge.sport_gateway_client import SportGatewayClient
from go2w_bridge.sport_gateway_server import SportGatewayServer


class FakeLeasedSport:
    def __init__(self, renewal_period=0.08, lease_term=1.0):
        self.calls = []
        self.disconnected_call_index = None
        self.renewal_times = []
        self.expired_count = 0
        self._renewal_period = renewal_period
        self._lease_term = lease_term
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._renew, daemon=True)
        self._thread.start()

    def _renew(self):
        previous = None
        while not self._stop.wait(self._renewal_period):
            now = time.monotonic()
            if previous is not None and now - previous >= self._lease_term:
                self.expired_count += 1
            self.renewal_times.append(now)
            previous = now

    @property
    def renewal_gap_max(self):
        return max(
            (right - left for left, right in zip(
                self.renewal_times, self.renewal_times[1:])),
            default=0.0,
        )

    @property
    def nonzero_after_disconnect(self):
        if self.disconnected_call_index is None:
            return 0
        return sum(
            any(abs(value) > 1e-9 for value in arguments)
            for _timestamp, arguments in self.calls[self.disconnected_call_index:]
        )

    @property
    def zero_calls_after_disconnect(self):
        if self.disconnected_call_index is None:
            return 0
        return sum(
            arguments == (0.0, 0.0, 0.0)
            for _timestamp, arguments in self.calls[self.disconnected_call_index:]
        )

    def Move(self, vx, vy, vyaw):
        self.calls.append((time.monotonic(), (vx, vy, vyaw)))
        return 0

    def StandUp(self):
        return 0

    def BalanceStand(self):
        return 0

    def StopMove(self):
        return 0

    def close(self):
        self._stop.set()
        self._thread.join(timeout=0.5)


class FakeSwitcher:
    def CheckMode(self):
        return 0, {"name": "ai-w"}


def _server(sport, tmp_path):
    if hasattr(socket, "AF_UNIX"):
        server = SportGatewayServer(
            sport, FakeSwitcher(), socket_path=tmp_path / "sport.sock",
            command_timeout=0.04, zero_period=0.02,
        )
    else:
        server = SportGatewayServer(
            sport, FakeSwitcher(), socket_path=("127.0.0.1", 0),
            socket_family=socket.AF_INET,
            command_timeout=0.04, zero_period=0.02,
        )
    server.start()
    return server


def _client(server):
    return SportGatewayClient(
        server.address,
        socket_family=server.socket_family,
        timeout=0.2,
    )


def test_motion_policy_restart_never_expires_lease_or_replays_velocity(tmp_path):
    sport = FakeLeasedSport()
    server = _server(sport, tmp_path)
    first = _client(server)
    second = _client(server)
    try:
        assert first.initialize().motion_service == "ai-w"
        assert first.execute(Effect(
            "Move", sequence=1, arguments=(0.1, 0.0, 0.0))).code == 0
        first.close()
        sport.disconnected_call_index = len(sport.calls)
        time.sleep(0.24)

        assert second.initialize().motion_service == "ai-w"
        assert sport.renewal_gap_max < 0.45
        assert sport.expired_count == 0
        assert sport.nonzero_after_disconnect == 0
        assert sport.zero_calls_after_disconnect >= 5
    finally:
        first.close()
        second.close()
        server.close()
        sport.close()
