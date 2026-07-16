from nx_navigation_gateway import MissionNavigationPort, NavigationGateway


class FakeActionPort:
    def __init__(self):
        self.sent = []
        self.canceled = []
        self.state = {"status": "idle", "drained": True, "healthy": True}

    def submit(self, x, y, yaw):
        self.sent.append((x, y, yaw))
        self.state = {"status": "active", "drained": False, "healthy": True}
        return {"ok": True, "generation": len(self.sent)}

    def cancel(self, reason):
        self.canceled.append(reason)
        self.state = {"status": "canceling", "drained": False, "healthy": True}
        return True

    def tick(self):
        return dict(self.state)

    def get_state(self):
        return dict(self.state)

    def stop(self):
        self.cancel("shutdown")


class FakePathPort:
    def __init__(self):
        self.calls = []

    def compute_path(self, pose, timeout):
        self.calls.append((pose, timeout))
        return {"ok": True, "path_length": 2.5, "poses": 4}


def test_point_and_mission_goals_share_one_action_owner():
    port = FakeActionPort()
    gateway = NavigationGateway(action_port=port)

    first = gateway.submit(owner="point", pose=(1, 0, 0))
    second = gateway.submit(owner="mission", pose=(2, 0, 0))

    assert first.accepted is True
    assert second.accepted is False
    assert second.reason == "navigation_owner_busy"
    assert port.sent == [(1.0, 0.0, 0.0)]


def test_owner_is_released_only_after_terminal_result():
    port = FakeActionPort()
    gateway = NavigationGateway(action_port=port)
    gateway.submit(owner="mission", pose=(1, 2, 0.3))

    canceled = gateway.cancel(owner="mission", reason="operator_stop")
    assert canceled.accepted is True
    assert gateway.snapshot()["owner"] == "mission"

    port.state = {"status": "canceled", "drained": True, "healthy": True}
    gateway.tick()
    assert gateway.snapshot()["owner"] is None


def test_late_terminal_from_old_generation_cannot_release_new_owner():
    port = FakeActionPort()
    gateway = NavigationGateway(action_port=port)
    first = gateway.submit(owner="point", pose=(1, 0, 0))
    port.state = {"status": "succeeded", "drained": True, "generation": 1, "healthy": True}
    gateway.tick()
    second = gateway.submit(owner="mission", pose=(2, 0, 0))

    gateway.observe_terminal(generation=first.generation, status="succeeded")

    assert second.accepted is True
    assert gateway.snapshot()["owner"] == "mission"


def test_compute_path_is_read_only_and_does_not_take_goal_ownership():
    path = FakePathPort()
    gateway = NavigationGateway(action_port=FakeActionPort(), path_port=path)

    result = gateway.compute_path((2, 3, 0.5), timeout=1.2)

    assert result.ok is True
    assert result.path_length == 2.5
    assert gateway.snapshot()["owner"] is None


def test_shutdown_keeps_quarantine_until_action_is_drained():
    port = FakeActionPort()
    gateway = NavigationGateway(action_port=port)
    gateway.submit(owner="point", pose=(1, 0, 0))

    gateway.shutdown()

    state = gateway.snapshot()
    assert state["stopped"] is True
    assert state["owner"] == "point"
    assert state["drained"] is False


def test_mission_wait_drained_is_true_when_gateway_is_idle():
    gateway = NavigationGateway(action_port=FakeActionPort())
    mission = MissionNavigationPort(gateway)

    assert mission.wait_drained(0.0) is True


def test_mission_wait_drained_ignores_a_different_gateway_owner():
    gateway = NavigationGateway(action_port=FakeActionPort())
    gateway.submit(owner="point", pose=(1, 0, 0))
    mission = MissionNavigationPort(gateway)

    assert mission.wait_drained(0.0) is True


def test_mission_wait_drained_tracks_its_own_action_until_terminal():
    port = FakeActionPort()
    gateway = NavigationGateway(action_port=port)
    gateway.submit(owner="mission", pose=(1, 0, 0))
    mission = MissionNavigationPort(gateway)

    assert mission.wait_drained(0.0) is False

    port.state = {"status": "canceled", "drained": True, "healthy": True}
    assert mission.wait_drained(0.0) is True
