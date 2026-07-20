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


def test_compute_path_preserves_endpoint_error_and_path_for_goal_validation():
    class EndpointPathPort:
        def compute_path(self, pose, timeout):
            return {
                "ok": True,
                "path_length": 2.5,
                "poses": 3,
                "endpoint_x": 1.8,
                "endpoint_y": 2.9,
                "goal_error_m": 0.2236,
                "path": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 1.0, "y": 2.0},
                    {"x": 1.8, "y": 2.9},
                ],
            }

    gateway = NavigationGateway(
        action_port=FakeActionPort(), path_port=EndpointPathPort())
    mission = MissionNavigationPort(gateway)

    result = mission.compute_path_to_pose(2.0, 3.0, 0.0)

    assert result["goal_error_m"] == 0.2236
    assert result["endpoint_x"] == 1.8
    assert result["endpoint_y"] == 2.9
    assert result["path"][-1] == {"x": 1.8, "y": 2.9}


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


def test_mission_cancel_preserves_the_internal_cancellation_reason():
    port = FakeActionPort()
    gateway = NavigationGateway(action_port=port)
    gateway.submit(owner="mission", pose=(1, 0, 0))
    mission = MissionNavigationPort(gateway)

    assert mission.cancel_current("goal_became_unreachable") is True

    assert port.canceled == ["goal_became_unreachable"]


def test_idle_mission_cancel_fences_late_goal_until_next_mission_begins():
    class InstantSuccessPort(FakeActionPort):
        def submit(self, x, y, yaw):
            self.sent.append((x, y, yaw))
            self.state = {
                "status": "succeeded",
                "drained": True,
                "healthy": True,
                "generation": len(self.sent),
            }
            return {"ok": True, "generation": len(self.sent)}

    port = InstantSuccessPort()
    mission = MissionNavigationPort(NavigationGateway(action_port=port))

    assert mission.cancel_current() is True
    late = mission.send_goal_and_wait(3.0, 0.0, 0.0)

    assert late == {"ok": False, "reason": "cancelled"}
    assert port.sent == []

    mission.begin_mission()
    accepted = mission.send_goal_and_wait(4.0, 0.0, 0.0)

    assert accepted["ok"] is True
    assert port.sent == [(4.0, 0.0, 0.0)]


def test_mission_recovers_motion_unhealthy_goal_without_resubmitting():
    recoveries = []

    class RecoverableActionPort(FakeActionPort):
        def submit(self, x, y, yaw):
            self.sent.append((x, y, yaw))
            self.state = {
                "status": "waiting_health",
                "reason": "motion_unhealthy",
                "drained": False,
                "healthy": False,
                "generation": len(self.sent),
            }
            return {"ok": True, "generation": len(self.sent)}

    port = RecoverableActionPort()
    gateway = NavigationGateway(action_port=port, poll_interval=0.001)

    def recover(reason):
        recoveries.append(reason)
        port.state = {
            "status": "succeeded",
            "reason": None,
            "drained": True,
            "healthy": True,
            "generation": 1,
        }
        return {"ok": True}

    mission = MissionNavigationPort(
        gateway, recovery_callback=recover, recovery_interval=0.0)

    result = mission.send_goal_and_wait(4.0, 0.0, 0.0)

    assert result["ok"] is True
    assert recoveries == ["motion_unhealthy"]
    assert port.sent == [(4.0, 0.0, 0.0)]


def test_mission_recovers_during_health_grace_before_active_goal_is_canceled():
    recoveries = []

    class DegradedActivePort(FakeActionPort):
        def submit(self, x, y, yaw):
            self.sent.append((x, y, yaw))
            self.state = {
                "status": "active",
                "reason": None,
                "health_degraded": True,
                "health_reason": "motion_unhealthy",
                "drained": False,
                "healthy": True,
                "generation": len(self.sent),
            }
            return {"ok": True, "generation": len(self.sent)}

    port = DegradedActivePort()
    clock = [0.0]

    def advance(seconds):
        clock[0] += seconds

    gateway = NavigationGateway(
        action_port=port,
        monotonic=lambda: clock[0],
        sleep=advance,
        poll_interval=0.1,
    )

    def recover(reason):
        recoveries.append(reason)
        port.state = {
            "status": "succeeded",
            "reason": None,
            "health_degraded": False,
            "health_reason": None,
            "drained": True,
            "healthy": True,
            "generation": 1,
        }
        return {"ok": True}

    mission = MissionNavigationPort(
        gateway, recovery_callback=recover, recovery_interval=0.0)

    result = mission.send_goal_and_wait(4.0, 0.0, 0.0)

    assert result["ok"] is True
    assert recoveries == ["motion_unhealthy"]
    assert port.canceled == []
    assert port.sent == [(4.0, 0.0, 0.0)]
