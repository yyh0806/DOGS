from pathlib import Path
from types import SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1] / "go2w_bridge"
ENTRY = PACKAGE / "nx_sport_gateway.py"
OBSERVER = PACKAGE / "nx_safety_observer.py"
SERVER = PACKAGE / "sport_gateway_server.py"
ROOT = Path(__file__).resolve().parents[3]


def test_gateway_is_the_only_sdk_lease_owner_and_has_no_unload_api():
    source = ENTRY.read_text(encoding="utf-8")

    assert source.count("SportClient(enableLease=True)") == 1
    assert "WaitLeaseApplied()" in source
    assert "ChannelFactory" in source
    assert "MotionSwitcherClient" in source
    for forbidden in (".Damp(", ".StandDown(", ".RecoveryStand("):
        assert forbidden not in source


def test_gateway_is_ros_independent_and_does_not_subscribe_raw_safety_state():
    source = ENTRY.read_text(encoding="utf-8")

    assert "import rclpy" not in source
    for forbidden in (
        "ChannelSubscriber", "LowState_", "SportModeState_",
        "RawSafetyObserver",
    ):
        assert forbidden not in source
    assert "SafetyEventRecorder(" in source
    assert "GO2W_GATEWAY_EVENT_LOG" in source
    assert "/home/nx/go2w/safety-events/gateway.jsonl" in source


def test_server_bootstraps_zero_in_actor_before_accepting_policy_client():
    source = SERVER.read_text(encoding="utf-8")
    actor_start = source.index("self._actor_thread.start()")
    ready_wait = source.index("self._actor_ready.wait", actor_start)
    accept_start = source.index("self._accept_thread.start()", ready_wait)

    assert actor_start < ready_wait < accept_start
    assert "self._switcher.CheckMode()" in source
    assert "for _ in range(2)" in source
    assert "self._sport.Move(0.0, 0.0, 0.0)" in source


def test_raw_observer_combines_sport_and_low_state_for_persistence():
    from go2w_bridge.nx_safety_observer import RawSafetyObserver

    class Recorder:
        def __init__(self):
            self.states = []

        def record_state(self, state):
            self.states.append(dict(state))

    recorder = Recorder()
    observer = RawSafetyObserver(recorder)
    motors = [
        SimpleNamespace(dq=float(index), lost=index == 2, mode=1,
                        temperature=30 + index)
        for index in range(20)
    ]
    observer.on_low(SimpleNamespace(
        motor_state=motors,
        imu_state=SimpleNamespace(rpy=[0.1, -0.2, 0.3]),
        foot_force=[1, 2, 3, 4],
        bms_state=SimpleNamespace(status=8, soc=81),
        power_v=31.2,
        power_a=2.1,
        level_flag=0,
        bit_flag=4,
    ))
    observer.on_sport(SimpleNamespace(
        error_code=42,
        mode=7,
        gait_type=0,
        progress=0.0,
        velocity=[0.0, 0.0, 0.0],
        yaw_speed=0.0,
        position=[0.0, 0.0, 0.0],
    ))
    state = recorder.states[-1]

    assert state["mode"] == 7
    assert state["error_code"] == 42
    assert state["wheel_dq"] == [12.0, 13.0, 14.0, 15.0]
    assert state["roll"] == 0.1
    assert state["pitch"] == -0.2
    assert state["motor_lost"][2] == 1


def test_raw_observer_is_read_only_and_owns_no_sdk_control_client():
    source = OBSERVER.read_text(encoding="utf-8")

    assert "ChannelSubscriber" in source
    assert '"rt/lf/sportmodestate"' in source
    assert '"rt/lowstate"' in source
    assert "SafetyEventRecorder(" in source
    assert "GO2W_SAFETY_EVENT_LOG" in source
    assert "/home/nx/go2w/safety-events/events.jsonl" in source
    for forbidden in (
        "SportClient", "MotionSwitcherClient", ".Move(",
        ".BalanceStand(", ".StandUp(",
    ):
        assert forbidden not in source


def test_safety_observer_service_is_independent_of_gateway_health():
    source = (ROOT / "docker" / "go2w-safety-observer.service").read_text(
        encoding="utf-8")

    assert "nx_safety_observer.py" in source
    assert "DOG_INTERFACE=" in source
    assert "GO2W_SAFETY_EVENT_LOG=" in source
    assert "Requires=go2w-sport-gateway.service" not in source
    assert "PartOf=go2w-sport-gateway.service" not in source
