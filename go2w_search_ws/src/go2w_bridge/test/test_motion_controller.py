import math
import types

from go2w_bridge.motion_machine import Go2WMotionMachine
from go2w_bridge.motion_protocol import (
    MotionIntentEnvelope,
    build_wheel_feedback_payload,
)
from go2w_bridge.motion_safety import (
    DriveExecutionWatchdog,
    ScanFreshnessWatchdog,
)
from go2w_bridge.motion_types import CommandReceipt, SessionState


class Clock:
    def __init__(self, value=100.0):
        self.value = float(value)

    def __call__(self):
        return self.value


class FakeAdapter:
    def __init__(self, codes=None):
        self.effects = []
        self.codes = dict(codes or {})

    def execute(self, effect):
        self.effects.append(effect)
        return CommandReceipt(
            effect.operation,
            self.codes.get(effect.operation, 0),
            effect.sequence,
        )


def scan(stamp=(10, 20)):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            frame_id="base_link",
            stamp=types.SimpleNamespace(sec=stamp[0], nanosec=stamp[1]),
        ),
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        range_min=0.15,
        range_max=10.0,
        ranges=[5.0] * 360,
    )


def feedback(
        sample_id, mode, *, wheel_dq=(0.0, 0.0, 0.0, 0.0),
        gait_type=None):
    extras = {} if gait_type is None else {"gait_type": gait_type}
    return build_wheel_feedback_payload(
        sample_id=sample_id,
        source_stamp=1000.0 + sample_id,
        wheel_dq=wheel_dq,
        battery_soc=80,
        bms_status=0,
        sport_mode=mode,
        sport_error_code=0,
        roll=0.0,
        pitch=0.0,
        motor_lost=(0, 0, 0, 0),
        extras=extras,
    )


def make_controller(clock, adapter=None):
    from go2w_bridge.motion_controller import MotionController

    machine = Go2WMotionMachine(now=clock)
    scan_watchdog = ScanFreshnessWatchdog(timeout=0.3, clock=clock)
    drive_watchdog = DriveExecutionWatchdog(
        timeout=0.2, min_wheel_speed=0.1, clock=clock)
    controller = MotionController(
        machine=machine,
        scan_watchdog=scan_watchdog,
        drive_watchdog=drive_watchdog,
        clock=clock,
        manual_timeout=0.5,
        nav_timeout=0.3,
        max_vx=0.3,
        max_vy=0.0,
        max_vyaw=0.15,
    )
    if adapter is not None:
        controller.attach_adapter(adapter, "ai-w")
    return controller, machine, scan_watchdog


def test_feedback_before_adapter_is_reduced_after_sdk_attach():
    clock = Clock()
    adapter = FakeAdapter()
    controller, machine, _ = make_controller(clock)

    controller.observe_feedback(feedback(1, 1))
    assert adapter.effects == []
    controller.attach_adapter(adapter, "ai-w")

    assert adapter.effects == []
    assert machine.snapshot().session is SessionState.BOOT_HOLD


def test_velocity_callback_only_updates_state_and_actor_tick_calls_sdk():
    clock = Clock()
    adapter = FakeAdapter()
    controller, machine, scan_watchdog = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6))
    assert scan_watchdog.observe_scan(scan()) is True
    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "nav-1",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }))
    controller.observe_feedback(feedback(2, 1))
    adapter.effects.clear()

    controller.update_velocity("nav", (0.5, 0.2, 0.4))
    assert adapter.effects == []
    controller.tick()

    assert len(adapter.effects) == 1
    effect = adapter.effects[0]
    assert effect.operation == "Move"
    assert effect.arguments == (0.3, 0.0, 0.15)
    assert machine.snapshot().session is SessionState.NAV_ACTIVE


def test_scan_is_rechecked_at_sdk_boundary_after_nav_callback():
    clock = Clock()
    adapter = FakeAdapter()
    controller, _, scan_watchdog = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6))
    scan_watchdog.observe_scan(scan())
    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "nav-2",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }))
    controller.observe_feedback(feedback(2, 1))
    controller.update_velocity("nav", (0.2, 0.0, 0.0))
    adapter.effects.clear()

    clock.value += 0.31
    controller.tick()

    assert [effect.operation for effect in adapter.effects] == ["MoveZero"]


def test_nav_reverse_is_removed_at_sdk_boundary_but_turn_is_preserved():
    clock = Clock()
    adapter = FakeAdapter()
    controller, _, scan_watchdog = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6))
    scan_watchdog.observe_scan(scan())
    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "nav-forward-only",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }))
    controller.observe_feedback(feedback(2, 1))
    controller.update_velocity("nav", (-0.2, 0.0, 0.15))
    adapter.effects.clear()

    controller.tick()

    assert adapter.effects[-1].operation == "Move"
    assert adapter.effects[-1].arguments == (0.0, 0.0, 0.15)


def test_manual_session_still_allows_explicit_reverse():
    clock = Clock()
    adapter = FakeAdapter()
    controller, _, _ = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6))
    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "manual-reverse",
        "intent": "start_manual",
        "source": "panel",
    }))
    controller.observe_feedback(feedback(2, 1))
    controller.update_velocity("manual", (-0.2, 0.0, 0.0))
    adapter.effects.clear()

    controller.tick()

    assert adapter.effects[-1].operation == "Move"
    assert adapter.effects[-1].arguments == (-0.2, 0.0, 0.0)


def test_non_wheel_gait_is_zeroed_before_any_velocity_reaches_sdk():
    clock = Clock()
    adapter = FakeAdapter()
    controller, machine, _ = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6, gait_type=0))
    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "manual-wheel-only",
        "intent": "start_manual",
        "source": "panel",
    }))
    controller.observe_feedback(feedback(
        2, 3, wheel_dq=(2.0, -2.0, 2.0, -2.0), gait_type=1))
    controller.update_velocity("manual", (0.2, 0.0, 0.0))
    adapter.effects.clear()

    controller.tick()

    assert [effect.operation for effect in adapter.effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.FAULT
    assert machine.snapshot().fault == "unexpected_gait"


def test_stale_velocity_command_cannot_be_extended_by_fresh_scan():
    clock = Clock()
    adapter = FakeAdapter()
    controller, _, scan_watchdog = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6))
    scan_watchdog.observe_scan(scan())
    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "nav-3",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }))
    controller.observe_feedback(feedback(2, 1))
    controller.update_velocity("nav", (0.2, 0.0, 0.0))
    adapter.effects.clear()

    clock.value += 0.31
    scan_watchdog.observe_scan(scan((10, 21)))
    controller.observe_feedback(feedback(3, 1))
    controller.tick()

    assert [effect.operation for effect in adapter.effects] == ["MoveZero"]


def test_sdk_failure_is_recorded_and_followed_by_zero():
    clock = Clock()
    adapter = FakeAdapter(codes={"BalanceStand": 3104})
    controller, machine, scan_watchdog = make_controller(clock, adapter)
    controller.observe_feedback(feedback(1, 6))
    scan_watchdog.observe_scan(scan())

    controller.handle_intent(MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": "nav-4",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }))
    controller.tick()

    assert machine.snapshot().session is SessionState.FAULT
    assert machine.snapshot().fault == "BalanceStand_transport_error"
    assert [effect.operation for effect in adapter.effects] == [
        "BalanceStand", "MoveZero"]


def test_shutdown_only_sends_two_unique_zero_velocity_effects():
    clock = Clock()
    adapter = FakeAdapter()
    controller, _, _ = make_controller(clock, adapter)

    controller.shutdown()

    assert [effect.operation for effect in adapter.effects] == [
        "MoveZero", "MoveZero"]
    assert adapter.effects[0].sequence != adapter.effects[1].sequence
