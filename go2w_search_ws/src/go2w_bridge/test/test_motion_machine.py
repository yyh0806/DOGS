from dataclasses import FrozenInstanceError

import pytest

from go2w_bridge.motion_types import (
    ActualMotionState,
    CommandReceipt,
    MotionIntent,
    PhysicalMode,
    SessionState,
    StopProfile,
    Telemetry,
)


class Clock:
    def __init__(self, value=100.0):
        self.value = float(value)

    def __call__(self):
        return self.value


class Samples:
    def __init__(self, clock):
        self.clock = clock
        self.sample_id = 0

    def make(self, raw_mode, wheel_dq=(0.0, 0.0, 0.0, 0.0), **updates):
        self.sample_id += 1
        values = {
            "sample_id": self.sample_id,
            "source_stamp": self.clock() - 0.01,
            "received_at": self.clock(),
            "raw_mode": raw_mode,
            "wheel_dq": tuple(float(value) for value in wheel_dq),
            "battery_soc": 80.0,
            "error_code": 0,
            "roll": 0.0,
            "pitch": 0.0,
            "motion_service": "ai-w",
            "motor_fault": False,
        }
        values.update(updates)
        return Telemetry(**values)


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def samples(clock):
    return Samples(clock)


@pytest.fixture
def machine(clock):
    from go2w_bridge.motion_machine import Go2WMotionMachine

    return Go2WMotionMachine(now=clock)


def ready(machine):
    assert machine.sdk_ready("ai-w") == []


def park_from_mode6(machine, samples):
    ready(machine)
    assert machine.observe(samples.make(raw_mode=6)) == []
    assert machine.snapshot().session is SessionState.PARKED


def test_feedback_before_sdk_and_sdk_before_feedback_converge(clock, samples):
    from go2w_bridge.motion_machine import Go2WMotionMachine

    feedback_first = Go2WMotionMachine(now=clock)
    assert feedback_first.observe(samples.make(raw_mode=6)) == []
    assert feedback_first.snapshot().session is SessionState.BOOT_HOLD
    assert feedback_first.sdk_ready("ai-w") == []

    sdk_first = Go2WMotionMachine(now=clock)
    assert sdk_first.sdk_ready("ai-w") == []
    assert sdk_first.observe(samples.make(raw_mode=6)) == []

    assert feedback_first.snapshot().session is SessionState.PARKED
    assert sdk_first.snapshot().session is SessionState.PARKED


def test_boot_mode6_adopts_parked_without_pose_effect(machine, samples):
    park_from_mode6(machine, samples)
    snapshot = machine.snapshot()
    assert snapshot.physical_mode is PhysicalMode.JOINT_LOCK
    assert snapshot.actual_motion is ActualMotionState.STOPPED
    assert snapshot.velocity_authorized is False
    with pytest.raises(FrozenInstanceError):
        snapshot.fault = "mutated"


def test_joint_lock_encoder_noise_does_not_create_false_parked_motion_fault(
        machine, samples):
    ready(machine)
    noise = (0.085, -0.035, 0.062, -0.039)

    assert machine.observe(samples.make(raw_mode=6, wheel_dq=noise)) == []
    assert machine.snapshot().session is SessionState.PARKED
    assert machine.snapshot().actual_motion is ActualMotionState.STOPPED
    assert machine.observe(samples.make(raw_mode=6, wheel_dq=noise)) == []
    assert machine.snapshot().fault is None


def test_joint_lock_real_wheel_motion_still_faults_after_parking(machine, samples):
    park_from_mode6(machine, samples)

    moving = (0.25, -0.25, 0.23, -0.23)
    for _ in range(14):
        effects = machine.observe(samples.make(raw_mode=6, wheel_dq=moving))
        assert [effect.operation for effect in effects] == ["MoveZero"]
        assert machine.snapshot().session is SessionState.PARKED

    effects = machine.observe(samples.make(raw_mode=6, wheel_dq=moving))
    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.FAULT
    assert machine.snapshot().fault == "parked_state_lost"


def test_boot_joint_lock_motion_zeroes_until_stopped_feedback(machine, samples):
    ready(machine)

    effects = machine.observe(samples.make(
        raw_mode=6, wheel_dq=(0.25, -0.25, 0.23, -0.23)))

    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.BOOT_HOLD
    assert machine.snapshot().fault is None

    assert machine.observe(samples.make(raw_mode=6)) == []
    assert machine.snapshot().session is SessionState.PARKED


def test_boot_stationary_wheel_mode_waits_for_explicit_park(machine, samples):
    ready(machine)
    first = machine.observe(samples.make(raw_mode=1))
    second = machine.observe(samples.make(raw_mode=1))

    assert first == []
    assert second == []
    assert machine.snapshot().session is SessionState.BOOT_HOLD


def test_boot_moving_wheels_never_issue_pose_before_stop(machine, samples):
    ready(machine)
    moving = machine.observe(samples.make(raw_mode=1, wheel_dq=(1, 1, 1, 1)))
    assert [effect.operation for effect in moving] == ["MoveZero"]
    assert all(effect.operation != "StandUp" for effect in moving)

    stopped = machine.observe(samples.make(raw_mode=1))
    assert stopped == []
    assert machine.snapshot().session is SessionState.BOOT_HOLD


def test_boot_balance_wheels_never_auto_park_after_zero_settle(
        machine, samples, clock):
    ready(machine)
    moving = (1.0, 1.0, 1.0, 1.0)
    first = machine.observe(samples.make(raw_mode=1, wheel_dq=moving))
    assert [effect.operation for effect in first] == ["MoveZero"]

    clock.value += 0.29
    assert machine.tick(clock()) == []
    clock.value += 0.02
    settled = machine.tick(clock())

    assert settled == []
    assert machine.snapshot().session is SessionState.BOOT_HOLD


def test_explicit_park_from_boot_hold_is_the_only_startup_standup_path(
        machine, samples, clock):
    ready(machine)
    assert machine.observe(samples.make(raw_mode=1)) == []
    effects = machine.request(MotionIntent.PARK)
    assert [effect.operation for effect in effects] == ["MoveZero"]
    clock.value += 0.51
    assert [effect.operation for effect in machine.tick(clock())] == ["StandUp"]


@pytest.mark.parametrize("service", [None, "normal", "ai"])
def test_wrong_motion_service_fails_closed(machine, service):
    assert machine.sdk_ready(service) == []
    snapshot = machine.snapshot()
    assert snapshot.session is SessionState.FAULT
    assert snapshot.velocity_authorized is False
    assert snapshot.fault == "wrong_motion_service"


@pytest.mark.parametrize(
    "updates,reason",
    [
        ({"error_code": 12}, "robot_error"),
        ({"motor_fault": True}, "motor_fault"),
        ({"roll": 0.8}, "attitude_unsafe"),
        ({"pitch": -0.8}, "attitude_unsafe"),
        ({"battery_soc": 2.0}, "battery_low"),
    ],
)
def test_unhealthy_boot_feedback_never_emits_pose(machine, samples, updates, reason):
    ready(machine)
    assert machine.observe(samples.make(raw_mode=1, **updates)) == []
    snapshot = machine.snapshot()
    assert snapshot.session is SessionState.FAULT
    assert snapshot.fault == reason


def test_explicit_nav_activation_requires_scan_and_feedback(machine, samples):
    park_from_mode6(machine, samples)

    assert machine.request(MotionIntent.START_NAV, scan_fresh=False) == []
    assert machine.snapshot().session is SessionState.PARKED
    assert machine.snapshot().fault == "nav_scan_stale"

    effects = machine.request(MotionIntent.START_NAV, scan_fresh=True)
    assert [effect.operation for effect in effects] == ["BalanceStand"]
    assert machine.snapshot().session is SessionState.ACTIVATING
    assert machine.snapshot().velocity_authorized is False

    receipt = CommandReceipt("BalanceStand", 0, effects[0].sequence)
    machine.record_receipt(receipt)
    assert machine.snapshot().session is SessionState.ACTIVATING

    assert machine.observe(samples.make(raw_mode=1)) == []
    assert machine.snapshot().session is SessionState.NAV_ACTIVE
    assert machine.snapshot().velocity_authorized is True


def test_stopped_manual_session_handoffs_to_nav_without_pose_effect(
        machine, samples):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_MANUAL)
    machine.observe(samples.make(raw_mode=1))

    assert machine.snapshot().session is SessionState.MANUAL_ACTIVE
    assert machine.request(MotionIntent.START_NAV, scan_fresh=True) == []
    snapshot = machine.snapshot()
    assert snapshot.session is SessionState.NAV_ACTIVE
    assert snapshot.owner == "nav"
    assert snapshot.fault is None
    assert snapshot.velocity_authorized is True


def test_moving_manual_session_rejects_nav_handoff(machine, samples):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_MANUAL)
    machine.observe(samples.make(raw_mode=1))
    machine.observe(samples.make(
        raw_mode=1,
        wheel_dq=(0.3, 0.3, 0.3, 0.3),
    ))

    assert machine.snapshot().session is SessionState.MANUAL_ACTIVE
    assert machine.snapshot().actual_motion is ActualMotionState.MOVING
    assert machine.request(MotionIntent.START_NAV, scan_fresh=True) == []
    assert machine.snapshot().session is SessionState.MANUAL_ACTIVE
    assert machine.snapshot().fault == "session_busy"


@pytest.mark.parametrize(
    "profile,expected",
    [
        (StopProfile.MOVE_ZERO_ONLY, ["MoveZero"]),
        (StopProfile.MOVE_ZERO_THEN_STOP_MOVE, ["MoveZero", "StopMove"]),
    ],
)
def test_park_zeroes_then_parks_after_stopped_feedback(
        clock, samples, profile, expected):
    from go2w_bridge.motion_machine import Go2WMotionMachine

    machine = Go2WMotionMachine(now=clock, stop_profile=profile)
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_MANUAL)
    machine.observe(samples.make(raw_mode=1))

    effects = machine.request(MotionIntent.PARK)
    assert [effect.operation for effect in effects] == expected
    assert machine.snapshot().session is SessionState.STOPPING
    assert machine.snapshot().velocity_authorized is False

    # One zero wheel sample must not bypass the bounded settling window.  On
    # hardware BalanceStand can momentarily report zero before its wheels have
    # actually finished reacting, and an immediate StandUp leaves a transient
    # that is later misclassified as parked-state loss.
    park_effects = machine.observe(samples.make(raw_mode=1))
    assert [effect.operation for effect in park_effects] == ["MoveZero"]
    clock.value += 0.49
    assert [effect.operation for effect in machine.tick(clock())] == ["MoveZero"]
    clock.value += 0.02
    assert [effect.operation for effect in machine.tick(clock())] == ["StandUp"]
    assert machine.observe(samples.make(raw_mode=1)) == []
    assert machine.observe(samples.make(raw_mode=6)) == []
    assert machine.snapshot().session is SessionState.PARKED


def test_park_balance_wheels_sends_standup_after_bounded_zero_settle(
        machine, samples, clock):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_NAV, scan_fresh=True)
    machine.observe(samples.make(
        raw_mode=1, wheel_dq=(1.0, 1.0, 1.0, 1.0)))

    effects = machine.request(MotionIntent.PARK)
    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.STOPPING

    clock.value += 0.49
    assert [effect.operation for effect in machine.tick(clock())] == ["MoveZero"]
    clock.value += 0.02
    parked = machine.tick(clock())

    assert [effect.operation for effect in parked] == ["StandUp"]
    assert machine.snapshot().session is SessionState.PARKING


def test_estop_never_emits_a_pose_operation(machine, samples):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_MANUAL)
    machine.observe(samples.make(raw_mode=1))

    effects = machine.request(MotionIntent.ESTOP)
    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.ESTOP
    assert machine.snapshot().velocity_authorized is False


def test_transition_timeout_faults_without_inverse_pose(machine, samples, clock):
    park_from_mode6(machine, samples)
    effects = machine.request(MotionIntent.START_MANUAL)
    assert [effect.operation for effect in effects] == ["BalanceStand"]

    clock.value += 6.0
    timeout_effects = machine.tick(clock())
    assert [effect.operation for effect in timeout_effects] == ["MoveZero"]
    assert all(effect.operation != "StandUp" for effect in timeout_effects)
    assert machine.snapshot().session is SessionState.FAULT
    assert machine.snapshot().fault == "transition_timeout"


def test_active_session_faults_when_telemetry_becomes_stale(
        machine, samples, clock):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_MANUAL)
    machine.observe(samples.make(raw_mode=1))

    clock.value += 1.0
    effects = machine.tick(clock())
    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.FAULT
    assert machine.snapshot().fault == "telemetry_stale"


def test_parked_telemetry_gap_inhibits_start_but_recovers_without_reset(
        machine, samples, clock):
    park_from_mode6(machine, samples)

    clock.value += 1.0
    assert machine.tick(clock()) == []
    stale = machine.snapshot()
    assert stale.session is SessionState.PARKED
    assert stale.telemetry_fresh is False
    assert stale.fault == "telemetry_stale"
    assert machine.request(MotionIntent.START_NAV, scan_fresh=True) == []

    machine.observe(samples.make(raw_mode=6))
    recovered = machine.snapshot()
    assert recovered.session is SessionState.PARKED
    assert recovered.telemetry_fresh is True
    assert recovered.fault is None


def test_duplicate_sample_id_is_rejected_without_replaying_pose(machine, samples):
    ready(machine)
    sample = samples.make(raw_mode=1)
    first = machine.observe(sample)
    second = machine.observe(sample)

    assert first == []
    assert second == []
    assert machine.snapshot().rejected_samples == 1


def test_feedback_publisher_restart_accepts_new_timestamp_epoch(machine, samples):
    ready(machine)
    old_epoch = samples.make(
        raw_mode=6, sample_id=153226, source_stamp=90.0)
    machine.observe(old_epoch)
    assert machine.snapshot().last_sample_id == 153226

    # nx_sensor_node restarts its sample counter at zero, while its wall-clock
    # source timestamp keeps advancing across the process boundary.
    new_epoch = samples.make(raw_mode=6, sample_id=1, source_stamp=91.0)
    machine.observe(new_epoch)

    assert machine.snapshot().last_sample_id == 1
    assert machine.snapshot().rejected_samples == 0
    assert machine.snapshot().session is SessionState.PARKED


def test_delayed_packet_from_old_feedback_epoch_is_rejected(machine, samples):
    ready(machine)
    machine.observe(samples.make(
        raw_mode=6, sample_id=153226, source_stamp=90.0))
    machine.observe(samples.make(
        raw_mode=6, sample_id=1, source_stamp=91.0))

    machine.observe(samples.make(
        raw_mode=6, sample_id=153227, source_stamp=90.5))

    assert machine.snapshot().last_sample_id == 1
    assert machine.snapshot().rejected_samples == 1


def test_velocity_authorization_invariant_across_event_sequences(machine, samples):
    park_from_mode6(machine, samples)
    events = [
        lambda: machine.request(MotionIntent.START_NAV, scan_fresh=True),
        lambda: machine.observe(samples.make(raw_mode=1)),
        lambda: machine.request(MotionIntent.PARK),
        lambda: machine.observe(samples.make(raw_mode=1)),
        lambda: machine.observe(samples.make(raw_mode=6)),
        lambda: machine.request(MotionIntent.ESTOP),
    ]

    for event in events:
        event()
        snapshot = machine.snapshot()
        if snapshot.velocity_authorized:
            assert snapshot.session in {
                SessionState.MANUAL_ACTIVE, SessionState.NAV_ACTIVE}
            assert snapshot.physical_mode in {
                PhysicalMode.WHEEL_BALANCE,
                PhysicalMode.WHEEL_LOCOMOTION,
            }
            assert snapshot.telemetry_fresh
            assert snapshot.error_code == 0


def test_only_matching_active_owner_can_obtain_nonzero_move_effect(
        machine, samples):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_NAV, scan_fresh=True)
    machine.observe(samples.make(raw_mode=1))

    allowed = machine.command_velocity(
        "nav", (0.2, 0.0, 0.1), scan_fresh=True)
    wrong_owner = machine.command_velocity(
        "manual", (0.2, 0.0, 0.1), scan_fresh=True)
    stale_scan = machine.command_velocity(
        "nav", (0.2, 0.0, 0.1), scan_fresh=False)

    assert allowed.operation == "Move"
    assert allowed.arguments == (0.2, 0.0, 0.1)
    assert wrong_owner.operation == "MoveZero"
    assert stale_scan.operation == "MoveZero"


def test_external_execution_fault_revokes_authority_and_zeroes(
        machine, samples):
    park_from_mode6(machine, samples)
    machine.request(MotionIntent.START_MANUAL)
    machine.observe(samples.make(raw_mode=1))

    effects = machine.report_fault("wheel_no_response")

    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert machine.snapshot().session is SessionState.FAULT
    assert machine.snapshot().fault == "wheel_no_response"
    assert machine.snapshot().velocity_authorized is False
