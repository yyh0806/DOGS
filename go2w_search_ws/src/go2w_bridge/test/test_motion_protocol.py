import json

import pytest

from go2w_bridge.motion_types import MotionIntent


def test_versioned_intent_round_trips_without_losing_identity():
    from go2w_bridge.motion_protocol import MotionIntentEnvelope

    payload = {
        "schema_version": 1,
        "request_id": "request-123",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }
    parsed = MotionIntentEnvelope.parse(json.dumps(payload))

    assert parsed.intent is MotionIntent.START_NAV
    assert parsed.request_id == "request-123"
    assert parsed.source == "navigation_arbiter"
    assert parsed.legacy is False
    assert parsed.to_dict() == payload


@pytest.mark.parametrize(
    "legacy,expected",
    [
        ("manual_start", MotionIntent.START_MANUAL),
        ("nav_start", MotionIntent.START_NAV),
        ("manual_stop", MotionIntent.PARK),
        ("nav_stop", MotionIntent.PARK),
        ("park", MotionIntent.PARK),
        ("estop", MotionIntent.ESTOP),
    ],
)
def test_exact_legacy_commands_are_supported_for_one_migration_release(
        legacy, expected):
    from go2w_bridge.motion_protocol import MotionIntentEnvelope

    parsed = MotionIntentEnvelope.parse(legacy)
    assert parsed.intent is expected
    assert parsed.legacy is True
    assert parsed.source == "legacy"


@pytest.mark.parametrize(
    "payload",
    [
        "drive",
        "stand",
        "",
        '{"schema_version":2,"request_id":"x","intent":"park","source":"web"}',
        '{"schema_version":1,"request_id":"","intent":"park","source":"web"}',
        '{"schema_version":1,"request_id":"x","intent":"fly","source":"web"}',
    ],
)
def test_unknown_or_invalid_intent_is_rejected(payload):
    from go2w_bridge.motion_protocol import MotionProtocolError, MotionIntentEnvelope

    with pytest.raises(MotionProtocolError):
        MotionIntentEnvelope.parse(payload)


def test_status_schema_serializes_enums_and_raw_feedback():
    from go2w_bridge.motion_machine import Go2WMotionMachine
    from go2w_bridge.motion_protocol import motion_status_dict
    from go2w_bridge.motion_types import Telemetry

    machine = Go2WMotionMachine(now=lambda: 10.0)
    machine.sdk_ready("ai-w")
    machine.observe(Telemetry(
        sample_id=4,
        source_stamp=9.9,
        received_at=10.0,
        raw_mode=6,
        wheel_dq=(0.0, 0.0, 0.0, 0.0),
        battery_soc=80.0,
        error_code=0,
        roll=0.0,
        pitch=0.0,
        motion_service="ai-w",
        motor_fault=False,
    ))

    status = motion_status_dict(
        machine.snapshot(),
        release_id="abc123",
        raw={"sport_mode": 6, "error_code": 0},
    )

    assert status["schema_version"] == 4
    assert status["release_id"] == "abc123"
    assert status["session"] == "parked"
    assert status["physical_mode"] == "joint_lock"
    assert status["actual_motion"] == "stopped"
    assert status["velocity_authorized"] is False
    assert status["raw"] == {"sport_mode": 6, "error_code": 0}


def test_feedback_decoder_requires_identity_attitude_and_motor_health():
    from go2w_bridge.motion_protocol import decode_wheel_feedback

    telemetry = decode_wheel_feedback({
        "schema_version": 2,
        "sample_id": 9,
        "source_stamp": 123.5,
        "wheel_dq": [0.0, 0.01, -0.01, 0.0],
        "battery_soc": 77,
        "sport_mode": 6,
        "sport_error_code": 0,
        "roll": 0.02,
        "pitch": -0.03,
        "motor_lost": [0, 0, 0, 0],
    }, received_at=20.0, motion_service="ai-w")

    assert telemetry.sample_id == 9
    assert telemetry.source_stamp == 123.5
    assert telemetry.received_at == 20.0
    assert telemetry.motion_service == "ai-w"
    assert telemetry.motor_fault is False


@pytest.mark.parametrize("missing", [
    "sample_id", "source_stamp", "wheel_dq", "battery_soc", "sport_mode",
    "sport_error_code", "roll", "pitch", "motor_lost",
])
def test_feedback_decoder_rejects_incomplete_samples(missing):
    from go2w_bridge.motion_protocol import decode_wheel_feedback

    payload = {
        "schema_version": 2,
        "sample_id": 9,
        "source_stamp": 123.5,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "battery_soc": 77,
        "sport_mode": 6,
        "sport_error_code": 0,
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0, 0, 0, 0],
    }
    payload.pop(missing)

    with pytest.raises(ValueError, match=missing):
        decode_wheel_feedback(
            payload, received_at=20.0, motion_service="ai-w")


def test_feedback_decoder_keeps_motor_lost_as_diagnostic_counter():
    from go2w_bridge.motion_protocol import decode_wheel_feedback

    telemetry = decode_wheel_feedback({
        "schema_version": 2,
        "sample_id": 10,
        "source_stamp": 124.0,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "battery_soc": 77,
        "sport_mode": 1,
        "sport_error_code": 0,
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0, 1, 0, 0],
    }, received_at=20.0, motion_service="ai-w")

    # Unitree MotorState.lost is a uint32 communication-loss counter.  It can
    # remain non-zero while SportModeState.error_code is clear and must not be
    # promoted to a permanent motor fault merely because history accumulated.
    assert telemetry.motor_fault is False


def test_feedback_builder_emits_the_complete_v2_contract():
    from go2w_bridge.motion_protocol import build_wheel_feedback_payload

    payload = build_wheel_feedback_payload(
        sample_id=12,
        source_stamp=200.25,
        wheel_dq=(1.0, 2.0, 3.0, 4.0),
        battery_soc=65,
        bms_status=3,
        sport_mode=1,
        sport_error_code=0,
        roll=0.1,
        pitch=-0.2,
        motor_lost=(0, 0, 1, 0),
        extras={"gait_type": 2},
    )

    assert payload == {
        "schema_version": 2,
        "sample_id": 12,
        "source_stamp": 200.25,
        "wheel_dq": [1.0, 2.0, 3.0, 4.0],
        "battery_soc": 65.0,
        "bms_status": 3,
        "sport_mode": 1,
        "sport_error_code": 0,
        "roll": 0.1,
        "pitch": -0.2,
        "motor_lost": [0, 0, 1, 0],
        "gait_type": 2,
    }
