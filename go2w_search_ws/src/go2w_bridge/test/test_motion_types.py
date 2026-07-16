import math

import pytest


def test_go2w_profile_fails_closed_for_unknown_mode():
    from go2w_bridge.motion_types import Go2WModeProfile, PhysicalMode

    profile = Go2WModeProfile()
    assert profile.decode(1) is PhysicalMode.WHEEL_BALANCE
    assert profile.decode(3) is PhysicalMode.WHEEL_LOCOMOTION
    assert profile.decode(6) is PhysicalMode.JOINT_LOCK
    assert profile.decode(7) is PhysicalMode.DAMPING
    assert profile.decode(255) is PhysicalMode.UNKNOWN
    assert profile.decode(None) is PhysicalMode.UNKNOWN


def test_move_transport_result_is_not_motion_confirmation():
    from go2w_bridge.motion_types import CommandReceipt

    receipt = CommandReceipt("Move", 0, sequence=4)
    assert receipt.transport_ok is True
    assert receipt.physical_confirmed is False


def test_telemetry_accepts_a_complete_fresh_sample():
    from go2w_bridge.motion_types import Telemetry

    sample = Telemetry(
        sample_id=4,
        source_stamp=10.0,
        received_at=10.1,
        raw_mode=6,
        wheel_dq=(0.0, 0.0, 0.0, 0.0),
        battery_soc=80.0,
        error_code=0,
        roll=0.01,
        pitch=-0.02,
        motion_service="ai-w",
        motor_fault=False,
    )

    assert sample.is_fresh(now=10.2, max_age=0.5)
    assert sample.wheels_stopped(threshold=0.05)


@pytest.mark.parametrize(
    "field,value",
    [
        ("sample_id", -1),
        ("source_stamp", math.nan),
        ("received_at", math.inf),
        ("raw_mode", -1),
        ("wheel_dq", (0.0, 0.0, math.nan, 0.0)),
        ("battery_soc", 101.0),
        ("error_code", -1),
        ("roll", math.inf),
        ("pitch", math.nan),
    ],
)
def test_telemetry_rejects_invalid_values(field, value):
    from go2w_bridge.motion_types import Telemetry

    values = {
        "sample_id": 1,
        "source_stamp": 10.0,
        "received_at": 10.1,
        "raw_mode": 6,
        "wheel_dq": (0.0, 0.0, 0.0, 0.0),
        "battery_soc": 80.0,
        "error_code": 0,
        "roll": 0.0,
        "pitch": 0.0,
        "motion_service": "ai-w",
        "motor_fault": False,
    }
    values[field] = value

    with pytest.raises(ValueError, match=field):
        Telemetry(**values)


def test_stale_telemetry_is_not_coerced_to_a_safe_mode():
    from go2w_bridge.motion_types import Telemetry

    sample = Telemetry(
        sample_id=1,
        source_stamp=1.0,
        received_at=1.1,
        raw_mode=6,
        wheel_dq=(0.0, 0.0, 0.0, 0.0),
        battery_soc=80.0,
        error_code=0,
        roll=0.0,
        pitch=0.0,
        motion_service="ai-w",
        motor_fault=False,
    )

    assert sample.is_fresh(now=2.0, max_age=0.5) is False
