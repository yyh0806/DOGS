from tools.sport_gateway_bootstrap_preflight import evaluate_snapshot


def safe_snapshot():
    return {
        "mode": 6,
        "error_code": 0,
        "wheel_dq": [0.01, -0.02, 0.0, 0.01],
        "roll": 0.03,
        "pitch": -0.04,
        "motor_lost": [0] * 20,
        "battery_soc": 82,
    }


def test_parked_joint_lock_snapshot_is_safe_for_gateway_handoff():
    assert evaluate_snapshot(safe_snapshot()) == []


def test_balance_mode_is_rejected_even_when_wheels_are_still():
    snapshot = {**safe_snapshot(), "mode": 1}

    assert "physical_mode_not_joint_lock" in evaluate_snapshot(snapshot)


def test_motion_robot_error_and_bad_low_state_are_all_rejected():
    snapshot = {
        **safe_snapshot(),
        "error_code": 3104,
        "wheel_dq": [0.2, 0.0, 0.0, 0.0],
        "roll": 0.8,
        "motor_lost": [0, 1],
        "battery_soc": 15,
    }

    failures = evaluate_snapshot(snapshot)
    assert "robot_error" in failures
    assert "wheels_not_stopped" in failures
    assert "attitude_unsafe" in failures
    assert "battery_low" in failures


def test_motor_lost_history_counter_is_not_treated_as_live_fault():
    snapshot = {
        **safe_snapshot(),
        "motor_lost": [0, 0, 0, 0, 0, 10, 0, 15, 0, 0, 0, 5],
    }

    assert evaluate_snapshot(snapshot) == []


def test_incomplete_snapshot_fails_closed():
    failures = evaluate_snapshot({"mode": 6})

    assert "incomplete_state" in failures
