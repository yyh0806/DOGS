import json
import os

from go2w_bridge.safety_event_recorder import SafetyEventRecorder


def _rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_recorder_persists_only_changes_and_command_transitions(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = SafetyEventRecorder(path, max_bytes=4096, backups=2)
    sample = {
        "mode": 6,
        "error_code": 0,
        "wheel_dq": [0, 0, 0, 0],
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0] * 16,
    }

    recorder.record_state(sample)
    recorder.record_state(sample)
    recorder.record_command("MoveZero", 0, reason="watchdog")
    recorder.record_command("MoveZero", 0, reason="watchdog")

    assert [row["kind"] for row in _rows(path)] == ["state", "command"]


def test_nonzero_error_is_fsynced_and_rotation_keeps_backups(
    tmp_path, monkeypatch,
):
    path = tmp_path / "events.jsonl"
    fsync_calls = []
    real_fsync = os.fsync

    def record_fsync(fd):
        fsync_calls.append(fd)
        real_fsync(fd)

    monkeypatch.setattr(os, "fsync", record_fsync)
    recorder = SafetyEventRecorder(path, max_bytes=320, backups=2)
    recorder.record_state({
        "mode": 7,
        "error_code": 3104,
        "wheel_dq": [0, 0, 0, 0],
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0] * 16,
    })
    for index in range(12):
        recorder.record_state({
            "mode": 7 if index % 2 else 6,
            "error_code": 3104 + index,
            "wheel_dq": [float(index), 0, 0, 0],
            "roll": 0.0,
            "pitch": 0.0,
            "motor_lost": [0] * 16,
        })

    assert fsync_calls
    assert path.with_name("events.jsonl.1").exists()


def test_recorder_keeps_raw_safety_fields_and_gateway_epoch(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = SafetyEventRecorder(
        path, max_bytes=4096, backups=2, process_epoch="epoch-1")

    recorder.record_state({
        "mode": 1,
        "error_code": 42,
        "wheel_dq": [1.0, 2.0, 3.0, 4.0],
        "roll": 0.8,
        "pitch": -0.1,
        "motor_lost": [0, 1],
        "battery_soc": 77,
        "bms_status": 8,
    })
    row = _rows(path)[0]

    assert row["process_epoch"] == "epoch-1"
    assert row["safety"] is True
    assert row["state"]["error_code"] == 42
    assert row["state"]["motor_lost"] == [0, 1]
    assert row["state"]["battery_soc"] == 77


def test_failed_sdk_command_and_connection_events_are_durable(
    tmp_path, monkeypatch,
):
    path = tmp_path / "events.jsonl"
    fsync_calls = []
    real_fsync = os.fsync
    monkeypatch.setattr(
        os, "fsync", lambda fd: (fsync_calls.append(fd), real_fsync(fd))[1])
    recorder = SafetyEventRecorder(path, max_bytes=4096, backups=2)

    recorder.record_event("client_disconnect")
    recorder.record_command(
        "BalanceStand", 500, error="transport failed")

    rows = _rows(path)
    assert [row["kind"] for row in rows] == ["event", "command"]
    assert rows[1]["error"] == "transport failed"
    assert len(fsync_calls) == 2


def test_normal_sensor_noise_is_rate_limited_but_mode_change_is_immediate(
    tmp_path,
):
    path = tmp_path / "events.jsonl"
    now = [10.0]
    recorder = SafetyEventRecorder(
        path,
        max_bytes=4096,
        backups=2,
        monotonic=lambda: now[0],
        normal_interval=1.0,
    )
    base = {
        "mode": 6,
        "error_code": 0,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0] * 16,
    }

    recorder.record_state(base)
    now[0] = 10.1
    recorder.record_state({**base, "wheel_dq": [0.05, 0.0, 0.0, 0.0]})
    now[0] = 10.2
    recorder.record_state({**base, "mode": 1})
    now[0] = 11.3
    recorder.record_state({**base, "mode": 6, "wheel_dq": [0.08, 0, 0, 0]})

    states = [row["state"] for row in _rows(path)]
    assert [state["mode"] for state in states] == [6, 1, 6]


def test_safety_noise_is_rate_limited_but_error_change_is_immediate(tmp_path):
    path = tmp_path / "events.jsonl"
    now = [20.0]
    recorder = SafetyEventRecorder(
        path,
        max_bytes=4096,
        backups=2,
        monotonic=lambda: now[0],
        normal_interval=1.0,
    )
    base = {
        "mode": 7,
        "error_code": 3104,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0] * 16,
    }

    recorder.record_state(base)
    now[0] = 20.1
    recorder.record_state({**base, "wheel_dq": [0.03, 0.0, 0.0, 0.0]})
    now[0] = 20.2
    recorder.record_state({**base, "error_code": 3105})
    now[0] = 21.3
    recorder.record_state({
        **base,
        "error_code": 3105,
        "wheel_dq": [0.04, 0.0, 0.0, 0.0],
    })

    states = [row["state"] for row in _rows(path)]
    assert [state["error_code"] for state in states] == [3104, 3105, 3105]


def test_motor_lost_counter_history_alone_is_not_a_live_safety_fault(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = SafetyEventRecorder(path, max_bytes=4096, backups=2)

    recorder.record_state({
        "mode": 6,
        "error_code": 0,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "roll": 0.0,
        "pitch": 0.0,
        "motor_lost": [0, 0, 10, 15],
    })

    assert _rows(path)[0]["safety"] is False
