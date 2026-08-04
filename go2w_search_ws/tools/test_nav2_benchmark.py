import json

from nav2_benchmark import BenchmarkRecorder, evaluate_report, main


def test_reference_report_passes_acceptance_gate():
    report = {
        "plan_latency_ms": 500,
        "time_to_first_cmd_ms": 400,
        "min_obstacle_clearance_m": 0.20,
        "terminal_parked": True,
        "measured_displacement_m": 0.30,
    }
    result = evaluate_report(report)
    assert result.passed is True
    assert result.failures == ()


def test_slow_or_unsafe_report_names_every_failed_gate():
    result = evaluate_report({
        "plan_latency_ms": 1600,
        "time_to_first_cmd_ms": 900,
        "min_obstacle_clearance_m": 0.10,
        "terminal_parked": False,
        "measured_displacement_m": 0.01,
    })
    assert result.passed is False
    assert set(result.failures) == {
        "plan_latency", "first_command_latency", "obstacle_clearance",
        "terminal_not_parked", "insufficient_displacement",
    }


def test_cli_without_execute_writes_read_only_template(tmp_path):
    output = tmp_path / "report.json"
    assert main(["--output", str(output)]) == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["execute"] is False
    assert report["goal_published"] is False


def test_read_only_recorder_derives_metrics_from_timestamped_events():
    recorder = BenchmarkRecorder(started_at=10.0)
    recorder.observe_action_status(10.1, "executing")
    recorder.observe_plan(10.55, poses=12)
    recorder.observe_cmd_vel(10.70, vx=0.2, vy=0.0, vyaw=0.0)
    recorder.observe_pose(10.0, x=1.0, y=2.0)
    recorder.observe_pose(11.0, x=1.3, y=2.0)
    recorder.observe_scan(10.8, [float("inf"), 0.25, 0.40])
    recorder.observe_costmap("local", 10.8)
    recorder.observe_costmap("global", 10.8)
    recorder.observe_dog_state(11.1, {
        "session": "PARKED", "actual_motion": "STOPPED"})
    recorder.observe_action_status(11.1, "succeeded")

    report = recorder.report()

    assert report["goal_published"] is False
    assert report["plan_latency_ms"] == 450.0
    assert report["time_to_first_cmd_ms"] == 600.0
    assert report["min_obstacle_clearance_m"] == 0.25
    assert report["measured_displacement_m"] == 0.3
    assert report["terminal_parked"] is True
    assert report["samples"]["local_costmap"] == 1
    assert report["samples"]["global_costmap"] == 1


def test_recorder_never_exposes_a_goal_publish_operation():
    recorder = BenchmarkRecorder(started_at=1.0)
    assert not hasattr(recorder, "publish_goal")
    assert recorder.report()["execute"] is False
