import json
import math
from pathlib import Path

import pytest

from nav_shuttle import ShuttleRunner, build_shuttle_goals, main


ROOT = Path(__file__).resolve().parents[1]


class FakeTransport:
    def __init__(self, states_by_generation=None, rejected_generation=None):
        self.states_by_generation = states_by_generation or {}
        self.rejected_generation = rejected_generation
        self.posts = []
        self.current_generation = 0

    def post(self, path, payload):
        self.posts.append((path, dict(payload)))
        if path == "/api/stop":
            return {"ok": True}
        self.current_generation += 1
        if self.current_generation == self.rejected_generation:
            return {"ok": False, "reason": "navigation_owner_busy"}
        return {"ok": True, "generation": self.current_generation}

    def get(self, path):
        assert path == "/api/status"
        states = self.states_by_generation.setdefault(
            self.current_generation,
            [{"generation": self.current_generation, "status": "succeeded"}],
        )
        state = states.pop(0) if len(states) > 1 else states[0]
        return {"point_nav": dict(state)}


def test_builds_three_complete_round_trips_in_order():
    goals = build_shuttle_goals((0, 0), (20, 0), trips=3)

    assert [(goal.x, goal.y) for goal in goals] == [
        (20.0, 0.0), (0.0, 0.0),
        (20.0, 0.0), (0.0, 0.0),
        (20.0, 0.0), (0.0, 0.0),
    ]
    assert [goal.yaw for goal in goals] == pytest.approx(
        [0.0, math.pi, 0.0, math.pi, 0.0, math.pi]
    )


def test_runner_submits_next_leg_only_after_matching_success():
    transport = FakeTransport({
        1: [
            {"generation": 0, "status": "succeeded"},
            {"generation": 1, "status": "active"},
            {"generation": 1, "status": "succeeded"},
        ],
        2: [{"generation": 2, "status": "succeeded"}],
    })
    runner = ShuttleRunner(
        transport,
        poll_interval=0.0,
        leg_timeout=5.0,
    )

    result = runner.run(build_shuttle_goals((0, 0), (20, 0), trips=1))

    assert result["ok"] is True
    assert result["legs_completed"] == 2
    assert [path for path, _ in transport.posts] == [
        "/api/navigate", "/api/navigate"
    ]


def test_runner_stops_on_first_terminal_failure_without_sending_later_legs():
    transport = FakeTransport({
        1: [{"generation": 1, "status": "aborted", "reason": "planner_failed"}],
    })
    runner = ShuttleRunner(transport, poll_interval=0.0, leg_timeout=5.0)

    result = runner.run(build_shuttle_goals((0, 0), (20, 0), trips=3))

    assert result == {
        "ok": False,
        "legs_completed": 0,
        "failed_leg": 1,
        "generation": 1,
        "status": "aborted",
        "reason": "planner_failed",
    }
    assert [path for path, _ in transport.posts] == [
        "/api/navigate", "/api/stop"
    ]


def test_runner_stops_if_a_later_submission_is_rejected():
    transport = FakeTransport(rejected_generation=2)
    runner = ShuttleRunner(transport, poll_interval=0.0, leg_timeout=5.0)

    result = runner.run(build_shuttle_goals((0, 0), (20, 0), trips=2))

    assert result["ok"] is False
    assert result["legs_completed"] == 1
    assert result["failed_leg"] == 2
    assert result["reason"] == "navigation_owner_busy"
    assert [path for path, _ in transport.posts] == [
        "/api/navigate", "/api/navigate", "/api/stop"
    ]


def test_runner_stops_a_leg_that_never_reaches_a_terminal_state():
    transport = FakeTransport({
        1: [{"generation": 1, "status": "active"}],
    })
    clock = {"now": 0.0}

    def monotonic():
        return clock["now"]

    def sleep(seconds):
        clock["now"] += seconds

    runner = ShuttleRunner(
        transport,
        poll_interval=0.1,
        leg_timeout=0.3,
        monotonic=monotonic,
        sleep=sleep,
    )

    result = runner.run(build_shuttle_goals((0, 0), (20, 0), trips=1))

    assert result["ok"] is False
    assert result["status"] == "timed_out"
    assert result["reason"] == "leg_timeout"
    assert [path for path, _ in transport.posts] == [
        "/api/navigate", "/api/stop"
    ]


def test_cli_is_dry_run_by_default_and_prints_six_legs(capsys):
    exit_code = main([])

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["mode"] == "dry-run"
    assert payload["trips"] == 3
    assert len(payload["goals"]) == 6
    assert payload["goals"][0]["x"] == 20.0
    assert payload["goals"][-1]["x"] == 0.0


@pytest.mark.parametrize("trips", [0, -1, 1.5, True, 101])
def test_trip_count_is_positive_bounded_integer(trips):
    with pytest.raises(ValueError):
        build_shuttle_goals((0, 0), (20, 0), trips=trips)


def test_release_artifact_packages_the_shuttle_tool():
    build = (ROOT / "docker/build_release.sh").read_text(encoding="utf-8")
    assert 'copy_path "tools/nav_shuttle.py"' in build
