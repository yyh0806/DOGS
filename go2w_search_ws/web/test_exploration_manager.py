from types import SimpleNamespace
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_exploration_manager import ExplorationManager
from nx_frontier_planner import score_frontier


def _map(data, width, height, revision=1):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=revision, nanosec=0)),
        info=SimpleNamespace(
            resolution=1.0,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=list(data),
    )


def _two_room_map(revision=1):
    # Known left/right rooms connected through a free doorway at row 3.
    width = height = 9
    data = [-1] * (width * height)
    for row in range(1, 8):
        for col in range(1, 8):
            data[row * width + col] = 0
    for row in range(1, 8):
        if row != 3:
            data[row * width + 4] = 100
    if revision > 1:
        # Simulate a moved obstacle; header churn alone is not a map revision.
        data[6 * width + 6] = 100
    return _map(data, width, height, revision)


class _PlannerPort:
    def __init__(self, blocked_x=()):
        self.blocked_x = tuple(blocked_x)
        self.probes = []

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=3.0):
        self.probes.append((x, y, yaw, frame_id, timeout))
        if any(abs(x - blocked) < 0.2 for blocked in self.blocked_x):
            return {"ok": False, "reason": "blocked"}
        return {
            "ok": True,
            "path_length": math.hypot(x - 2.0, y - 3.0),
            "poses": 4,
        }


def test_score_uses_path_information_heading_and_failure_count():
    candidate = {"size": 20}
    preferred = score_frontier(
        candidate, path_length=2.0, heading_change=0.1, failure_count=0)
    penalized = score_frontier(
        candidate, path_length=3.0, heading_change=1.0, failure_count=2)
    assert preferred > penalized


def test_manager_selects_only_reachable_candidate_and_persists_visit():
    nav = _PlannerPort(blocked_x=(1.5,))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(2.0, 3.0, 0.0),
        mode="whole_floor",
        reject_map_edge=False,
        max_failures_per_cell=1,
    )

    selected = manager.choose_next(_two_room_map(), (2.0, 3.0, 0.0))

    assert selected is not None
    assert selected["path_length"] >= 0.0
    manager.mark_visited(selected)
    snapshot = manager.snapshot()
    assert snapshot["visited_frontiers"] == [
        {"x": selected["x"], "y": selected["y"]}
    ]
    assert snapshot["current_goal"] is None


def test_blacklist_is_bounded_and_spatial_failures_survive_revision_churn(
        monkeypatch):
    nav = _PlannerPort(blocked_x=(1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(2.0, 3.0, 0.0),
        mode="whole_floor",
        reject_map_edge=False,
        max_failures_per_cell=1,
        max_blacklist_entries=2,
    )
    fake_candidates = [
        {"x": 1.5, "y": 2.5, "yaw": 0.0, "size": 10,
         "center_cell": (2, 1), "distance": 1.0, "information_gain": 10.0},
        {"x": 3.5, "y": 2.5, "yaw": 0.0, "size": 9,
         "center_cell": (2, 3), "distance": 1.0, "information_gain": 9.0},
        {"x": 5.5, "y": 2.5, "yaw": 0.0, "size": 8,
         "center_cell": (2, 5), "distance": 3.0, "information_gain": 8.0},
    ]
    monkeypatch.setattr(
        "nx_exploration_manager.select_frontier_candidates",
        lambda *_args, **_kwargs: list(fake_candidates),
    )

    assert manager.choose_next(_two_room_map(1), (2.0, 3.0, 0.0)) is None
    first = manager.snapshot()
    assert len(first["blacklist"]) == 2
    assert first["map_revision"]
    assert manager.choose_next(_two_room_map(1), (2.0, 3.0, 0.0)) is None
    probes_same_revision = len(nav.probes)

    assert manager.choose_next(_two_room_map(2), (2.0, 3.0, 0.0)) is None
    assert len(nav.probes) == probes_same_revision


def test_current_room_policy_rejects_candidate_outside_polygon(monkeypatch):
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=5.0,
        room_polygon=[(-1.0, -1.0), (2.0, -1.0), (2.0, 2.0), (-1.0, 2.0)],
        reject_map_edge=False,
    )
    monkeypatch.setattr(
        "nx_exploration_manager.select_frontier_candidates",
        lambda *_args, **_kwargs: [
            {"x": 4.0, "y": 0.0, "yaw": 0.0, "size": 100,
             "center_cell": (1, 4), "distance": 4.0, "information_gain": 100.0},
            {"x": 1.0, "y": 1.0, "yaw": 0.0, "size": 5,
             "center_cell": (1, 1), "distance": 1.4, "information_gain": 5.0},
        ],
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected["x"] == pytest.approx(1.0)
    assert all(probe[0] != pytest.approx(4.0) for probe in nav.probes)


def test_budget_status_distinguishes_information_exhaustion_and_safety_limits():
    now = [100.0]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        max_time_s=10.0,
        max_distance_m=20.0,
        battery_reserve_percent=25.0,
        monotonic=lambda: now[0],
    )
    assert manager.budget_status(battery_percent=80.0) is None
    now[0] = 111.0
    assert manager.budget_status(battery_percent=80.0) == "time_budget_exhausted"
    now[0] = 100.0
    assert manager.budget_status(battery_percent=20.0) == "battery_reserve_reached"


def test_current_room_expands_radius_when_local_frontiers_are_exhausted():
    seen_radii = []

    def candidates(_map_msg, _pose, _visited, **kwargs):
        radius = kwargs.get("max_radius")
        seen_radii.append(radius)
        if radius is not None and radius >= 12.0:
            return [{
                "x": 10.0, "y": 0.0, "yaw": 0.0, "size": 10,
                "center_cell": (0, 10), "distance": 10.0,
                "information_gain": 10.0, "score": 1.0,
            }]
        return []

    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=30.0,
        initial_radius_m=6.0,
        radius_step_m=6.0,
        tile_size_m=6.0,
        stable_exhaustion_cycles=3,
        candidate_selector=candidates,
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(10.0)
    assert seen_radii[:2] == [6.0, 12.0]
    assert manager.snapshot()["active_radius_m"] == pytest.approx(12.0)


def test_candidates_in_active_tile_are_exhausted_before_switching_tiles():
    local = {
        "x": 1.0, "y": 1.0, "yaw": 0.0, "size": 2,
        "center_cell": (1, 1), "distance": 1.0,
        "information_gain": 2.0, "score": 1.0,
    }
    remote = {
        "x": 7.0, "y": 1.0, "yaw": 0.0, "size": 100,
        "center_cell": (1, 7), "distance": 7.0,
        "information_gain": 100.0, "score": 50.0,
    }

    def candidates(_map_msg, _pose, visited, **_kwargs):
        if visited:
            return [dict(remote)]
        return [dict(remote), dict(local)]

    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=30.0,
        initial_radius_m=12.0,
        tile_size_m=6.0,
        candidate_selector=candidates,
        reject_map_edge=False,
    )

    first = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert first["x"] == pytest.approx(1.0)
    assert first["tile"] == [0, 0]
    manager.mark_visited(first)

    second = manager.choose_next(_two_room_map(), (1.0, 1.0, 0.0))
    assert second["x"] == pytest.approx(7.0)
    assert second["tile"] == [1, 0]


def test_origin_tile_is_centered_around_mission_start():
    candidates = [
        {
            "x": -1.0, "y": -1.0, "yaw": 0.0, "size": 4,
            "center_cell": (-1, -1), "distance": 1.4,
            "information_gain": 4.0, "score": 4.0,
        },
        {
            "x": 1.0, "y": 1.0, "yaw": 0.0, "size": 3,
            "center_cell": (1, 1), "distance": 1.4,
            "information_gain": 3.0, "score": 3.0,
        },
    ]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=30.0,
        initial_radius_m=6.0,
        tile_size_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(c) for c in candidates],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["tile"] == [0, 0]
    assert len(manager.navigation_port.probes) == 2


def test_plan_probe_budget_resets_each_selection_cycle():
    blocked = {
        "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 1), "distance": 1.0,
        "information_gain": 10.0, "score": 10.0,
    }
    reachable = {
        "x": 2.0, "y": 0.0, "yaw": 0.0, "size": 9,
        "center_cell": (0, 2), "distance": 2.0,
        "information_gain": 9.0, "score": 9.0,
    }
    nav = _PlannerPort(blocked_x=(1.0,))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        max_failures_per_cell=1,
        max_plan_probes=1,
        candidate_selector=lambda *_a, **_k: [dict(blocked), dict(reachable)],
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["last_selection_reason"] == "retry_pending"

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["x"] == pytest.approx(2.0)
    assert manager.snapshot()["plan_probes"] == 2


def test_same_spatial_frontier_is_bounded_across_rapid_map_revisions():
    blocked = {
        "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 1), "distance": 1.0,
        "information_gain": 10.0, "score": 10.0,
    }
    nav = _PlannerPort(blocked_x=(1.0,))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        max_failures_per_cell=2,
        candidate_selector=lambda *_a, **_k: [dict(blocked)],
        reject_map_edge=False,
    )

    manager.choose_next(_two_room_map(1), (0.0, 0.0, 0.0))
    manager.choose_next(_two_room_map(1), (0.0, 0.0, 0.0))
    probes_before_revision_churn = len(nav.probes)
    manager.choose_next(_two_room_map(2), (0.0, 0.0, 0.0))

    assert probes_before_revision_churn == 2
    assert len(nav.probes) == probes_before_revision_churn


def test_exhaustion_requires_configured_stable_confirmations():
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        stable_exhaustion_cycles=3,
        candidate_selector=lambda *_a, **_k: [],
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["last_selection_reason"] == (
        "stability_confirmation_pending")
    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["last_selection_reason"] == (
        "stability_confirmation_pending")
    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    snapshot = manager.snapshot()
    assert snapshot["last_selection_reason"] == "reachable_frontiers_exhausted"
    assert snapshot["exhaustion_streak"] == 3
