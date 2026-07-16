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


def test_blacklist_is_bounded_and_keyed_by_map_revision(monkeypatch):
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
    assert len(nav.probes) > probes_same_revision


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
