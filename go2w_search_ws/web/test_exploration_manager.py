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
    def __init__(self, blocked_x=(), origin=(0.0, 0.0)):
        self.blocked_x = tuple(blocked_x)
        self.origin = tuple(float(value) for value in origin)
        self.probes = []

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=3.0):
        self.probes.append((x, y, yaw, frame_id, timeout))
        if any(abs(x - blocked) < 0.2 for blocked in self.blocked_x):
            return {"ok": False, "reason": "blocked"}
        return {
            "ok": True,
            "path_length": math.hypot(
                x - self.origin[0], y - self.origin[1]),
            "poses": 4,
        }


class _VisibilityTracker:
    def __init__(
            self, *, adaptive_step_m=6.0, scene_complexity=0.0,
            gains=None, coverage_candidates=None, lidar_candidates=None,
            coverage_ratio=0.25):
        self.adaptive_step_m = float(adaptive_step_m)
        self.scene_complexity = float(scene_complexity)
        self.gains = dict(gains or {})
        self._coverage_candidates = list(coverage_candidates or [])
        self._lidar_candidates = list(lidar_candidates or [])
        self.coverage_ratio = float(coverage_ratio)
        self.observations = []

    def observe(self, map_msg, robot_pose, scan_snapshot):
        self.observations.append((map_msg, tuple(robot_pose), scan_snapshot))
        return self.snapshot(map_msg)

    def rank_candidates(self, _map_msg, robot_pose, candidates):
        ranked = []
        for source in candidates:
            candidate = dict(source)
            gain = float(self.gains.get(float(candidate["x"]), 0.0))
            candidate.update({
                "base_information_gain": float(
                    candidate.get("information_gain", candidate.get("size", 0.0))),
                "visual_gain": gain,
                "information_gain": gain,
                "adaptive_step_m": self.adaptive_step_m,
                "scene_complexity": self.scene_complexity,
                "forward_clearance_m": self.adaptive_step_m,
                "heading_change": abs(math.atan2(
                    math.sin(float(candidate["yaw"]) - float(robot_pose[2])),
                    math.cos(float(candidate["yaw"]) - float(robot_pose[2])),
                )),
            })
            ranked.append(candidate)
        return ranked

    def coverage_candidates(
            self, _map_msg, _robot_pose, _visited, *, limit=32):
        return [dict(item) for item in self._coverage_candidates[:limit]]

    def lidar_candidates(self, _robot_pose, _visited, *, limit=16):
        return [dict(item) for item in self._lidar_candidates[:limit]]

    def snapshot(self, _map_msg=None):
        return {
            "observed_cells": [{"x": 0.25, "y": 0.25}],
            "visual_coverage_ratio": self.coverage_ratio,
            "coverage_cell_size_m": 0.5,
            "visual_range_m": 8.0,
            "scan_usable": True,
            "forward_clearance_m": self.adaptive_step_m,
            "scene_complexity": self.scene_complexity,
            "adaptive_step_m": self.adaptive_step_m,
        }


def test_score_uses_path_information_heading_and_failure_count():
    candidate = {"size": 20}
    preferred = score_frontier(
        candidate, path_length=2.0, heading_change=0.1, failure_count=0)
    penalized = score_frontier(
        candidate, path_length=3.0, heading_change=1.0, failure_count=2)
    assert preferred > penalized


def test_manager_selects_only_reachable_candidate_and_persists_visit():
    nav = _PlannerPort(blocked_x=(1.5,), origin=(2.0, 3.0))
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


def test_current_room_prefers_nearest_reachable_frontier_over_distant_large_cluster():
    candidates = [
        {
            "x": 5.0, "y": 0.0, "yaw": 0.0, "size": 100,
            "center_cell": (0, 50), "distance": 5.0,
            "information_gain": 100.0,
        },
        {
            "x": 1.5, "y": 0.0, "yaw": 0.0, "size": 5,
            "center_cell": (0, 15), "distance": 1.5,
            "information_gain": 5.0,
        },
    ]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=8.0,
        initial_radius_m=8.0,
        tile_size_m=16.0,
        candidate_selector=lambda *_a, **_k: [dict(item) for item in candidates],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# Mixed-utility mode (2026-07-20): α·size + β·visual_gain − γ·path_cost
# ---------------------------------------------------------------------------
_MIXED_CANDIDATES = [
    {
        "x": 5.0, "y": 0.0, "yaw": 0.0, "size": 100,
        "center_cell": (0, 50), "distance": 5.0,
        "information_gain": 100.0,
    },
    {
        "x": 1.5, "y": 0.0, "yaw": 0.0, "size": 5,
        "center_cell": (0, 15), "distance": 1.5,
        "information_gain": 5.0,
    },
]


def test_mixed_mode_prefers_large_distant_frontier_over_near_small():
    """Same input as the nearest-mode test above, but utility_mode='mixed'.

    With α=0.5, γ=0.5 (β irrelevant — no visibility tracker injected):
      utility(x=5)  = 0.5·100 − 0.5·5  = 47.5
      utility(x=1.5) = 0.5·5   − 0.5·1.5 =  1.75
    The large distant frontier wins, so the dog actively seeks more gain
    instead of walking the nearest small boundary first.
    """
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=8.0,
        initial_radius_m=8.0,
        tile_size_m=16.0,
        candidate_selector=lambda *_a, **_k: [dict(item) for item in _MIXED_CANDIDATES],
        reject_map_edge=False,
        utility_mode="mixed",
        mixed_frontier_weight=0.5,
        mixed_visual_gain_weight=1.0,
        mixed_path_cost_penalty=0.5,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(5.0)
    assert manager.snapshot()["utility_mode"] == "mixed"


def test_mixed_mode_visual_gain_breaks_size_tie():
    """Two frontiers with identical size/distance but different visual_gain.

    The candidate whose viewpoint reveals more unseen cells must win in
    mixed mode (visual_gain is the 'actively explore unexplored area' term).
    """
    candidates = [
        {
            "x": 3.0, "y": 0.0, "yaw": 0.0, "size": 10,
            "center_cell": (0, 30), "distance": 3.0,
            "information_gain": 10.0, "visual_gain": 0.0,
        },
        {
            "x": -3.0, "y": 0.0, "yaw": math.pi, "size": 10,
            "center_cell": (0, -30), "distance": 3.0,
            "information_gain": 10.0, "visual_gain": 40.0,
        },
    ]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(item) for item in candidates],
        reject_map_edge=False,
        utility_mode="mixed",
        mixed_frontier_weight=0.5,
        mixed_visual_gain_weight=1.0,
        mixed_path_cost_penalty=0.5,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(-3.0)
    assert selected["visual_gain"] == 40.0


def test_mixed_mode_env_override_switches_behaviour(monkeypatch):
    """GO2W_FRONTIER_UTILITY_MODE=mixed env var flips the default nearest mode.

    Constructor default is utility_mode='nearest'; the env var must override
    it at construction time so deploy jobs can opt in without code changes.
    """
    monkeypatch.setenv("GO2W_FRONTIER_UTILITY_MODE", "mixed")
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=8.0,
        initial_radius_m=8.0,
        tile_size_m=16.0,
        candidate_selector=lambda *_a, **_k: [dict(item) for item in _MIXED_CANDIDATES],
        reject_map_edge=False,
        # utility_mode intentionally left at default 'nearest'
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert manager.snapshot()["utility_mode"] == "mixed"
    assert selected is not None
    assert selected["x"] == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# Parallel Nav2 probe (2026-07-20): collapse serial probe stall
# ---------------------------------------------------------------------------
def test_parallel_probe_picks_first_reachable_in_eligible_order():
    """parallel_probe_workers>0 must still respect first-reachable-wins.

    Eligible order is nearest-first (x=1.5, 3.0, 5.0). x=1.5 is blocked, so
    the parallel path must return x=3.0 — not the larger x=5.0 — exactly
    matching the serial loop's behaviour.
    """
    candidates = [
        {"x": 1.5, "y": 0.0, "yaw": 0.0, "size": 5,
         "center_cell": (0, 15), "distance": 1.5, "information_gain": 5.0},
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 8,
         "center_cell": (0, 30), "distance": 3.0, "information_gain": 8.0},
        {"x": 5.0, "y": 0.0, "yaw": 0.0, "size": 12,
         "center_cell": (0, 50), "distance": 5.0, "information_gain": 12.0},
    ]
    nav = _PlannerPort(blocked_x=(1.5,))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in candidates],
        reject_map_edge=False,
        parallel_probe_workers=2,
    )
    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["x"] == pytest.approx(3.0)
    # At least two probes fired (blocked + first reachable).
    assert manager.snapshot()["plan_probes"] >= 2


def test_parallel_probe_env_override_enables_it(monkeypatch):
    """GO2W_FRONTIER_PROBE_WORKERS env var opts the manager into parallel probes."""
    monkeypatch.setenv("GO2W_FRONTIER_PROBE_WORKERS", "2")
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in _MIXED_CANDIDATES],
        reject_map_edge=False,
    )
    assert manager.parallel_probe_workers == 2


def test_plan_endpoint_must_reach_candidate_instead_of_stopping_beside_obstacle():
    class ApproximatePlanner:
        def compute_path_to_pose(
                self, x, y, yaw, frame_id="map", timeout=3.0):
            return {
                "ok": True,
                "path_length": 1.0,
                "poses": 4,
                "goal_error_m": 0.65,
            }

    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=ApproximatePlanner(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        max_failures_per_cell=1,
        max_goal_endpoint_error_m=0.25,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["blacklist"][0]["last_reason"] == (
        "plan_endpoint_mismatch")


def test_default_endpoint_tolerance_rejects_a_twenty_centimetre_approximation():
    """Regression: field goals in cost 253 cells ended 0.20 m away."""

    class ApproximatePlanner:
        def compute_path_to_pose(
                self, x, y, yaw, frame_id="map", timeout=3.0):
            return {
                "ok": True,
                "path_length": 1.0,
                "poses": 4,
                "goal_error_m": 0.20,
            }

    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=ApproximatePlanner(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        max_failures_per_cell=1,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["blacklist"][0]["last_reason"] == (
        "plan_endpoint_mismatch")


def test_dynamic_obstacle_revalidation_cancels_goal_after_sustained_failure():
    class DynamicPlanner:
        def __init__(self):
            self.responses = [
                {"ok": True, "path_length": 1.2, "poses": 4,
                 "goal_error_m": 0.0},
                {"ok": False, "reason": "unreachable"},
                {"ok": False, "reason": "unreachable"},
            ]

        def compute_path_to_pose(
                self, x, y, yaw, frame_id="map", timeout=3.0):
            return self.responses.pop(0)

    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=DynamicPlanner(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        goal_revalidation_failures=2,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )
    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is not None

    first = manager.revalidate_current_goal()
    second = manager.revalidate_current_goal()

    assert first["ok"] is True
    assert first["goal_revalidation_failures"] == 1
    assert second["ok"] is False
    assert second["reason"] == "goal_became_unreachable"


def test_transient_revalidation_timeouts_do_not_cancel_an_active_goal():
    class BusyPlanner:
        def __init__(self):
            self.responses = [
                {"ok": True, "path_length": 1.2, "poses": 4,
                 "goal_error_m": 0.0},
                {"ok": False, "reason": "plan_timeout"},
                {"ok": False, "reason": "plan_timeout"},
            ]

        def compute_path_to_pose(
                self, x, y, yaw, frame_id="map", timeout=3.0):
            return self.responses.pop(0)

    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=BusyPlanner(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        goal_revalidation_failures=2,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )
    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is not None

    first = manager.revalidate_current_goal()
    second = manager.revalidate_current_goal()

    assert first["ok"] is True
    assert second["ok"] is True
    assert second["reason"] == "goal_revalidation_inconclusive"
    assert second["goal_revalidation_failures"] == 0


def test_blacklist_is_bounded_and_spatial_failures_survive_revision_churn(
        monkeypatch):
    nav = _PlannerPort(
        blocked_x=(1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5),
        origin=(2.0, 3.0),
    )
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


def test_manager_passes_frontier_spacing_to_candidate_selector():
    seen = []

    def candidates(_map_msg, _pose, _visited, **kwargs):
        seen.append(kwargs.get("frontier_spacing_m"))
        return []

    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        frontier_spacing_m=0.8,
        candidate_selector=candidates,
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert seen == [pytest.approx(0.8)]
    assert manager.snapshot()["frontier_spacing_m"] == pytest.approx(0.8)


def test_manager_supports_legacy_candidate_selector_without_spacing_keyword():
    calls = []

    def legacy_candidates(
            map_msg, robot_pose, visited, *, revisit_radius, origin_pose,
            max_radius, room_polygon, reject_map_edge, failure_counts,
            distance_weight, heading_weight, failure_penalty):
        del (
            map_msg, robot_pose, visited, revisit_radius, origin_pose,
            max_radius, room_polygon, reject_map_edge, failure_counts,
            distance_weight, heading_weight, failure_penalty,
        )
        calls.append(1)
        return []

    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=legacy_candidates,
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert calls == [1]


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


def test_current_room_expands_after_local_frontiers_become_unreachable():
    blocked = {
        "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 1), "distance": 1.0,
        "information_gain": 10.0, "score": 10.0,
    }
    remote = {
        "x": 7.0, "y": 0.0, "yaw": 0.0, "size": 9,
        "center_cell": (0, 7), "distance": 7.0,
        "information_gain": 9.0, "score": 9.0,
    }

    def candidates(_map_msg, _pose, _visited, **kwargs):
        return [dict(remote)] if kwargs.get("max_radius", 0.0) >= 12.0 else [dict(blocked)]

    manager = ExplorationManager(
        navigation_port=_PlannerPort(blocked_x=(1.0, 0.7, 0.4, 0.1)),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=30.0,
        initial_radius_m=6.0,
        radius_step_m=6.0,
        max_failures_per_cell=1,
        candidate_selector=candidates,
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["last_selection_reason"] == "retry_pending"
    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    assert manager.snapshot()["last_selection_reason"] == "search_boundary_expanded"
    assert manager.snapshot()["active_radius_m"] == pytest.approx(12.0)
    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["x"] == pytest.approx(7.0)


def test_frontier_inside_goal_tolerance_is_not_selected():
    near = {
        "x": 0.09, "y": 0.0, "yaw": 0.0, "size": 100,
        "center_cell": (0, 0), "distance": 0.09,
        "information_gain": 100.0, "score": 100.0,
    }
    useful = {
        "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 5,
        "center_cell": (0, 1), "distance": 1.0,
        "information_gain": 5.0, "score": 5.0,
    }
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(near), dict(useful)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(1.0)
    assert manager.navigation_port.probes[0][0] == pytest.approx(1.0)


def test_only_frontier_inside_goal_tolerance_uses_outward_probe():
    frontier = {
        "x": 0.05, "y": 0.0, "yaw": 0.0, "size": 324,
        "center_cell": (121, 132), "distance": 0.05,
        "information_gain": 324.0, "score": 324.0,
    }
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert manager.navigation_port.probes[0][0] == pytest.approx(1.2)
    assert selected["x"] == pytest.approx(1.2)
    assert selected["frontier_x"] == pytest.approx(0.05)
    assert selected["approach_lookahead_m"] == pytest.approx(1.15)
    assert manager.snapshot()["last_selection_reason"] is None


def test_near_frontier_uses_outward_probe_to_escape_tiny_initial_map():
    frontier = {
        "x": 0.45, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 9), "distance": 0.45,
        "information_gain": 10.0, "score": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert manager.navigation_port.probes[0][0] == pytest.approx(1.2)
    assert selected["x"] == pytest.approx(1.2)
    assert selected["frontier_x"] == pytest.approx(0.45)
    assert selected["approach_lookahead_m"] == pytest.approx(0.75)


def test_zero_progress_planner_path_is_rejected_as_false_reachability():
    class ZeroProgressPlanner:
        def __init__(self):
            self.probes = []

        def compute_path_to_pose(
                self, x, y, yaw, frame_id="map", timeout=3.0):
            self.probes.append((x, y, yaw, frame_id, timeout))
            return {"ok": True, "path_length": 0.0, "poses": 1}

    frontier = {
        "x": 0.45, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 9), "distance": 0.45,
        "information_gain": 10.0, "score": 10.0,
    }
    nav = ZeroProgressPlanner()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        max_failures_per_cell=1,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    snapshot = manager.snapshot()
    assert nav.probes[0][0] == pytest.approx(1.2)
    assert snapshot["blacklist"][0]["last_reason"] == "degenerate_plan"


def test_excessive_detour_plan_is_rejected_before_navigation():
    class DetourPlanner:
        def __init__(self):
            self.probes = []

        def compute_path_to_pose(
                self, x, y, yaw, frame_id="map", timeout=3.0):
            self.probes.append((x, y, yaw, frame_id, timeout))
            return {"ok": True, "path_length": 8.47, "poses": 158}

    frontier = {
        "x": -0.84, "y": -0.83, "yaw": -2.37, "size": 10,
        "center_cell": (10, 10), "distance": 1.18,
        "information_gain": 10.0, "score": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=DetourPlanner(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        max_failures_per_cell=1,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0)) is None
    snapshot = manager.snapshot()
    assert snapshot["blacklist"][0]["last_reason"] == "excessive_plan_detour"
    assert snapshot["max_path_stretch_ratio"] == pytest.approx(3.0)
    assert snapshot["max_path_detour_m"] == pytest.approx(1.5)


def test_navigation_progress_guard_latches_after_repeated_motion_away_from_goal():
    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 10.0, "score": 10.0,
    }
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )
    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None

    assert manager.observe_navigation_pose((0.0, 0.0, 0.0))["ok"] is True
    first = manager.observe_navigation_pose((-1.0, 0.0, 0.0))
    second = manager.observe_navigation_pose((-1.0, 0.0, 0.0))
    third = manager.observe_navigation_pose((-1.0, 0.0, 0.0))

    assert first["ok"] is True
    assert second["ok"] is True
    assert third["ok"] is False
    assert third["reason"] == "navigation_diverging"
    assert third["distance_to_goal_m"] == pytest.approx(2.2)
    assert third["allowed_distance_m"] == pytest.approx(1.8)


def test_unreachable_frontier_uses_inward_standoff_goal():
    frontier = {
        "x": 1.4, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 14), "distance": 1.4,
        "information_gain": 10.0, "score": 10.0,
    }
    nav = _PlannerPort(blocked_x=(1.4,))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert [probe[0] for probe in nav.probes[:2]] == pytest.approx([1.4, 1.1])
    assert selected["x"] == pytest.approx(1.1)
    assert selected["frontier_x"] == pytest.approx(1.4)
    assert selected["approach_standoff_m"] == pytest.approx(0.3)


def test_reachable_frontier_prefers_farthest_inward_standoff():
    frontier = {
        "x": 2.0, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 20), "distance": 2.0,
        "information_gain": 10.0, "score": 10.0,
        "prefer_standoff": True,
    }
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert nav.probes[0][0] == pytest.approx(1.1)
    assert selected["x"] == pytest.approx(1.1)
    assert selected["frontier_x"] == pytest.approx(2.0)
    assert selected["approach_standoff_m"] == pytest.approx(0.9)


def test_large_initial_turn_uses_heading_aligned_staging_probe():
    frontier = {
        "x": -2.0, "y": 0.0, "yaw": math.pi, "size": 10,
        "center_cell": (0, -20), "distance": 2.0,
        "information_gain": 10.0, "score": 10.0,
        "heading_change": math.pi,
        "prefer_standoff": True,
    }
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert nav.probes[0][0] == pytest.approx(0.8)
    assert nav.probes[0][1] == pytest.approx(0.0)
    assert nav.probes[0][2] == pytest.approx(0.0)
    assert selected["x"] == pytest.approx(0.8)
    assert selected["y"] == pytest.approx(0.0)
    assert selected["frontier_x"] == pytest.approx(-2.0)
    assert selected["approach_staging_m"] == pytest.approx(0.8)
    assert selected["staging_for_heading_change_rad"] == pytest.approx(math.pi)


def test_turn_staging_is_transition_not_completed_frontier():
    frontier = {
        "x": -2.0, "y": 0.0, "yaw": math.pi, "size": 10,
        "center_cell": (0, -20), "distance": 2.0,
        "information_gain": 10.0, "score": 10.0,
        "heading_change": math.pi,
        "prefer_standoff": True,
    }
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    staging = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert staging["approach_staging_m"] == pytest.approx(0.8)
    manager.mark_visited(staging)

    assert manager.snapshot()["visited_frontiers"] == []
    target = manager.choose_next(_two_room_map(), (0.8, 0.0, 0.0))
    assert target is not None
    assert "approach_staging_m" not in target
    assert target["x"] < 0.0


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
    assert len(manager.navigation_port.probes) == 1


def test_ranked_reachable_candidate_starts_without_probing_every_alternative():
    candidates = [
        {
            "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 8,
            "center_cell": (0, 10), "distance": 1.0,
            "information_gain": 8.0,
        },
        {
            "x": 2.0, "y": 0.0, "yaw": 0.0, "size": 7,
            "center_cell": (0, 20), "distance": 2.0,
            "information_gain": 7.0,
        },
        {
            "x": 3.0, "y": 0.0, "yaw": 0.0, "size": 6,
            "center_cell": (0, 30), "distance": 3.0,
            "information_gain": 6.0,
        },
    ]
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=8.0,
        initial_radius_m=8.0,
        tile_size_m=16.0,
        candidate_selector=lambda *_a, **_k: [dict(item) for item in candidates],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(1.0)
    assert len(nav.probes) == 1


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


def test_same_world_frontier_is_bounded_when_grid_origin_shifts():
    """Growing SLAM maps may reindex a fixed world point every revision."""
    nav = _PlannerPort(blocked_x=(1.0,))
    calls = [0]

    def candidates(_map_msg, _pose, _visited, **_kwargs):
        calls[0] += 1
        return [{
            "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 10,
            "center_cell": (100, 100 + calls[0]),
            "distance": 1.0, "information_gain": 10.0, "score": 10.0,
        }]

    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        max_failures_per_cell=1,
        candidate_selector=candidates,
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(1), (0.0, 0.0, 0.0)) is None
    probes_after_first_rejection = len(nav.probes)
    assert manager.choose_next(_two_room_map(2), (0.0, 0.0, 0.0)) is None

    assert probes_after_first_rejection == 1
    assert len(nav.probes) == probes_after_first_rejection


def test_small_world_drift_cannot_cross_spatial_failure_bucket_boundary():
    nav = _PlannerPort(blocked_x=(1.25,))
    calls = [0]

    def candidates(_map_msg, _pose, _visited, **_kwargs):
        calls[0] += 1
        x = 1.24 if calls[0] == 1 else 1.26
        return [{
            "x": x, "y": 0.0, "yaw": 0.0, "size": 10,
            "center_cell": (100, 100 + calls[0]),
            "distance": x, "information_gain": 10.0, "score": 10.0,
        }]

    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        max_failures_per_cell=1,
        candidate_selector=candidates,
        reject_map_edge=False,
    )

    assert manager.choose_next(_two_room_map(1), (0.0, 0.0, 0.0)) is None
    assert manager.choose_next(_two_room_map(2), (0.0, 0.0, 0.0)) is None

    assert len(nav.probes) == 1


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


@pytest.mark.parametrize(("adaptive_step_m", "expected_x"), [
    (6.0, 6.0),
    (1.5, 1.5),
])
def test_visibility_profile_adapts_long_frontier_step_to_scene(
        adaptive_step_m, expected_x):
    tracker = _VisibilityTracker(
        adaptive_step_m=adaptive_step_m,
        scene_complexity=0.0 if adaptive_step_m > 2.0 else 0.8,
        gains={9.0: 50.0},
    )
    frontier = {
        "x": 9.0, "y": 0.0, "yaw": 0.0, "size": 50,
        "center_cell": (0, 90), "distance": 9.0,
        "information_gain": 50.0,
    }
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=12.0,
        initial_radius_m=12.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        max_frontier_standoff_steps=0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(expected_x)
    assert selected["frontier_x"] == pytest.approx(9.0)
    assert selected["approach_adaptive_m"] == pytest.approx(adaptive_step_m)
    assert nav.probes[0][0] == pytest.approx(expected_x)


def test_near_frontier_looks_ahead_to_lidar_confirmed_safe_step():
    tracker = _VisibilityTracker(
        adaptive_step_m=6.0,
        scene_complexity=0.05,
        gains={1.2: 20.0},
    )
    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 20,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 20.0, "score": 20.0,
        "prefer_standoff": True,
    }
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=12.0,
        initial_radius_m=12.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert nav.probes[0][0] == pytest.approx(6.0)
    assert selected["x"] == pytest.approx(6.0)
    assert selected["frontier_x"] == pytest.approx(1.2)
    assert selected["approach_lidar_lookahead_m"] == pytest.approx(4.8)


def test_open_corridor_does_not_fall_back_to_tiny_goal_when_lookahead_is_blocked():
    tracker = _VisibilityTracker(
        adaptive_step_m=6.0,
        scene_complexity=0.05,
        gains={1.2: 20.0},
    )
    frontier = {
        "x": 1.2, "y": 0.0, "yaw": 0.0, "size": 20,
        "center_cell": (0, 12), "distance": 1.2,
        "information_gain": 20.0, "score": 20.0,
        "prefer_standoff": True,
    }
    nav = _PlannerPort(blocked_x=(6.0,))
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=12.0,
        initial_radius_m=12.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is None
    assert [probe[0] for probe in nav.probes] == [pytest.approx(6.0)]


def test_lidar_fallback_prevents_false_exhaustion_with_collapsed_map_coverage():
    lidar_goal = {
        "x": 7.0, "y": 0.0, "yaw": 0.0, "size": 50,
        "center_cell": (0, 14), "distance": 7.0,
        "information_gain": 50.0, "visual_gain": 50,
        "coverage_candidate": True, "lidar_candidate": True,
        "prefer_standoff": False, "adaptive_step_m": 7.0,
        "scene_complexity": 0.0, "forward_clearance_m": 8.0,
        "path_clearance_m": 8.0, "heading_change": 0.0,
    }
    tracker = _VisibilityTracker(
        coverage_ratio=1.0,
        coverage_candidates=[],
        lidar_candidates=[lidar_goal],
    )
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=12.0,
        initial_radius_m=12.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(7.0)
    assert selected["lidar_candidate"] is True
    assert manager.snapshot()["last_selection_reason"] is None


def test_lidar_fallback_prefers_long_open_corridor_over_short_side_step():
    """Distance cost must not turn safe LiDAR fallback back into small steps."""
    candidates = [
        {
            "x": 3.4, "y": 0.0, "yaw": math.pi / 2.0, "size": 205,
            "center_cell": (0, 7), "distance": 3.4,
            "information_gain": 71.75, "visual_gain": 205,
            "coverage_candidate": True, "lidar_candidate": True,
            "prefer_standoff": False, "adaptive_step_m": 3.4,
            "scene_complexity": 1.0, "forward_clearance_m": 5.1,
            "path_clearance_m": 5.1, "heading_change": math.pi / 2.0,
        },
        {
            "x": 7.389, "y": 0.0, "yaw": 0.0, "size": 205,
            "center_cell": (0, 15), "distance": 7.389,
            "information_gain": 71.75, "visual_gain": 205,
            "coverage_candidate": True, "lidar_candidate": True,
            "prefer_standoff": False, "adaptive_step_m": 7.389,
            "scene_complexity": 0.0, "forward_clearance_m": 7.989,
            "path_clearance_m": 7.989, "heading_change": 0.0,
        },
    ]
    tracker = _VisibilityTracker(
        coverage_ratio=1.0,
        coverage_candidates=[],
        lidar_candidates=candidates,
    )
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=12.0,
        initial_radius_m=12.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(7.389)
    assert selected["lidar_progress_bonus"] > 0.0


def test_open_lidar_corridor_skips_short_turn_staging_for_long_lookahead():
    """Open-factory frontiers must not become repeated 0.8 m staging goals."""
    tracker = _VisibilityTracker(
        adaptive_step_m=6.0,
        scene_complexity=0.05,
        gains={-1.2: 20.0},
    )
    frontier = {
        "x": -1.2, "y": 0.0, "yaw": math.pi, "size": 20,
        "center_cell": (0, -12), "distance": 1.2,
        "information_gain": 20.0, "score": 20.0,
        "prefer_standoff": True,
    }
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=12.0,
        initial_radius_m=12.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert nav.probes[0][0] == pytest.approx(-6.0)
    assert selected["x"] == pytest.approx(-6.0)
    assert selected["approach_lidar_lookahead_m"] == pytest.approx(4.8)
    assert "approach_staging_m" not in selected


def test_visual_gain_prioritizes_unobserved_viewpoint():
    tracker = _VisibilityTracker(
        adaptive_step_m=8.0,
        gains={1.0: 0.0, 2.0: 20.0},
    )
    candidates = [
        {
            "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 100,
            "center_cell": (0, 10), "distance": 1.0,
            "information_gain": 100.0,
        },
        {
            "x": 2.0, "y": 0.0, "yaw": 0.0, "size": 1,
            "center_cell": (0, 20), "distance": 2.0,
            "information_gain": 1.0,
        },
    ]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [dict(c) for c in candidates],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(2.0)
    assert selected["visual_gain"] == pytest.approx(20.0)


def test_already_observed_frontier_is_dropped_when_new_visual_coverage_exists():
    class BaseGainTracker(_VisibilityTracker):
        def rank_candidates(self, _map_msg, _robot_pose, candidates):
            ranked = []
            for source in candidates:
                item = dict(source)
                gain = 0.0 if item["x"] == 1.0 else 2.0
                item.update({
                    "visual_gain": gain,
                    "information_gain": float(item["size"]) + gain * 0.35,
                    "adaptive_step_m": 1.0,
                    "scene_complexity": 1.0,
                    "forward_clearance_m": 1.0,
                    "path_clearance_m": 1.0,
                    "heading_change": 0.0,
                })
                ranked.append(item)
            return ranked

    candidates = [
        {
            "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 100,
            "center_cell": (0, 10), "distance": 1.0,
            "information_gain": 100.0,
        },
        {
            "x": 2.0, "y": 0.0, "yaw": 0.0, "size": 1,
            "center_cell": (0, 20), "distance": 2.0,
            "information_gain": 1.0,
        },
    ]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        tile_size_m=20.0,
        visibility_tracker=BaseGainTracker(),
        candidate_selector=lambda *_a, **_k: [dict(c) for c in candidates],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected.get("frontier_x", selected["x"]) == pytest.approx(2.0)


def test_frontier_goal_inside_already_visited_viewpoint_radius_is_not_revisited():
    frontier = {
        "x": 1.1, "y": 0.0, "yaw": 0.0, "size": 10,
        "center_cell": (0, 11), "distance": 1.1,
        "information_gain": 10.0, "prefer_standoff": True,
    }
    nav = _PlannerPort()
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        revisit_radius_m=1.0,
        candidate_selector=lambda *_a, **_k: [dict(frontier)],
        reject_map_edge=False,
    )
    manager.mark_visited({"x": 1.0, "y": 0.0, "path_length": 1.0})

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is None
    assert nav.probes == []


def test_open_space_heading_penalty_avoids_unnecessary_back_and_forth():
    tracker = _VisibilityTracker(
        adaptive_step_m=8.0,
        scene_complexity=0.0,
        gains={4.0: 10.0, -4.0: 11.0},
    )
    candidates = [
        {
            "x": 4.0, "y": 0.0, "yaw": 0.0, "size": 10,
            "center_cell": (0, 40), "distance": 4.0,
            "information_gain": 10.0,
        },
        {
            "x": -4.0, "y": 0.0, "yaw": math.pi, "size": 11,
            "center_cell": (0, -40), "distance": 4.0,
            "information_gain": 11.0,
        },
    ]
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        heading_weight=0.0,
        open_space_heading_weight=1.0,
        max_frontier_standoff_steps=0,
        candidate_selector=lambda *_a, **_k: [dict(c) for c in candidates],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["x"] == pytest.approx(4.0)


def test_visual_coverage_candidate_prevents_false_frontier_exhaustion():
    coverage_candidate = {
        "x": 2.0, "y": 0.0, "yaw": math.pi / 2.0, "size": 30,
        "center_cell": (0, 20), "distance": 2.0,
        "information_gain": 10.0, "visual_gain": 30,
        "adaptive_step_m": 4.0, "scene_complexity": 0.2,
        "coverage_candidate": True,
    }
    tracker = _VisibilityTracker(
        adaptive_step_m=4.0,
        coverage_candidates=[coverage_candidate],
        coverage_ratio=0.25,
    )
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        tile_size_m=20.0,
        visibility_tracker=tracker,
        visual_coverage_threshold=0.9,
        candidate_selector=lambda *_a, **_k: [],
        reject_map_edge=False,
    )

    selected = manager.choose_next(_two_room_map(), (0.0, 0.0, 0.0))

    assert selected is not None
    assert selected["coverage_candidate"] is True
    assert manager.snapshot()["last_selection_reason"] is None


def test_visibility_snapshot_and_observation_are_exposed_for_frontend():
    tracker = _VisibilityTracker(adaptive_step_m=5.0, scene_complexity=0.1)
    manager = ExplorationManager(
        navigation_port=_PlannerPort(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=6.0,
        initial_radius_m=6.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [],
        reject_map_edge=False,
    )
    scan = {"ranges": [8.0], "age_sec": 0.0}

    observed = manager.observe_environment(
        _two_room_map(), (0.0, 0.0, 0.0), scan)
    snapshot = manager.snapshot()

    assert len(tracker.observations) == 1
    assert observed["adaptive_step_m"] == pytest.approx(5.0)
    assert snapshot["visibility"]["observed_cells"] == [
        {"x": 0.25, "y": 0.25}
    ]


def test_find_frontier_clusters_reports_adjacent_wall_count():
    """墙角 frontier 的 adjacent_wall_count >= 1；朝开阔区的可能 == 0。

    10m x 10m 房间, 四周墙, robot 在 (5,5)。靠墙的 frontier 邻接 occupied。
    """
    from nx_frontier_planner import find_frontier_clusters
    resolution = 0.5
    width = height = 20
    data = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            if row in {0, height - 1} or col in {0, width - 1}:
                data[row * width + col] = 100  # 墙
    # 在 (1,1) 放 unknown, 紧贴两面墙；frontier 代表 cell 落入墙的 support_radius。
    data[1 * width + 1] = -1
    map_msg = type("M", (), {
        "info": type("I", (), {
            "resolution": resolution, "width": width, "height": height,
            "origin": type("O", (), {
                "position": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0}),
                "orientation": type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
            }),
        })(),
        "data": data,
    })()
    clusters = find_frontier_clusters(map_msg, (5.0, 5.0, 0.0), [], min_cluster_size=1)
    assert clusters, "应至少有一个 frontier cluster"
    assert all("adjacent_wall_count" in c for c in clusters), \
        "每个 cluster 必须带 adjacent_wall_count 字段"
    assert any(c["adjacent_wall_count"] >= 1 for c in clusters), \
        "靠墙 frontier 的 adjacent_wall_count 应 >= 1"


def test_visual_gain_at_counts_unobserved_cells_for_yaw():
    """同一 (x,y)，不同 yaw 的 visual_gain > 0（朝向敏感）。"""
    from nx_visibility_coverage import VisibilityCoverageTracker
    resolution = 0.5
    width = height = 20
    data = [-1] * (width * height)
    for dr in range(-2, 3):
        for dc in range(-2, 3):
            data[(10 + dr) * width + (10 + dc)] = 0
    map_msg = type("M", (), {
        "info": type("I", (), {
            "resolution": resolution, "width": width, "height": height,
            "origin": type("O", (), {
                "position": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0}),
                "orientation": type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
            }),
        })(),
        "data": data,
    })()
    tracker = VisibilityCoverageTracker(camera_hfov_rad=1.2, visual_range_m=5.0)
    gain_east = tracker.visual_gain_at(map_msg, 5.0, 5.0, 0.0)
    gain_west = tracker.visual_gain_at(map_msg, 5.0, 5.0, math.pi)
    assert gain_east > 0, "朝东应扫到未观测 cell"
    assert gain_west > 0, "朝西应扫到未观测 cell"
