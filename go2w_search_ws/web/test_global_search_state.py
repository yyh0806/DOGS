"""Contracts for entrance-gated global exploration completion."""

import importlib
import math
from types import SimpleNamespace

import pytest


def _api():
    try:
        return importlib.import_module("nx_global_search_state")
    except ModuleNotFoundError:
        pytest.fail("nx_global_search_state is not implemented yet")


def _grid(*, far_door_open: bool, enclosed_pocket: bool = False):
    """Two-sided room: entrance at west, another opening at east."""

    width, height = 15, 11
    data = [-1] * (width * height)

    # Search room interior.
    for row in range(2, 9):
        for col in range(3, 10):
            data[row * width + col] = 0

    # One-cell-thick room shell.
    for col in range(2, 11):
        data[1 * width + col] = 100
        data[9 * width + col] = 100
    for row in range(1, 10):
        data[row * width + 2] = 100
        data[row * width + 10] = 100

    # Initial west entrance. The dog starts in this opening, facing east.
    data[5 * width + 2] = 0
    data[5 * width + 1] = 0
    data[5 * width + 0] = 0

    # A non-entry opening must remain an exploration obligation.
    if far_door_open:
        data[5 * width + 10] = 0

    if enclosed_pocket:
        # Occupied ring around one unknown cell inside the room.
        for row in range(3, 6):
            for col in range(5, 8):
                data[row * width + col] = 100
        data[4 * width + 6] = -1

    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0),
        ),
        info=SimpleNamespace(
            resolution=1.0,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0,
                ),
            ),
        ),
        data=data,
    )


def _all_known_free_centers(map_msg):
    width = map_msg.info.width
    return [
        {"x": (index % width) + 0.5, "y": (index // width) + 0.5}
        for index, value in enumerate(map_msg.data)
        if value == 0
    ]


def test_initial_entrance_excludes_rear_but_preserves_other_opening():
    api = _api()
    map_msg = _grid(far_door_open=True)
    mission_origin = (2.5, 5.5, 0.0)

    gate = api.infer_entrance_gate(map_msg, mission_origin)
    state = api.analyze_global_search_state(
        map_msg,
        mission_origin=mission_origin,
        entrance_gate=gate,
        observed_cells=_all_known_free_centers(map_msg),
        traversal_clearance_m=0.0,
    )

    assert gate is not None
    assert gate["width_m"] == pytest.approx(2.0)
    assert state["entrance_excluded_edge_count"] > 0
    assert state["traversable_opening_count"] == 1
    assert state["opening_components"][0]["center_x"] > 10.0
    assert state["completion_eligible"] is False


def test_closed_non_entry_boundary_can_complete_at_95_percent_coverage():
    api = _api()
    map_msg = _grid(far_door_open=False)
    mission_origin = (2.5, 5.5, 0.0)
    gate = api.infer_entrance_gate(map_msg, mission_origin)

    state = api.analyze_global_search_state(
        map_msg,
        mission_origin=mission_origin,
        entrance_gate=gate,
        observed_cells=_all_known_free_centers(map_msg),
        coverage_threshold=0.95,
        traversal_clearance_m=0.0,
    )

    assert state["valid"] is True
    assert state["traversable_opening_count"] == 0
    assert state["explainable_coverage_ratio"] == pytest.approx(1.0)
    assert state["completion_eligible"] is True


def test_entrance_gate_is_finite_and_only_rearward_crossing_is_rejected():
    api = _api()
    gate = {
        "center_x": 2.5,
        "center_y": 5.5,
        "yaw": 0.0,
        "width_m": 2.0,
    }

    forward_path = [(2.5, 5.5), (3.5, 5.5), (5.5, 5.5)]
    rearward_path = [(5.5, 5.5), (3.5, 5.5), (2.5, 5.5), (1.5, 5.5)]
    around_gate_endpoint = [
        (5.5, 5.5), (3.5, 7.0), (1.5, 7.0), (1.0, 6.5),
    ]

    assert api.path_crosses_entrance_gate(forward_path, gate) is False
    assert api.path_crosses_entrance_gate(rearward_path, gate) is True
    assert api.path_crosses_entrance_gate(around_gate_endpoint, gate) is False


def test_path_starting_on_entrance_gate_cannot_step_rearward():
    api = _api()
    gate = {
        "center_x": 2.5,
        "center_y": 5.5,
        "yaw": 0.0,
        "width_m": 2.0,
    }

    assert api.path_crosses_entrance_gate(
        [(2.5, 5.5), (1.5, 5.5)], gate
    ) is True
    assert api.path_crosses_entrance_gate(
        [(2.5, 5.5), (3.5, 5.5)], gate
    ) is False


def test_enclosed_unknown_is_occluded_but_an_open_door_never_is():
    api = _api()
    map_msg = _grid(far_door_open=True, enclosed_pocket=True)
    mission_origin = (2.5, 5.5, 0.0)
    gate = api.infer_entrance_gate(map_msg, mission_origin)

    state = api.analyze_global_search_state(
        map_msg,
        mission_origin=mission_origin,
        entrance_gate=gate,
        observed_cells=_all_known_free_centers(map_msg),
        traversal_clearance_m=0.0,
    )

    assert state["certified_occluded_unknown_cell_count"] == 1
    assert state["traversable_opening_count"] == 1
    assert math.isclose(state["explainable_coverage_ratio"], 1.0)
    assert state["completion_eligible"] is False


def test_frontier_selection_excludes_entrance_rear_and_keeps_far_door():
    from nx_frontier_planner import select_frontier_candidates

    api = _api()
    map_msg = _grid(far_door_open=True)
    mission_origin = (2.5, 5.5, 0.0)
    gate = api.infer_entrance_gate(map_msg, mission_origin)

    candidates = select_frontier_candidates(
        map_msg,
        (3.5, 5.5, 0.0),
        [],
        min_cluster_size=1,
        revisit_radius=0.0,
        reject_map_edge=False,
        entrance_gate=gate,
    )

    assert candidates
    assert all(candidate["x"] > 10.0 for candidate in candidates)


def test_manager_rejects_a_planner_path_that_returns_through_entrance():
    from nx_exploration_manager import ExplorationManager

    gate = {
        "center_x": 2.5,
        "center_y": 5.5,
        "yaw": 0.0,
        "width_m": 2.0,
    }

    class RearCrossingPlanner:
        def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=3.0):
            return {
                "ok": True,
                "path_length": 2.0,
                "poses": 3,
                "goal_error_m": 0.0,
                "path": [
                    {"x": 3.5, "y": 5.5},
                    {"x": 2.5, "y": 5.5},
                    {"x": 1.5, "y": 5.5},
                ],
            }

    candidate = {
        "x": 1.5,
        "y": 5.5,
        "yaw": math.pi,
        "size": 4,
        "information_gain": 4.0,
        "distance": 2.0,
        "center_cell": (5, 1),
    }
    manager = ExplorationManager(
        navigation_port=RearCrossingPlanner(),
        mission_origin=(2.5, 5.5, 0.0),
        entrance_gate=gate,
        mode="current_room",
        room_radius_m=10.0,
        initial_radius_m=10.0,
        max_frontier_standoff_steps=0,
        max_plan_probes=2,
        reject_map_edge=False,
        candidate_selector=lambda *_args, **_kwargs: [dict(candidate)],
    )

    selected = manager.choose_next(_grid(far_door_open=True), (3.5, 5.5, 0.0))

    assert selected is None
    assert manager.snapshot()["blacklist"][-1]["last_reason"] == (
        "path_crosses_entrance_gate"
    )


class _CompleteVisibility:
    def observe(self, map_msg, robot_pose, scan_snapshot):
        del robot_pose, scan_snapshot
        return self.snapshot(map_msg)

    def rank_candidates(self, map_msg, robot_pose, candidates):
        del map_msg, robot_pose
        return list(candidates)

    def snapshot(self, map_msg=None):
        return {
            "observed_cells": (
                _all_known_free_centers(map_msg) if map_msg is not None else []
            ),
            "observed_cell_count": (
                len(_all_known_free_centers(map_msg)) if map_msg is not None else 0
            ),
            "visual_coverage_ratio": 1.0,
        }


def _exhaustion_manager(map_msg, *, stable_cycles=3):
    from nx_exploration_manager import ExplorationManager

    mission_origin = (2.5, 5.5, 0.0)
    gate = _api().infer_entrance_gate(map_msg, mission_origin)
    return ExplorationManager(
        navigation_port=object(),
        mission_origin=mission_origin,
        entrance_gate=gate,
        mode="current_room",
        room_radius_m=20.0,
        initial_radius_m=20.0,
        stable_exhaustion_cycles=stable_cycles,
        visibility_tracker=_CompleteVisibility(),
        candidate_selector=lambda *_args, **_kwargs: [],
        reject_map_edge=False,
    )


def test_manager_never_confirms_exhaustion_with_a_non_entry_opening():
    map_msg = _grid(far_door_open=True)
    manager = _exhaustion_manager(map_msg, stable_cycles=1)

    selected = manager.choose_next(map_msg, (3.5, 5.5, 0.0))

    assert selected is None
    state = manager.snapshot()
    assert state["last_selection_reason"] == "traversable_opening_blocked"
    assert state["exhaustion_streak"] == 0
    assert state["global_search"]["traversable_opening_count"] == 1


def test_manager_confirms_closed_boundary_on_three_distinct_map_revisions():
    first = _grid(far_door_open=False)
    second = _grid(far_door_open=False)
    third = _grid(far_door_open=False)
    # Occupancy probability changes while geometry/topology remains closed.
    second.data[1 * second.info.width + 3] = 99
    third.data[1 * third.info.width + 3] = 98
    manager = _exhaustion_manager(first, stable_cycles=3)

    assert manager.choose_next(first, (3.5, 5.5, 0.0)) is None
    assert manager.snapshot()["exhaustion_streak"] == 1
    assert manager.choose_next(first, (3.5, 5.5, 0.0)) is None
    assert manager.snapshot()["exhaustion_streak"] == 1
    assert manager.choose_next(second, (3.5, 5.5, 0.0)) is None
    assert manager.snapshot()["exhaustion_streak"] == 2
    assert manager.choose_next(third, (3.5, 5.5, 0.0)) is None

    state = manager.snapshot()
    assert state["last_selection_reason"] == "reachable_frontiers_exhausted"
    assert state["exhaustion_streak"] == 3
    assert state["global_search"]["completion_eligible"] is True


def test_global_search_default_traversable_width_is_0_8m():
    map_msg = _grid(far_door_open=False)
    mission_origin = (2.5, 5.5, 0.0)
    gate = _api().infer_entrance_gate(map_msg, mission_origin)

    state = _api().analyze_global_search_state(
        map_msg,
        mission_origin=mission_origin,
        entrance_gate=gate,
        observed_cells=_all_known_free_centers(map_msg),
    )
    manager = _exhaustion_manager(map_msg, stable_cycles=1)

    assert state["traversal_clearance_m"] == pytest.approx(0.40)
    assert manager.snapshot()["global_traversal_clearance_m"] == pytest.approx(0.40)


def test_global_search_without_an_entrance_gate_uses_an_arbitrary_start_pose():
    api = _api()
    map_msg = _grid(far_door_open=False)
    mission_origin = (6.5, 5.5, math.pi / 2.0)

    state = api.analyze_global_search_state(
        map_msg,
        mission_origin=mission_origin,
        entrance_gate=None,
        observed_cells=_all_known_free_centers(map_msg),
        traversal_clearance_m=0.0,
    )

    assert state["valid"] is True
    assert state["entrance_gate"] is None
    assert state["entrance_excluded_edge_count"] == 0
    assert state["reachable_free_cell_count"] > 0


def test_manager_keeps_global_completion_checks_without_an_entrance_gate():
    from nx_exploration_manager import ExplorationManager

    map_msg = _grid(far_door_open=False)
    manager = ExplorationManager(
        navigation_port=object(),
        mission_origin=(6.5, 5.5, 0.0),
        entrance_gate=None,
        mode="current_room",
        room_radius_m=20.0,
        initial_radius_m=20.0,
        stable_exhaustion_cycles=1,
        visibility_tracker=_CompleteVisibility(),
        candidate_selector=lambda *_args, **_kwargs: [],
        reject_map_edge=False,
        global_traversal_clearance_m=0.0,
    )

    assert manager.choose_next(map_msg, (6.5, 5.5, 0.0)) is None
    state = manager.snapshot()
    assert state["global_search"]["valid"] is True
    assert state["global_search"]["entrance_gate"] is None
    assert state["global_search"]["entrance_excluded_edge_count"] == 0


def test_visibility_cells_cover_their_full_area_on_a_finer_occupancy_grid():
    api = _api()
    width = height = 10
    map_msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        info=SimpleNamespace(
            resolution=0.05,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * (width * height),
    )

    state = api.analyze_global_search_state(
        map_msg,
        mission_origin=(0.2, 0.2, 0.0),
        entrance_gate=None,
        observed_cells=[{"x": 0.25, "y": 0.25}],
        observed_cell_size_m=0.5,
        traversal_clearance_m=0.0,
    )

    assert state["observed_reachable_free_cell_count"] == 100
    assert state["explainable_coverage_ratio"] == pytest.approx(1.0)
    assert state["completion_eligible"] is True
