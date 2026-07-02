import sys
from pathlib import Path

import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_active_search import ActiveSearchPlanner


def test_generates_candidates_inside_room_only():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.4)
    room_area = {"origin_x": 2.0, "origin_y": -1.0, "width": 3.0, "height": 2.0}

    candidates = planner.generate_candidates(
        room_area,
        robot_pose=(2.0, -1.0, 0.0),
        obstacles=[],
    )

    assert candidates
    assert all(2.0 <= c["x"] <= 5.0 and -1.0 <= c["y"] <= 1.0 for c in candidates)
    assert {
        "x",
        "y",
        "yaw",
        "information_gain",
        "visual_coverage_gain",
        "obstacle_risk_cost",
        "repeated_observation_penalty",
    }.issubset(candidates[0])


@pytest.mark.parametrize("spacing", [0.0, -1.0, float("inf"), float("nan")])
def test_rejects_invalid_spacing(spacing):
    with pytest.raises(ValueError, match="spacing must be a finite positive value"):
        ActiveSearchPlanner(spacing=spacing)


@pytest.mark.parametrize("obstacle_clearance", [0.0, -0.1, float("inf"), float("nan")])
def test_rejects_invalid_obstacle_clearance(obstacle_clearance):
    with pytest.raises(ValueError, match="obstacle_clearance must be a finite positive value"):
        ActiveSearchPlanner(obstacle_clearance=obstacle_clearance)


@pytest.mark.parametrize(
    "room_area",
    [
        {"origin_x": float("inf"), "origin_y": 0.0, "width": 2.0, "height": 2.0},
        {"origin_x": 0.0, "origin_y": float("nan"), "width": 2.0, "height": 2.0},
        {"origin_x": 0.0, "origin_y": 0.0, "width": 0.0, "height": 2.0},
        {"origin_x": 0.0, "origin_y": 0.0, "width": -1.0, "height": 2.0},
        {"origin_x": 0.0, "origin_y": 0.0, "width": float("inf"), "height": 2.0},
        {"origin_x": 0.0, "origin_y": 0.0, "width": 2.0, "height": float("nan")},
    ],
)
def test_rejects_invalid_room_geometry(room_area):
    planner = ActiveSearchPlanner()

    with pytest.raises(ValueError, match="room_area"):
        planner.generate_candidates(room_area, robot_pose=(0.0, 0.0, 0.0), obstacles=[])


def test_malformed_obstacles_are_ignored_without_crashing():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.6)
    room_area = {"origin_x": 0.0, "origin_y": 0.0, "width": 2.0, "height": 2.0}

    candidates = planner.generate_candidates(
        room_area,
        robot_pose=(0.0, 0.0, 0.0),
        obstacles=[None, 7, "bad", {"x": 1.0}, [1.0], object()],
    )

    assert candidates


def test_non_finite_obstacles_do_not_disable_valid_obstacle_filtering():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.6)
    room_area = {"origin_x": 0.0, "origin_y": 0.0, "width": 2.0, "height": 2.0}

    candidates = planner.generate_candidates(
        room_area,
        robot_pose=(0.0, 0.0, 0.0),
        obstacles=[[float("nan"), 0.0], [float("inf"), 1.0], [1.0, 1.0]],
    )

    assert (1.0, 1.0) not in {(c["x"], c["y"]) for c in candidates}


def test_filters_candidates_near_obstacle():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.6)
    room_area = {"origin_x": 0.0, "origin_y": 0.0, "width": 3.0, "height": 2.0}

    candidates = planner.generate_candidates(
        room_area,
        robot_pose=(0.0, 0.0, 0.0),
        obstacles=[[1.0, 1.0]],
    )

    assert candidates
    assert all(((c["x"] - 1.0) ** 2 + (c["y"] - 1.0) ** 2) ** 0.5 >= 0.6 for c in candidates)


def test_mark_blocked_and_mark_visited_affect_future_generation():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.2)
    room_area = {"origin_x": 0.0, "origin_y": 0.0, "width": 1.0, "height": 1.0}

    planner.mark_blocked({"x": 0.0, "y": 0.0})
    planner.mark_visited({"x": 1.0, "y": 0.0})
    candidates = planner.generate_candidates(room_area, robot_pose=(0.0, 0.0, 0.0), obstacles=[])

    assert (0.0, 0.0) not in {(c["x"], c["y"]) for c in candidates}
    visited = next(c for c in candidates if (c["x"], c["y"]) == (1.0, 0.0))
    assert visited["repeated_observation_penalty"] > 0.0


def test_selects_lower_path_cost_candidate_when_scores_tie():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.2)
    candidates = [
        {
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "information_gain": 1.0,
            "visual_coverage_gain": 0.0,
            "obstacle_risk_cost": 0.0,
            "repeated_observation_penalty": 0.0,
        },
        {
            "x": 2.0,
            "y": 0.0,
            "yaw": 0.0,
            "information_gain": 2.0,
            "visual_coverage_gain": 0.0,
            "obstacle_risk_cost": 0.0,
            "repeated_observation_penalty": 0.0,
        },
    ]

    selected = planner.select_next_best(candidates, robot_pose=(0.0, 0.0, 0.0))

    assert selected["x"] == 1.0
    assert selected["score"] == 0.0
    assert "score" not in candidates[0]


def test_select_next_best_returns_none_for_empty_candidates():
    planner = ActiveSearchPlanner()

    assert planner.select_next_best([], robot_pose=(0.0, 0.0, 0.0)) is None
