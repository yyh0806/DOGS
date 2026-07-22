from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


WEB = Path(__file__).resolve().parent
if str(WEB) not in sys.path:
    sys.path.insert(0, str(WEB))


def _grid(width=12, height=8, resolution=1.0, data=None):
    values = list(data) if data is not None else [0] * (width * height)
    return SimpleNamespace(
        info=SimpleNamespace(
            resolution=float(resolution),
            width=int(width),
            height=int(height),
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=values,
    )


def _scan(distance, *, count=361, range_max=12.0):
    return {
        "angle_min": -math.pi,
        "angle_increment": 2.0 * math.pi / float(count - 1),
        "range_min": 0.1,
        "range_max": float(range_max),
        "ranges": [float(distance)] * count,
        "age_sec": 0.0,
    }


def _tracker(**kwargs):
    from nx_visibility_coverage import VisibilityCoverageTracker

    defaults = {
        "camera_hfov_rad": math.radians(90.0),
        "camera_yaw_offset_rad": 0.0,
        "visual_range_m": 8.0,
        "coverage_cell_size_m": 1.0,
        "min_step_m": 1.0,
        "max_step_m": 8.0,
    }
    defaults.update(kwargs)
    return VisibilityCoverageTracker(**defaults)


def _observed_key(snapshot, x, y):
    return any(
        abs(float(point["x"]) - x) < 0.51
        and abs(float(point["y"]) - y) < 0.51
        for point in snapshot["observed_cells"]
    )


def test_wall_stops_camera_visibility():
    width, height = 12, 8
    data = [0] * (width * height)
    for row in range(height):
        data[row * width + 5] = 100
    tracker = _tracker()

    snapshot = tracker.observe(
        _grid(width, height, data=data),
        (1.5, 3.5, 0.0),
        _scan(8.0),
    )

    assert _observed_key(snapshot, 4.5, 3.5)
    assert _observed_key(snapshot, 5.5, 3.5)
    assert not _observed_key(snapshot, 6.5, 3.5)


def test_fresh_lidar_reveals_unknown_cells_until_measured_obstacle():
    """A live clear ray is stronger evidence than a lagging unknown map cell."""
    tracker = _tracker()

    snapshot = tracker.observe(
        _grid(width=20, height=12, data=[-1] * (20 * 12)),
        (2.5, 5.5, 0.0),
        _scan(4.0, range_max=8.0),
    )

    assert snapshot["scan_usable"] is True
    assert snapshot["visible_cell_count"] > 0
    assert _observed_key(snapshot, 5.5, 5.5)
    assert _observed_key(snapshot, 6.5, 5.5)
    assert not _observed_key(snapshot, 7.5, 5.5)


def test_open_unknown_space_produces_long_lidar_fallback_candidates():
    tracker = _tracker()
    grid = _grid(width=30, height=30, data=[-1] * (30 * 30))
    pose = (10.5, 10.5, 0.0)
    tracker.observe(grid, pose, _scan(8.0, range_max=8.0))

    candidates = tracker.lidar_candidates(pose, [], limit=8)

    assert candidates
    assert len(candidates) <= 8
    assert max(item["distance"] for item in candidates) >= 6.0
    assert all(item["lidar_candidate"] is True for item in candidates)
    assert all(item["prefer_standoff"] is False for item in candidates)
    assert all(item["visual_gain"] > 0 for item in candidates)


def test_open_scan_selects_long_step():
    snapshot = _tracker().observe(
        _grid(width=20, height=12),
        (2.5, 5.5, 0.0),
        _scan(12.0),
    )

    assert snapshot["forward_clearance_m"] >= 8.0
    assert snapshot["adaptive_step_m"] >= 5.0
    assert snapshot["scene_complexity"] < 0.35


def test_cluttered_scan_selects_short_step():
    count = 361
    ranges = []
    for index in range(count):
        angle = -math.pi + index * (2.0 * math.pi / (count - 1))
        ranges.append(1.2 if abs(angle) < math.radians(30.0) else 3.0)
    scan = _scan(3.0, count=count)
    scan["ranges"] = ranges

    snapshot = _tracker().observe(
        _grid(width=20, height=12),
        (2.5, 5.5, 0.0),
        scan,
    )

    assert snapshot["forward_clearance_m"] < 2.0
    assert snapshot["adaptive_step_m"] <= 2.0
    assert snapshot["scene_complexity"] > 0.5


def test_candidate_step_uses_its_lidar_corridor_not_side_returns():
    count = 361
    scan = _scan(12.0, count=count)
    ranges = []
    for index in range(count):
        angle = -math.pi + index * (2.0 * math.pi / (count - 1))
        # Close objects sit inside the camera sector, but outside the swept
        # body corridor of the straight-ahead candidate.
        side_clutter = math.radians(25.0) < abs(angle) < math.radians(45.0)
        ranges.append(1.2 if side_clutter else 12.0)
    scan["ranges"] = ranges
    tracker = _tracker()
    grid = _grid(width=24, height=12)
    pose = (2.5, 5.5, 0.0)
    tracker.observe(grid, pose, scan)

    ranked = tracker.rank_candidates(grid, pose, [{
        "x": 12.5, "y": 5.5, "yaw": 0.0, "size": 10,
        "information_gain": 10.0, "distance": 10.0,
        "center_cell": (5, 12),
    }])

    assert ranked[0]["path_clearance_m"] >= 8.0
    assert ranked[0]["adaptive_step_m"] >= 7.0


def test_candidate_step_stops_before_obstacle_inside_lidar_corridor():
    count = 361
    scan = _scan(12.0, count=count)
    ranges = list(scan["ranges"])
    center = count // 2
    for index in range(center - 2, center + 3):
        ranges[index] = 2.0
    scan["ranges"] = ranges
    tracker = _tracker()
    grid = _grid(width=24, height=12)
    pose = (2.5, 5.5, 0.0)
    tracker.observe(grid, pose, scan)

    ranked = tracker.rank_candidates(grid, pose, [{
        "x": 12.5, "y": 5.5, "yaw": 0.0, "size": 10,
        "information_gain": 10.0, "distance": 10.0,
        "center_cell": (5, 12),
    }])

    assert ranked[0]["path_clearance_m"] == pytest.approx(2.0, abs=0.1)
    assert ranked[0]["adaptive_step_m"] <= 1.5


def test_visual_gain_excludes_already_observed_cells():
    grid = _grid(width=14, height=10)
    candidate = {
        "x": 3.5,
        "y": 4.5,
        "yaw": 0.0,
        "size": 4,
        "information_gain": 4.0,
        "distance": 1.0,
        "center_cell": (4, 3),
    }
    fresh = _tracker()
    gain_before = fresh.rank_candidates(
        grid, (2.5, 4.5, 0.0), [candidate])[0]["visual_gain"]

    fresh.observe(grid, (2.5, 4.5, 0.0), _scan(8.0))
    gain_after = fresh.rank_candidates(
        grid, (2.5, 4.5, 0.0), [candidate])[0]["visual_gain"]

    assert gain_before > 0
    assert gain_after < gain_before


def test_coverage_viewpoints_target_unswept_free_space():
    grid = _grid(width=14, height=12)
    tracker = _tracker()
    tracker.observe(grid, (7.5, 5.5, 0.0), _scan(8.0))

    candidates = tracker.coverage_candidates(
        grid, (7.5, 5.5, 0.0), [], limit=12)

    assert candidates
    assert len(candidates) <= 12
    assert all(candidate["visual_gain"] > 0 for candidate in candidates)
    assert any(
        abs(math.atan2(
            math.sin(candidate["yaw"] - math.pi),
            math.cos(candidate["yaw"] - math.pi),
        )) < math.radians(70.0)
        for candidate in candidates
    )


def test_open_room_coverage_candidates_reach_lidar_confirmed_distance():
    grid = _grid(width=32, height=32)
    tracker = _tracker(visual_range_m=8.0)
    pose = (15.5, 15.5, 0.0)
    tracker.observe(grid, pose, _scan(12.0))

    candidates = tracker.coverage_candidates(grid, pose, [], limit=12)

    assert candidates
    assert max(candidate["distance"] for candidate in candidates) >= 6.0
    assert candidates[0]["distance"] >= 5.0


def test_coverage_candidates_do_not_miss_a_narrow_offset_free_corridor():
    width = height = 80
    data = [-1] * (width * height)
    for row in range(7, 18):
        for col in range(7, 68):
            data[row * width + col] = 0
    grid = _grid(width, height, resolution=0.05, data=data)
    tracker = _tracker(
        visual_range_m=2.0,
        coverage_cell_size_m=0.5,
        min_step_m=1.0,
        max_step_m=4.0,
    )

    candidates = tracker.coverage_candidates(
        grid, (0.525, 0.525, 0.0), [], limit=8)

    assert candidates
    assert all(0.35 <= candidate["y"] <= 0.9 for candidate in candidates)


def test_invalid_or_stale_scan_falls_back_to_conservative_step():
    snapshot = _tracker().observe(
        _grid(width=14, height=10),
        (2.5, 4.5, 0.0),
        {**_scan(8.0), "age_sec": 5.0},
    )

    assert snapshot["scan_usable"] is False
    assert snapshot["adaptive_step_m"] == pytest.approx(1.0)


def test_snapshot_publishes_exact_camera_calibration_and_current_visible_cells():
    tracker = _tracker(
        camera_hfov_rad=math.radians(77.4),
        camera_yaw_offset_rad=math.radians(-12.25),
        visual_range_m=4.0,
        coverage_cell_size_m=0.5,
    )

    snapshot = tracker.observe(
        _grid(width=20, height=16, resolution=0.5),
        (2.25, 3.25, 0.0),
        _scan(4.0, range_max=6.0),
    )

    assert snapshot["camera_hfov_deg"] == pytest.approx(77.4)
    assert snapshot["camera_yaw_offset_deg"] == pytest.approx(-12.25)
    assert snapshot["visible_cells"]
    assert len(snapshot["visible_cells"]) == snapshot["visible_cell_count"]
    assert all(set(cell) == {"x", "y"} for cell in snapshot["visible_cells"])


def test_snapshot_visible_cells_are_clipped_by_the_current_obstacle_map():
    width, height = 12, 8
    data = [0] * (width * height)
    for row in range(height):
        data[row * width + 5] = 100
    tracker = _tracker(camera_hfov_rad=math.radians(60.0))

    snapshot = tracker.observe(
        _grid(width, height, data=data),
        (1.5, 3.5, 0.0),
        _scan(8.0),
    )

    assert any(cell["x"] >= 5.0 for cell in snapshot["visible_cells"])
    assert not any(cell["x"] >= 6.0 for cell in snapshot["visible_cells"])


def test_exploration_live_fields_forward_tracker_frustum_without_recalibration():
    from nx_room_orchestrator import RoomSearchOrchestrator

    visibility = {
        "camera_hfov_deg": 77.412345,
        "camera_yaw_offset_deg": -12.234567,
        "visible_cells": [{"x": 1.25, "y": 2.75}],
        "coverage_cell_size_m": 0.5,
        "visual_range_m": 4.0,
    }
    exploration = SimpleNamespace(snapshot=lambda: {
        "visibility": visibility,
        "visited_frontiers": [],
    })

    live = RoomSearchOrchestrator._exploration_live_fields(
        object(), exploration)

    assert live["camera_hfov_deg"] == visibility["camera_hfov_deg"]
    assert live["camera_yaw_offset_deg"] == visibility["camera_yaw_offset_deg"]
    assert live["visible_cells"] == visibility["visible_cells"]
