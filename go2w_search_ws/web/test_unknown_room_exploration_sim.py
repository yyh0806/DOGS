"""Closed-loop occupancy simulation for large unknown-room exploration."""

from collections import deque
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_exploration_manager import ExplorationManager  # noqa: E402


def _grid(data, width, height, resolution):
    return SimpleNamespace(
        info=SimpleNamespace(
            resolution=resolution,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=list(data),
    )


def _reveal(truth, observed, width, height, resolution, x, y, radius):
    radius_sq = radius * radius
    for row in range(height):
        cy = (row + 0.5) * resolution
        for col in range(width):
            cx = (col + 0.5) * resolution
            if (cx - x) ** 2 + (cy - y) ** 2 <= radius_sq:
                observed[row * width + col] = truth[row * width + col]


class _KnownFreePlanner:
    """Read-only plan facade backed by the currently observed free grid."""

    def __init__(self, observed, width, height, resolution, pose):
        self.observed = observed
        self.width = width
        self.height = height
        self.resolution = resolution
        self.pose = list(pose)

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=3.0):
        del frame_id, timeout
        start = self._cell(self.pose[0], self.pose[1])
        goal = self._cell(x, y)
        if start is None or goal is None:
            return {"ok": False, "reason": "outside_map"}
        distances = {start: 0}
        queue = deque([start])
        while queue:
            row, col = queue.popleft()
            if (row, col) == goal:
                steps = distances[(row, col)]
                return {
                    "ok": True,
                    "poses": steps + 1,
                    "path_length": steps * self.resolution,
                }
            for dr, dc in ((-1, 0), (0, -1), (0, 1), (1, 0)):
                neighbor = (row + dr, col + dc)
                if neighbor in distances or not self._free(*neighbor):
                    continue
                distances[neighbor] = distances[(row, col)] + 1
                queue.append(neighbor)
        return {"ok": False, "reason": "no_known_free_path"}

    def _cell(self, x, y):
        row = int(math.floor(float(y) / self.resolution))
        col = int(math.floor(float(x) / self.resolution))
        if not (0 <= row < self.height and 0 <= col < self.width):
            return None
        return row, col

    def _free(self, row, col):
        return (
            0 <= row < self.height
            and 0 <= col < self.width
            and self.observed[row * self.width + col] == 0
        )


def test_large_unknown_room_reaches_stable_frontier_exhaustion_with_high_coverage():
    resolution = 0.25
    width, height = 80, 48  # 20 m x 12 m
    truth = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            if row in {0, height - 1} or col in {0, width - 1}:
                truth[row * width + col] = 100
    # A large internal obstacle with free circulation on every side.
    for row in range(14, 34):
        for col in range(36, 44):
            truth[row * width + col] = 100

    observed = [-1] * (width * height)
    pose = [2.0, 6.0, 0.0]
    sensor_radius = 2.25
    _reveal(
        truth, observed, width, height, resolution,
        pose[0], pose[1], sensor_radius)
    nav = _KnownFreePlanner(observed, width, height, resolution, pose)
    manager = ExplorationManager(
        navigation_port=nav,
        mission_origin=tuple(pose),
        mode="current_room",
        room_radius_m=30.0,
        initial_radius_m=6.0,
        radius_step_m=6.0,
        tile_size_m=6.0,
        frontier_spacing_m=1.5,
        stable_exhaustion_cycles=3,
        max_time_s=1800.0,
        max_plan_probes=12,
        reject_map_edge=True,
    )

    terminal_reason = None
    goals = []
    for _ in range(200):
        nav.pose[:] = pose
        target = manager.choose_next(
            _grid(observed, width, height, resolution), tuple(pose))
        if target is None:
            reason = manager.snapshot()["last_selection_reason"]
            if reason == "reachable_frontiers_exhausted":
                terminal_reason = reason
                break
            continue
        goals.append((target["x"], target["y"]))
        pose[:] = [target["x"], target["y"], target["yaw"]]
        _reveal(
            truth, observed, width, height, resolution,
            pose[0], pose[1], sensor_radius)
        manager.mark_visited(target)

    free_cells = [index for index, value in enumerate(truth) if value == 0]
    known_free = sum(1 for index in free_cells if observed[index] == 0)
    coverage = known_free / len(free_cells)

    assert terminal_reason == "reachable_frontiers_exhausted"
    assert coverage >= 0.95
    assert len(goals) >= 8
    assert max(x for x, _ in goals) >= 16.0
    assert max(y for _, y in goals) >= 9.0
    assert manager.snapshot()["active_radius_m"] == pytest.approx(30.0)
