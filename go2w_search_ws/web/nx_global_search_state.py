"""Pure global-map evidence for entrance-gated exploration completion.

The mission entrance is a finite graph cut, not a rear half-plane.  Unknown
space behind that one cut is excluded from the mission; every other reachable
free/unknown boundary remains an exploration obligation.
"""

from __future__ import annotations

from collections import deque
import math
from typing import Any, Iterable, Optional


_NEIGHBORS_4 = ((-1, 0), (0, -1), (0, 1), (1, 0))
_NEIGHBORS_8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
)


def _field(value: Any, name: str, default: Any = None) -> Any:
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _finite(value: Any) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite value")
    return number


class _Grid:
    def __init__(self, map_msg: Any) -> None:
        info = _field(map_msg, "info")
        self.resolution = _finite(_field(info, "resolution"))
        self.width = int(_field(info, "width", 0) or 0)
        self.height = int(_field(info, "height", 0) or 0)
        self.data = list(_field(map_msg, "data", []) or [])
        if (
            self.resolution <= 0.0
            or self.width <= 0
            or self.height <= 0
            or len(self.data) != self.width * self.height
        ):
            raise ValueError("invalid occupancy grid")
        origin = _field(info, "origin")
        position = _field(origin, "position")
        orientation = _field(origin, "orientation")
        self.origin_x = _finite(_field(position, "x", 0.0))
        self.origin_y = _finite(_field(position, "y", 0.0))
        qx = _finite(_field(orientation, "x", 0.0))
        qy = _finite(_field(orientation, "y", 0.0))
        qz = _finite(_field(orientation, "z", 0.0))
        qw = _finite(_field(orientation, "w", 1.0))
        self.origin_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self._cos = math.cos(self.origin_yaw)
        self._sin = math.sin(self.origin_yaw)

    def value(self, cell: tuple[int, int]) -> int:
        return int(self.data[cell[0] * self.width + cell[1]])

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.height and 0 <= cell[1] < self.width

    def world_to_cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        local_x = self._cos * dx + self._sin * dy
        local_y = -self._sin * dx + self._cos * dy
        col = int(math.floor(local_x / self.resolution))
        row = int(math.floor(local_y / self.resolution))
        cell = (row, col)
        return cell if self.in_bounds(cell) else None

    def cell_center(self, cell: tuple[int, int]) -> tuple[float, float]:
        row, col = cell
        local_x = (col + 0.5) * self.resolution
        local_y = (row + 0.5) * self.resolution
        return (
            self.origin_x + self._cos * local_x - self._sin * local_y,
            self.origin_y + self._sin * local_x + self._cos * local_y,
        )


def _validated_pose(pose: Iterable) -> tuple[float, float, float]:
    values = tuple(pose)
    if len(values) < 3:
        raise ValueError("pose must contain x, y, yaw")
    return _finite(values[0]), _finite(values[1]), _finite(values[2])


def _validated_gate(gate: dict) -> dict:
    result = {
        "center_x": _finite(gate["center_x"]),
        "center_y": _finite(gate["center_y"]),
        "yaw": _finite(gate["yaw"]),
        "width_m": _finite(gate["width_m"]),
    }
    if result["width_m"] <= 0.0:
        raise ValueError("entrance gate width must be positive")
    for key in ("left_support_m", "right_support_m"):
        if key in gate:
            result[key] = _finite(gate[key])
    return result


def infer_entrance_gate(
    map_msg: Any,
    mission_origin: Iterable,
    *,
    obstacle_threshold: int = 50,
    min_gate_width_m: float = 0.6,
    max_gate_width_m: float = 3.0,
) -> Optional[dict]:
    """Infer the doorway cross-section through the initial pose.

    The dog is expected to face into the search domain.  The nearest occupied
    support on each side of that heading bounds the finite virtual gate.
    """

    try:
        grid = _Grid(map_msg)
        x, y, yaw = _validated_pose(mission_origin)
        maximum = _finite(max_gate_width_m)
        minimum = max(0.0, _finite(min_gate_width_m))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if maximum <= 0.0 or minimum > maximum:
        return None
    tangent_x = -math.sin(yaw)
    tangent_y = math.cos(yaw)
    max_steps = max(1, int(math.ceil(maximum / grid.resolution)))
    supports = {}
    for side in (-1, 1):
        for step in range(1, max_steps + 1):
            distance = step * grid.resolution
            if distance > maximum + 1e-9:
                break
            cell = grid.world_to_cell(
                x + side * distance * tangent_x,
                y + side * distance * tangent_y,
            )
            if cell is None:
                break
            if grid.value(cell) >= int(obstacle_threshold):
                supports[side] = distance
                break
    if -1 not in supports or 1 not in supports:
        return None
    width = supports[-1] + supports[1]
    if width < minimum - 1e-9 or width > maximum + 1e-9:
        return None
    return {
        "center_x": x,
        "center_y": y,
        "yaw": yaw,
        "width_m": width,
        "left_support_m": supports[1],
        "right_support_m": supports[-1],
    }


def _gate_coordinates(point: tuple[float, float], gate: dict) -> tuple[float, float]:
    dx = float(point[0]) - gate["center_x"]
    dy = float(point[1]) - gate["center_y"]
    cosine = math.cos(gate["yaw"])
    sine = math.sin(gate["yaw"])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def _segment_crossing(
    start: tuple[float, float],
    end: tuple[float, float],
    gate: dict,
    *,
    rearward_only: bool,
) -> bool:
    start_forward, start_lateral = _gate_coordinates(start, gate)
    end_forward, end_lateral = _gate_coordinates(end, gate)
    epsilon = 1e-9
    if rearward_only:
        crosses = start_forward >= -epsilon and end_forward < -epsilon
    else:
        crosses = (
            (start_forward > epsilon and end_forward <= epsilon)
            or (end_forward > epsilon and start_forward <= epsilon)
        )
    if not crosses:
        return False
    denominator = start_forward - end_forward
    if abs(denominator) <= epsilon:
        return False
    ratio = start_forward / denominator
    if ratio < -epsilon or ratio > 1.0 + epsilon:
        return False
    lateral = start_lateral + ratio * (end_lateral - start_lateral)
    return abs(lateral) <= gate["width_m"] * 0.5 + epsilon


def path_crosses_entrance_gate(path: Iterable, entrance_gate: dict) -> bool:
    """Return True only when a path exits rearward through the entrance."""

    try:
        gate = _validated_gate(entrance_gate)
        points = [
            (
                float(_field(item, "x", item[0] if not isinstance(item, dict) else None)),
                float(_field(item, "y", item[1] if not isinstance(item, dict) else None)),
            )
            for item in path
        ]
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return True
    return any(
        _segment_crossing(start, end, gate, rearward_only=True)
        for start, end in zip(points, points[1:])
    )


def edge_crosses_entrance_gate(
    start: Iterable,
    end: Iterable,
    entrance_gate: dict,
) -> bool:
    """Return whether an undirected graph edge intersects the finite gate."""

    try:
        gate = _validated_gate(entrance_gate)
        first = (float(start[0]), float(start[1]))
        second = (float(end[0]), float(end[1]))
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return True
    return _segment_crossing(first, second, gate, rearward_only=False)


def _inflate_occupied(
    grid: _Grid,
    occupied: set[tuple[int, int]],
    clearance_m: float,
) -> set[tuple[int, int]]:
    radius = max(0, int(math.ceil(clearance_m / grid.resolution - 1e-9)))
    if radius <= 0:
        return set(occupied)
    forbidden = set(occupied)
    for row, col in occupied:
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if math.hypot(dr, dc) * grid.resolution > clearance_m + 1e-9:
                    continue
                cell = (row + dr, col + dc)
                if grid.in_bounds(cell):
                    forbidden.add(cell)
    return forbidden


def _nearest_forward_seed(
    grid: _Grid,
    passable: set[tuple[int, int]],
    pose: tuple[float, float, float],
    gate: Optional[dict],
) -> Optional[tuple[int, int]]:
    if gate is None:
        target = (pose[0], pose[1])
    else:
        step = max(grid.resolution, 0.25)
        target = (
            pose[0] + step * math.cos(pose[2]),
            pose[1] + step * math.sin(pose[2]),
        )
    direct = grid.world_to_cell(*target)
    if direct in passable:
        return direct
    best = None
    for cell in passable:
        center = grid.cell_center(cell)
        if gate is not None:
            forward, _ = _gate_coordinates(center, gate)
            if forward <= 0.0:
                continue
        distance_sq = (center[0] - target[0]) ** 2 + (center[1] - target[1]) ** 2
        if distance_sq > 1.0:
            continue
        key = (distance_sq, cell)
        if best is None or key < best[0]:
            best = (key, cell)
    return None if best is None else best[1]


def _reachable_component(
    grid: _Grid,
    passable: set[tuple[int, int]],
    seed: tuple[int, int],
    gate: Optional[dict],
) -> tuple[set[tuple[int, int]], set[tuple[tuple[int, int], tuple[int, int]]]]:
    reachable = {seed}
    queue = deque([seed])
    excluded_edges = set()
    while queue:
        cell = queue.popleft()
        start = grid.cell_center(cell)
        for dr, dc in _NEIGHBORS_4:
            neighbor = (cell[0] + dr, cell[1] + dc)
            if neighbor not in passable:
                continue
            if gate is not None and _segment_crossing(
                    start, grid.cell_center(neighbor), gate,
                    rearward_only=False):
                excluded_edges.add(tuple(sorted((cell, neighbor))))
                continue
            if neighbor not in reachable:
                reachable.add(neighbor)
                queue.append(neighbor)
    return reachable, excluded_edges


def _frontier_components(
    grid: _Grid,
    frontier_cells: set[tuple[int, int]],
) -> list[dict]:
    remaining = set(frontier_cells)
    output = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        queue = deque([seed])
        component = []
        while queue:
            cell = queue.popleft()
            component.append(cell)
            for dr, dc in _NEIGHBORS_8:
                neighbor = (cell[0] + dr, cell[1] + dc)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
        centers = [grid.cell_center(cell) for cell in component]
        output.append({
            "cell_count": len(component),
            "center_x": round(sum(item[0] for item in centers) / len(centers), 3),
            "center_y": round(sum(item[1] for item in centers) / len(centers), 3),
            "cells": [list(cell) for cell in sorted(component)],
        })
    return sorted(output, key=lambda item: (item["center_x"], item["center_y"]))


def _classify_unknown(
    grid: _Grid,
    unknown: set[tuple[int, int]],
    reachable: set[tuple[int, int]],
) -> tuple[set[tuple[int, int]], set[tuple[int, int]]]:
    remaining = set(unknown)
    exterior = set()
    occluded = set()
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        queue = deque([seed])
        component = set()
        touches_edge = False
        borders_reachable = False
        while queue:
            cell = queue.popleft()
            component.add(cell)
            if (
                cell[0] in (0, grid.height - 1)
                or cell[1] in (0, grid.width - 1)
            ):
                touches_edge = True
            for dr, dc in _NEIGHBORS_8:
                neighbor = (cell[0] + dr, cell[1] + dc)
                if neighbor in reachable:
                    borders_reachable = True
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
        if touches_edge:
            exterior.update(component)
        elif not borders_reachable:
            occluded.update(component)
    return exterior, occluded


def analyze_global_search_state(
    map_msg: Any,
    *,
    mission_origin: Iterable,
    entrance_gate: Optional[dict] = None,
    observed_cells: Iterable = (),
    observed_cell_size_m: Optional[float] = None,
    obstacle_threshold: int = 50,
    traversal_clearance_m: float = 0.40,
    coverage_threshold: float = 0.95,
) -> dict:
    """Return global boundary and explainable-coverage evidence."""

    try:
        grid = _Grid(map_msg)
        pose = _validated_pose(mission_origin)
        gate = (
            None if entrance_gate is None
            else _validated_gate(entrance_gate)
        )
        clearance = max(0.0, _finite(traversal_clearance_m))
        threshold = min(1.0, max(0.0, _finite(coverage_threshold)))
        observed_size = (
            None if observed_cell_size_m is None
            else _finite(observed_cell_size_m)
        )
        if observed_size is not None and observed_size <= 0.0:
            raise ValueError("observed cell size must be positive")
    except (KeyError, TypeError, ValueError, OverflowError):
        return {
            "valid": False,
            "reason": "invalid_global_search_evidence",
            "completion_eligible": False,
        }

    free = set()
    occupied = set()
    unknown = set()
    for index, raw_value in enumerate(grid.data):
        cell = divmod(index, grid.width)
        value = int(raw_value)
        if value == 0:
            free.add(cell)
        elif value < 0:
            unknown.add(cell)
        elif value >= int(obstacle_threshold):
            occupied.add(cell)

    forbidden = _inflate_occupied(grid, occupied, clearance)
    passable = free.difference(forbidden)
    seed = _nearest_forward_seed(grid, passable, pose, gate)
    if seed is None:
        return {
            "valid": False,
            "reason": "no_reachable_seed",
            "entrance_gate": gate,
            "completion_eligible": False,
        }
    reachable, excluded_edges = _reachable_component(grid, passable, seed, gate)

    frontier_cells = set()
    occupied_boundary = set()
    for cell in reachable:
        start = grid.cell_center(cell)
        for dr, dc in _NEIGHBORS_8:
            neighbor = (cell[0] + dr, cell[1] + dc)
            if not grid.in_bounds(neighbor):
                # A rolling/global map edge is not a physical wall. Reachable
                # free space touching it remains an exploration opening unless
                # that exact graph edge is the explicitly excluded entrance.
                if (
                    gate is None
                    or not _segment_crossing(
                        start,
                        grid.cell_center(neighbor),
                        gate,
                        rearward_only=False,
                    )
                ):
                    frontier_cells.add(cell)
                continue
            if neighbor in occupied:
                occupied_boundary.add(neighbor)
            elif neighbor in unknown and (
                gate is None
                or not _segment_crossing(
                    start, grid.cell_center(neighbor), gate,
                    rearward_only=False,
                )
            ):
                frontier_cells.add(cell)

    opening_components = _frontier_components(grid, frontier_cells)
    exterior_unknown, occluded_unknown = _classify_unknown(
        grid, unknown, reachable)

    observed_points = []
    for item in observed_cells or ():
        try:
            x = _finite(_field(item, "x"))
            y = _finite(_field(item, "y"))
        except (TypeError, ValueError, OverflowError):
            continue
        observed_points.append((x, y))

    observed_reachable = set()
    if observed_size is not None:
        observed_keys = {
            (
                int(math.floor(x / observed_size)),
                int(math.floor(y / observed_size)),
            )
            for x, y in observed_points
        }
        for cell in reachable:
            center_x, center_y = grid.cell_center(cell)
            key = (
                int(math.floor(center_x / observed_size)),
                int(math.floor(center_y / observed_size)),
            )
            if key in observed_keys:
                observed_reachable.add(cell)
    else:
        for x, y in observed_points:
            cell = grid.world_to_cell(x, y)
            if cell in reachable:
                observed_reachable.add(cell)

    explainable_total = (
        len(reachable) + len(occupied_boundary) + len(occluded_unknown)
    )
    explainable_covered = (
        len(observed_reachable) + len(occupied_boundary) + len(occluded_unknown)
    )
    ratio = (
        explainable_covered / float(explainable_total)
        if explainable_total > 0 else 0.0
    )
    completion = (
        bool(reachable)
        and not opening_components
        and ratio + 1e-12 >= threshold
    )
    return {
        "valid": True,
        "reason": None,
        "entrance_gate": gate,
        "entrance_excluded_edge_count": len(excluded_edges),
        "reachable_free_cell_count": len(reachable),
        "observed_reachable_free_cell_count": len(observed_reachable),
        "occupied_boundary_cell_count": len(occupied_boundary),
        "frontier_cell_count": len(frontier_cells),
        "traversable_opening_count": len(opening_components),
        "opening_components": opening_components,
        "exterior_unknown_cell_count": len(exterior_unknown),
        "certified_occluded_unknown_cell_count": len(occluded_unknown),
        "explainable_total_cell_count": explainable_total,
        "explainable_covered_cell_count": explainable_covered,
        "explainable_coverage_ratio": round(ratio, 6),
        "coverage_threshold": threshold,
        "traversal_clearance_m": clearance,
        "minimum_traversable_width_m": round(2.0 * clearance, 6),
        "completion_eligible": completion,
    }


__all__ = [
    "analyze_global_search_state",
    "edge_crosses_entrance_gate",
    "infer_entrance_gate",
    "path_crosses_entrance_gate",
]
