"""Pure occupancy-grid frontier extraction and deterministic scoring."""

from __future__ import annotations

import math
import os
from typing import Callable, Iterable, Mapping, Optional


_NEIGHBORS_8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
)


def _finite_float(value, default=0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _map_geometry(map_msg):
    if map_msg is None:
        return None
    info = getattr(map_msg, "info", None)
    if info is None:
        return None
    resolution = _finite_float(getattr(info, "resolution", 0.0))
    width = int(getattr(info, "width", 0) or 0)
    height = int(getattr(info, "height", 0) or 0)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return None
    try:
        data = list(getattr(map_msg, "data", None) or [])
    except Exception:
        return None
    if len(data) != width * height:
        return None
    origin = getattr(info, "origin", None)
    position = getattr(origin, "position", None)
    orientation = getattr(origin, "orientation", None)
    origin_x = _finite_float(getattr(position, "x", 0.0))
    origin_y = _finite_float(getattr(position, "y", 0.0))
    qx = _finite_float(getattr(orientation, "x", 0.0))
    qy = _finite_float(getattr(orientation, "y", 0.0))
    qz = _finite_float(getattr(orientation, "z", 0.0))
    qw = _finite_float(getattr(orientation, "w", 1.0), 1.0)
    origin_yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return resolution, width, height, data, origin_x, origin_y, origin_yaw


def find_frontier_clusters(
    map_msg,
    robot_pose,
    visited,
    min_cluster_size: int = 3,
    revisit_radius: float = 1.0,
) -> list[dict]:
    """Return connected free cells bordering unknown occupancy cells.

    The commanded representative is the closest real free cell, never the
    arithmetic centroid of a large frontier ring. ``touches_map_edge`` lets
    the persistent policy reject rolling-window boundary artifacts.
    """

    geometry = _map_geometry(map_msg)
    if geometry is None:
        return []
    resolution, width, height, data, origin_x, origin_y, origin_yaw = geometry
    try:
        robot_x = _finite_float(robot_pose[0])
        robot_y = _finite_float(robot_pose[1])
    except (TypeError, IndexError):
        robot_x = robot_y = 0.0

    def value(row, col):
        if row < 0 or row >= height or col < 0 or col >= width:
            return None
        return data[row * width + col]

    frontier_cells = []
    for row in range(height):
        for col in range(width):
            if data[row * width + col] != 0:
                continue
            if any(value(row + dr, col + dc) == -1
                   for dr, dc in _NEIGHBORS_8):
                frontier_cells.append((row, col))
    if not frontier_cells:
        return []

    frontier_set = set(frontier_cells)
    consumed = set()
    components = []
    for seed in frontier_cells:
        if seed in consumed:
            continue
        queue = [seed]
        consumed.add(seed)
        component = []
        for row, col in queue:
            component.append((row, col))
            for dr, dc in _NEIGHBORS_8:
                neighbor = (row + dr, col + dc)
                if neighbor in frontier_set and neighbor not in consumed:
                    consumed.add(neighbor)
                    queue.append(neighbor)
        if len(component) >= max(1, int(min_cluster_size)):
            components.append(component)

    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)

    def cell_world(cell):
        row, col = cell
        local_x = (col + 0.5) * resolution
        local_y = (row + 0.5) * resolution
        return (
            origin_x + cos_yaw * local_x - sin_yaw * local_y,
            origin_y + sin_yaw * local_x + cos_yaw * local_y,
        )

    result = []
    for component in components:
        representative = min(
            component,
            key=lambda cell: (
                (cell_world(cell)[0] - robot_x) ** 2
                + (cell_world(cell)[1] - robot_y) ** 2,
                cell[0], cell[1],
            ),
        )
        world_x, world_y = cell_world(representative)
        if any(
            math.hypot(
                world_x - _finite_float(item.get("x")),
                world_y - _finite_float(item.get("y")),
            ) < max(0.0, float(revisit_radius))
            for item in (visited or [])
            if isinstance(item, dict)
        ):
            continue
        result.append({
            "center_cell": representative,
            "center_world": (world_x, world_y),
            "size": len(component),
            "information_gain": float(len(component)),
            "distance": math.hypot(world_x - robot_x, world_y - robot_y),
            "touches_map_edge": any(
                row <= 1 or col <= 1 or row >= height - 2 or col >= width - 2
                for row, col in component
            ),
        })
    return result


def score_frontier(
    candidate: Mapping,
    *,
    path_length: Optional[float] = None,
    heading_change: float = 0.0,
    failure_count: int = 0,
    distance_weight: float = 1.0,
    heading_weight: float = 0.15,
    failure_penalty: float = 1.0,
) -> float:
    """Score information gain against real path cost and retry evidence."""

    gain = _finite_float(
        candidate.get("information_gain", candidate.get("size", 0.0)))
    path_cost = _finite_float(
        candidate.get("distance", 0.0) if path_length is None else path_length)
    denominator = 1.0 + max(0.0, path_cost) * max(0.0, distance_weight)
    return (
        gain / denominator
        - abs(_finite_float(heading_change)) * max(0.0, heading_weight)
        - max(0, int(failure_count)) * max(0.0, failure_penalty)
    )


def point_in_polygon(x: float, y: float, polygon: Iterable) -> bool:
    points = [(float(px), float(py)) for px, py in polygon]
    if len(points) < 3:
        return False
    inside = False
    previous = points[-1]
    for current in points:
        x1, y1 = previous
        x2, y2 = current
        if ((y1 > y) != (y2 > y)):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x <= crossing_x:
                inside = not inside
        previous = current
    return inside


def _angle_delta(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def select_frontier_candidates(
    map_msg,
    robot_pose,
    visited,
    min_cluster_size: int = 3,
    revisit_radius: float = 1.0,
    origin_pose=None,
    max_radius=None,
    *,
    room_polygon=None,
    reject_map_edge: bool = False,
    failure_counts: Optional[Mapping] = None,
    path_lengths: Optional[Mapping] = None,
    cluster_finder: Callable = find_frontier_clusters,
    distance_weight: Optional[float] = None,
    heading_weight: float = 0.0,
    failure_penalty: float = 1.0,
) -> list[dict]:
    """Return deterministic frontier candidates in utility order.

    This implements cost-distance method (c). Nearest-frontier (a) and maximum
    information-gain (b) remain special cases of the configurable weights.
    """

    clusters = cluster_finder(
        map_msg, robot_pose, visited, min_cluster_size, revisit_radius)
    if reject_map_edge:
        clusters = [item for item in clusters if not item.get("touches_map_edge")]
    try:
        robot_x, robot_y, robot_yaw = (float(value) for value in robot_pose[:3])
    except (TypeError, ValueError, IndexError):
        return []
    if max_radius is not None:
        radius = _finite_float(max_radius, -1.0)
        origin = origin_pose if origin_pose is not None else robot_pose
        if radius <= 0.0:
            return []
        origin_x, origin_y = _finite_float(origin[0]), _finite_float(origin[1])
        clusters = [
            item for item in clusters
            if math.hypot(
                item["center_world"][0] - origin_x,
                item["center_world"][1] - origin_y,
            ) <= radius
        ]
    if room_polygon:
        clusters = [
            item for item in clusters
            if point_in_polygon(
                item["center_world"][0], item["center_world"][1], room_polygon)
        ]
    if distance_weight is None:
        try:
            distance_weight = float(os.environ.get("GO2W_FRONTIER_ALPHA", "1.0"))
        except (TypeError, ValueError):
            distance_weight = 1.0
        if not math.isfinite(distance_weight) or distance_weight <= 0.0:
            distance_weight = 1.0

    candidates = []
    for cluster in clusters:
        world_x, world_y = cluster["center_world"]
        yaw = math.atan2(world_y - robot_y, world_x - robot_x)
        cell = tuple(cluster["center_cell"])
        failure_count = int((failure_counts or {}).get(cell, 0))
        path_length = (path_lengths or {}).get(cell)
        heading_change = abs(_angle_delta(yaw, robot_yaw))
        candidate = {
            "x": world_x,
            "y": world_y,
            "yaw": yaw,
            "size": int(cluster["size"]),
            "information_gain": _finite_float(cluster.get(
                "information_gain", cluster["size"])),
            "distance": _finite_float(cluster["distance"]),
            "center_cell": cell,
            "touches_map_edge": bool(cluster.get("touches_map_edge")),
            "heading_change": heading_change,
            "failure_count": failure_count,
        }
        if path_length is not None:
            candidate["path_length"] = _finite_float(path_length)
        candidate["score"] = score_frontier(
            candidate,
            path_length=path_length,
            heading_change=heading_change,
            failure_count=failure_count,
            distance_weight=distance_weight,
            heading_weight=heading_weight,
            failure_penalty=failure_penalty,
        )
        candidates.append(candidate)
    return sorted(candidates, key=lambda item: (
        -item["score"], item.get("path_length", item["distance"]),
        item["x"], item["y"], item["center_cell"],
    ))


def select_next_frontier(map_msg, robot_pose, visited, **kwargs):
    candidates = select_frontier_candidates(
        map_msg, robot_pose, visited, **kwargs)
    return candidates[0] if candidates else None


# Compatibility name used by older tests and offline scripts.
_find_frontier_clusters = find_frontier_clusters

