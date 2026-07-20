"""Pure occupancy-grid frontier extraction and deterministic scoring."""

from __future__ import annotations

from collections import deque
import inspect
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


def callable_accepts_keyword(callback: Callable, keyword: str) -> bool:
    """Return whether a callback supports one keyword or arbitrary kwargs."""
    try:
        parameters = inspect.signature(callback).parameters
    except (TypeError, ValueError):
        return True
    return (
        keyword in parameters
        or any(
            parameter.kind is inspect.Parameter.VAR_KEYWORD
            for parameter in parameters.values()
        )
    )


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


def _reachable_free_cells(
    data,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    robot_x: float,
    robot_y: float,
    seed_search_radius_m: float = 1.0,
) -> list[int]:
    """Return the known-free component containing the robot.

    Localization and occupancy grids are asynchronous, so the robot cell can
    briefly be unknown or occupied. In that case use the nearest known-free
    seed within a small physical radius instead of admitting remote islands.
    Four-connected flood fill is deliberately conservative around diagonal
    obstacle corners; Nav2 remains the final reachability authority.
    """

    dx = float(robot_x) - origin_x
    dy = float(robot_y) - origin_y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    seed_col = int(math.floor(local_x / resolution))
    seed_row = int(math.floor(local_y / resolution))

    def is_free(row, col):
        return (
            0 <= row < height
            and 0 <= col < width
            and data[row * width + col] == 0
        )

    seed = None
    if is_free(seed_row, seed_col):
        seed = (seed_row, seed_col)
    else:
        radius_cells = max(
            0, int(math.ceil(max(0.0, seed_search_radius_m) / resolution)))
        best = None
        for row in range(seed_row - radius_cells, seed_row + radius_cells + 1):
            for col in range(seed_col - radius_cells, seed_col + radius_cells + 1):
                if not is_free(row, col):
                    continue
                cell_dx = (col + 0.5) * resolution - local_x
                cell_dy = (row + 0.5) * resolution - local_y
                distance_sq = cell_dx * cell_dx + cell_dy * cell_dy
                if distance_sq > seed_search_radius_m * seed_search_radius_m:
                    continue
                key = (distance_sq, row, col)
                if best is None or key < best:
                    best = key
                    seed = (row, col)
    if seed is None:
        return []

    seed_index = seed[0] * width + seed[1]
    discovered = bytearray(width * height)
    discovered[seed_index] = 1
    reachable = []
    queue = deque([seed_index])
    while queue:
        index = queue.popleft()
        reachable.append(index)
        row, col = divmod(index, width)
        neighbors = []
        if row > 0:
            neighbors.append(index - width)
        if col > 0:
            neighbors.append(index - 1)
        if col + 1 < width:
            neighbors.append(index + 1)
        if row + 1 < height:
            neighbors.append(index + width)
        for neighbor in neighbors:
            if not discovered[neighbor] and data[neighbor] == 0:
                discovered[neighbor] = 1
                queue.append(neighbor)
    return reachable


def find_frontier_clusters(
    map_msg,
    robot_pose,
    visited,
    min_cluster_size: int = 3,
    revisit_radius: float = 1.0,
    frontier_spacing_m: float = 1.5,
    max_candidates_per_cluster: int = 64,
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

    reachable_free = _reachable_free_cells(
        data, width, height, resolution,
        origin_x, origin_y, origin_yaw, robot_x, robot_y)
    cell_count = width * height
    frontier_mask = bytearray(cell_count)
    frontier_count = 0
    for index in reachable_free:
        row, col = divmod(index, width)
        is_frontier = False
        for dr, dc in _NEIGHBORS_8:
            neighbor_row = row + dr
            neighbor_col = col + dc
            if (0 <= neighbor_row < height and 0 <= neighbor_col < width
                    and data[neighbor_row * width + neighbor_col] == -1):
                is_frontier = True
                break
        if is_frontier:
            frontier_mask[index] = 1
            frontier_count += 1
    if frontier_count == 0:
        return []

    components = []
    for seed in range(cell_count):
        if not frontier_mask[seed]:
            continue
        queue = [seed]
        frontier_mask[seed] = 0
        component = []
        for index in queue:
            component.append(index)
            row, col = divmod(index, width)
            for dr, dc in _NEIGHBORS_8:
                neighbor_row = row + dr
                neighbor_col = col + dc
                if not (0 <= neighbor_row < height
                        and 0 <= neighbor_col < width):
                    continue
                neighbor = neighbor_row * width + neighbor_col
                if frontier_mask[neighbor]:
                    frontier_mask[neighbor] = 0
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

    spacing = _finite_float(frontier_spacing_m, 1.5)
    if spacing <= 0.0:
        spacing = 1.5
    candidate_limit = max(1, int(max_candidates_per_cluster))
    revisit_distance = max(0.0, _finite_float(revisit_radius))
    visited_points = [
        (_finite_float(item.get("x")), _finite_float(item.get("y")))
        for item in (visited or [])
        if isinstance(item, dict)
    ]

    result = []
    component_mask = bytearray(cell_count)
    for component in components:
        # Reduce a potentially huge/noisy component to one representative per
        # spacing-sized grid bucket in O(N). Sorting every frontier cell and
        # rescanning the whole component for every selected candidate caused
        # multi-second stalls on large valid occupancy grids.
        bucket_cells = max(1, int(math.ceil(spacing / resolution)))
        bucket_representatives = {}
        for index in component:
            component_mask[index] = 1
            row, col = divmod(index, width)
            cell = (row, col)
            key = (row // bucket_cells, col // bucket_cells)
            world_x, world_y = cell_world(cell)
            rank = (
                (world_x - robot_x) ** 2 + (world_y - robot_y) ** 2,
                row, col,
            )
            current = bucket_representatives.get(key)
            if current is None or rank < current[0]:
                bucket_representatives[key] = (rank, index)
        ordered = [
            item[1] for item in sorted(
                bucket_representatives.values(), key=lambda item: item[0])
        ]
        support_radius_cells = int(math.ceil(spacing / resolution))
        support_radius_sq = (spacing / resolution) ** 2
        selected_cells = []
        for representative_index in ordered:
            rep_row, rep_col = divmod(representative_index, width)
            representative = (rep_row, rep_col)
            world_x, world_y = cell_world(representative)
            if any(
                ((rep_row - row) ** 2 + (rep_col - col) ** 2)
                * resolution * resolution < spacing * spacing
                for row, col in selected_cells
            ):
                continue
            if any(math.hypot(world_x - vx, world_y - vy) < revisit_distance
                   for vx, vy in visited_points):
                continue
            support_count = 0
            touches_map_edge = False
            for row in range(
                    max(0, rep_row - support_radius_cells),
                    min(height, rep_row + support_radius_cells + 1)):
                dr_sq = (row - rep_row) ** 2
                for col in range(
                        max(0, rep_col - support_radius_cells),
                        min(width, rep_col + support_radius_cells + 1)):
                    if dr_sq + (col - rep_col) ** 2 > support_radius_sq:
                        continue
                    if not component_mask[row * width + col]:
                        continue
                    support_count += 1
                    touches_map_edge = bool(
                        touches_map_edge
                        or row <= 1 or col <= 1
                        or row >= height - 2 or col >= width - 2)
            selected_cells.append(representative)
            result.append({
                "center_cell": representative,
                "center_world": (world_x, world_y),
                "size": support_count,
                "cluster_size": len(component),
                "information_gain": float(support_count),
                "distance": math.hypot(
                    world_x - robot_x, world_y - robot_y),
                "touches_map_edge": touches_map_edge,
            })
            if len(selected_cells) >= candidate_limit:
                break
        for index in component:
            component_mask[index] = 0
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
    frontier_spacing_m: float = 1.5,
    max_candidates_per_cluster: int = 64,
    distance_weight: Optional[float] = None,
    heading_weight: float = 0.0,
    failure_penalty: float = 1.0,
) -> list[dict]:
    """Return deterministic frontier candidates in utility order.

    This implements cost-distance method (c). Nearest-frontier (a) and maximum
    information-gain (b) remain special cases of the configurable weights.
    """

    cluster_kwargs = {}
    if callable_accepts_keyword(cluster_finder, "frontier_spacing_m"):
        cluster_kwargs["frontier_spacing_m"] = frontier_spacing_m
    if callable_accepts_keyword(cluster_finder, "max_candidates_per_cluster"):
        cluster_kwargs["max_candidates_per_cluster"] = (
            max_candidates_per_cluster)
    clusters = cluster_finder(
        map_msg, robot_pose, visited, min_cluster_size, revisit_radius,
        **cluster_kwargs)
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
            "prefer_standoff": True,
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
