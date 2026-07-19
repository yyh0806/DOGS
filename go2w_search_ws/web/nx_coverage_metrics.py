"""Pure-function ROI-bounded coverage metrics for exploration success validation.

No rclpy dependency. Reads OccupancyGrid via getattr so it works with both real
nav_msgs/OccupancyGrid and SimpleNamespace stubs in unit tests.

Used by RoomSearchOrchestrator._run_frontier_explore at REPORT time to validate
that the closed-room exploration actually covered the reachable free space
*inside the mission ROI*, not the whole grid (which is inflated by
map_padding_bridge's 2m unknown padding around /map_frontier).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Optional


_NEIGHBORS_8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
)


def _finite_float(value, default=0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _map_geometry(map_msg):
    """Return (resolution, width, height, data, origin_x, origin_y, origin_yaw) or None."""
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


def _world_to_cell(wx, wy, resolution, origin_x, origin_y, origin_yaw):
    dx = wx - origin_x
    dy = wy - origin_y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return (int(math.floor(local_x / resolution + 1e-9)),
            int(math.floor(local_y / resolution + 1e-9)))


def _cell_to_world_center(row, col, resolution, origin_x, origin_y, origin_yaw):
    local_x = (col + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    wx = origin_x + cos_yaw * local_x - sin_yaw * local_y
    wy = origin_y + sin_yaw * local_x + cos_yaw * local_y
    return wx, wy


def _in_roi(row, col, resolution, origin_x, origin_y, origin_yaw, roi):
    """Is cell center inside the ROI polygon/circle? roi=None → True (whole map)."""
    if roi is None:
        return True
    wx, wy = _cell_to_world_center(
        row, col, resolution, origin_x, origin_y, origin_yaw)
    rtype = str(roi.get("type") or "")
    if rtype == "circle":
        cx, cy = float(roi["center"][0]), float(roi["center"][1])
        radius = float(roi["radius"])
        return math.hypot(wx - cx, wy - cy) <= radius + 1e-9
    if rtype == "polygon":
        return _point_in_polygon(wx, wy, roi.get("points") or [])
    return True


def _point_in_polygon(x, y, polygon):
    inside = False
    n = len(polygon)
    if n > 256:  # DoS guard: real room polygons are far smaller; conservatively reject huge inputs
        return False
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = float(polygon[i][0]), float(polygon[i][1])
        xj, yj = float(polygon[j][0]), float(polygon[j][1])
        if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _inflate_obstacles(forbidden, occupied_cells_roi, width, height,
                       inflation_radius_cells):
    """Mark cells within inflation_radius of any occupied cell as forbidden.

    Disk inflation: for each occupied seed, every cell whose Chebyshev-distance
    falls inside the radius² disk (Euclidean) is added to ``forbidden``.
    """
    if inflation_radius_cells <= 0:
        return
    radius_sq = inflation_radius_cells * inflation_radius_cells
    for sr, sc in occupied_cells_roi:
        for dr in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for dc in range(-inflation_radius_cells, inflation_radius_cells + 1):
                if dr * dr + dc * dc > radius_sq:
                    continue
                nr, nc = sr + dr, sc + dc
                if 0 <= nr < height and 0 <= nc < width:
                    forbidden.add((nr, nc))


def compute_coverage(map_msg, roi=None, mission_origin=None,
                     inflation_radius_m: float = 0.3) -> Optional[dict]:
    """Scan final occupancy grid inside ROI; return coverage metrics or None.

    Fields:
      - coverage_valid: True iff ROI non-empty and map geometry valid
      - roi: echo of the ROI used (for the report)
      - free_cells/occupied_cells/unknown_cells/total_cells: ROI-bounded counts
      - explored_ratio: (free+occupied)/total in [0,1], None if total=0
      - enclosed_unknown_regions: connected unknown regions inside ROI that
        neither border reachable_free (after inflation) nor touch the ROI/map
        boundary. Each entry: {min_x,min_y,max_x,max_y,cell_count} world frame.
      - map_stamp: header stamp seconds if parseable, else None
      - inflation_radius_m: echo

    roi=None → coverage_valid=False (whole-map ratio is polluted by padding).
    """
    geometry = _map_geometry(map_msg)
    if geometry is None:
        return None
    resolution, width, height, data, origin_x, origin_y, origin_yaw = geometry

    map_stamp = _parse_stamp(getattr(map_msg, "header", None))

    inflation_cells = max(
        0, int(math.ceil(_finite_float(inflation_radius_m, 0.0) / resolution - 1e-9)))

    free_cells_roi = set()
    occupied_cells_roi = set()
    unknown_cells_roi = set()
    for index, value in enumerate(data):
        row, col = divmod(index, width)
        if not _in_roi(row, col, resolution, origin_x, origin_y, origin_yaw, roi):
            continue
        if value == 0:
            free_cells_roi.add((row, col))
        elif value < 0:
            unknown_cells_roi.add((row, col))
        else:
            occupied_cells_roi.add((row, col))

    free = len(free_cells_roi)
    occupied = len(occupied_cells_roi)
    unknown = len(unknown_cells_roi)
    total = free + occupied + unknown
    coverage_valid = (roi is not None) and (total > 0)
    explored_ratio = ((free + occupied) / total) if total > 0 else None

    enclosed = _enclosed_unknown_regions(
        unknown_cells_roi, free_cells_roi, occupied_cells_roi,
        width, height, inflation_cells, mission_origin,
        resolution, origin_x, origin_y, origin_yaw, roi)

    roi_echo = (
        {"type": "whole_map"} if roi is None
        else {"type": str(roi.get("type")), **{
            k: list(v) if isinstance(v, (list, tuple)) else v
            for k, v in roi.items() if k != "type"}})

    return {
        "coverage_valid": bool(coverage_valid),
        "roi": roi_echo,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "total_cells": total,
        "explored_ratio": (round(explored_ratio, 6) if explored_ratio is not None else None),
        "enclosed_unknown_regions": enclosed,
        "map_stamp": map_stamp,
        "inflation_radius_m": _finite_float(inflation_radius_m, 0.0),
    }


def _enclosed_unknown_regions(unknown_cells, free_cells, occupied_cells,
                              width, height, inflation_cells, mission_origin,
                              resolution, origin_x, origin_y, origin_yaw, roi):
    """Connected unknown regions that are enclosed (review #4 corrected rule).

    A region is enclosed iff:
      - all its cells are inside ROI (already true by construction), AND
      - none of its cells borders a reachable_free cell (free cell reachable
        from mission_origin after obstacle inflation), AND
      - none of its cells touches the ROI boundary or the map boundary.
    Regions touching reachable_free are frontiers (reachable); regions touching
    a boundary may be wall interior / building exterior / padding — not reported.
    """
    if not unknown_cells:
        return []

    # 1. forbidden = inflated obstacles; passable_free = free not forbidden
    forbidden = set(occupied_cells)
    _inflate_obstacles(forbidden, occupied_cells, width, height, inflation_cells)
    passable_free = free_cells - forbidden
    if not passable_free:
        # No passable free → every unknown region is enclosed (conservative)
        passable_free = set()

    # 2. reachable_free from mission_origin
    reachable_free = set()
    start_cell = None
    if mission_origin is not None and passable_free:
        try:
            mx, my = float(mission_origin[0]), float(mission_origin[1])
            start_cell = _world_to_cell(
                mx, my, resolution, origin_x, origin_y, origin_yaw)
        except (TypeError, ValueError, IndexError):
            start_cell = None
    if start_cell is not None and start_cell in passable_free:
        reachable_free = _flood_fill(start_cell, passable_free, width, height)
    else:
        # mission_origin not on passable free (or None): treat all passable_free
        # as reachable (conservative — do not over-report enclosed regions).
        reachable_free = set(passable_free)

    # 3. connected components of unknown; check enclosure
    visited = set()
    enclosed = []
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    for seed in unknown_cells:
        if seed in visited:
            continue
        queue = deque([seed])
        visited.add(seed)
        component = []
        borders_reachable = False
        touches_boundary = False
        while queue:
            cell = queue.popleft()
            component.append(cell)
            row, col = cell
            if _cell_touches_boundary_or_roi_edge(
                    row, col, width, height, resolution,
                    origin_x, origin_y, origin_yaw, roi):
                touches_boundary = True
            for dr, dc in _NEIGHBORS_8:
                nr, nc = row + dr, col + dc
                neighbor = (nr, nc)
                if neighbor in reachable_free:
                    borders_reachable = True
                if neighbor in unknown_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        if borders_reachable or touches_boundary:
            continue
        min_row = min(c[0] for c in component)
        max_row = max(c[0] for c in component)
        min_col = min(c[1] for c in component)
        max_col = max(c[1] for c in component)
        corners = []
        for r in (min_row, max_row):
            for c in (min_col, max_col):
                local_x = (c + 0.5) * resolution
                local_y = (r + 0.5) * resolution
                wx = origin_x + cos_yaw * local_x - sin_yaw * local_y
                wy = origin_y + sin_yaw * local_x + cos_yaw * local_y
                corners.append((wx, wy))
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        enclosed.append({
            "min_x": round(min(xs), 3),
            "min_y": round(min(ys), 3),
            "max_x": round(max(xs), 3),
            "max_y": round(max(ys), 3),
            "cell_count": len(component),
        })
    enclosed.sort(key=lambda z: (z["min_x"], z["min_y"]))
    return enclosed


def _cell_touches_boundary_or_roi_edge(row, col, width, height, resolution,
                                       origin_x, origin_y, origin_yaw, roi):
    # Map boundary
    if row <= 0 or col <= 0 or row >= height - 1 or col >= width - 1:
        return True
    if roi is None:
        return False
    # ROI boundary: probe the 4-neighbors; if any is outside the ROI, this cell
    # is on the ROI edge.
    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        nr, nc = row + dr, col + dc
        if not _in_roi(nr, nc, resolution, origin_x, origin_y, origin_yaw, roi):
            return True
    return False


def _flood_fill(start, passable_free, width, height):
    visited = {start}
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for dr, dc in _NEIGHBORS_8:
            neighbor = (row + dr, col + dc)
            if neighbor in passable_free and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return visited


def _parse_stamp(header):
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        if sec == 0 and nanosec == 0:
            return None
        return float(sec) + float(nanosec) * 1e-9
    except (TypeError, ValueError):
        return None


__all__ = ["compute_coverage"]
