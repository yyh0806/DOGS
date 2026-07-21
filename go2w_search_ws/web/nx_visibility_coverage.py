"""LiDAR-bounded C13 visibility coverage for unknown-room exploration.

The tracker is deliberately ROS-free.  It consumes OccupancyGrid-shaped and
LaserScan-shaped objects/dicts, stores coverage in stable world-space buckets,
and exposes bounded candidate scoring for the existing Nav2 frontier manager.
"""

from __future__ import annotations

from collections import deque
import math
import threading
from typing import Any, Iterable, Optional


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _field(value: Any, name: str, default: Any = None) -> Any:
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _angle_delta(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class _Grid:
    def __init__(self, map_msg: Any) -> None:
        info = _field(map_msg, "info")
        self.resolution = _finite(_field(info, "resolution"))
        self.width = int(_field(info, "width", 0) or 0)
        self.height = int(_field(info, "height", 0) or 0)
        data = list(_field(map_msg, "data", []) or [])
        if (
            self.resolution <= 0.0
            or self.width <= 0
            or self.height <= 0
            or len(data) != self.width * self.height
        ):
            raise ValueError("invalid occupancy grid")
        self.data = data
        origin = _field(info, "origin")
        position = _field(origin, "position")
        orientation = _field(origin, "orientation")
        self.origin_x = _finite(_field(position, "x"))
        self.origin_y = _finite(_field(position, "y"))
        qx = _finite(_field(orientation, "x"))
        qy = _finite(_field(orientation, "y"))
        qz = _finite(_field(orientation, "z"))
        qw = _finite(_field(orientation, "w", 1.0), 1.0)
        self.origin_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self._cos = math.cos(self.origin_yaw)
        self._sin = math.sin(self.origin_yaw)

    def world_to_cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        local_x = self._cos * dx + self._sin * dy
        local_y = -self._sin * dx + self._cos * dy
        col = int(math.floor(local_x / self.resolution))
        row = int(math.floor(local_y / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        local_x = (float(col) + 0.5) * self.resolution
        local_y = (float(row) + 0.5) * self.resolution
        return (
            self.origin_x + self._cos * local_x - self._sin * local_y,
            self.origin_y + self._sin * local_x + self._cos * local_y,
        )

    def value(self, row: int, col: int) -> int:
        return int(self.data[row * self.width + col])

    def nearest_free_seed(self, x: float, y: float) -> Optional[tuple[int, int]]:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        if self.value(*cell) == 0:
            return cell
        radius = max(1, int(math.ceil(1.0 / self.resolution)))
        best = None
        for row in range(max(0, cell[0] - radius), min(self.height, cell[0] + radius + 1)):
            for col in range(max(0, cell[1] - radius), min(self.width, cell[1] + radius + 1)):
                if self.value(row, col) != 0:
                    continue
                wx, wy = self.cell_to_world(row, col)
                key = ((wx - x) ** 2 + (wy - y) ** 2, row, col)
                if best is None or key < best[0]:
                    best = (key, (row, col))
        return None if best is None else best[1]

    def reachable_free(self, x: float, y: float) -> list[tuple[int, int]]:
        seed = self.nearest_free_seed(x, y)
        if seed is None:
            return []
        seen = bytearray(self.width * self.height)
        seed_index = seed[0] * self.width + seed[1]
        seen[seed_index] = 1
        queue = deque([seed_index])
        result = []
        while queue:
            index = queue.popleft()
            row, col = divmod(index, self.width)
            result.append((row, col))
            for nr, nc in ((row - 1, col), (row, col - 1),
                           (row, col + 1), (row + 1, col)):
                if not (0 <= nr < self.height and 0 <= nc < self.width):
                    continue
                neighbor = nr * self.width + nc
                if not seen[neighbor] and self.value(nr, nc) == 0:
                    seen[neighbor] = 1
                    queue.append(neighbor)
        return result


class VisibilityCoverageTracker:
    """Accumulate wall-occluded C13 coverage and estimate useful Nav2 steps."""

    def __init__(
        self,
        *,
        camera_hfov_rad: float,
        camera_yaw_offset_rad: float = 0.0,
        visual_range_m: float = 8.0,
        coverage_cell_size_m: float = 0.5,
        min_step_m: float = 1.0,
        max_step_m: float = 8.0,
        obstacle_threshold: int = 50,
        max_scan_age_sec: float = 1.0,
        ray_angle_step_rad: float = math.radians(2.0),
        visual_gain_weight: float = 0.35,
        path_corridor_half_width_m: float = 0.45,
        obstacle_standoff_m: float = 0.6,
    ) -> None:
        self.camera_hfov_rad = min(
            2.0 * math.pi, max(math.radians(5.0), float(camera_hfov_rad)))
        self.camera_yaw_offset_rad = float(camera_yaw_offset_rad)
        self.visual_range_m = max(0.2, float(visual_range_m))
        self.coverage_cell_size_m = max(0.1, float(coverage_cell_size_m))
        self.min_step_m = max(0.1, float(min_step_m))
        self.max_step_m = max(self.min_step_m, float(max_step_m))
        self.obstacle_threshold = max(1, int(obstacle_threshold))
        self.max_scan_age_sec = max(0.0, float(max_scan_age_sec))
        self.ray_angle_step_rad = max(
            math.radians(0.5), float(ray_angle_step_rad))
        self.visual_gain_weight = max(0.0, float(visual_gain_weight))
        self.path_corridor_half_width_m = max(
            0.1, float(path_corridor_half_width_m))
        self.obstacle_standoff_m = max(0.0, float(obstacle_standoff_m))
        self._observed: set[tuple[int, int]] = set()
        # RLock (not Lock) because observe() calls self.snapshot() internally.
        # The lock lets the en-route / progress helper threads call observe()
        # while the main planning thread reads rank_candidates()/snapshot()
        # without racing on the _observed set (which would raise
        # "Set changed size during iteration" mid-ray-cast).
        self._lock = threading.RLock()
        self._last_profile = self._conservative_profile()
        self._last_pose: Optional[tuple[float, float, float]] = None
        self._last_visible: set[tuple[int, int]] = set()
        self._last_scan: Any = None

    def observe(self, map_msg: Any, robot_pose: Iterable, scan_snapshot: Any) -> dict:
        with self._lock:
            grid = _Grid(map_msg)
            pose = self._pose(robot_pose)
            scan_usable = self._scan_usable(scan_snapshot)
            self._last_profile = (
                self._scene_profile(scan_snapshot)
                if scan_usable else self._conservative_profile())
            self._last_scan = scan_snapshot if scan_usable else None
            self._last_pose = pose
            visible = self._visible_buckets(
                grid, pose, scan_snapshot if scan_usable else None)
            self._last_visible = visible
            self._observed.update(visible)
            return self.snapshot(map_msg)

    def rank_candidates(
        self, map_msg: Any, robot_pose: Iterable, candidates: Iterable[dict]
    ) -> list[dict]:
        with self._lock:
            grid = _Grid(map_msg)
            pose = self._pose(robot_pose)
            result = []
            for source in candidates or []:
                candidate = dict(source)
                candidate_pose = (
                    _finite(candidate.get("x")),
                    _finite(candidate.get("y")),
                    _finite(candidate.get("yaw")),
                )
                visible = self._visible_buckets(grid, candidate_pose, None)
                visual_gain = len(visible.difference(self._observed))
                base_gain = _finite(candidate.get(
                    "information_gain", candidate.get("size", 0.0)))
                path_profile = self._candidate_path_profile(pose, candidate_pose)
                candidate.update({
                    "base_information_gain": base_gain,
                    "visual_gain": int(visual_gain),
                    "information_gain": (
                        base_gain + visual_gain * self.visual_gain_weight),
                    "adaptive_step_m": path_profile["adaptive_step_m"],
                    "scene_complexity": path_profile["scene_complexity"],
                    "forward_clearance_m": path_profile["forward_clearance_m"],
                    "path_clearance_m": path_profile["forward_clearance_m"],
                    "heading_change": abs(_angle_delta(candidate_pose[2], pose[2])),
                })
                result.append(candidate)
            return sorted(result, key=lambda item: (
                -_finite(item.get("visual_gain")),
                -_finite(item.get("information_gain")),
                _finite(item.get("heading_change")),
                _finite(item.get("distance")),
                _finite(item.get("x")),
                _finite(item.get("y")),
            ))

    def visual_gain_at(
        self, map_msg: Any, x: float, y: float, yaw: float,
    ) -> int:
        """从 (x,y,yaw) 出发的视锥 frustum 内, 尚不在 _observed 的 bucket 数。

        供 ExplorationManager 的 yaw 优化调用。线程安全 (RLock)。
        """
        with self._lock:
            try:
                grid = _Grid(map_msg)
            except ValueError:
                return 0
            visible = self._visible_buckets(
                grid, (float(x), float(y), float(yaw)), None)
            return len(visible.difference(self._observed))

    def coverage_candidates(
        self,
        map_msg: Any,
        robot_pose: Iterable,
        visited: Iterable[dict],
        *,
        limit: int = 32,
    ) -> list[dict]:
        grid = _Grid(map_msg)
        pose = self._pose(robot_pose)
        output_limit = max(1, int(limit))
        reachable = grid.reachable_free(pose[0], pose[1])
        if not reachable:
            return []
        sample_spacing = max(
            self.coverage_cell_size_m, self.min_step_m)
        visited_points = [
            (_finite(item.get("x")), _finite(item.get("y")))
            for item in (visited or []) if isinstance(item, dict)
        ]
        # Sample in world-space buckets rather than ``row % stride``.  A
        # narrow corridor (or a shifted/rotated SLAM origin) may contain no
        # cell whose absolute row and column both hit that modulo, producing
        # a false "coverage complete" result even with visible free space.
        sampled = {}
        for row, col in reachable:
            x, y = grid.cell_to_world(row, col)
            key = (
                int(math.floor(y / sample_spacing)),
                int(math.floor(x / sample_spacing)),
            )
            center_x = (key[1] + 0.5) * sample_spacing
            center_y = (key[0] + 0.5) * sample_spacing
            rank = ((x - center_x) ** 2 + (y - center_y) ** 2, row, col)
            previous = sampled.get(key)
            if previous is None or rank < previous[0]:
                sampled[key] = (rank, row, col, x, y)
        pool = []
        for _rank, row, col, x, y in sampled.values():
            distance = math.hypot(x - pose[0], y - pose[1])
            if distance < self.min_step_m * 0.5:
                continue
            if any(math.hypot(x - vx, y - vy) < self.coverage_cell_size_m
                   for vx, vy in visited_points):
                continue
            path_profile = self._candidate_path_profile(
                pose, (x, y, pose[2]))
            adaptive_step = path_profile["adaptive_step_m"]
            # A coverage waypoint must itself lie in the currently confirmed
            # LiDAR corridor.  Keep one sample bucket of tolerance so grid
            # quantisation cannot discard the safe shell completely.
            if distance > adaptive_step + sample_spacing * 0.75:
                continue
            pool.append((
                abs(distance - adaptive_step), -distance,
                row, col, x, y, path_profile,
            ))
        # Prefer the outer safe shell instead of truncating to the nearest
        # samples.  The previous nearest-first truncation limited open rooms
        # to roughly two-metre hops despite an eight-metre clear scan.
        pool.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
        pool = pool[:max(128, output_limit * 16)]

        candidates = []
        yaw_options = tuple(
            pose[2] + offset
            for offset in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
        )
        for _step_error, negative_distance, row, col, x, y, path_profile in pool:
            distance = -negative_distance
            best = None
            for yaw in yaw_options:
                visible = self._visible_buckets(grid, (x, y, yaw), None)
                gain = len(visible.difference(self._observed))
                key = (-gain, abs(_angle_delta(yaw, pose[2])), yaw)
                if best is None or key < best[0]:
                    best = (key, gain, yaw)
            if best is None or best[1] <= 0:
                continue
            gain, yaw = best[1], best[2]
            candidates.append({
                "x": x,
                "y": y,
                "yaw": yaw,
                "size": gain,
                "information_gain": gain * self.visual_gain_weight,
                "base_information_gain": 0.0,
                "visual_gain": gain,
                "distance": distance,
                "center_cell": (row, col),
                "touches_map_edge": False,
                "prefer_standoff": False,
                "coverage_candidate": True,
                "adaptive_step_m": path_profile["adaptive_step_m"],
                "scene_complexity": path_profile["scene_complexity"],
                "forward_clearance_m": path_profile["forward_clearance_m"],
                "path_clearance_m": path_profile["forward_clearance_m"],
                "heading_change": abs(_angle_delta(yaw, pose[2])),
                "failure_count": 0,
            })
        candidates.sort(key=lambda item: (
            -item["visual_gain"],
            abs(item["distance"] - item["adaptive_step_m"]),
            item["heading_change"], -item["distance"],
            item["x"], item["y"],
        ))
        return candidates[:output_limit]

    def lidar_candidates(
        self,
        robot_pose: Iterable,
        visited: Iterable[dict],
        *,
        limit: int = 16,
    ) -> list[dict]:
        """Return long safe-corridor goals when the occupancy map lags.

        These goals are not a replacement for Nav2 reachability checks.  They
        only bridge the period where MID360 has measured open space but the
        online OccupancyGrid still contains no usable frontier or free-area
        denominator.
        """

        if self._last_scan is None or not self._scan_usable(self._last_scan):
            return []
        pose = self._pose(robot_pose)
        output_limit = max(1, int(limit))
        visited_points = [
            (_finite(item.get("x")), _finite(item.get("y")))
            for item in (visited or []) if isinstance(item, dict)
        ]
        revisit_radius = max(
            self.coverage_cell_size_m, self.min_step_m * 0.75)
        offsets = [0.0]
        for index in range(1, 9):
            offset = math.pi * index / 8.0
            offsets.extend((offset, -offset))

        candidates = []
        for offset in offsets:
            bearing = pose[2] + offset
            profile = self._path_profile(self._last_scan, offset)
            clearance = _finite(profile.get("forward_clearance_m"))
            distance = _finite(profile.get("adaptive_step_m"))
            if (
                distance < self.min_step_m
                or clearance - self.obstacle_standoff_m
                < self.min_step_m - 1e-9
            ):
                continue
            x = pose[0] + distance * math.cos(bearing)
            y = pose[1] + distance * math.sin(bearing)
            if any(math.hypot(x - vx, y - vy) < revisit_radius
                   for vx, vy in visited_points):
                continue

            projected = set()
            ray_count = max(
                3,
                int(math.ceil(
                    self.camera_hfov_rad / self.ray_angle_step_rad)) + 1,
            )
            ray_step = max(0.1, self.coverage_cell_size_m * 0.5)
            for ray_index in range(ray_count):
                ray_offset = (
                    -self.camera_hfov_rad * 0.5
                    + self.camera_hfov_rad * ray_index
                    / float(ray_count - 1)
                )
                ray_yaw = bearing + self.camera_yaw_offset_rad + ray_offset
                ray_distance = 0.0
                while ray_distance <= self.visual_range_m + 1e-9:
                    projected.add(self._bucket(
                        x + ray_distance * math.cos(ray_yaw),
                        y + ray_distance * math.sin(ray_yaw),
                    ))
                    ray_distance += ray_step
            visual_gain = len(projected.difference(self._observed))
            if visual_gain <= 0:
                continue
            bucket = self._bucket(x, y)
            candidates.append({
                "x": x,
                "y": y,
                "yaw": bearing,
                "size": visual_gain,
                "information_gain": visual_gain * self.visual_gain_weight,
                "base_information_gain": 0.0,
                "visual_gain": visual_gain,
                "distance": distance,
                "center_cell": bucket,
                "touches_map_edge": False,
                "prefer_standoff": False,
                "coverage_candidate": True,
                "lidar_candidate": True,
                "adaptive_step_m": distance,
                "scene_complexity": profile["scene_complexity"],
                "forward_clearance_m": clearance,
                "path_clearance_m": clearance,
                "heading_change": abs(_angle_delta(bearing, pose[2])),
                "failure_count": 0,
            })
        candidates.sort(key=lambda item: (
            -item["visual_gain"],
            item["heading_change"],
            -item["distance"],
            item["x"],
            item["y"],
        ))
        return candidates[:output_limit]

    def snapshot(self, map_msg: Any = None) -> dict:
        with self._lock:
            total_free = 0
            observed_free = 0
            if map_msg is not None and self._last_pose is not None:
                try:
                    grid = _Grid(map_msg)
                    free_buckets = {
                        self._bucket(*grid.cell_to_world(row, col))
                        for row, col in grid.reachable_free(
                            self._last_pose[0], self._last_pose[1])
                    }
                    total_free = len(free_buckets)
                    observed_free = len(free_buckets.intersection(self._observed))
                except (TypeError, ValueError):
                    total_free = observed_free = 0
            ratio = (
                float(observed_free) / float(total_free)
                if total_free > 0 else 0.0)
            observed_cells = [
                {
                    "x": (col + 0.5) * self.coverage_cell_size_m,
                    "y": (row + 0.5) * self.coverage_cell_size_m,
                }
                for row, col in sorted(self._observed)
            ]
            return {
                "observed_cells": observed_cells[-5000:],
                "observed_cell_count": len(self._observed),
                "visible_cell_count": len(self._last_visible),
                "reachable_free_cell_count": total_free,
                "observed_reachable_cell_count": observed_free,
                "visual_coverage_ratio": round(ratio, 6),
                "coverage_cell_size_m": self.coverage_cell_size_m,
                "visual_range_m": self.visual_range_m,
                **self._last_profile,
            }

    def _visible_buckets(
        self,
        grid: _Grid,
        pose: tuple[float, float, float],
        scan_snapshot: Any,
    ) -> set[tuple[int, int]]:
        ray_count = max(
            3, int(math.ceil(self.camera_hfov_rad / self.ray_angle_step_rad)) + 1)
        step = max(0.05, min(
            grid.resolution * 0.5, self.coverage_cell_size_m * 0.5))
        visible = set()
        start = -self.camera_hfov_rad * 0.5
        for index in range(ray_count):
            offset = (
                start + self.camera_hfov_rad * index / float(ray_count - 1))
            relative_angle = self.camera_yaw_offset_rad + offset
            world_angle = pose[2] + relative_angle
            ray_limit = self.visual_range_m
            if scan_snapshot is not None:
                ray_limit = min(
                    ray_limit,
                    self._scan_range(scan_snapshot, relative_angle),
                )
            distance = 0.0
            while distance <= ray_limit + 1e-9:
                x = pose[0] + distance * math.cos(world_angle)
                y = pose[1] + distance * math.sin(world_angle)
                cell = grid.world_to_cell(x, y)
                if cell is None:
                    break
                occupancy = grid.value(*cell)
                if occupancy < 0:
                    # The SLAM occupancy map may trail the live MID360 scan by
                    # several metres in a newly entered factory aisle.  A
                    # fresh per-bearing range still proves that the C13 ray is
                    # unobstructed up to ``ray_limit``; only candidate scoring
                    # without a live scan must stop at unknown space.
                    if scan_snapshot is None:
                        break
                    visible.add(self._bucket(x, y))
                    distance += step
                    continue
                if occupancy >= self.obstacle_threshold:
                    # The blocking surface itself was seen; only the space
                    # behind it remains fogged/unknown.
                    visible.add(self._bucket(x, y))
                    break
                visible.add(self._bucket(x, y))
                distance += step
        return visible

    def _scan_usable(self, scan: Any) -> bool:
        ranges = _field(scan, "ranges", []) or []
        age = _finite(_field(scan, "age_sec", 0.0), float("inf"))
        increment = _finite(_field(scan, "angle_increment", 0.0))
        return bool(
            ranges and increment > 0.0
            and 0.0 <= age <= self.max_scan_age_sec)

    def _scan_range(self, scan: Any, relative_angle: float) -> float:
        ranges = list(_field(scan, "ranges", []) or [])
        angle_min = _finite(_field(scan, "angle_min", 0.0))
        increment = _finite(_field(scan, "angle_increment", 0.0))
        range_min = max(0.0, _finite(_field(scan, "range_min", 0.0)))
        range_max = max(range_min, _finite(
            _field(scan, "range_max", self.visual_range_m), self.visual_range_m))
        if not ranges or increment <= 0.0:
            return min(self.visual_range_m, range_max)
        normalized = relative_angle
        while normalized < angle_min:
            normalized += 2.0 * math.pi
        while normalized > angle_min + increment * (len(ranges) - 1):
            normalized -= 2.0 * math.pi
        index = int(round((normalized - angle_min) / increment))
        if not (0 <= index < len(ranges)):
            return min(self.visual_range_m, range_max)
        value = _finite(ranges[index], range_max)
        if value < range_min:
            return range_min
        return min(range_max, value)

    def _scene_profile(self, scan: Any) -> dict:
        return self._path_profile(scan, self.camera_yaw_offset_rad)

    def _candidate_path_profile(
        self,
        pose: tuple[float, float, float],
        candidate_pose: tuple[float, float, float],
    ) -> dict:
        if self._last_scan is None or not self._scan_usable(self._last_scan):
            return self._conservative_profile()
        bearing = math.atan2(
            candidate_pose[1] - pose[1], candidate_pose[0] - pose[0])
        return self._path_profile(
            self._last_scan, _angle_delta(bearing, pose[2]))

    def _path_profile(self, scan: Any, relative_bearing: float) -> dict:
        ranges = list(_field(scan, "ranges", []) or [])
        angle_min = _finite(_field(scan, "angle_min", 0.0))
        increment = _finite(_field(scan, "angle_increment", 0.0))
        range_min = max(0.0, _finite(_field(scan, "range_min", 0.0)))
        range_max = max(range_min, _finite(
            _field(scan, "range_max", self.visual_range_m), self.visual_range_m))
        if not ranges or increment <= 0.0:
            return self._conservative_profile()
        clearance = range_max
        corridor_samples = 0
        corridor_hits = 0
        hit_tolerance = max(0.05, range_max * 0.01)
        for index, raw in enumerate(ranges):
            angle = angle_min + increment * index
            delta = _angle_delta(angle, relative_bearing)
            cosine = math.cos(delta)
            if cosine <= 0.0:
                continue
            value = _finite(raw, range_max)
            if value < range_min:
                continue
            value = min(range_max, value)
            forward = value * cosine
            lateral = abs(value * math.sin(delta))
            if lateral > self.path_corridor_half_width_m:
                continue
            corridor_samples += 1
            clearance = min(clearance, forward)
            if value < range_max - hit_tolerance:
                corridor_hits += 1
        if corridor_samples <= 0:
            return self._conservative_profile()
        complexity = min(1.0, corridor_hits / float(corridor_samples))
        usable_clearance = max(0.0, clearance - self.obstacle_standoff_m)
        step = usable_clearance * (1.0 - 0.25 * complexity)
        step = min(self.max_step_m, max(self.min_step_m, step))
        return {
            "scan_usable": True,
            "forward_clearance_m": round(clearance, 3),
            "scene_complexity": round(complexity, 4),
            "adaptive_step_m": round(step, 3),
        }

    def _conservative_profile(self) -> dict:
        return {
            "scan_usable": False,
            "forward_clearance_m": self.min_step_m,
            "scene_complexity": 1.0,
            "adaptive_step_m": self.min_step_m,
        }

    def _bucket(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(math.floor(float(y) / self.coverage_cell_size_m)),
            int(math.floor(float(x) / self.coverage_cell_size_m)),
        )

    @staticmethod
    def _pose(value: Iterable) -> tuple[float, float, float]:
        try:
            pose = tuple(float(item) for item in value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("pose must contain x, y, yaw") from exc
        if len(pose) < 3 or not all(math.isfinite(item) for item in pose[:3]):
            raise ValueError("pose must contain finite x, y, yaw")
        return pose[0], pose[1], pose[2]
