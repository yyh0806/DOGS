"""Persistent bounded frontier policy; owns no ROS clients or subscriptions."""

from __future__ import annotations

from collections import OrderedDict
import hashlib
import math
import time
from typing import Any, Callable, Optional

from nx_frontier_planner import (
    point_in_polygon,
    score_frontier,
    select_frontier_candidates,
)


def map_revision(map_msg) -> str:
    """Return a stable revision that changes when occupancy or geometry changes."""

    info = getattr(map_msg, "info", None)
    data = getattr(map_msg, "data", None)
    if info is None or data is None:
        return "unavailable"
    origin = getattr(info, "origin", None)
    position = getattr(origin, "position", None)
    orientation = getattr(origin, "orientation", None)
    geometry = (
        getattr(info, "width", 0), getattr(info, "height", 0),
        getattr(info, "resolution", 0.0),
        getattr(position, "x", 0.0), getattr(position, "y", 0.0),
        getattr(orientation, "z", 0.0), getattr(orientation, "w", 1.0),
    )
    digest = hashlib.sha256(repr(geometry).encode("ascii", "backslashreplace"))
    try:
        digest.update(bytes((int(value) + 1) & 0xFF for value in data))
    except Exception:
        digest.update(repr(list(data)).encode("ascii", "backslashreplace"))
    return digest.hexdigest()[:16]


class ExplorationManager:
    """Own exploration state across map updates and navigation attempts.

    ``navigation_port`` is the mission facade of the process-wide
    ``NavigationGateway``. This class performs read-only path preflights and
    never creates an action client or a ROS subscription.
    """

    MODES = frozenset({"current_room", "whole_floor"})

    def __init__(
        self,
        *,
        navigation_port: Any,
        mission_origin,
        observation_sync: Any = None,
        mode: str = "current_room",
        room_radius_m: Optional[float] = 6.0,
        room_polygon=None,
        initial_radius_m: Optional[float] = None,
        radius_step_m: float = 6.0,
        tile_size_m: float = 6.0,
        stable_exhaustion_cycles: int = 1,
        max_time_s: float = 300.0,
        max_distance_m: Optional[float] = None,
        battery_reserve_percent: float = 20.0,
        max_failures_per_cell: int = 2,
        max_blacklist_entries: int = 256,
        revisit_radius_m: float = 1.0,
        reject_map_edge: bool = True,
        planning_timeout_s: float = 3.0,
        max_plan_probes: int = 60,
        min_goal_distance_m: float = 0.35,
        frontier_standoff_step_m: float = 0.3,
        max_frontier_standoff_steps: int = 3,
        distance_weight: float = 1.0,
        heading_weight: float = 0.15,
        failure_penalty: float = 1.0,
        candidate_selector: Optional[Callable] = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        normalized_mode = str(mode).strip().lower()
        if normalized_mode not in self.MODES:
            raise ValueError("mode must be current_room or whole_floor")
        try:
            origin = tuple(float(value) for value in mission_origin)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("mission_origin must contain x, y, yaw") from exc
        if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
            raise ValueError("mission_origin must contain finite x, y, yaw")
        self.navigation_port = navigation_port
        self.observation_sync = observation_sync
        self.mode = normalized_mode
        self.mission_origin = origin
        self.room_radius_m = (
            None if normalized_mode == "whole_floor" or room_radius_m is None
            else max(0.01, float(room_radius_m)))
        self.room_polygon = (
            [(float(x), float(y)) for x, y in room_polygon]
            if room_polygon else None)
        if self.room_radius_m is None:
            self.initial_radius_m = None
            self._active_radius_m = None
        else:
            initial = (
                self.room_radius_m if initial_radius_m is None
                else max(0.01, float(initial_radius_m)))
            self.initial_radius_m = min(initial, self.room_radius_m)
            self._active_radius_m = self.initial_radius_m
        self.radius_step_m = max(0.01, float(radius_step_m))
        self.tile_size_m = max(0.01, float(tile_size_m))
        self.stable_exhaustion_cycles = max(
            1, int(stable_exhaustion_cycles))
        self.max_time_s = max(0.01, float(max_time_s))
        self.max_distance_m = (
            None if max_distance_m is None else max(0.01, float(max_distance_m)))
        self.battery_reserve_percent = max(
            0.0, min(100.0, float(battery_reserve_percent)))
        self.max_failures_per_cell = max(1, int(max_failures_per_cell))
        self.max_blacklist_entries = max(1, int(max_blacklist_entries))
        self.max_spatial_failure_entries = max(
            256, self.max_blacklist_entries)
        self.revisit_radius_m = max(0.0, float(revisit_radius_m))
        self.reject_map_edge = bool(reject_map_edge)
        self.planning_timeout_s = max(0.05, float(planning_timeout_s))
        self.max_plan_probes = max(1, int(max_plan_probes))
        self.min_goal_distance_m = max(0.0, float(min_goal_distance_m))
        self.frontier_standoff_step_m = max(
            0.01, float(frontier_standoff_step_m))
        self.max_frontier_standoff_steps = max(
            0, int(max_frontier_standoff_steps))
        self.distance_weight = max(0.0, float(distance_weight))
        self.heading_weight = max(0.0, float(heading_weight))
        self.failure_penalty = max(0.0, float(failure_penalty))
        self._candidate_selector = candidate_selector
        self._monotonic = monotonic
        self._started_at = monotonic()
        self._map_revision: Optional[str] = None
        self._visited: list[dict] = []
        self._blacklist: OrderedDict[tuple, dict] = OrderedDict()
        self._spatial_failures: OrderedDict[tuple, int] = OrderedDict()
        self._current_goal: Optional[dict] = None
        self._active_tile: Optional[tuple[int, int]] = (
            (0, 0) if normalized_mode == "current_room" else None)
        self._visited_tiles: set[tuple[int, int]] = set()
        self._exhaustion_streak = 0
        self._distance_m = 0.0
        self._plan_probes = 0
        self._plan_rejections = 0
        self._navigation_failures = 0
        self._last_selection_reason: Optional[str] = None

    def budget_status(self, *, battery_percent: Optional[float] = None) -> Optional[str]:
        if self._monotonic() - self._started_at >= self.max_time_s:
            return "time_budget_exhausted"
        if self.max_distance_m is not None and self._distance_m >= self.max_distance_m:
            return "distance_budget_exhausted"
        if battery_percent is not None:
            try:
                battery = float(battery_percent)
            except (TypeError, ValueError, OverflowError):
                return "battery_state_invalid"
            if not math.isfinite(battery):
                return "battery_state_invalid"
            if battery <= self.battery_reserve_percent:
                return "battery_reserve_reached"
        return None

    def choose_next(self, map_msg, robot_pose) -> Optional[dict]:
        if self.budget_status() is not None:
            self._last_selection_reason = self.budget_status()
            return None
        revision = map_revision(map_msg)
        self._map_revision = revision
        candidate_selector = self._candidate_selector or select_frontier_candidates
        candidates = self._select_candidates(
            candidate_selector, map_msg, robot_pose)
        while (not candidates and self._can_expand_radius()):
            self._expand_radius()
            candidates = self._select_candidates(
                candidate_selector, map_msg, robot_pose)
        if not candidates:
            self._current_goal = None
            self._confirm_exhaustion()
            return None

        eligible = [
            candidate for candidate in candidates
            if self._spatial_failures.get(self._candidate_cell(candidate), 0)
            < self.max_failures_per_cell
            and self._distance_from_pose(candidate, robot_pose)
            >= self.min_goal_distance_m
        ]
        if not eligible:
            self._current_goal = None
            if self._can_expand_radius():
                self._expand_radius()
                self._last_selection_reason = "search_boundary_expanded"
            else:
                self._confirm_exhaustion()
            return None

        self._exhaustion_streak = 0
        eligible = self._prioritize_active_tile(eligible, robot_pose)

        reachable = []
        probes_this_cycle = 0
        for candidate in eligible:
            if probes_this_cycle >= self.max_plan_probes:
                break
            approaches = self._candidate_approaches(candidate, robot_pose)
            last_reason = "unreachable"
            attempts_complete = True
            candidate_reachable = False
            for approach in approaches:
                if probes_this_cycle >= self.max_plan_probes:
                    attempts_complete = False
                    break
                probes_this_cycle += 1
                self._plan_probes += 1
                result = self.navigation_port.compute_path_to_pose(
                    approach["x"], approach["y"], approach["yaw"],
                    frame_id="map", timeout=self.planning_timeout_s)
                if not result.get("ok") or not self._path_respects_room(result):
                    last_reason = str(result.get("reason") or "unreachable")
                    if result.get("ok"):
                        last_reason = "path_leaves_room_polygon"
                    continue
                chosen = dict(approach)
                chosen["path_length"] = float(
                    result.get("path_length", approach.get("distance", 0.0)))
                chosen["path_poses"] = int(result.get("poses", 0) or 0)
                chosen["score"] = score_frontier(
                    chosen,
                    path_length=chosen["path_length"],
                    heading_change=chosen.get("heading_change", 0.0),
                    failure_count=chosen.get("failure_count", 0),
                    distance_weight=self.distance_weight,
                    heading_weight=self.heading_weight,
                    failure_penalty=self.failure_penalty,
                )
                reachable.append(chosen)
                candidate_reachable = True
                break
            else:
                attempts_complete = True
            if not candidate_reachable and attempts_complete:
                self._record_failure(candidate, last_reason, "plan")
        if not reachable:
            # A per-cycle probe cap prevents planner storms. Remaining spatial
            # candidates are retried on the next selection cycle; the lifetime
            # counter is telemetry only and never terminates a large mission.
            self._last_selection_reason = "retry_pending"
            self._current_goal = None
            return None
        reachable.sort(key=lambda item: (
            -item["score"], item["path_length"], item["x"], item["y"]))
        self._current_goal = dict(reachable[0])
        self._last_selection_reason = None
        return dict(self._current_goal)

    def mark_visited(self, candidate: Optional[dict] = None) -> None:
        target = dict(candidate or self._current_goal or {})
        if "x" not in target or "y" not in target:
            return
        self._visited.append({"x": float(target["x"]), "y": float(target["y"])})
        tile = self._tile_key(float(target["x"]), float(target["y"]))
        self._visited_tiles.add(tile)
        self._distance_m += max(0.0, float(target.get("path_length", 0.0)))
        # Successful motion changes the planning vantage point. Previously
        # unreachable spatial cells may now be valid and get one fresh epoch.
        self._spatial_failures.clear()
        self._exhaustion_streak = 0
        self._current_goal = None

    def mark_navigation_failed(self, reason: str, candidate: Optional[dict] = None) -> None:
        target = dict(candidate or self._current_goal or {})
        if target:
            self._record_failure(target, str(reason or "navigation_failed"), "navigation")
        self._current_goal = None

    def snapshot(self) -> dict:
        return {
            "mode": self.mode,
            "mission_origin": list(self.mission_origin),
            "map_revision": self._map_revision,
            "initial_radius_m": self.initial_radius_m,
            "active_radius_m": self._active_radius_m,
            "max_radius_m": self.room_radius_m,
            "radius_step_m": self.radius_step_m,
            "tile_size_m": self.tile_size_m,
            "active_tile": (
                None if self._active_tile is None else list(self._active_tile)),
            "visited_tiles": [
                list(tile) for tile in sorted(self._visited_tiles)],
            "exhaustion_streak": self._exhaustion_streak,
            "stable_exhaustion_cycles": self.stable_exhaustion_cycles,
            "min_goal_distance_m": self.min_goal_distance_m,
            "frontier_standoff_step_m": self.frontier_standoff_step_m,
            "max_frontier_standoff_steps": self.max_frontier_standoff_steps,
            "visited_frontiers": [dict(item) for item in self._visited],
            "blacklist": [dict(record) for record in self._blacklist.values()],
            "current_goal": (
                None if self._current_goal is None else dict(self._current_goal)),
            "distance_m": self._distance_m,
            "plan_probes": self._plan_probes,
            "plan_rejections": self._plan_rejections,
            "navigation_failures": self._navigation_failures,
            "last_selection_reason": self._last_selection_reason,
            "elapsed_s": max(0.0, self._monotonic() - self._started_at),
        }

    def _record_failure(self, candidate: dict, reason: str, stage: str) -> None:
        cell = self._candidate_cell(candidate)
        spatial_count = self._spatial_failures.pop(cell, 0) + 1
        self._spatial_failures[cell] = spatial_count
        while len(self._spatial_failures) > self.max_spatial_failure_entries:
            self._spatial_failures.popitem(last=False)
        key = (self._map_revision, cell)
        record = self._blacklist.pop(key, {
            "map_revision": self._map_revision,
            "center_cell": list(cell),
            "x": float(candidate["x"]),
            "y": float(candidate["y"]),
            "failures": 0,
            "last_reason": reason,
            "last_stage": stage,
        })
        record["failures"] += 1
        record["last_reason"] = reason
        record["last_stage"] = stage
        self._blacklist[key] = record
        while len(self._blacklist) > self.max_blacklist_entries:
            self._blacklist.popitem(last=False)
        if stage == "plan":
            self._plan_rejections += 1
        else:
            self._navigation_failures += 1

    @staticmethod
    def _candidate_cell(candidate: dict) -> tuple[int, int]:
        cell = candidate.get("center_cell")
        if cell is not None:
            return tuple(int(value) for value in cell)
        bucket = 0.25
        return (
            int(round(float(candidate["y"]) / bucket)),
            int(round(float(candidate["x"]) / bucket)),
        )

    @staticmethod
    def _distance_from_pose(candidate: dict, robot_pose) -> float:
        try:
            return math.hypot(
                float(candidate["x"]) - float(robot_pose[0]),
                float(candidate["y"]) - float(robot_pose[1]),
            )
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            return 0.0

    def _candidate_approaches(self, candidate: dict, robot_pose) -> list[dict]:
        approaches = [dict(candidate)]
        if self.mode != "current_room" or self.max_frontier_standoff_steps <= 0:
            return approaches
        try:
            robot_x, robot_y = float(robot_pose[0]), float(robot_pose[1])
            frontier_x = float(candidate["x"])
            frontier_y = float(candidate["y"])
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            return approaches
        distance = math.hypot(frontier_x - robot_x, frontier_y - robot_y)
        if not math.isfinite(distance) or distance <= 1e-9:
            return approaches
        for step in range(1, self.max_frontier_standoff_steps + 1):
            standoff = step * self.frontier_standoff_step_m
            remaining = distance - standoff
            if remaining + 1e-9 < self.min_goal_distance_m:
                break
            ratio = remaining / distance
            approach = dict(candidate)
            approach.update({
                "x": robot_x + (frontier_x - robot_x) * ratio,
                "y": robot_y + (frontier_y - robot_y) * ratio,
                "frontier_x": frontier_x,
                "frontier_y": frontier_y,
                "approach_standoff_m": standoff,
            })
            approaches.append(approach)
        return approaches

    def _path_respects_room(self, path_result: dict) -> bool:
        if self.mode != "current_room" or not self.room_polygon:
            return True
        path = path_result.get("path")
        if not path:
            return True
        for pose in path:
            try:
                x = float(pose.get("x") if isinstance(pose, dict) else pose[0])
                y = float(pose.get("y") if isinstance(pose, dict) else pose[1])
            except (TypeError, ValueError, IndexError):
                return False
            if not point_in_polygon(x, y, self.room_polygon):
                return False
        return True

    def _select_candidates(self, candidate_selector, map_msg, robot_pose):
        failures = {
            cell: int(count) for cell, count in self._spatial_failures.items()
        }
        candidates = candidate_selector(
            map_msg,
            robot_pose,
            self._visited,
            revisit_radius=self.revisit_radius_m,
            origin_pose=self.mission_origin,
            max_radius=self._active_radius_m,
            room_polygon=self.room_polygon,
            reject_map_edge=self.reject_map_edge,
            failure_counts=failures,
            distance_weight=self.distance_weight,
            heading_weight=self.heading_weight,
            failure_penalty=self.failure_penalty,
        )
        # Enforce the active mission bound here too, so custom/test candidate
        # sources cannot bypass dynamic current-room containment.
        if self._active_radius_m is not None:
            candidates = [
                item for item in candidates
                if math.hypot(
                    float(item["x"]) - self.mission_origin[0],
                    float(item["y"]) - self.mission_origin[1],
                ) <= self._active_radius_m
            ]
        if self.room_polygon:
            candidates = [
                item for item in candidates
                if point_in_polygon(
                    float(item["x"]), float(item["y"]), self.room_polygon)
            ]
        return [dict(item) for item in candidates]

    def _can_expand_radius(self) -> bool:
        return bool(
            self.mode == "current_room"
            and not self.room_polygon
            and self._active_radius_m is not None
            and self.room_radius_m is not None
            and self._active_radius_m + 1e-9 < self.room_radius_m
        )

    def _expand_radius(self) -> None:
        if not self._can_expand_radius():
            return
        self._active_radius_m = min(
            self.room_radius_m, self._active_radius_m + self.radius_step_m)
        self._spatial_failures.clear()
        self._exhaustion_streak = 0

    def _tile_key(self, x: float, y: float) -> tuple[int, int]:
        half = self.tile_size_m * 0.5
        return (
            int(math.floor((float(x) - self.mission_origin[0] + half) /
                           self.tile_size_m)),
            int(math.floor((float(y) - self.mission_origin[1] + half) /
                           self.tile_size_m)),
        )

    def _prioritize_active_tile(self, candidates, robot_pose):
        if self.mode != "current_room" or not candidates:
            return candidates
        annotated = []
        for item in candidates:
            candidate = dict(item)
            tile = self._tile_key(candidate["x"], candidate["y"])
            candidate["tile"] = list(tile)
            annotated.append(candidate)
        active = [
            item for item in annotated
            if tuple(item["tile"]) == self._active_tile
        ]
        if active:
            return active

        if self._active_tile is not None:
            self._visited_tiles.add(self._active_tile)
        try:
            robot_tile = self._tile_key(robot_pose[0], robot_pose[1])
        except (TypeError, ValueError, IndexError):
            robot_tile = (0, 0)
        tile_keys = {tuple(item["tile"]) for item in annotated}
        self._active_tile = min(tile_keys, key=lambda tile: (
            abs(tile[0] - robot_tile[0]) + abs(tile[1] - robot_tile[1]),
            tile[0], tile[1],
        ))
        return [
            item for item in annotated
            if tuple(item["tile"]) == self._active_tile
        ]

    def _confirm_exhaustion(self) -> None:
        self._exhaustion_streak += 1
        if self._exhaustion_streak >= self.stable_exhaustion_cycles:
            self._last_selection_reason = "reachable_frontiers_exhausted"
        else:
            self._last_selection_reason = "stability_confirmation_pending"
