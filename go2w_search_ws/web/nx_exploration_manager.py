"""Persistent bounded frontier policy; owns no ROS clients or subscriptions."""

from __future__ import annotations

from collections import OrderedDict
import hashlib
import math
import os
import threading
import time
import logging
from typing import Any, Callable, Optional

logger = logging.getLogger("go2w.exploration")

from nx_frontier_planner import (
    callable_accepts_keyword,
    find_frontier_clusters,
    point_in_polygon,
    score_frontier,
    select_frontier_candidates,
)
from nx_global_search_state import (
    analyze_global_search_state,
    path_crosses_entrance_gate,
)


def _abs_angle_delta(a: float, b: float) -> float:
    """abs 角度差, 结果在 [0, π]。"""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


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
        entrance_gate=None,
        initial_radius_m: Optional[float] = None,
        radius_step_m: float = 6.0,
        tile_size_m: float = 6.0,
        frontier_spacing_m: float = 1.5,
        stable_exhaustion_cycles: int = 3,
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
        frontier_lookahead_trigger_m: float = 0.75,
        frontier_lookahead_goal_m: float = 1.2,
        initial_turn_staging_threshold_rad: float = math.radians(75.0),
        initial_turn_staging_distance_m: float = 0.8,
        local_turn_threshold_rad: float = 0.2,
        min_path_progress_m: float = 0.25,
        max_goal_endpoint_error_m: float = 0.05,
        goal_revalidation_failures: int = 2,
        max_path_stretch_ratio: float = 3.0,
        max_path_detour_m: float = 1.5,
        max_navigation_distance_ratio: float = 1.5,
        max_navigation_distance_increase_m: float = 0.5,
        navigation_divergence_samples: int = 3,
        frontier_standoff_step_m: float = 0.3,
        max_frontier_standoff_steps: int = 3,
        distance_weight: float = 1.0,
        heading_weight: float = 0.15,
        open_space_heading_weight: float = 1.0,
        failure_penalty: float = 1.0,
        visibility_tracker: Any = None,
        visual_coverage_threshold: float = 0.9,
        global_coverage_threshold: float = 0.95,
        global_traversal_clearance_m: float = 0.40,
        coverage_candidate_limit: int = 32,
        candidate_selector: Optional[Callable] = None,
        monotonic: Callable[[], float] = time.monotonic,
        # Mixed-utility scoring (2026-07-20). When utility_mode != "mixed"
        # the manager preserves the historical nearest-reachable-first
        # behaviour (locked by test_frontier_explore score contracts).
        # In "mixed" mode the eligible.sort primary key becomes a linear
        # combination that rewards frontier size + visual gain and penalises
        # path cost, so the dog actively seeks unexplored area instead of
        # always walking the closest small frontier first.
        utility_mode: str = "nearest",
        mixed_frontier_weight: float = 0.5,
        mixed_visual_gain_weight: float = 1.0,
        mixed_path_cost_penalty: float = 0.5,
        # v3 (2026-07-21): heading 时间归一化 + wall_bonus + yaw 优化
        mixed_heading_penalty: float = 0.0,
        mixed_wall_bonus: float = 0.0,
        mixed_expansion_bonus: float = 0.1,
        unknown_eta_band_s: float = 10.0,
        yaw_step_deg: float = 45.0,
        max_vel_x: float = 1.5,
        max_vel_theta: float = 1.0,
        # Parallel Nav2 probe (2026-07-20). 0 = serial (historical, locked
        # by plan_calls ordering tests). >0 = ThreadPoolExecutor workers
        # used to probe the first approach of each eligible candidate
        # concurrently, reducing worst-case selection stall from
        # max_plan_probes × planning_timeout_s (e.g. 12×3s=36s) to a single
        # planning_timeout_s round (~3s) when many candidates are reachable.
        parallel_probe_workers: int = 0,
        candidate_analysis_limit: int = 24,
        yaw_optimization_candidate_limit: int = 12,
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
        if entrance_gate is None:
            self.entrance_gate = None
        else:
            try:
                normalized_gate = {
                    "center_x": float(entrance_gate["center_x"]),
                    "center_y": float(entrance_gate["center_y"]),
                    "yaw": float(entrance_gate["yaw"]),
                    "width_m": float(entrance_gate["width_m"]),
                }
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                raise ValueError("entrance_gate is invalid") from exc
            if (
                normalized_gate["width_m"] <= 0.0
                or not all(math.isfinite(value) for value in normalized_gate.values())
            ):
                raise ValueError("entrance_gate is invalid")
            self.entrance_gate = normalized_gate
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
        self.frontier_spacing_m = max(0.05, float(frontier_spacing_m))
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
        self.frontier_lookahead_trigger_m = max(
            self.min_goal_distance_m,
            float(frontier_lookahead_trigger_m),
        )
        self.frontier_lookahead_goal_m = max(
            self.frontier_lookahead_trigger_m,
            float(frontier_lookahead_goal_m),
        )
        self.initial_turn_staging_threshold_rad = min(
            math.pi,
            max(0.0, float(initial_turn_staging_threshold_rad)),
        )
        self.initial_turn_staging_distance_m = max(
            self.min_goal_distance_m,
            float(initial_turn_staging_distance_m),
        )
        self.local_turn_threshold_rad = min(
            math.pi, max(0.0, float(local_turn_threshold_rad)))
        self.min_path_progress_m = max(0.0, float(min_path_progress_m))
        self.max_goal_endpoint_error_m = max(
            0.0, float(max_goal_endpoint_error_m))
        self.goal_revalidation_failures = max(
            1, int(goal_revalidation_failures))
        self.max_path_stretch_ratio = max(
            1.0, float(max_path_stretch_ratio))
        self.max_path_detour_m = max(0.0, float(max_path_detour_m))
        self.max_navigation_distance_ratio = max(
            1.0, float(max_navigation_distance_ratio))
        self.max_navigation_distance_increase_m = max(
            0.0, float(max_navigation_distance_increase_m))
        self.navigation_divergence_samples = max(
            1, int(navigation_divergence_samples))
        self.frontier_standoff_step_m = max(
            0.01, float(frontier_standoff_step_m))
        self.max_frontier_standoff_steps = max(
            0, int(max_frontier_standoff_steps))
        self.distance_weight = max(0.0, float(distance_weight))
        self.heading_weight = max(0.0, float(heading_weight))
        self.open_space_heading_weight = max(
            0.0, float(open_space_heading_weight))
        self.failure_penalty = max(0.0, float(failure_penalty))
        self.visibility_tracker = visibility_tracker
        self.visual_coverage_threshold = min(
            1.0, max(0.0, float(visual_coverage_threshold)))
        self.global_coverage_threshold = min(
            1.0, max(0.0, float(global_coverage_threshold)))
        self.global_traversal_clearance_m = max(
            0.0, float(global_traversal_clearance_m))
        self.coverage_candidate_limit = max(1, int(coverage_candidate_limit))
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
        self._last_exhaustion_revision = None
        self._global_search_state = {}
        self._distance_m = 0.0
        self._plan_probes = 0
        self._plan_rejections = 0
        self._navigation_failures = 0
        self._last_selection_reason: Optional[str] = None
        self._last_selected_metrics: dict = {}
        self._navigation_start_distance: Optional[float] = None
        self._navigation_start_pose: Optional[tuple[float, float, float]] = None
        self._navigation_divergence_count = 0
        self._goal_revalidation_failure_count = 0
        self._visibility_snapshot: dict = {}
        # Trap-escape counter (2026-07-21). When the dog gets stuck in a
        # dense-obstacle pocket, DWB aborts back-to-back on nearby frontiers.
        # After escape_abort_threshold consecutive failures with no success,
        # choose_next drops near candidates and picks a far one to jump out.
        self._consecutive_nav_failures = 0
        self._escape_abort_threshold = max(1, int(3))
        self._escape_min_distance_m = max(0.0, float(3.0))
        self._motion_trap: dict = {}
        self._raw_candidate_count = 0
        self._analyzed_candidate_count = 0
        self._failure_filtered_candidate_count = 0
        self._yaw_optimized_candidate_count = 0
        self._candidate_analysis_ms = 0.0
        self._yaw_optimization_ms = 0.0
        # worker thread writing while the main planning/broadcast thread
        # reads via snapshot()/_effective_heading_weight(). The visibility
        # tracker's own RLock protects _observed; this lock is the
        # ExplorationManager-side mirror for its cached snapshot field.
        self._visibility_lock = threading.Lock()
        self._staging_transition_pending = False
        # Mixed-utility mode (env-overridable). GO2W_FRONTIER_UTILITY_MODE
        # accepts "nearest" (default, historical behaviour) or "mixed"
        # (linear combination of frontier size, visual gain, path cost).
        env_mode = str(os.environ.get(
            "GO2W_FRONTIER_UTILITY_MODE", str(utility_mode))).strip().lower()
        self.utility_mode = "mixed" if env_mode == "mixed" else "nearest"
        self.mixed_frontier_weight = max(0.0, float(mixed_frontier_weight))
        self.mixed_visual_gain_weight = max(0.0, float(mixed_visual_gain_weight))
        self.mixed_path_cost_penalty = max(0.0, float(mixed_path_cost_penalty))
        # v3 (2026-07-21): heading 时间归一化 + wall_bonus + yaw 优化.
        # 所有 env override 走 max() 下限保护, 避免误配 0/负数导致除零
        # 或反向 selection.
        self.mixed_heading_penalty = max(0.0, float(os.environ.get(
            "GO2W_FRONTIER_TIME_PENALTY", str(mixed_heading_penalty))))
        self.mixed_wall_bonus = max(0.0, float(os.environ.get(
            "GO2W_FRONTIER_MIXED_WALL_BONUS", str(mixed_wall_bonus))))
        self.mixed_expansion_bonus = max(0.0, float(os.environ.get(
            "GO2W_FRONTIER_MIXED_EXPANSION_BONUS", str(mixed_expansion_bonus))))
        self.unknown_eta_band_s = max(0.1, float(unknown_eta_band_s))
        self.yaw_step_deg = max(5.0, float(os.environ.get(
            "GO2W_FRONTIER_YAW_STEP_DEG", str(yaw_step_deg))))
        self.max_vel_x = max(0.1, float(os.environ.get(
            "GO2W_FRONTIER_MAX_VEL_X", str(max_vel_x))))
        self.max_vel_theta = max(0.05, float(os.environ.get(
            "GO2W_FRONTIER_MAX_VEL_THETA", str(max_vel_theta))))
        try:
            env_workers = int(os.environ.get(
                "GO2W_FRONTIER_PROBE_WORKERS",
                str(parallel_probe_workers)))
        except (TypeError, ValueError):
            env_workers = 0
        self.parallel_probe_workers = max(0, int(env_workers))
        try:
            env_analysis_limit = int(os.environ.get(
                "GO2W_FRONTIER_ANALYSIS_LIMIT",
                str(candidate_analysis_limit)))
        except (TypeError, ValueError):
            env_analysis_limit = candidate_analysis_limit
        self.candidate_analysis_limit = max(1, int(env_analysis_limit))
        try:
            env_yaw_limit = int(os.environ.get(
                "GO2W_FRONTIER_YAW_CANDIDATE_LIMIT",
                str(yaw_optimization_candidate_limit)))
        except (TypeError, ValueError):
            env_yaw_limit = yaw_optimization_candidate_limit
        self.yaw_optimization_candidate_limit = max(
            1, min(self.candidate_analysis_limit, int(env_yaw_limit)))

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
        self._motion_trap = {}
        budget_reason = self.budget_status()
        if budget_reason is not None:
            self._clear_selected_goal()
            self._last_selection_reason = budget_reason
            return None
        if (self.mode == "current_room"
                and os.environ.get(
                    "GO2W_FRONTIER_ROOM_ENCLOSURE_CHECK", "0") == "1"
                and self.detect_room_enclosure(map_msg, robot_pose)):
            self._clear_selected_goal()
            self._last_selection_reason = "room_enclosed"
            return None

        self._map_revision = map_revision(map_msg)
        candidate_selector = (
            self._candidate_selector or select_frontier_candidates)
        candidates = self._select_candidates(
            candidate_selector, map_msg, robot_pose)
        candidates = self._optimize_yaw_for_candidates(
            candidates, robot_pose, map_msg)
        if (
            not candidates
            and self._failure_filtered_candidate_count > 0
            and self._can_expand_radius()
        ):
            self._clear_selected_goal()
            self._expand_radius()
            self._last_selection_reason = "search_boundary_expanded"
            return None
        while self._can_expand_radius():
            truncated = getattr(self, "_last_radius_truncated", 0)
            if candidates and truncated == 0:
                break
            self._expand_radius()
            candidates = self._select_candidates(
                candidate_selector, map_msg, robot_pose)
            candidates = self._optimize_yaw_for_candidates(
                candidates, robot_pose, map_msg)

        seeded = {"frontier": [], "coverage": [], "lidar": []}
        for candidate in candidates:
            seeded[self._exploration_source(candidate)].append(candidate)

        remaining_budget = self.max_plan_probes
        parallel_batch_available = True
        for source in ("frontier", "coverage", "lidar"):
            tier_candidates = list(seeded[source])
            if source == "coverage":
                tier_candidates.extend(
                    self._select_visual_coverage_candidates(
                        map_msg, robot_pose))
            elif source == "lidar":
                tier_candidates.extend(
                    self._select_lidar_candidates(robot_pose))
            if not tier_candidates:
                continue
            if remaining_budget <= 0:
                self._clear_selected_goal()
                self._exhaustion_streak = 0
                self._last_selection_reason = "retry_pending"
                return None

            eligible = self._eligible_candidates(
                tier_candidates, robot_pose, map_msg)
            if not eligible:
                if self._motion_trap:
                    self._clear_selected_goal()
                    self._exhaustion_streak = 0
                    self._last_selection_reason = "motion_trapped"
                    return None
                if source == "frontier" and self._can_expand_radius():
                    self._clear_selected_goal()
                    self._expand_radius()
                    self._last_selection_reason = "search_boundary_expanded"
                    return None
                continue

            chosen, probes_used, incomplete, batch_used = (
                self._probe_ordered_candidates(
                    eligible, robot_pose, remaining_budget,
                    allow_parallel=parallel_batch_available))
            if batch_used:
                parallel_batch_available = False
            remaining_budget -= probes_used
            if chosen is not None:
                return self._activate_selected_goal(
                    chosen, robot_pose, map_msg)
            if incomplete:
                self._clear_selected_goal()
                self._exhaustion_streak = 0
                self._last_selection_reason = "retry_pending"
                return None
            if source == "frontier" and self._can_expand_radius():
                self._clear_selected_goal()
                self._expand_radius()
                self._last_selection_reason = "search_boundary_expanded"
                return None

        self._clear_selected_goal()
        self._confirm_exhaustion(map_msg)
        return None

    def _activate_selected_goal(
            self, chosen: dict, robot_pose, map_msg) -> dict:
        ranked = self._rank_exploration_candidates(
            [chosen], robot_pose, map_msg)
        self._current_goal = dict(ranked[0])
        self._exhaustion_streak = 0
        self._last_exhaustion_revision = None
        self._navigation_start_distance = self._distance_from_pose(
            self._current_goal, robot_pose)
        self._navigation_start_pose = tuple(
            float(value) for value in robot_pose[:3])
        self._navigation_divergence_count = 0
        self._goal_revalidation_failure_count = 0
        self._last_selection_reason = None
        self._last_selected_metrics = {
            "source": self._current_goal["exploration_source"],
            "unknown_gain": float(self._current_goal["unknown_gain"]),
            "wall_proximity_bonus": float(
                self._current_goal["wall_proximity_bonus"]),
            "eta_s": float(self._current_goal["eta_s"]),
        }
        logger.info(
            "exploration selected source=%s unknown_gain=%.3f "
            "wall_proximity_bonus=%.3f eta_s=%.3f",
            self._last_selected_metrics["source"],
            self._last_selected_metrics["unknown_gain"],
            self._last_selected_metrics["wall_proximity_bonus"],
            self._last_selected_metrics["eta_s"],
        )
        return dict(self._current_goal)

    def _clear_selected_goal(self) -> None:
        self._current_goal = None
        self._last_selected_metrics = {}

    def observe_environment(self, map_msg, robot_pose, scan_snapshot) -> dict:
        """Accumulate LiDAR-bounded camera coverage at the current pose.

        Thread-safe: called from the main frontier loop AND from the
        en-route progress worker concurrently. The lock ensures the
        _visibility_snapshot rebind and the visibility_tracker.observe
        mutation are not interleaved with snapshot() readers.
        """

        if self.visibility_tracker is None:
            with self._visibility_lock:
                self._visibility_snapshot = {}
            return {}
        observed = dict(self.visibility_tracker.observe(
            map_msg, robot_pose, scan_snapshot) or {})
        with self._visibility_lock:
            self._visibility_snapshot = observed
        return dict(observed)

    def _visibility_snapshot_snapshot(self) -> dict:
        """Lock-guarded copy of the cached visibility snapshot."""
        with self._visibility_lock:
            return dict(self._visibility_snapshot)

    def mark_visited(self, candidate: Optional[dict] = None) -> None:
        target = dict(candidate or self._current_goal or {})
        if "x" not in target or "y" not in target:
            return
        self._distance_m += max(0.0, float(target.get("path_length", 0.0)))
        if "approach_staging_m" in target:
            # A heading-aligned escape point only creates room for the next
            # turn.  It neither observes nor completes the physical frontier.
            # Suppress another staging goal until one real exploration target
            # has been attempted from the improved vantage point.
            self._staging_transition_pending = True
            self._spatial_failures.clear()
            self._exhaustion_streak = 0
            self._clear_selected_goal()
            self._reset_navigation_progress()
            return
        self._staging_transition_pending = False
        self._consecutive_nav_failures = 0
        self._visited.append({"x": float(target["x"]), "y": float(target["y"])})
        tile = self._tile_key(float(target["x"]), float(target["y"]))
        self._visited_tiles.add(tile)
        # Successful motion changes the planning vantage point. Previously
        # unreachable spatial cells may now be valid and get one fresh epoch.
        self._spatial_failures.clear()
        self._exhaustion_streak = 0
        self._clear_selected_goal()
        self._reset_navigation_progress()

    def mark_navigation_failed(
            self, reason: str, candidate: Optional[dict] = None,
            robot_pose=None) -> Optional[str]:
        target = dict(candidate or self._current_goal or {})
        normalized_reason = str(reason or "navigation_failed")
        start_pose = self._navigation_start_pose
        visibility = self._visibility_snapshot_snapshot()
        motion_trapped = False
        if (
                normalized_reason in {
                    "nav2_aborted", "controller_abort", "controller_failed"}
                and start_pose is not None
                and robot_pose is not None):
            try:
                progress = math.hypot(
                    float(robot_pose[0]) - float(start_pose[0]),
                    float(robot_pose[1]) - float(start_pose[1]),
                )
            except (TypeError, ValueError, IndexError, OverflowError):
                progress = math.inf
            motion_trapped = (
                progress < 0.10
                and bool(visibility.get("path_blocked"))
                and bool(visibility.get("turn_motion_blocked"))
            )
        if target:
            self._record_failure(target, normalized_reason, "navigation")
        self._consecutive_nav_failures += 1
        self._clear_selected_goal()
        self._reset_navigation_progress()
        if motion_trapped:
            self._motion_trap = self._motion_trap_evidence(
                target, visibility, reason="nav2_zero_progress")
            self._last_selection_reason = "motion_trapped"
            return "motion_trapped"
        return None

    def observe_navigation_pose(self, robot_pose) -> dict:
        """Fail closed after repeated motion materially away from the goal."""

        if self._current_goal is None or self._navigation_start_distance is None:
            return {"ok": True, "reason": "no_active_goal"}
        distance = self._distance_from_pose(self._current_goal, robot_pose)
        if not math.isfinite(distance):
            return {"ok": True, "reason": "pose_unavailable"}
        allowed = max(
            self._navigation_start_distance * self.max_navigation_distance_ratio,
            self._navigation_start_distance
            + self.max_navigation_distance_increase_m,
        )
        if distance > allowed + 1e-9:
            self._navigation_divergence_count += 1
        else:
            self._navigation_divergence_count = 0
        failed = (
            self._navigation_divergence_count
            >= self.navigation_divergence_samples)
        return {
            "ok": not failed,
            "reason": "navigation_diverging" if failed else None,
            "distance_to_goal_m": distance,
            "allowed_distance_m": allowed,
            "divergence_samples": self._navigation_divergence_count,
            "required_samples": self.navigation_divergence_samples,
        }

    def revalidate_current_goal(self) -> dict:
        """Reject a moving leg when fresh Nav2 planning no longer reaches it."""

        if self._current_goal is None:
            return {"ok": True, "reason": "no_active_goal"}
        target = self._current_goal
        try:
            result = self.navigation_port.compute_path_to_pose(
                target["x"], target["y"], target.get("yaw", 0.0),
                frame_id="map", timeout=self.planning_timeout_s)
        except Exception:
            result = {"ok": False, "reason": "planner_error"}
        planner_reason = str(result.get("reason") or "")
        if not result.get("ok") and planner_reason == "plan_timeout":
            # This auxiliary probe shares the planner with Nav2's active
            # replanning. A probe timeout is no evidence that the commanded
            # goal became blocked; canceling after two busy samples caused
            # healthy legs to stop and enter a long reselection cycle.
            self._goal_revalidation_failure_count = 0
            return {
                "ok": True,
                "reason": "goal_revalidation_inconclusive",
                "goal_revalidation_failures": 0,
                "required_failures": self.goal_revalidation_failures,
                "planner_reason": planner_reason,
                "goal_error_m": result.get("goal_error_m"),
            }
        reachable = bool(result.get("ok")) and self._path_endpoint_reaches_goal(
            result)
        if reachable:
            self._goal_revalidation_failure_count = 0
        else:
            self._goal_revalidation_failure_count += 1
        failed = (
            self._goal_revalidation_failure_count
            >= self.goal_revalidation_failures)
        return {
            "ok": not failed,
            "reason": "goal_became_unreachable" if failed else (
                "goal_revalidation_pending" if not reachable else None),
            "goal_revalidation_failures": (
                self._goal_revalidation_failure_count),
            "required_failures": self.goal_revalidation_failures,
            "planner_reason": result.get("reason"),
            "goal_error_m": result.get("goal_error_m"),
        }

    def snapshot(self) -> dict:
        return {
            "mode": self.mode,
            "mission_origin": list(self.mission_origin),
            "entrance_gate": (
                dict(self.entrance_gate) if self.entrance_gate else None),
            "map_revision": self._map_revision,
            "initial_radius_m": self.initial_radius_m,
            "active_radius_m": self._active_radius_m,
            "max_radius_m": self.room_radius_m,
            "radius_step_m": self.radius_step_m,
            "tile_size_m": self.tile_size_m,
            "frontier_spacing_m": self.frontier_spacing_m,
            "active_tile": (
                None if self._active_tile is None else list(self._active_tile)),
            "visited_tiles": [
                list(tile) for tile in sorted(self._visited_tiles)],
            "exhaustion_streak": self._exhaustion_streak,
            "stable_exhaustion_cycles": self.stable_exhaustion_cycles,
            "min_goal_distance_m": self.min_goal_distance_m,
            "frontier_lookahead_trigger_m": self.frontier_lookahead_trigger_m,
            "frontier_lookahead_goal_m": self.frontier_lookahead_goal_m,
            "initial_turn_staging_threshold_rad": (
                self.initial_turn_staging_threshold_rad),
            "initial_turn_staging_distance_m": (
                self.initial_turn_staging_distance_m),
            "local_turn_threshold_rad": self.local_turn_threshold_rad,
            "min_path_progress_m": self.min_path_progress_m,
            "max_goal_endpoint_error_m": self.max_goal_endpoint_error_m,
            "goal_revalidation_failures": self.goal_revalidation_failures,
            "goal_revalidation_failure_count": (
                self._goal_revalidation_failure_count),
            "max_path_stretch_ratio": self.max_path_stretch_ratio,
            "max_path_detour_m": self.max_path_detour_m,
            "max_navigation_distance_ratio": self.max_navigation_distance_ratio,
            "max_navigation_distance_increase_m": (
                self.max_navigation_distance_increase_m),
            "navigation_divergence_samples": self.navigation_divergence_samples,
            "navigation_start_distance_m": self._navigation_start_distance,
            "navigation_divergence_count": self._navigation_divergence_count,
            "frontier_standoff_step_m": self.frontier_standoff_step_m,
            "max_frontier_standoff_steps": self.max_frontier_standoff_steps,
            "open_space_heading_weight": self.open_space_heading_weight,
            "visual_coverage_threshold": self.visual_coverage_threshold,
            "global_coverage_threshold": self.global_coverage_threshold,
            "global_traversal_clearance_m": self.global_traversal_clearance_m,
            "global_search": dict(self._global_search_state),
            "utility_mode": self.utility_mode,
            "mixed_frontier_weight": self.mixed_frontier_weight,
            "mixed_visual_gain_weight": self.mixed_visual_gain_weight,
            "mixed_path_cost_penalty": self.mixed_path_cost_penalty,
            "parallel_probe_workers": self.parallel_probe_workers,
            "candidate_analysis_limit": self.candidate_analysis_limit,
            "yaw_optimization_candidate_limit": (
                self.yaw_optimization_candidate_limit),
            "raw_candidate_count": self._raw_candidate_count,
            "analyzed_candidate_count": self._analyzed_candidate_count,
            "failure_filtered_candidate_count": (
                self._failure_filtered_candidate_count),
            "yaw_optimized_candidate_count": (
                self._yaw_optimized_candidate_count),
            "candidate_analysis_ms": round(self._candidate_analysis_ms, 3),
            "yaw_optimization_ms": round(self._yaw_optimization_ms, 3),
            "mixed_heading_penalty": self.mixed_heading_penalty,
            "mixed_wall_bonus": self.mixed_wall_bonus,
            "mixed_expansion_bonus": self.mixed_expansion_bonus,
            "unknown_eta_band_s": self.unknown_eta_band_s,
            "yaw_step_deg": self.yaw_step_deg,
            "max_vel_x": self.max_vel_x,
            "max_vel_theta": self.max_vel_theta,
            "coverage_candidate_limit": self.coverage_candidate_limit,
            "staging_transition_pending": self._staging_transition_pending,
            "visibility": self._visibility_snapshot_snapshot(),
            "visited_frontiers": [dict(item) for item in self._visited],
            "blacklist": [dict(record) for record in self._blacklist.values()],
            "current_goal": (
                None if self._current_goal is None else dict(self._current_goal)),
            "distance_m": self._distance_m,
            "plan_probes": self._plan_probes,
            "plan_rejections": self._plan_rejections,
            "navigation_failures": self._navigation_failures,
            "last_selection_reason": self._last_selection_reason,
            "motion_trap": dict(self._motion_trap),
            "selected_candidate_metrics": dict(self._last_selected_metrics),
            "elapsed_s": max(0.0, self._monotonic() - self._started_at),
        }

    def _record_failure(self, candidate: dict, reason: str, stage: str) -> None:
        cell = self._candidate_cell(candidate)
        x, y = self._candidate_spatial_position(candidate)
        spatial_key = self._matching_spatial_failure_key(candidate)
        if spatial_key is None:
            spatial_key = self._spatial_bucket(x, y)
            record = {"x": x, "y": y, "count": 0}
        else:
            record = self._spatial_failures.pop(spatial_key)
        # v3 修复 (P1): degenerate_plan 是结构性退化 (goal 落在障碍/已观测区/
        # plan 端点不达 goal), 1 次即应排除候选. 加速 spatial count 累积达
        # max_failures_per_cell, 避免退化点被反复 probe (实测 (0.047,-0.062)
        # 被重试 8 次/20 probe 浪费 40%).
        increment = (max(1, self.max_failures_per_cell)
                     if reason == "degenerate_plan" else 1)
        record["count"] = int(record.get("count", 0)) + increment
        self._spatial_failures[spatial_key] = record
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
    def _candidate_spatial_position(candidate: dict) -> tuple[float, float]:
        """Return the physical frontier, not an inward standoff approach."""
        return (
            float(candidate.get("frontier_x", candidate["x"])),
            float(candidate.get("frontier_y", candidate["y"])),
        )

    @staticmethod
    def _spatial_bucket(x: float, y: float) -> tuple[int, int]:
        bucket = 0.5
        return (
            int(math.floor(y / bucket)),
            int(math.floor(x / bucket)),
        )

    def _matching_spatial_failure_key(self, candidate: dict):
        """Find a prior failure by distance, including adjacent hash buckets."""
        x, y = self._candidate_spatial_position(candidate)
        center_row, center_col = self._spatial_bucket(x, y)
        best = None
        for row in range(center_row - 1, center_row + 2):
            for col in range(center_col - 1, center_col + 2):
                key = (row, col)
                record = self._spatial_failures.get(key)
                if record is None:
                    continue
                distance = math.hypot(
                    x - float(record["x"]), y - float(record["y"]))
                if distance <= 0.5 and (
                        best is None or (distance, key) < best):
                    best = (distance, key)
        return None if best is None else best[1]

    def _spatial_failure_count(self, candidate: dict) -> int:
        key = self._matching_spatial_failure_key(candidate)
        if key is None:
            return 0
        return int(self._spatial_failures[key].get("count", 0))

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
        approaches = []
        try:
            robot_x, robot_y = float(robot_pose[0]), float(robot_pose[1])
            robot_yaw = float(robot_pose[2])
            frontier_x = float(candidate["x"])
            frontier_y = float(candidate["y"])
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            return approaches
        distance = math.hypot(frontier_x - robot_x, frontier_y - robot_y)
        if not math.isfinite(distance) or distance <= 1e-9:
            return [dict(candidate)]
        try:
            heading_change = abs(math.atan2(
                math.sin(float(candidate.get("heading_change", 0.0))),
                math.cos(float(candidate.get("heading_change", 0.0))),
            ))
        except (TypeError, ValueError, OverflowError):
            heading_change = 0.0
        use_room_approaches = (
            self.mode == "current_room"
            and self.max_frontier_standoff_steps > 0)
        try:
            adaptive_step = float(candidate.get("adaptive_step_m", 0.0))
            scene_complexity = float(candidate.get("scene_complexity", 1.0))
            path_clearance = float(candidate.get(
                "path_clearance_m",
                candidate.get("forward_clearance_m", 0.0)))
        except (TypeError, ValueError, OverflowError):
            adaptive_step = 0.0
            scene_complexity = 1.0
            path_clearance = 0.0
        path_blocked = bool(candidate.get("path_blocked", False))
        current_path_blocked = bool(
            candidate.get("current_path_blocked", False))
        turn_motion_blocked = bool(
            candidate.get("turn_motion_blocked", False))
        try:
            current_adaptive_step = float(candidate.get(
                "current_adaptive_step_m", adaptive_step))
        except (TypeError, ValueError, OverflowError):
            current_adaptive_step = 0.0
        if (
                turn_motion_blocked
                and heading_change > self.local_turn_threshold_rad + 1e-9):
            if (
                    current_path_blocked
                    or not math.isfinite(current_adaptive_step)
                    or current_adaptive_step + 1e-9 < self.min_goal_distance_m):
                return []
            staging_distance = min(
                self.initial_turn_staging_distance_m,
                current_adaptive_step,
            )
            staging = dict(candidate)
            staging.update({
                "x": robot_x + staging_distance * math.cos(robot_yaw),
                "y": robot_y + staging_distance * math.sin(robot_yaw),
                "yaw": robot_yaw,
                "frontier_x": frontier_x,
                "frontier_y": frontier_y,
                "heading_change": 0.0,
                "approach_staging_m": staging_distance,
                "staging_for_heading_change_rad": heading_change,
                "staging_reason": "turn_clearance_blocked",
            })
            return [staging] if self._approach_within_bounds(staging) else []
        if path_blocked:
            return []
        open_lidar_corridor = (
            math.isfinite(adaptive_step)
            and math.isfinite(scene_complexity)
            and math.isfinite(path_clearance)
            and adaptive_step >= max(
                self.frontier_lookahead_goal_m,
                self.initial_turn_staging_distance_m * 2.0)
            and path_clearance + 1e-9 >= adaptive_step
            and scene_complexity <= 0.35
        )
        if (
                use_room_approaches
                and
                candidate.get("prefer_standoff")
                and not self._staging_transition_pending
                and not open_lidar_corridor
                and heading_change
                > self.initial_turn_staging_threshold_rad + 1e-9):
            # The motion layer deliberately rejects a pure turn when any
            # MID360 return is inside its swept-clearance radius.  Before a
            # large reorientation, probe short heading-aligned escape points;
            # the global planner still has final authority over each probe.
            for heading_offset in (
                    0.0, math.radians(30.0), -math.radians(30.0)):
                staging_yaw = robot_yaw + heading_offset
                staging = dict(candidate)
                staging.update({
                    "x": robot_x + self.initial_turn_staging_distance_m
                    * math.cos(staging_yaw),
                    "y": robot_y + self.initial_turn_staging_distance_m
                    * math.sin(staging_yaw),
                    "yaw": staging_yaw,
                    "frontier_x": frontier_x,
                    "frontier_y": frontier_y,
                    "heading_change": abs(heading_offset),
                    "approach_staging_m": self.initial_turn_staging_distance_m,
                    "approach_staging_heading_offset_rad": heading_offset,
                    "staging_for_heading_change_rad": heading_change,
                })
                if self._approach_within_bounds(staging):
                    approaches.append(staging)
        adaptive_goal_added = False
        lidar_lookahead_added = False
        if (
                math.isfinite(adaptive_step)
                and adaptive_step >= self.min_goal_distance_m
                and distance > adaptive_step + self.min_goal_distance_m):
            ratio = adaptive_step / distance
            adaptive = dict(candidate)
            adaptive.update({
                "x": robot_x + (frontier_x - robot_x) * ratio,
                "y": robot_y + (frontier_y - robot_y) * ratio,
                "frontier_x": frontier_x,
                "frontier_y": frontier_y,
                "approach_adaptive_m": adaptive_step,
            })
            if self._approach_within_bounds(adaptive):
                approaches.append(adaptive)
                adaptive_goal_added = True
        elif (
                math.isfinite(adaptive_step)
                and not candidate.get("coverage_candidate")
                and adaptive_step
                > distance + self.min_goal_distance_m):
            # A frontier can lag far behind the current LiDAR scan boundary.
            # Probe the end of the candidate-direction safe corridor first;
            # Nav2 still validates map reachability. Complex scenes may retain
            # the physical frontier fallback below; open scenes must not turn
            # a rejected long probe into a tiny stop-and-go target.
            ratio = adaptive_step / distance
            lookahead = dict(candidate)
            lookahead.update({
                "x": robot_x + (frontier_x - robot_x) * ratio,
                "y": robot_y + (frontier_y - robot_y) * ratio,
                "frontier_x": frontier_x,
                "frontier_y": frontier_y,
                "approach_lidar_lookahead_m": adaptive_step - distance,
            })
            if self._approach_within_bounds(lookahead):
                approaches.append(lookahead)
                lidar_lookahead_added = True
        if open_lidar_corridor and (
                adaptive_goal_added or lidar_lookahead_added):
            # In an open scene the long approach is the actual exploration
            # decision. Falling back to frontier standoffs after Nav2 rejects
            # it caused 0.3-1.2 m stop-and-go motion. Let the manager try a
            # different long corridor on the next candidate/cycle instead.
            return approaches
        if (
                use_room_approaches
                and not lidar_lookahead_added
                and distance < self.frontier_lookahead_trigger_m):
            ratio = self.frontier_lookahead_goal_m / distance
            lookahead = dict(candidate)
            lookahead.update({
                "x": robot_x + (frontier_x - robot_x) * ratio,
                "y": robot_y + (frontier_y - robot_y) * ratio,
                "frontier_x": frontier_x,
                "frontier_y": frontier_y,
                "approach_lookahead_m": (
                    self.frontier_lookahead_goal_m - distance),
            })
            if self._approach_within_bounds(lookahead):
                approaches.append(lookahead)
        standoff_approaches = []
        standoff_steps = (
            self.max_frontier_standoff_steps if use_room_approaches else 0)
        for step in range(1, standoff_steps + 1):
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
            standoff_approaches.append(approach)
        if candidate.get("prefer_standoff"):
            approaches.extend(reversed(standoff_approaches))
            approaches.append(dict(candidate))
        else:
            if not approaches or adaptive_step <= 0.0:
                approaches.append(dict(candidate))
            else:
                # Probe the scene-sized step first. The physical frontier is
                # retained as a later fallback only when that intermediate
                # point cannot be planned.
                approaches.append(dict(candidate))
            approaches.extend(standoff_approaches)
        return approaches

    def _approach_within_bounds(self, approach: dict) -> bool:
        try:
            x, y = float(approach["x"]), float(approach["y"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if self._active_radius_m is not None:
            if math.hypot(
                    x - float(self.mission_origin[0]),
                    y - float(self.mission_origin[1])) > self._active_radius_m:
                return False
        if self.room_polygon and not point_in_polygon(x, y, self.room_polygon):
            return False
        return True

    def _approach_revisits_viewpoint(self, approach: dict) -> bool:
        """Return true when a real observation goal repeats a prior viewpoint."""

        if self.revisit_radius_m <= 0.0 or "approach_staging_m" in approach:
            return False
        try:
            x, y = float(approach["x"]), float(approach["y"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        return any(
            math.hypot(
                x - float(visited["x"]),
                y - float(visited["y"]),
            ) < self.revisit_radius_m - 1e-9
            for visited in self._visited
        )

    def _path_makes_progress(self, path_result: dict) -> bool:
        try:
            path_length = float(path_result.get("path_length"))
            poses = int(path_result.get("poses", 0))
        except (TypeError, ValueError, OverflowError):
            return False
        return (
            math.isfinite(path_length)
            and poses >= 2
            and path_length + 1e-9 >= self.min_path_progress_m
        )

    def _path_endpoint_reaches_goal(self, path_result: dict) -> bool:
        raw = path_result.get("goal_error_m")
        if raw is None:
            # Compatibility ports may omit this. The production ROS adapter
            # always exposes the final path pose and endpoint error.
            return True
        try:
            error = float(raw)
        except (TypeError, ValueError, OverflowError):
            return False
        return (
            math.isfinite(error)
            and error <= self.max_goal_endpoint_error_m + 1e-9
        )

    def _path_detour_is_safe(
            self, path_result: dict, approach: dict, robot_pose) -> bool:
        try:
            path_length = float(path_result.get("path_length"))
        except (TypeError, ValueError, OverflowError):
            return False
        direct_distance = self._distance_from_pose(approach, robot_pose)
        if (not math.isfinite(path_length)
                or not math.isfinite(direct_distance)
                or direct_distance <= 1e-9):
            return False
        allowed_length = max(
            direct_distance * self.max_path_stretch_ratio,
            direct_distance + self.max_path_detour_m,
        )
        return path_length <= allowed_length + 1e-9

    def _reset_navigation_progress(self) -> None:
        self._navigation_start_distance = None
        self._navigation_start_pose = None
        self._navigation_divergence_count = 0
        self._goal_revalidation_failure_count = 0

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

    def _path_respects_entrance_gate(self, path_result: dict) -> bool:
        if self.entrance_gate is None:
            return True
        path = path_result.get("path")
        if not path:
            return False
        return not path_crosses_entrance_gate(path, self.entrance_gate)

    def _select_candidates(self, candidate_selector, map_msg, robot_pose):
        started = time.perf_counter()
        selector_kwargs = dict(
            revisit_radius=self.revisit_radius_m,
            origin_pose=self.mission_origin,
            max_radius=self._active_radius_m,
            room_polygon=self.room_polygon,
            reject_map_edge=self.reject_map_edge,
            failure_counts={},
            distance_weight=self.distance_weight,
            heading_weight=self.heading_weight,
            failure_penalty=self.failure_penalty,
        )
        if (
            self.entrance_gate is not None
            and callable_accepts_keyword(candidate_selector, "entrance_gate")
        ):
            selector_kwargs["entrance_gate"] = dict(self.entrance_gate)
        if callable_accepts_keyword(candidate_selector, "frontier_spacing_m"):
            selector_kwargs["frontier_spacing_m"] = self.frontier_spacing_m
        candidates = candidate_selector(
            map_msg,
            robot_pose,
            self._visited,
            **selector_kwargs,
        )
        candidates = [dict(item) for item in (candidates or [])]
        self._raw_candidate_count = len(candidates)
        self._failure_filtered_candidate_count = 0
        preanalysis_radius_truncated = 0
        prefiltered = []
        for item in candidates:
            if self._active_radius_m is not None and math.hypot(
                    float(item["x"]) - self.mission_origin[0],
                    float(item["y"]) - self.mission_origin[1],
            ) > self._active_radius_m:
                preanalysis_radius_truncated += 1
                continue
            if self.room_polygon and not point_in_polygon(
                    float(item["x"]), float(item["y"]), self.room_polygon):
                continue
            count = self._spatial_failure_count(item)
            if count >= self.max_failures_per_cell:
                self._failure_filtered_candidate_count += 1
                continue
            item["failure_count"] = int(count)
            item["score"] = score_frontier(
                item,
                path_length=item.get("path_length"),
                heading_change=item.get("heading_change", 0.0),
                failure_count=count,
                distance_weight=self.distance_weight,
                heading_weight=self._effective_heading_weight(item),
                failure_penalty=self.failure_penalty,
            )
            prefiltered.append(item)
        prefiltered.sort(key=lambda item: (
            -float(item.get("score", 0.0)),
            float(item.get("distance", 0.0)),
            float(item["x"]), float(item["y"]),
        ))
        candidates = prefiltered[:self.candidate_analysis_limit]
        self._analyzed_candidate_count = len(candidates)
        if self.visibility_tracker is not None:
            candidates = self.visibility_tracker.rank_candidates(
                map_msg, robot_pose, candidates)
            # Visual gain is ranking evidence, never frontier eligibility.
            # A zero-gain frontier may be the only reachable path into still
            # unknown space after a positive-gain frontier fails preflight.
        for item in candidates:
            count = self._spatial_failure_count(item)
            item["failure_count"] = int(count)
            item["score"] = score_frontier(
                item,
                path_length=item.get("path_length"),
                heading_change=item.get("heading_change", 0.0),
                failure_count=count,
                distance_weight=self.distance_weight,
                heading_weight=self._effective_heading_weight(item),
                failure_penalty=self.failure_penalty,
            )
        candidates.sort(key=lambda item: (
            -float(item.get("score", 0.0)),
            float(item.get("distance", 0.0)),
            float(item["x"]), float(item["y"]),
        ))
        # Enforce the active mission bound here too, so custom/test candidate
        # sources cannot bypass dynamic current-room containment. Track how
        # many candidates were truncated so choose_next can grow the radius
        # proactively — otherwise a room larger than initial_radius_m leaves
        #远 frontier permanently out of reach while near frontiers keep the
        # candidate list non-empty (2026-07-21 production: (21,-10) at 23m
        # never explored because initial_radius=16m never expanded).
        self._last_radius_truncated = 0
        if self._active_radius_m is not None:
            kept = []
            for item in candidates:
                if math.hypot(
                    float(item["x"]) - self.mission_origin[0],
                    float(item["y"]) - self.mission_origin[1],
                ) <= self._active_radius_m:
                    kept.append(item)
                else:
                    self._last_radius_truncated += 1
            candidates = kept
        if self.room_polygon:
            candidates = [
                item for item in candidates
                if point_in_polygon(
                    float(item["x"]), float(item["y"]), self.room_polygon)
            ]
        self._last_radius_truncated += preanalysis_radius_truncated
        self._candidate_analysis_ms = (
            time.perf_counter() - started) * 1000.0
        return [dict(item) for item in candidates]

    def _select_visual_coverage_candidates(self, map_msg, robot_pose):
        if self.visibility_tracker is None:
            return []
        snapshot = dict(
            self.visibility_tracker.snapshot(map_msg) or {})
        self._visibility_snapshot = snapshot
        try:
            coverage_ratio = float(snapshot.get(
                "visual_coverage_ratio", 0.0))
        except (TypeError, ValueError, OverflowError):
            coverage_ratio = 0.0
        if coverage_ratio >= self.visual_coverage_threshold:
            return []
        selector = getattr(self.visibility_tracker, "coverage_candidates", None)
        if not callable(selector):
            return []
        candidates = selector(
            map_msg,
            robot_pose,
            self._visited,
            limit=self.coverage_candidate_limit,
        )
        prepared = []
        for source in candidates:
            item = dict(source)
            if self._active_radius_m is not None and math.hypot(
                    float(item["x"]) - self.mission_origin[0],
                    float(item["y"]) - self.mission_origin[1],
            ) > self._active_radius_m:
                continue
            if self.room_polygon and not point_in_polygon(
                    float(item["x"]), float(item["y"]), self.room_polygon):
                continue
            count = self._spatial_failure_count(item)
            item["failure_count"] = count
            item["score"] = score_frontier(
                item,
                path_length=item.get("path_length"),
                heading_change=item.get("heading_change", 0.0),
                failure_count=count,
                distance_weight=self.distance_weight,
                heading_weight=self._effective_heading_weight(item),
                failure_penalty=self.failure_penalty,
            )
            prepared.append(item)
        prepared.sort(key=lambda item: (
            -float(item.get("score", 0.0)),
            float(item.get("distance", 0.0)),
            float(item["x"]), float(item["y"]),
        ))
        return prepared

    def _select_lidar_candidates(self, robot_pose):
        if self.visibility_tracker is None:
            return []
        selector = getattr(self.visibility_tracker, "lidar_candidates", None)
        if not callable(selector):
            return []
        candidates = selector(
            robot_pose,
            self._visited,
            limit=self.coverage_candidate_limit,
        )
        prepared = []
        for source in candidates or []:
            item = dict(source)
            if self._active_radius_m is not None and math.hypot(
                    float(item["x"]) - self.mission_origin[0],
                    float(item["y"]) - self.mission_origin[1],
            ) > self._active_radius_m:
                continue
            if self.room_polygon and not point_in_polygon(
                    float(item["x"]), float(item["y"]), self.room_polygon):
                continue
            count = self._spatial_failure_count(item)
            item["failure_count"] = count
            # Generic frontier scoring subtracts path length because nearby
            # map frontiers are cheaper.  A LiDAR fallback exists specifically
            # because the map has collapsed or lagged; retaining that bias
            # would select short side corridors even when a long, clear one is
            # available.  Flipping the net distance term rewards safe progress
            # while Nav2 preflight and mission bounds remain authoritative.
            lidar_progress_bonus = (
                3.0 * self.distance_weight
                * max(0.0, float(item.get("distance", 0.0)))
            )
            item["lidar_progress_bonus"] = lidar_progress_bonus
            item["score"] = score_frontier(
                item,
                path_length=item.get("path_length"),
                heading_change=item.get("heading_change", 0.0),
                failure_count=count,
                distance_weight=self.distance_weight,
                heading_weight=self._effective_heading_weight(item),
                failure_penalty=self.failure_penalty,
            ) + lidar_progress_bonus
            prepared.append(item)
        prepared.sort(key=lambda item: (
            -float(item.get("score", 0.0)),
            float(item.get("heading_change", 0.0)),
            -float(item.get("distance", 0.0)),
            float(item["x"]), float(item["y"]),
        ))
        return prepared

    def _effective_heading_weight(self, candidate: Optional[dict] = None) -> float:
        source = candidate if candidate is not None else (
            self._visibility_snapshot_snapshot())
        try:
            complexity = float(source.get("scene_complexity", 1.0))
        except (AttributeError, TypeError, ValueError, OverflowError):
            complexity = 1.0
        complexity = min(1.0, max(0.0, complexity))
        return (
            self.heading_weight
            + self.open_space_heading_weight * (1.0 - complexity)
        )

    @staticmethod
    def _exploration_source(candidate: dict) -> str:
        if candidate.get("lidar_candidate"):
            return "lidar"
        if candidate.get("coverage_candidate"):
            return "coverage"
        return "frontier"

    def _unknown_sample_stats(self, candidate: dict, map_msg) -> tuple:
        """Return unknown cells and the exact clipped disk population."""

        try:
            provided_unknown = max(
                0.0, float(candidate.get("adjacent_unknown_count", 0.0)))
        except (TypeError, ValueError, OverflowError):
            provided_unknown = 0.0
        for key in ("adjacent_support_count", "unknown_support_count"):
            try:
                value = int(candidate.get(key, 0))
            except (TypeError, ValueError, OverflowError):
                value = 0
            if value > 0:
                return provided_unknown, value
        try:
            width = int(map_msg.info.width)
            height = int(map_msg.info.height)
            resolution = float(map_msg.info.resolution)
            center_row, center_col = (
                int(value) for value in candidate["center_cell"])
            data = map_msg.data
        except (AttributeError, KeyError, TypeError, ValueError,
                OverflowError):
            width = height = 0
            resolution = 0.0
            center_row = center_col = 0
            data = None
        geometry_available = (
            width > 0 and height > 0
            and math.isfinite(resolution) and resolution > 0.0
            and 0 <= center_row < height and 0 <= center_col < width
            and data is not None and len(data) >= width * height
        )
        if not geometry_available:
            resolution = resolution if resolution > 0.0 else 1.0
        radius_cells = self.frontier_spacing_m / resolution
        radius_limit = int(math.ceil(radius_cells))
        radius_sq = radius_cells * radius_cells
        if geometry_available:
            cells = [
                int(data[row * width + col])
                for row in range(
                    max(0, center_row - radius_limit),
                    min(height, center_row + radius_limit + 1))
                for col in range(
                    max(0, center_col - radius_limit),
                    min(width, center_col + radius_limit + 1))
                if ((row - center_row) ** 2 + (col - center_col) ** 2
                    <= radius_sq + 1e-9)
            ]
            support_count = max(1, len(cells))
            unknown_count = (
                provided_unknown
                if "adjacent_unknown_count" in candidate
                else (
                    float(sum(1 for value in cells if value < 0))
                    if "touches_map_edge" in candidate else 0.0
                )
            )
            return unknown_count, support_count
        support_count = max(1, sum(
            1
            for row in range(-radius_limit, radius_limit + 1)
            for col in range(-radius_limit, radius_limit + 1)
            if row * row + col * col <= radius_sq + 1e-9
        ))
        return provided_unknown, support_count

    def _annotate_exploration_candidate(
            self, source: dict, robot_pose, map_msg) -> dict:
        candidate = dict(source)
        exploration_source = self._exploration_source(candidate)
        unknown_count, support_count = self._unknown_sample_stats(
            candidate, map_msg)
        if exploration_source != "frontier":
            unknown_count = 0.0
        unknown_gain = min(1.0, unknown_count / float(support_count))
        try:
            wall_bonus = float(candidate.get(
                "wall_proximity_bonus",
                1.0 if int(candidate.get("adjacent_wall_count", 0)) >= 2
                else 0.0,
            ))
        except (TypeError, ValueError, OverflowError):
            wall_bonus = 0.0
        wall_bonus = max(0.0, wall_bonus)
        path_cost = self._path_cost_for_utility(candidate, robot_pose)
        try:
            heading_change = abs(float(candidate.get("heading_change", 0.0)))
        except (TypeError, ValueError, OverflowError):
            heading_change = 0.0
        eta_s = (
            path_cost / max(self.max_vel_x, 1e-6)
            + heading_change / max(self.max_vel_theta, 1e-6)
        )
        candidate.update({
            "exploration_source": exploration_source,
            "adjacent_support_count": support_count,
            "adjacent_unknown_count": unknown_count,
            "unknown_gain": unknown_gain,
            "wall_proximity_bonus": wall_bonus,
            "eta_s": eta_s,
            "eta_band": int(math.floor(eta_s / self.unknown_eta_band_s)),
        })
        return candidate

    @staticmethod
    def _exploration_priority_tier(candidate: dict) -> tuple:
        source_rank = {"frontier": 0, "coverage": 1, "lidar": 2}
        source = str(candidate.get("exploration_source", "frontier"))
        unknown_gain = max(0.0, float(candidate.get("unknown_gain", 0.0)))
        return (
            source_rank.get(source, 3),
            0 if unknown_gain > 1e-9 else 1,
            int(candidate.get("eta_band", 0)),
            -unknown_gain,
        )

    def _rank_exploration_candidates(
            self, candidates: list, robot_pose, map_msg) -> list:
        """Apply source, unknown, bounded-ETA, then utility ordering."""

        annotated = [
            self._annotate_exploration_candidate(item, robot_pose, map_msg)
            for item in candidates
        ]

        def rank_key(candidate):
            if self.utility_mode == "mixed":
                utility_key = self._mixed_utility_sort_key(
                    candidate, robot_pose)[0]
                return (
                    self._exploration_priority_tier(candidate),
                    -float(candidate.get("wall_proximity_bonus", 0.0)),
                    utility_key,
                    float(candidate.get("eta_s", 0.0)),
                    float(candidate.get("x", 0.0)),
                    float(candidate.get("y", 0.0)),
                )
            # Preserve historical nearest-mode motion inside the explicit
            # source/unknown tier. Wall affinity is a mixed-mode tie-breaker.
            if candidate.get("exploration_source") != "frontier":
                return (
                    self._exploration_priority_tier(candidate),
                    -float(candidate.get("score", 0.0)),
                    float(candidate.get("eta_s", 0.0)),
                    float(candidate.get("x", 0.0)),
                    float(candidate.get("y", 0.0)),
                )
            try:
                visual_gain = float(candidate.get("visual_gain", 0.0))
            except (TypeError, ValueError, OverflowError):
                visual_gain = 0.0
            return (
                self._exploration_priority_tier(candidate),
                0 if visual_gain > 1e-9 else 1,
                float(candidate.get("eta_s", 0.0)),
                -float(candidate.get("score", 0.0)),
                float(candidate.get("x", 0.0)),
                float(candidate.get("y", 0.0)),
            )

        annotated.sort(key=rank_key)
        return annotated

    def _mixed_utility_sort_key(self, candidate: dict, robot_pose) -> tuple:
        """v3: α·size + β·visual_gain + δ·wall − k_time·(t_travel+t_turn).

        path_cost 和 heading 都换算成秒 (时间归一化).
        ``path_length`` is preferred over raw euclidean ``distance`` when
        available — the rank/coverage pipelines leave ``path_length`` unset
        until Nav2 preflight fills it, so we fall back to ``distance`` and
        let ``choose_next`` re-rank reachable paths with their real
        ``path_length`` after probing.
        """
        try:
            information_gain = float(candidate.get(
                "information_gain", candidate.get("size", 0.0)))
        except (TypeError, ValueError, OverflowError):
            information_gain = 0.0
        try:
            visual_gain = float(candidate.get("visual_gain", 0.0))
        except (TypeError, ValueError, OverflowError):
            visual_gain = 0.0
        try:
            wall_bonus = float(candidate.get("wall_proximity_bonus", 0.0))
        except (TypeError, ValueError, OverflowError):
            wall_bonus = 0.0
        try:
            expansion_potential = float(candidate.get(
                "unknown_gain", candidate.get("adjacent_unknown_count", 0.0)))
        except (TypeError, ValueError, OverflowError):
            expansion_potential = 0.0
        path_cost = self._path_cost_for_utility(candidate, robot_pose)
        try:
            heading_change = abs(float(candidate.get("heading_change", 0.0)))
        except (TypeError, ValueError, OverflowError):
            heading_change = 0.0
        t_travel = path_cost / max(self.max_vel_x, 1e-6)
        t_turn = heading_change / max(self.max_vel_theta, 1e-6)
        utility = (
            self.mixed_frontier_weight * information_gain
            + self.mixed_visual_gain_weight * visual_gain
            + self.mixed_wall_bonus * wall_bonus
            + self.mixed_expansion_bonus * expansion_potential
            - self.mixed_heading_penalty * (t_travel + t_turn)
        )
        # Return as tuple so callers can compose tiebreakers (distance, x, y).
        return (-utility,)

    def _optimize_yaw_for_candidates(
            self, candidates: list, robot_pose, map_msg) -> list:
        """v3 yaw 优化: 每个 candidate 试全360° K 个 yaw, 选 mixed-utility 最优.

        仅 mixed 模式 + visibility_tracker 存在时调用. 否则只填 wall_proximity_bonus.
        候选集: robot_yaw + ±k·yaw_step (覆盖全360°含 90/180) + 朝frontier方向.
        不硬排除大角度 — 靠 k_time·t_turn 加权偏好小角度, 前方受阻时 180° 自然胜出.
        """
        started = time.perf_counter()
        optimized_count = 0
        try:
            robot_yaw = float(robot_pose[2])
        except (TypeError, IndexError, ValueError):
            robot_yaw = 0.0
        try:
            rx = float(robot_pose[0]); ry = float(robot_pose[1])
        except (TypeError, IndexError, ValueError):
            rx = ry = 0.0
        for index, cand in enumerate(candidates):
            awc = int(cand.get("adjacent_wall_count", 0))
            wall_bonus = 1.0 if awc >= 2 else 0.0
            if (self.utility_mode != "mixed"
                    or self.visibility_tracker is None
                    or index >= self.yaw_optimization_candidate_limit):
                cand["wall_proximity_bonus"] = wall_bonus
                continue
            try:
                cx = float(cand["x"]); cy = float(cand["y"])
            except (KeyError, TypeError, ValueError):
                cand["wall_proximity_bonus"] = wall_bonus
                continue
            frontier_yaw = math.atan2(cy - ry, cx - rx)
            step = math.radians(max(5.0, float(self.yaw_step_deg)))
            yaw_offsets = [0.0]
            k = 1
            while k * step < math.pi - 1e-9:
                yaw_offsets.append(k * step)
                yaw_offsets.append(-k * step)
                k += 1
            yaw_offsets.append(math.pi)
            yaw_offsets.append(_abs_angle_delta(frontier_yaw, robot_yaw)
                               * (1.0 if frontier_yaw >= robot_yaw else -1.0))
            path_cost = self._distance_from_pose(cand, robot_pose)
            t_travel = path_cost / max(self.max_vel_x, 1e-6)
            try:
                base_ig = float(cand.get(
                    "information_gain", cand.get("size", 0.0)))
            except (TypeError, ValueError, OverflowError):
                base_ig = 0.0
            best = None  # (key_tuple, yaw, vg, hc)
            for offset in set(yaw_offsets):
                yaw = robot_yaw + offset
                vg = self.visibility_tracker.visual_gain_at(map_msg, cx, cy, yaw)
                hc = _abs_angle_delta(yaw, robot_yaw)
                t_turn = hc / max(self.max_vel_theta, 1e-6)
                utility = (
                    self.mixed_frontier_weight * base_ig
                    + self.mixed_visual_gain_weight * float(vg)
                    + self.mixed_wall_bonus * wall_bonus
                    - self.mixed_heading_penalty * (t_travel + t_turn)
                )
                key = (utility, -hc, -vg)
                if best is None or key > best[0]:
                    best = (key, yaw, vg, hc)
            if best is not None:
                optimized_count += 1
                _, yaw, vg, hc = best
                cand["yaw"] = yaw
                cand["visual_gain"] = vg
                cand["heading_change"] = hc
                try:
                    logger.debug(
                        "v3 yaw_opt: x=%.2f y=%.2f size=%.0f awc=%d "
                        "yaw=%.0fdeg vg=%d hc=%.2frad util=%.3f",
                        cx, cy, base_ig, awc,
                        math.degrees(yaw), int(vg), hc, best[0][0])
                except Exception:
                    pass
            cand["wall_proximity_bonus"] = wall_bonus
        self._yaw_optimized_candidate_count = optimized_count
        self._yaw_optimization_ms = (
            time.perf_counter() - started) * 1000.0
        logger.info(
            "exploration candidate analysis raw=%d analyzed=%d yaw=%d "
            "analysis_ms=%.1f yaw_ms=%.1f",
            self._raw_candidate_count,
            self._analyzed_candidate_count,
            self._yaw_optimized_candidate_count,
            self._candidate_analysis_ms,
            self._yaw_optimization_ms,
        )
        return candidates

    def _path_cost_for_utility(self, candidate: dict, robot_pose) -> float:
        """Return Nav2 path_length when known, else euclidean distance.

        At preflight time only ``distance`` is set; ``path_length`` is filled
        after ``compute_path_to_pose``. Using whichever is available keeps the
        utility meaningful both before and after probing.
        """
        path_length = candidate.get("path_length")
        try:
            if path_length is not None:
                value = float(path_length)
                if math.isfinite(value) and value >= 0.0:
                    return value
        except (TypeError, ValueError, OverflowError):
            pass
        return self._distance_from_pose(candidate, robot_pose)

    def _probe_validation_reason(
            self, result: dict, approach: dict, robot_pose) -> Optional[str]:
        if not result.get("ok"):
            return str(result.get("reason") or "unreachable")
        if not self._path_respects_room(result):
            return "path_leaves_room_polygon"
        if not self._path_respects_entrance_gate(result):
            return "path_crosses_entrance_gate"
        if not self._path_makes_progress(result):
            return "degenerate_plan"
        if not self._path_endpoint_reaches_goal(result):
            return "plan_endpoint_mismatch"
        if not self._path_detour_is_safe(result, approach, robot_pose):
            return "excessive_plan_detour"
        return None

    def _chosen_from_probe(self, approach: dict, result: dict) -> dict:
        chosen = dict(approach)
        chosen["path_length"] = float(
            result.get("path_length", approach.get("distance", 0.0)))
        chosen["path_poses"] = int(result.get("poses", 0) or 0)
        if result.get("goal_error_m") is not None:
            chosen["goal_error_m"] = float(result["goal_error_m"])
        chosen["score"] = score_frontier(
            chosen,
            path_length=chosen["path_length"],
            heading_change=chosen.get("heading_change", 0.0),
            failure_count=chosen.get("failure_count", 0),
            distance_weight=self.distance_weight,
            heading_weight=self._effective_heading_weight(chosen),
            failure_penalty=self.failure_penalty,
        ) + float(chosen.get("lidar_progress_bonus", 0.0))
        return chosen

    @staticmethod
    def _physical_probe_key(
            approach: dict, frame_id: str = "map") -> tuple:
        """Quantize one physical pose so equivalent probes can share work."""

        position_scale = 1000.0
        orientation_scale = 1000.0
        yaw = float(approach["yaw"])
        return (
            str(frame_id),
            int(round(float(approach["x"]) * position_scale)),
            int(round(float(approach["y"]) * position_scale)),
            int(round(math.cos(yaw) * orientation_scale)),
            int(round(math.sin(yaw) * orientation_scale)),
        )

    def _safe_compute_path(self, approach: dict) -> dict:
        """Convert planner exceptions into ordinary rejected-probe evidence."""

        try:
            return self.navigation_port.compute_path_to_pose(
                approach["x"], approach["y"], approach["yaw"],
                frame_id="map", timeout=self.planning_timeout_s)
        except Exception as exc:
            logger.warning(
                "exploration planner probe failed x=%s y=%s: %s",
                approach.get("x"), approach.get("y"), exc,
            )
            return {
                "ok": False,
                "reason": "planner_error",
                "error": str(exc),
            }

    def _parallel_probe_first_approaches(
            self, eligible: list, robot_pose, budget: int):
        """Return ordered first-approach evidence without selecting a goal."""

        import concurrent.futures

        evidence = []
        batch_limit = max(0, min(
            int(budget), int(self.parallel_probe_workers)))
        for index, candidate in enumerate(eligible):
            approaches = [
                approach
                for approach in self._candidate_approaches(
                    candidate, robot_pose)
                if not self._approach_revisits_viewpoint(approach)
            ]
            entry = {
                "candidate_index": index,
                "candidate": candidate,
                "approaches": approaches,
                "first_result": None,
                "validation_reason": None,
                "first_probed": False,
                "first_probe_key": None,
            }
            if approaches:
                entry["first_probe_key"] = self._physical_probe_key(
                    approaches[0])
            evidence.append(entry)

        # Speculate only when the remaining budget can still exhaust every
        # approach of all earlier candidates.  Otherwise a lower candidate's
        # first probe could consume the exact call the serial path needs for a
        # higher candidate's second/third standoff.
        submitted_by_key = {}
        reserved_ordered_continuations = 0
        accepting_new_keys = True
        for entry in evidence:
            approaches = entry["approaches"]
            if not approaches:
                continue
            key = entry["first_probe_key"]
            if key in submitted_by_key:
                reserved_ordered_continuations += max(
                    0, len(approaches) - 1)
                continue
            projected_calls = (
                len(submitted_by_key) + 1
                + reserved_ordered_continuations)
            if (not accepting_new_keys
                    or len(submitted_by_key) >= batch_limit
                    or projected_calls > int(budget)):
                accepting_new_keys = False
                continue
            submitted_by_key[key] = entry
            reserved_ordered_continuations += max(
                0, len(approaches) - 1)

        for entry in evidence:
            entry["first_probed"] = (
                entry["first_probe_key"] in submitted_by_key)

        submitted = list(submitted_by_key.items())

        if not submitted:
            return evidence, 0

        workers = max(1, min(
            int(self.parallel_probe_workers), len(submitted)))

        def probe(key, entry):
            approach = entry["approaches"][0]
            return key, self._safe_compute_path(approach)

        results = {}
        with concurrent.futures.ThreadPoolExecutor(
                max_workers=workers) as pool:
            futures = {
                pool.submit(probe, key, entry): key
                for key, entry in submitted
            }
            for future in concurrent.futures.as_completed(futures):
                key = futures[future]
                try:
                    _returned_key, result = future.result()
                except Exception as exc:
                    logger.warning(
                        "exploration planner worker failed key=%s: %s",
                        key, exc,
                    )
                    result = {
                        "ok": False,
                        "reason": "planner_error",
                        "error": str(exc),
                    }
                results[key] = result

        for entry in evidence:
            if not entry["first_probed"]:
                continue
            result = results[entry["first_probe_key"]]
            entry["first_result"] = result
            entry["validation_reason"] = self._probe_validation_reason(
                result, entry["approaches"][0], robot_pose)
        return evidence, len(submitted)

    def _eligible_candidates(self, candidates, robot_pose, map_msg):
        ranked = self._rank_exploration_candidates(
            candidates, robot_pose, map_msg)
        eligible = [
            candidate for candidate in ranked
            if self._spatial_failure_count(candidate)
            < self.max_failures_per_cell
            and self._distance_from_pose(candidate, robot_pose)
            >= self.min_goal_distance_m
        ]
        if not eligible:
            eligible = [
                candidate for candidate in ranked
                if self._spatial_failure_count(candidate)
                < self.max_failures_per_cell
                and any(
                    self._distance_from_pose(approach, robot_pose)
                    >= self.min_goal_distance_m
                    for approach in self._candidate_approaches(
                        candidate, robot_pose)
                )
            ]
        if not eligible:
            return []
        locally_executable = []
        for candidate in eligible:
            if self._candidate_approaches(candidate, robot_pose):
                locally_executable.append(candidate)
                continue
            if (
                    bool(candidate.get("current_path_blocked"))
                    and bool(candidate.get("turn_motion_blocked"))):
                self._motion_trap = self._motion_trap_evidence(
                    candidate, candidate, reason="scan_start_infeasible")
        eligible = locally_executable
        if not eligible:
            return []
        eligible = self._prioritize_active_tile(eligible, robot_pose)
        eligible = self._rank_exploration_candidates(
            eligible, robot_pose, map_msg)
        if (self._consecutive_nav_failures >= self._escape_abort_threshold
                and self._escape_min_distance_m > 0.0
                and len(eligible) > 1):
            best_tier = self._exploration_priority_tier(eligible[0])
            tier_candidates = [
                item for item in eligible
                if self._exploration_priority_tier(item) == best_tier
            ]
            tier_candidates.sort(
                key=lambda candidate: self._distance_from_pose(
                    candidate, robot_pose),
                reverse=True,
            )
            eligible = tier_candidates + [
                item for item in eligible
                if self._exploration_priority_tier(item) != best_tier
            ]
            self._last_selection_reason = "escape_trap_pending"
        return eligible

    @staticmethod
    def _motion_trap_evidence(
            candidate: dict, evidence: dict, *, reason: str) -> dict:
        def finite_value(key, default=0.0):
            try:
                value = float(evidence.get(key, default))
            except (TypeError, ValueError, OverflowError):
                value = float(default)
            return value if math.isfinite(value) else float(default)

        return {
            "reason": str(reason),
            "forward_clearance_m": finite_value(
                "current_forward_clearance_m",
                evidence.get("forward_clearance_m", 0.0)),
            "adaptive_step_m": finite_value(
                "current_adaptive_step_m",
                evidence.get("adaptive_step_m", 0.0)),
            "turn_clearance_m": finite_value("turn_clearance_m"),
            "path_blocked": bool(evidence.get(
                "current_path_blocked", evidence.get("path_blocked"))),
            "turn_motion_blocked": bool(
                evidence.get("turn_motion_blocked")),
            "candidate_x": finite_value("x", candidate.get("x", 0.0)),
            "candidate_y": finite_value("y", candidate.get("y", 0.0)),
        }

    def _probe_ordered_candidates(
            self, eligible, robot_pose, budget: int, *,
            allow_parallel: bool = True):
        """Probe one source tier, preserving serial candidate semantics."""

        budget = max(0, int(budget))
        evidence = []
        probes_used = 0
        batch_used = False
        if (allow_parallel and self.parallel_probe_workers > 0
                and len(eligible) > 1):
            evidence, probes_used = self._parallel_probe_first_approaches(
                eligible, robot_pose, budget)
            self._plan_probes += probes_used
            batch_used = probes_used > 0
        evidence_by_index = {
            item["candidate_index"]: item for item in evidence
        }

        for index, candidate in enumerate(eligible):
            entry = evidence_by_index.get(index)
            if entry is None:
                approaches = [
                    approach
                    for approach in self._candidate_approaches(
                        candidate, robot_pose)
                    if not self._approach_revisits_viewpoint(approach)
                ]
            else:
                approaches = entry["approaches"]
            if not approaches:
                self._record_failure(
                    candidate, "revisited_viewpoint", "plan")
                continue

            last_reason = "unreachable"
            next_approach = 0
            if entry is not None and entry["first_probed"]:
                next_approach = 1
                last_reason = str(
                    entry["validation_reason"] or "unreachable")
                if entry["validation_reason"] is None:
                    return (
                        self._chosen_from_probe(
                            approaches[0], entry["first_result"]),
                        probes_used,
                        False,
                        batch_used,
                    )

            attempts_complete = True
            for approach in approaches[next_approach:]:
                if probes_used >= budget:
                    attempts_complete = False
                    break
                result = self._safe_compute_path(approach)
                probes_used += 1
                self._plan_probes += 1
                reason = self._probe_validation_reason(
                    result, approach, robot_pose)
                if reason is not None:
                    last_reason = reason
                    continue
                return (
                    self._chosen_from_probe(approach, result),
                    probes_used,
                    False,
                    batch_used,
                )
            if not attempts_complete:
                return None, probes_used, True, batch_used
            self._record_failure(candidate, last_reason, "plan")
        return None, probes_used, False, batch_used

    def detect_room_enclosure(self, map_msg, robot_pose, door_radius_m: float = 2.5,
                              min_wall_ratio: float = 0.8) -> bool:
        """Return True when the reachable free space is walled in.

        封闭房间判据 (2026-07-21, rev 2): a room is enclosed when BOTH hold:
          1. every remaining frontier cluster is at the entry door (within
             ``door_radius_m`` of ``mission_origin``), AND
          2. the reachable free perimeter is mostly occupied walls
             (``wall_ratio >= min_wall_ratio``).

        Condition (2) is the user's literal spec — "外围全部被遮挡".
        Without it the first version triggered on a rolling-window artefact:
        the dog walked 17m from the door, MID360's window showed only the
        explored neighbourhood, frontier clusters came back empty, and the
        check returned True even though 51% of the room was still unknown
        (production mission 4903b09d: explored_ratio=0.485, 154 enclosed
        unknown regions, but completion_reason=room_enclosed).

        frontier-empty no longer claims enclosure; it defers to the regular
        stable_exhaustion path so radius expansion + further exploration
        can run.
        """
        wall_ratio = self._reachable_free_wall_ratio(map_msg, robot_pose)
        if wall_ratio < min_wall_ratio:
            return False
        try:
            clusters = find_frontier_clusters(
                map_msg, robot_pose, [], min_cluster_size=int(os.environ.get("GO2W_FRONTIER_MIN_CLUSTER_SIZE", "1")),
                revisit_radius=0.0, frontier_spacing_m=self.frontier_spacing_m)
        except Exception:
            return False
        if not clusters:
            # Reachable free is walled in AND no frontier at all → fully
            # mapped room. This is a true enclosure (distinct from the
            # rolling-window empty-frontier case, which fails wall_ratio).
            return True
        ox = float(self.mission_origin[0])
        oy = float(self.mission_origin[1])
        door_radius = max(0.5, float(door_radius_m))
        for cluster in clusters:
            world = cluster.get("center_world") or (0.0, 0.0)
            try:
                wx, wy = float(world[0]), float(world[1])
            except (TypeError, ValueError, IndexError):
                continue
            if math.hypot(wx - ox, wy - oy) > door_radius:
                return False
        return True

    def _reachable_free_wall_ratio(self, map_msg, robot_pose) -> float:
        """Fraction of reachable-free perimeter neighbors that are walls.

        Flood-fills the 4-connected free component containing the robot,
        then walks its 8-neighbors: every non-free neighbor is a perimeter
        sample, classified as wall (occupied, value > 0) or opening
        (unknown, value < 0). Returns walls / non-free. Low ratio means
        the explored region still borders meaningful unknown area
        (unexplored room); ratio near 1 means the region is hemmed in by
        walls (truly enclosed).
        """
        from collections import deque
        info = getattr(map_msg, "info", None)
        if info is None:
            return 0.0
        resolution = float(getattr(info, "resolution", 0.0) or 0.0)
        width = int(getattr(info, "width", 0) or 0)
        height = int(getattr(info, "height", 0) or 0)
        try:
            data = list(getattr(map_msg, "data", []) or [])
        except Exception:
            return 0.0
        if (resolution <= 0.0 or width <= 0 or height <= 0
                or len(data) != width * height):
            return 0.0
        origin = getattr(info, "origin", None)
        position = getattr(origin, "position", None)
        orientation = getattr(origin, "orientation", None)
        ox = float(getattr(position, "x", 0.0))
        oy = float(getattr(position, "y", 0.0))
        qx = float(getattr(orientation, "x", 0.0))
        qy = float(getattr(orientation, "y", 0.0))
        qz = float(getattr(orientation, "z", 0.0))
        qw = float(getattr(orientation, "w", 1.0))
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        try:
            rx, ry = float(robot_pose[0]), float(robot_pose[1])
        except (TypeError, ValueError, IndexError):
            return 0.0
        dx = rx - ox
        dy = ry - oy
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        col = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if not (0 <= row < height and 0 <= col < width):
            return 0.0
        seed = row * width + col
        if data[seed] != 0:
            best = None
            radius_cells = max(1, int(math.ceil(1.0 / resolution)))
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    nr, nc = row + dr, col + dc
                    if not (0 <= nr < height and 0 <= nc < width):
                        continue
                    if data[nr * width + nc] != 0:
                        continue
                    key = (dr * dr + dc * dc, nr, nc)
                    if best is None or key < best[0]:
                        best = (key, nr * width + nc)
            if best is None:
                return 0.0
            seed = best[1]
        seen = bytearray(width * height)
        seen[seed] = 1
        queue = deque([seed])
        reachable = []
        while queue:
            index = queue.popleft()
            reachable.append(index)
            r, c = divmod(index, width)
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                neighbor = nr * width + nc
                if not seen[neighbor] and data[neighbor] == 0:
                    seen[neighbor] = 1
                    queue.append(neighbor)
        wall = 0
        non_free = 0
        for index in reachable:
            r, c = divmod(index, width)
            for nr, nc in ((r - 1, c - 1), (r - 1, c), (r - 1, c + 1),
                           (r, c - 1), (r, c + 1),
                           (r + 1, c - 1), (r + 1, c), (r + 1, c + 1)):
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                value = data[nr * width + nc]
                if value == 0:
                    continue
                non_free += 1
                if value > 0:
                    wall += 1
        if non_free == 0:
            return 0.0
        return wall / float(non_free)

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
        # v3 修复 (大场地 50x50m): 原逻辑 `if active: return active` 丢弃非
        # active_tile 候选, 把狗锁在 16x16m tile (1/9 面积). 现在只跟踪 tile
        # 不 filter — 让 mixed_utility + expansion_bonus 自由排序, path_cost
        # (-k_time*t_travel) 已惩罚远 frontier 防乱跳.
        if self._active_tile is not None:
            try:
                robot_tile = self._tile_key(robot_pose[0], robot_pose[1])
                if tuple(self._active_tile) != robot_tile:
                    self._visited_tiles.add(self._active_tile)
            except (TypeError, ValueError, IndexError):
                pass
        try:
            robot_tile = self._tile_key(robot_pose[0], robot_pose[1])
        except (TypeError, ValueError, IndexError):
            robot_tile = (0, 0)
        tile_keys = {tuple(item["tile"]) for item in annotated}
        self._active_tile = min(tile_keys, key=lambda tile: (
            abs(tile[0] - robot_tile[0]) + abs(tile[1] - robot_tile[1]),
            tile[0], tile[1],
        ))
        return annotated

    def _confirm_exhaustion(self, map_msg=None) -> None:
        if self.entrance_gate is None and (
                self.mode != "current_room"
                or self.visibility_tracker is None):
            self._exhaustion_streak += 1
            if self._exhaustion_streak >= self.stable_exhaustion_cycles:
                self._last_selection_reason = "reachable_frontiers_exhausted"
            else:
                self._last_selection_reason = "stability_confirmation_pending"
            return
        observed_cells = []
        observed_cell_size_m = None
        if self.visibility_tracker is not None:
            try:
                visibility = dict(
                    self.visibility_tracker.snapshot(map_msg) or {})
                observed_cells = list(
                    visibility.get("observed_cells") or [])
                observed_cell_size_m = visibility.get("coverage_cell_size_m")
            except Exception:
                observed_cells = []
        state = analyze_global_search_state(
            map_msg,
            mission_origin=self.mission_origin,
            entrance_gate=self.entrance_gate,
            observed_cells=observed_cells,
            observed_cell_size_m=observed_cell_size_m,
            traversal_clearance_m=self.global_traversal_clearance_m,
            coverage_threshold=self.global_coverage_threshold,
        )
        self._global_search_state = dict(state)
        if not state.get("valid"):
            self._exhaustion_streak = 0
            self._last_exhaustion_revision = None
            self._last_selection_reason = "global_evidence_unverified"
            return
        if int(state.get("traversable_opening_count", 0) or 0) > 0:
            self._exhaustion_streak = 0
            self._last_exhaustion_revision = None
            self._last_selection_reason = "traversable_opening_blocked"
            return
        if not state.get("completion_eligible"):
            self._exhaustion_streak = 0
            self._last_exhaustion_revision = None
            self._last_selection_reason = "global_coverage_incomplete"
            return
        if self._last_exhaustion_revision == self._map_revision:
            self._last_selection_reason = "stability_confirmation_pending"
            return
        self._last_exhaustion_revision = self._map_revision
        self._exhaustion_streak += 1
        if self._exhaustion_streak >= self.stable_exhaustion_cycles:
            self._last_selection_reason = "reachable_frontiers_exhausted"
        else:
            self._last_selection_reason = "stability_confirmation_pending"
