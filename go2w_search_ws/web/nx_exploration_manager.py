"""Persistent bounded frontier policy; owns no ROS clients or subscriptions."""

from __future__ import annotations

from collections import OrderedDict
import hashlib
import math
import os
import threading
import time
from typing import Any, Callable, Optional

from nx_frontier_planner import (
    callable_accepts_keyword,
    find_frontier_clusters,
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
        frontier_spacing_m: float = 1.5,
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
        frontier_lookahead_trigger_m: float = 0.75,
        frontier_lookahead_goal_m: float = 1.2,
        initial_turn_staging_threshold_rad: float = math.radians(75.0),
        initial_turn_staging_distance_m: float = 0.8,
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
        # Parallel Nav2 probe (2026-07-20). 0 = serial (historical, locked
        # by plan_calls ordering tests). >0 = ThreadPoolExecutor workers
        # used to probe the first approach of each eligible candidate
        # concurrently, reducing worst-case selection stall from
        # max_plan_probes × planning_timeout_s (e.g. 12×3s=36s) to a single
        # planning_timeout_s round (~3s) when many candidates are reachable.
        parallel_probe_workers: int = 0,
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
        self._distance_m = 0.0
        self._plan_probes = 0
        self._plan_rejections = 0
        self._navigation_failures = 0
        self._last_selection_reason: Optional[str] = None
        self._navigation_start_distance: Optional[float] = None
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
        try:
            env_workers = int(os.environ.get(
                "GO2W_FRONTIER_PROBE_WORKERS",
                str(parallel_probe_workers)))
        except (TypeError, ValueError):
            env_workers = 0
        self.parallel_probe_workers = max(0, int(env_workers))

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
        # 封闭房间判据 (2026-07-21): if every remaining frontier cluster
        # lies within door_radius of mission_origin, the room is walled
        # off except for the entry door — stop rather than push into the
        # corridor. Only fires in current_room mode with a visibility
        # tracker injected (production current_room path); test stubs
        # rarely set up a tracker, so this guard keeps them on the
        # historical frontier-driven path.
        if (self.mode == "current_room"
                and os.environ.get("GO2W_FRONTIER_ROOM_ENCLOSURE_CHECK", "0") == "1"
                and self.detect_room_enclosure(map_msg, robot_pose)):
            self._last_selection_reason = "room_enclosed"
            return None
        revision = map_revision(map_msg)
        self._map_revision = revision
        candidate_selector = self._candidate_selector or select_frontier_candidates
        candidates = self._select_candidates(
            candidate_selector, map_msg, robot_pose)
        # Grow the active radius until the in-radius list has candidates AND
        # nothing was truncated by it (or we hit max_radius). The old "only
        # when list completely empty" condition stranded far frontier (e.g.
        # (21,-10) at 23m) behind a wall of always-present near frontiers,
        # because the list never went empty once the dog was inside the room.
        while self._can_expand_radius():
            truncated = getattr(self, "_last_radius_truncated", 0)
            if candidates and truncated == 0:
                break
            self._expand_radius()
            candidates = self._select_candidates(
                candidate_selector, map_msg, robot_pose)
        if not candidates:
            candidates = self._select_visual_coverage_candidates(
                map_msg, robot_pose)
        if not candidates:
            candidates = self._select_lidar_candidates(robot_pose)
        if not candidates:
            self._current_goal = None
            self._confirm_exhaustion()
            return None

        eligible = [
            candidate for candidate in candidates
            if self._spatial_failure_count(candidate)
            < self.max_failures_per_cell
            and self._distance_from_pose(candidate, robot_pose)
            >= self.min_goal_distance_m
        ]
        if not eligible:
            eligible = [
                candidate for candidate in candidates
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
            self._current_goal = None
            if self._can_expand_radius():
                self._expand_radius()
                self._last_selection_reason = "search_boundary_expanded"
            else:
                self._confirm_exhaustion()
            return None

        self._exhaustion_streak = 0
        eligible = self._prioritize_active_tile(eligible, robot_pose)
        # Trap-escape (2026-07-21): if recent nav failures stacked, the dog
        # is stuck in a dense-obstacle pocket. Pick the FARTHEST eligible
        # frontier to maximise the chance Nav2's global planner can route
        # around the trap. A fixed distance threshold failed in production
        # because small rooms had no candidate beyond it (far list empty,
        # escape silently no-op'd, dog kept abort-cycling on near goals).
        # Resets to normal once one goal succeeds.
        if (self._consecutive_nav_failures >= self._escape_abort_threshold
                and self._escape_min_distance_m > 0.0
                and len(eligible) > 1):
            farthest = max(
                eligible,
                key=lambda candidate: self._distance_from_pose(
                    candidate, robot_pose),
            )
            eligible = [farthest]
            self._last_selection_reason = "escape_trap_pending"
        if eligible and all(
                not item.get("coverage_candidate")
                and not item.get("lidar_candidate")
                for item in eligible):
            if self.utility_mode == "mixed":
                # 2026-07-20 mixed mode: rank by α·size + β·visual_gain
                # - γ·path_cost so the dog actively approaches unexplored
                # area. Distance/heading remain as tiebreakers only.
                eligible.sort(key=lambda item: (
                    self._mixed_utility_sort_key(item, robot_pose),
                    self._distance_from_pose(item, robot_pose),
                    float(item.get("heading_change", 0.0)),
                    -float(item.get("visual_gain", 0.0)),
                    float(item["x"]), float(item["y"]),
                ))
            else:
                # Physical frontier travel dominates mission duration. Probe
                # the nearest unknown boundary first; visual gain breaks
                # distance ties. Long movement in open space is still
                # produced by the LiDAR-confirmed lookahead in
                # _candidate_approaches().
                eligible.sort(key=lambda item: (
                    self._distance_from_pose(item, robot_pose),
                    -float(item.get("score", 0.0)),
                    -float(item.get("visual_gain", 0.0)),
                    -float(item.get("information_gain", item.get("size", 0.0))),
                    float(item.get("heading_change", 0.0)),
                    float(item["x"]), float(item["y"]),
                ))

        reachable = []
        probes_this_cycle = 0
        # Parallel fast path (2026-07-20): concurrently probe the first
        # approach of each eligible candidate. When enabled this collapses
        # the worst-case selection stall from max_plan_probes × timeout to
        # a single timeout round. The serial loop below still runs (it
        # breaks on the first reachable candidate, which the parallel path
        # already inserted), so semantics are preserved. A known P2
        # inefficiency: the serial sweep re-probes candidates the parallel
        # path already examined, roughly doubling probe count for the same
        # waypoint output (sim: 68 → 132). Acceptable because ComputePath
        # calls are cheap vs send_goal_and_wait latency.
        if self.parallel_probe_workers > 0 and len(eligible) > 1:
            chosen_parallel, parallel_probes = (
                self._parallel_probe_first_approaches(
                    eligible, robot_pose, self.max_plan_probes))
            probes_this_cycle += parallel_probes
            self._plan_probes += parallel_probes
            if chosen_parallel is not None:
                reachable.append(chosen_parallel)
        for candidate in eligible:
            # NOTE: critic #2 asked for a short-circuit `if reachable: break`
            # here, but doing so skips lower-priority candidates' standoff
            # fallbacks (the serial loop probes multiple standoff offsets
            # per candidate). Sim showed mixed+parallel dropping coverage
            # from 98.44% to 96.48% with the short-circuit. We keep the
            # serial sweep running so behaviour matches the serial path
            # exactly (sim H3 PASS), accepting ~2× probe count (68→132)
            # as the cost. ComputePath calls are cheaper than the goal
            # send_goal_and_wait they precede, and parallel_probe_workers
            # defaults to 0 (opt-in) so this only fires when an operator
            # explicitly enables it.
            if probes_this_cycle >= self.max_plan_probes:
                break
            approaches = self._candidate_approaches(candidate, robot_pose)
            approaches = [
                approach for approach in approaches
                if not self._approach_revisits_viewpoint(approach)
            ]
            if not approaches:
                self._record_failure(candidate, "revisited_viewpoint", "plan")
                continue
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
                path_respects_room = self._path_respects_room(result)
                path_makes_progress = self._path_makes_progress(result)
                path_detour_is_safe = self._path_detour_is_safe(
                    result, approach, robot_pose)
                endpoint_reaches_goal = self._path_endpoint_reaches_goal(
                    result)
                if (not result.get("ok")
                        or not path_respects_room
                        or not path_makes_progress
                        or not path_detour_is_safe
                        or not endpoint_reaches_goal):
                    last_reason = str(result.get("reason") or "unreachable")
                    if result.get("ok") and not path_respects_room:
                        last_reason = "path_leaves_room_polygon"
                    elif result.get("ok") and not path_makes_progress:
                        last_reason = "degenerate_plan"
                    elif result.get("ok") and not endpoint_reaches_goal:
                        last_reason = "plan_endpoint_mismatch"
                    elif result.get("ok"):
                        last_reason = "excessive_plan_detour"
                    continue
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
                reachable.append(chosen)
                candidate_reachable = True
                break
            else:
                attempts_complete = True
            if candidate_reachable:
                # Candidates are already ordered by mission utility. Starting
                # the first verified path avoids serially blocking on up to a
                # dozen ComputePathToPose calls while the dog stands still.
                break
            if not candidate_reachable and attempts_complete:
                self._record_failure(candidate, last_reason, "plan")
        if not reachable:
            # A per-cycle probe cap prevents planner storms. Remaining spatial
            # candidates are retried on the next selection cycle; the lifetime
            # counter is telemetry only and never terminates a large mission.
            self._last_selection_reason = "retry_pending"
            self._current_goal = None
            return None
        if all(
                not item.get("coverage_candidate")
                and not item.get("lidar_candidate")
                for item in reachable):
            if self.utility_mode == "mixed":
                # path_length is now known (post-probe); utility uses the
                # real Nav2 cost instead of euclidean distance.
                reachable.sort(key=lambda item: (
                    self._mixed_utility_sort_key(item, robot_pose),
                    item["path_length"],
                    -float(item.get("visual_gain", 0.0)),
                    item["x"], item["y"],
                ))
            else:
                reachable.sort(key=lambda item: (
                    item["path_length"],
                    -float(item.get("visual_gain", 0.0)),
                    -float(item.get("information_gain", item.get("size", 0.0))),
                    -item["score"], item["x"], item["y"],
                ))
        else:
            reachable.sort(key=lambda item: (
                -item["score"], item["path_length"], item["x"], item["y"]))
        self._current_goal = dict(reachable[0])
        self._navigation_start_distance = self._distance_from_pose(
            self._current_goal, robot_pose)
        self._navigation_divergence_count = 0
        self._goal_revalidation_failure_count = 0
        self._last_selection_reason = None
        return dict(self._current_goal)

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
            self._current_goal = None
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
        self._current_goal = None
        self._reset_navigation_progress()

    def mark_navigation_failed(self, reason: str, candidate: Optional[dict] = None) -> None:
        target = dict(candidate or self._current_goal or {})
        if target:
            self._record_failure(target, str(reason or "navigation_failed"), "navigation")
        self._consecutive_nav_failures += 1
        self._current_goal = None
        self._reset_navigation_progress()

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
            "utility_mode": self.utility_mode,
            "mixed_frontier_weight": self.mixed_frontier_weight,
            "mixed_visual_gain_weight": self.mixed_visual_gain_weight,
            "mixed_path_cost_penalty": self.mixed_path_cost_penalty,
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
        record["count"] = int(record.get("count", 0)) + 1
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

    def _select_candidates(self, candidate_selector, map_msg, robot_pose):
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
        if callable_accepts_keyword(candidate_selector, "frontier_spacing_m"):
            selector_kwargs["frontier_spacing_m"] = self.frontier_spacing_m
        candidates = candidate_selector(
            map_msg,
            robot_pose,
            self._visited,
            **selector_kwargs,
        )
        if self.visibility_tracker is not None:
            candidates = self.visibility_tracker.rank_candidates(
                map_msg, robot_pose, candidates)
            # Hard-filtering visual_gain==0 frontiers was too aggressive in
            # mixed mode: once the dog's视锥 swept a region, every remaining
            # map frontier got dropped, the candidate list went empty, and
            # stable_exhaustion fired after 3 cycles — ending the mission
            # with most of the room unexplored (sim critique #3, confirmed
            # in 2026-07-21 production run: only 3 waypoints in a 4-wall
            # room). Mixed mode already uses visual_gain as a utility term,
            # so the filter is redundant there. Keep it only for nearest
            # mode where it was the historical tie-breaker.
            if self.utility_mode != "mixed":
                positive_visual_gain = [
                    item for item in candidates
                    if float(item.get("visual_gain", 0.0)) > 0.0
                ]
                if positive_visual_gain:
                    candidates = positive_visual_gain
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
        candidates = self.visibility_tracker.coverage_candidates(
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

    def _mixed_utility_sort_key(self, candidate: dict, robot_pose) -> tuple:
        """Primary sort key for mixed-utility mode.

        Linear combination: ``alpha * information_gain + beta * visual_gain
        - gamma * path_length``. Unlike ``score_frontier``'s gain/cost ratio,
        this rewards large frontiers AND fresh visibility gain at the cost of
        longer travel, so the dog actively approaches unexplored area instead
        of always picking the closest small frontier.

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
        path_cost = self._path_cost_for_utility(candidate, robot_pose)
        utility = (
            self.mixed_frontier_weight * information_gain
            + self.mixed_visual_gain_weight * visual_gain
            - self.mixed_path_cost_penalty * path_cost
        )
        # Return as tuple so callers can compose tiebreakers (distance, x, y).
        return (-utility,)

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

    def _parallel_probe_first_approaches(
            self, eligible: list, robot_pose, budget: int):
        """Concurrently probe the first approach of each eligible candidate.

        Opt-in fast path (2026-07-20) used when ``parallel_probe_workers`` > 0
        and at least two candidates are eligible. Submits one
        ``compute_path_to_pose`` per candidate to a ThreadPoolExecutor, then
        walks results in submission order so the highest-priority reachable
        candidate wins — matching the serial loop's first-reachable-wins rule.

        Returns ``(chosen_or_None, probes_used)``. The caller falls through
        to the serial loop when ``chosen`` is None so per-candidate standoff
        fallbacks still run for hard-to-reach frontiers.
        """
        import concurrent.futures

        candidates_to_probe: list = []
        first_approaches: list = []
        for candidate in eligible:
            if len(candidates_to_probe) >= budget:
                break
            approaches = self._candidate_approaches(candidate, robot_pose)
            approaches = [
                approach for approach in approaches
                if not self._approach_revisits_viewpoint(approach)
            ]
            if not approaches:
                continue
            candidates_to_probe.append(candidate)
            first_approaches.append(approaches[0])
        if not candidates_to_probe:
            return None, 0

        workers = max(1, min(
            int(self.parallel_probe_workers), len(candidates_to_probe)))

        def _probe(index):
            approach = first_approaches[index]
            result = self.navigation_port.compute_path_to_pose(
                approach["x"], approach["y"], approach["yaw"],
                frame_id="map", timeout=self.planning_timeout_s)
            return index, result

        results_in_order: list = [None] * len(candidates_to_probe)
        probes_used = 0
        with concurrent.futures.ThreadPoolExecutor(
                max_workers=workers) as pool:
            futures = {
                pool.submit(_probe, i): i
                for i in range(len(candidates_to_probe))
            }
            for future in concurrent.futures.as_completed(futures):
                probes_used += 1
                index, result = future.result()
                results_in_order[index] = result

        chosen = None
        for index, result in enumerate(results_in_order):
            if result is None:
                continue
            approach = first_approaches[index]
            ok = (
                result.get("ok")
                and self._path_respects_room(result)
                and self._path_makes_progress(result)
                and self._path_detour_is_safe(result, approach, robot_pose)
                and self._path_endpoint_reaches_goal(result))
            if not ok:
                continue
            chosen_approach = dict(approach)
            chosen_approach["path_length"] = float(
                result.get("path_length", approach.get("distance", 0.0)))
            chosen_approach["path_poses"] = int(result.get("poses", 0) or 0)
            if result.get("goal_error_m") is not None:
                chosen_approach["goal_error_m"] = float(result["goal_error_m"])
            chosen_approach["score"] = score_frontier(
                chosen_approach,
                path_length=chosen_approach["path_length"],
                heading_change=chosen_approach.get("heading_change", 0.0),
                failure_count=chosen_approach.get("failure_count", 0),
                distance_weight=self.distance_weight,
                heading_weight=self._effective_heading_weight(chosen_approach),
                failure_penalty=self.failure_penalty,
            ) + float(chosen_approach.get("lidar_progress_bonus", 0.0))
            if chosen is None:
                chosen = chosen_approach
        return chosen, probes_used

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
                map_msg, robot_pose, [], min_cluster_size=3,
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
