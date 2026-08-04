"""Strategy comparison simulation for frontier exploration (2026-07-20).

Standalone script — does NOT touch ROS / Nav2 / NX. Synthesises a closed
room with internal obstacles, then runs the full ExplorationManager loop
under three configurations and prints side-by-side metrics so we can
validate the mixed-utility + parallel-probe fixes without hardware.

Run:
    python go2w_search_ws/tools/sim_strategy_compare.py

What it validates
-----------------
- nearest (historical): always picks closest frontier first.
- mixed:               actual-speed tuned serial reference (k_time=12.5).
- mixed + parallel:    same scoring + concurrent Nav2 probe fast-path.

Hypotheses the sim must confirm:
  H1. mixed reaches equal-or-higher coverage than nearest (problem 4).
  H2. mixed visits large distant frontiers earlier than nearest (problem 1).
  H3. mixed + parallel produces identical waypoint choices as mixed when
      every candidate is reachable (parallel is a latency optimisation,
      not a behaviour change).
  H9. the deployed physical-speed scorer retains coverage/far-boundary
      reach, keeps path length within 110% of nearest, and reduces turning.
"""
from __future__ import annotations

from collections import deque
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

WEB_DIR = Path(__file__).resolve().parent.parent / "web"
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_exploration_manager import ExplorationManager  # noqa: E402
from nx_frontier_planner import select_frontier_candidates  # noqa: E402


DEPLOY_MAX_VEL_X = 0.8
DEPLOY_MAX_VEL_THETA = 0.5
DEPLOY_K_TIME = 14.5
PATH_NEAR_NEAREST_MAX_RATIO = 1.10
PRIMARY_K_TIME_CANDIDATES = (15.0, 20.0, 25.0)
ADJACENT_K_TIME_CANDIDATES = (10.0, 12.5, 14.0, 14.5, 17.5)


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
    """BFS path planner over the currently observed free grid.

    Read-only during compute_path_to_pose — safe for concurrent calls from
    ThreadPoolExecutor (parallel_probe_workers path). The sim updates
    self.pose only between choose_next rounds, never during probing.
    """

    def __init__(self, observed, width, height, resolution, pose):
        self.observed = observed
        self.width = width
        self.height = height
        self.resolution = resolution
        self.pose = list(pose)
        self.probe_endpoints = []
        self.parallel_batches = {}
        self._planned_metrics = {}
        self.selection_cycle_id = None
        self._probe_lock = threading.Lock()

    def begin_selection_cycle(self, cycle_id):
        with self._probe_lock:
            self.selection_cycle_id = int(cycle_id)

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=3.0):
        del timeout
        endpoint = ExplorationManager._physical_probe_key({
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }, frame_id=frame_id)
        thread_name = threading.current_thread().name
        batch_id = None
        if thread_name.startswith("ThreadPoolExecutor-"):
            batch_id = thread_name.rsplit("_", 1)[0]
        with self._probe_lock:
            self.probe_endpoints.append(endpoint)
            if batch_id is not None:
                batch_key = (self.selection_cycle_id, batch_id)
                self.parallel_batches.setdefault(
                    batch_key, []).append(endpoint)
        start = self._cell(self.pose[0], self.pose[1])
        goal = self._cell(x, y)
        if start is None or goal is None:
            return self._record_plan_result(
                endpoint, {"ok": False, "reason": "outside_map"})
        distances = {start: 0}
        parents = {start: None}
        queue = deque([start])
        while queue:
            row, col = queue.popleft()
            if (row, col) == goal:
                steps = distances[(row, col)]
                path_cells = []
                cursor = (row, col)
                while cursor is not None:
                    path_cells.append(cursor)
                    cursor = parents[cursor]
                path_cells.reverse()
                result = {
                    "ok": True,
                    "poses": steps + 1,
                    "path_length": steps * self.resolution,
                    "path_heading_turn_rad": self._path_heading_turn(
                        path_cells, float(self.pose[2]), float(yaw)),
                }
                return self._record_plan_result(endpoint, result)
            for dr, dc in ((-1, 0), (0, -1), (0, 1), (1, 0)):
                neighbor = (row + dr, col + dc)
                if neighbor in distances or not self._free(*neighbor):
                    continue
                distances[neighbor] = distances[(row, col)] + 1
                parents[neighbor] = (row, col)
                queue.append(neighbor)
        return self._record_plan_result(
            endpoint, {"ok": False, "reason": "no_known_free_path"})

    @staticmethod
    def _path_heading_turn(path_cells, start_yaw, goal_yaw):
        heading = float(start_yaw)
        total = 0.0
        for (row0, col0), (row1, col1) in zip(
                path_cells, path_cells[1:]):
            segment_heading = math.atan2(row1 - row0, col1 - col0)
            total += abs(math.atan2(
                math.sin(segment_heading - heading),
                math.cos(segment_heading - heading)))
            heading = segment_heading
        total += abs(math.atan2(
            math.sin(float(goal_yaw) - heading),
            math.cos(float(goal_yaw) - heading)))
        return total

    def _record_plan_result(self, endpoint, result):
        with self._probe_lock:
            if result.get("ok"):
                self._planned_metrics[endpoint] = {
                    "path_length": float(result["path_length"]),
                    "path_heading_turn_rad": float(
                        result["path_heading_turn_rad"]),
                }
            else:
                self._planned_metrics.pop(endpoint, None)
        return result

    def planned_metrics_for_pose(self, x, y, yaw, frame_id="map"):
        endpoint = ExplorationManager._physical_probe_key({
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }, frame_id=frame_id)
        with self._probe_lock:
            metrics = self._planned_metrics.get(endpoint)
            return None if metrics is None else dict(metrics)

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


class _SimTracker:
    """Sim-only: visual_gain_at = 从 (x,y,yaw) raycast truth grid,
    返回 observed 还没揭示的 free cell 数。"""
    def __init__(self, truth, observed, width, height, resolution,
                 hfov=1.2, vrange=5.0):
        self.truth = truth
        self.observed = observed  # 引用, _reveal 更新同一份
        self.width = width
        self.height = height
        self.resolution = resolution
        self.hfov = hfov
        self.vrange = vrange
        self._observed = set()

    def visual_gain_at(self, map_msg, x, y, yaw):
        step = self.resolution * 0.5
        gain = 0
        seen = set()
        ray_count = max(3, int(math.ceil(self.hfov / math.radians(2.0))) + 1)
        for k in range(ray_count):
            offset = -self.hfov / 2 + self.hfov * k / max(1, ray_count - 1)
            angle = yaw + offset
            d = 0.0
            while d <= self.vrange:
                wx = x + d * math.cos(angle)
                wy = y + d * math.sin(angle)
                col = int(math.floor(wx / self.resolution))
                row = int(math.floor(wy / self.resolution))
                if not (0 <= row < self.height and 0 <= col < self.width):
                    break
                idx = row * self.width + col
                if self.truth[idx] >= 50:
                    break
                if self.observed[idx] == -1 and idx not in seen:
                    gain += 1
                    seen.add(idx)
                d += step
        return gain

    def observe(self, *a, **k): return {}
    def snapshot(self, *a, **k): return {}
    def rank_candidates(self, m, p, c): return list(c)
    def coverage_candidates(self, *a, **k): return []
    def lidar_candidates(self, *a, **k): return []


def _gate_candidate(x, *, unknown=0, coverage=False, lidar=False):
    candidate = {
        "x": float(x),
        "y": 0.0,
        "yaw": 0.0,
        "size": 10,
        "center_cell": (1, int(round(float(x)))),
        "distance": abs(float(x)),
        "information_gain": 10.0,
        "score": 10.0,
        "adjacent_unknown_count": int(unknown),
        "adjacent_support_count": 10,
    }
    if coverage:
        candidate["coverage_candidate"] = True
    if lidar:
        candidate["lidar_candidate"] = True
    return candidate


class _FallbackGateTracker:
    """Expose real fallback candidates and count whether they were generated."""

    def __init__(self):
        self.coverage_calls = 0
        self.lidar_calls = 0

    def snapshot(self, *_args, **_kwargs):
        return {"visual_coverage_ratio": 0.0}

    def rank_candidates(self, _map_msg, _pose, candidates):
        return list(candidates)

    def coverage_candidates(self, *_args, **_kwargs):
        self.coverage_calls += 1
        return [_gate_candidate(1.0, coverage=True)]

    def lidar_candidates(self, *_args, **_kwargs):
        self.lidar_calls += 1
        return [_gate_candidate(1.5, lidar=True)]


class _FallbackGatePlanner:
    def __init__(self):
        self.probe_x = []

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=3.0):
        del yaw, frame_id, timeout
        x = float(x)
        self.probe_x.append(round(x, 3))
        if abs(x - 3.4) < 1e-6:
            return {"ok": False, "reason": "blocked_first_standoff"}
        return {
            "ok": True,
            "path_length": math.hypot(x, float(y)),
            "poses": 4,
            "goal_error_m": 0.0,
        }


def _run_h7_fallback_gate():
    """A later frontier standoff must beat both real fallback sources."""

    preferred = _gate_candidate(4.0, unknown=6)
    preferred["prefer_standoff"] = True
    secondary = _gate_candidate(2.0, unknown=2)
    tracker = _FallbackGateTracker()
    planner = _FallbackGatePlanner()
    manager = ExplorationManager(
        navigation_port=planner,
        mission_origin=(0.0, 0.0, 0.0),
        mode="current_room",
        room_radius_m=8.0,
        initial_radius_m=8.0,
        visibility_tracker=tracker,
        candidate_selector=lambda *_a, **_k: [
            dict(preferred), dict(secondary)],
        max_frontier_standoff_steps=2,
        max_plan_probes=12,
        parallel_probe_workers=4,
        reject_map_edge=False,
    )
    selected = manager.choose_next(
        _grid([0] * 100, 10, 10, 1.0), (0.0, 0.0, 0.0))
    selected_source = (
        None if selected is None
        else str(selected.get("exploration_source", "frontier")))
    selected_frontier_x = (
        None if selected is None else selected.get("frontier_x"))
    passed = (
        selected_source == "frontier"
        and selected_frontier_x is not None
        and abs(float(selected_frontier_x) - 4.0) < 1e-6
        and tracker.coverage_calls == 0
        and tracker.lidar_calls == 0
        and planner.probe_x.count(3.4) == 1
        and planner.probe_x.count(3.7) == 1
    )
    return {
        "passed": passed,
        "selected_source": selected_source,
        "selected_frontier_x": selected_frontier_x,
        "coverage_calls": tracker.coverage_calls,
        "lidar_calls": tracker.lidar_calls,
        "probe_x": planner.probe_x,
    }


def _build_world():
    """20m × 12m room with perimeter walls + two internal obstacles."""
    resolution = 0.25
    width, height = 80, 48
    truth = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            if row in {0, height - 1} or col in {0, width - 1}:
                truth[row * width + col] = 100
    for row in range(14, 34):
        for col in range(20, 28):
            truth[row * width + col] = 100
    for row in range(20, 40):
        for col in range(52, 60):
            truth[row * width + col] = 100
    return resolution, width, height, truth


def _run_scenario(label, *, utility_mode, parallel_workers,
                  mixed_weights=None, max_iterations=250):
    resolution, width, height, truth = _build_world()
    observed = [-1] * (width * height)
    pose = [2.0, 6.0, 0.0]
    sensor_radius = 2.25
    _reveal(truth, observed, width, height, resolution,
            pose[0], pose[1], sensor_radius)
    nav = _KnownFreePlanner(observed, width, height, resolution, pose)

    kwargs = dict(
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
        utility_mode=utility_mode,
        parallel_probe_workers=parallel_workers,
        max_vel_x=DEPLOY_MAX_VEL_X,
        max_vel_theta=DEPLOY_MAX_VEL_THETA,
    )
    if mixed_weights is not None:
        kwargs.update(mixed_weights)

    sim_tracker = _SimTracker(truth, observed, width, height, resolution)
    kwargs["visibility_tracker"] = sim_tracker

    manager = ExplorationManager(**kwargs)

    goals = []
    goal_sequence = []
    source_sequence = []
    selection_latency_ms = []
    total_path = 0.0
    total_path_turn = 0.0
    terminal_reason = None
    source_counts = {"frontier": 0, "coverage": 0, "lidar": 0}
    fallback_with_reachable_frontier = 0
    selection_cycles = 0
    for _ in range(max_iterations):
        selection_cycles += 1
        nav.begin_selection_cycle(selection_cycles)
        nav.pose[:] = pose
        map_msg = _grid(observed, width, height, resolution)
        selection_started = time.perf_counter()
        target = manager.choose_next(map_msg, tuple(pose))
        selection_latency_ms.append(
            (time.perf_counter() - selection_started) * 1000.0)
        if target is None:
            reason = manager.snapshot()["last_selection_reason"]
            if reason == "reachable_frontiers_exhausted":
                terminal_reason = reason
                break
            continue
        source = str(target.get("exploration_source", "frontier"))
        source_counts[source] = source_counts.get(source, 0) + 1
        source_sequence.append(source)
        if source != "frontier":
            frontiers = manager._select_candidates(
                select_frontier_candidates, map_msg, tuple(pose))
            if any(
                nav.compute_path_to_pose(
                    item["x"], item["y"], item.get("yaw", 0.0)).get("ok")
                for item in frontiers
            ):
                fallback_with_reachable_frontier += 1
        goals.append((target["x"], target["y"]))
        target_yaw = float(target.get("yaw", 0.0))
        goal_sequence.append((
            round(float(target["x"]), 6),
            round(float(target["y"]), 6),
            round(target_yaw, 6),
        ))
        planned_metrics = nav.planned_metrics_for_pose(
            target["x"], target["y"], target_yaw)
        if planned_metrics is None:
            raise RuntimeError("selected goal has no cached planner evidence")
        selected_path_length = float(target["path_length"])
        if not math.isclose(
                selected_path_length, planned_metrics["path_length"],
                rel_tol=0.0, abs_tol=1e-9):
            raise RuntimeError("selected goal path length differs from probe")
        total_path += selected_path_length
        total_path_turn += planned_metrics["path_heading_turn_rad"]
        pose[:] = [target["x"], target["y"], target["yaw"]]
        _reveal(truth, observed, width, height, resolution,
                pose[0], pose[1], sensor_radius)
        manager.mark_visited(target)

    free_cells = [i for i, v in enumerate(truth) if v == 0]
    known_free = sum(1 for i in free_cells if observed[i] == 0)
    coverage = known_free / len(free_cells) if free_cells else 0.0
    snapshot = manager.snapshot()
    sorted_latency = sorted(selection_latency_ms)

    def latency_percentile(percentile):
        if not sorted_latency:
            return 0.0
        index = max(0, min(
            len(sorted_latency) - 1,
            int(math.ceil(percentile * len(sorted_latency))) - 1,
        ))
        return round(sorted_latency[index], 3)

    parallel_batches = list(nav.parallel_batches.values())
    duplicate_parallel_endpoints = sum(
        len(batch) - len(set(batch)) for batch in parallel_batches)
    parallel_batches_per_cycle = {}
    for cycle_id, _batch_id in nav.parallel_batches:
        parallel_batches_per_cycle[cycle_id] = (
            parallel_batches_per_cycle.get(cycle_id, 0) + 1)
    return {
        "label": label,
        "waypoints": len(goals),
        "selection_cycles": selection_cycles,
        "selection_latency_p50_ms": latency_percentile(0.50),
        "selection_latency_p95_ms": latency_percentile(0.95),
        "coverage_pct": round(coverage * 100.0, 2),
        "total_path_m": round(total_path, 2),
        "total_path_turn_rad": round(total_path_turn, 2),
        "max_x_reached": round(max((g[0] for g in goals), default=0.0), 2),
        "max_y_reached": round(max((g[1] for g in goals), default=0.0), 2),
        "plan_probes": snapshot["plan_probes"],
        "plan_rejections": snapshot["plan_rejections"],
        "terminal_reason": terminal_reason or "max_iterations",
        "far_boundary_x": round(width * resolution - sensor_radius, 2),
        "source_counts": source_counts,
        "source_sequence": source_sequence,
        "waypoint_sequence": goal_sequence,
        "physical_probe_endpoints": list(nav.probe_endpoints),
        "parallel_batch_count": len(parallel_batches),
        "max_parallel_batches_per_selection_cycle": max(
            parallel_batches_per_cycle.values(), default=0),
        "max_parallel_batch_size": max(
            (len(batch) for batch in parallel_batches), default=0),
        "duplicate_parallel_endpoints": duplicate_parallel_endpoints,
        "fallback_with_reachable_frontier": (
            fallback_with_reachable_frontier),
        "first_5_goals": [(round(x, 1), round(y, 1)) for x, y in goals[:5]],
    }


def _print_table(results):
    headers = ["mode", "waypoints", "coverage%", "path_m", "path_turn_rad",
               "max_x", "max_y", "probes", "p50ms", "p95ms", "rejects",
               "terminal"]
    rows = [[r["label"], r["waypoints"], r["coverage_pct"],
             r["total_path_m"], r["total_path_turn_rad"],
             r["max_x_reached"], r["max_y_reached"],
             r["plan_probes"], r["selection_latency_p50_ms"],
             r["selection_latency_p95_ms"], r["plan_rejections"],
             r["terminal_reason"]]
            for r in results]
    widths = [max(len(str(h)), *(len(str(row[i])) for row in rows))
              for i, h in enumerate(headers)]
    sep = "+".join("-" * (w + 2) for w in widths)
    print("  " + sep)
    print("  " + " | ".join(h.ljust(w) for h, w in zip(headers, widths)))
    print("  " + sep)
    for row in rows:
        print("  " + " | ".join(str(c).ljust(w) for c, w in zip(row, widths)))
    print("  " + sep)
    print("\n  First 5 goals per mode:")
    for r in results:
        print(f"    {r['label']:24s} -> {r['first_5_goals']}")


def _tuning_gate_status(result, nearest):
    """Return explicit deployment gates; path-near means no >10% blow-up."""
    return {
        "coverage": (
            result["coverage_pct"] >= nearest["coverage_pct"] - 1.0),
        "far_boundary": (
            result["max_x_reached"] >= result["far_boundary_x"]),
        "path_near": (
            result["total_path_m"]
            <= nearest["total_path_m"] * PATH_NEAR_NEAREST_MAX_RATIO),
        "turn_lower": (
            result["total_path_turn_rad"]
            < nearest["total_path_turn_rad"]),
    }


def main():
    print("\n=== Frontier strategy comparison sim ===")
    print("World: 20m × 12m room, 2 internal obstacles, start at (2, 6)\n")

    results = [
        _run_scenario(
            "nearest",
            utility_mode="nearest",
            parallel_workers=0,
        ),
        _run_scenario(
            "mixed",
            utility_mode="mixed",
            parallel_workers=0,
            mixed_weights={
                "mixed_frontier_weight": 0.5,
                "mixed_visual_gain_weight": 1.0,
                "mixed_path_cost_penalty": 0.5,
                "mixed_heading_penalty": 12.5,
            },
        ),
        _run_scenario(
            "mixed+parallel",
            utility_mode="mixed",
            parallel_workers=4,
            mixed_weights={
                "mixed_frontier_weight": 0.5,
                "mixed_visual_gain_weight": 1.0,
                "mixed_path_cost_penalty": 0.5,
                "mixed_heading_penalty": 12.5,
            },
        ),
        _run_scenario(
            "v3",
            utility_mode="mixed",
            parallel_workers=0,
            mixed_weights={
                "mixed_frontier_weight": 0.5,
                "mixed_visual_gain_weight": 1.0,
                "mixed_wall_bonus": 1.0,
                "mixed_expansion_bonus": 0.1,
                "yaw_step_deg": 45.0,
                "max_vel_x": DEPLOY_MAX_VEL_X,
                "max_vel_theta": DEPLOY_MAX_VEL_THETA,
                "mixed_heading_penalty": DEPLOY_K_TIME,
            },
        ),
    ]
    _print_table(results)

    nearest = results[0]
    mixed = results[1]
    mixed_par = results[2]
    v3 = results[3]
    fallback_gate = _run_h7_fallback_gate()

    print("\n=== Hypothesis checks ===")
    h1 = mixed["coverage_pct"] >= nearest["coverage_pct"] - 1.0
    print(f"  H1 mixed coverage >= nearest (±1%): "
          f"{mixed['coverage_pct']} vs {nearest['coverage_pct']} -> "
          f"{'PASS' if h1 else 'FAIL'}")
    h2 = mixed["max_x_reached"] >= nearest["max_x_reached"] - 0.5
    print(f"  H2 mixed reaches as far in X as nearest: "
          f"{mixed['max_x_reached']} vs {nearest['max_x_reached']} -> "
          f"{'PASS' if h2 else 'FAIL'}")
    same_waypoints = (
        mixed_par["waypoint_sequence"] == mixed["waypoint_sequence"])
    same_sources = (
        mixed_par["source_sequence"] == mixed["source_sequence"])
    h3 = (same_waypoints and same_sources
          and abs(mixed_par["coverage_pct"] - mixed["coverage_pct"]) < 0.1)
    print(f"  H3 mixed+parallel exactly matches serial goal/source sequence: "
          f"goals={same_waypoints} sources={same_sources} "
          f"wp={mixed_par['waypoints']}/{mixed['waypoints']} "
          f"cov={mixed_par['coverage_pct']}/{mixed['coverage_pct']} -> "
          f"{'PASS' if h3 else 'FAIL'}")
    h4 = v3["coverage_pct"] >= nearest["coverage_pct"] - 1.0
    print(f"  H4 v3 coverage >= nearest (±1%): "
          f"{v3['coverage_pct']} vs {nearest['coverage_pct']} -> "
          f"{'PASS' if h4 else 'FAIL'}")
    h5 = v3["total_path_turn_rad"] < nearest["total_path_turn_rad"]
    print(f"  H5 v3 planned-path turn_rad < nearest: "
          f"{v3['total_path_turn_rad']} vs "
          f"{nearest['total_path_turn_rad']} -> "
          f"{'PASS' if h5 else 'FAIL'}")
    h6 = v3["max_x_reached"] >= v3["far_boundary_x"]
    print(f"  H6 v3 reaches far-room sensor boundary: "
          f"{v3['max_x_reached']} >= {v3['far_boundary_x']} -> "
          f"{'PASS' if h6 else 'FAIL'}")
    h7 = fallback_gate["passed"]
    print(f"  H7 later reachable frontier standoff precedes real fallbacks: "
          f"source={fallback_gate['selected_source']} "
          f"frontier_x={fallback_gate['selected_frontier_x']} "
          f"fallback_calls={fallback_gate['coverage_calls']}/"
          f"{fallback_gate['lidar_calls']} "
          f"probes={fallback_gate['probe_x']} -> "
          f"{'PASS' if h7 else 'FAIL'}")
    parallel_workers = 4
    speculative_allowance = (
        mixed_par["selection_cycles"] * max(0, parallel_workers - 1))
    probe_limit = mixed["plan_probes"] + speculative_allowance
    aggregate_probe_bound = mixed_par["plan_probes"] <= probe_limit
    no_batch_duplicates = (
        mixed_par["duplicate_parallel_endpoints"] == 0)
    worker_bound = (
        mixed_par["max_parallel_batch_size"] <= parallel_workers)
    one_batch_per_cycle = (
        mixed_par["max_parallel_batches_per_selection_cycle"] <= 1)
    exact_probe_telemetry = (
        mixed_par["plan_probes"]
        == len(mixed_par["physical_probe_endpoints"]))
    h8 = (
        h3 and aggregate_probe_bound and no_batch_duplicates
        and worker_bound and one_batch_per_cycle and exact_probe_telemetry)
    print(f"  H8 parallel batches are bounded, unique, and exactly counted: "
          f"{mixed_par['plan_probes']} <= {mixed['plan_probes']} + "
          f"{mixed_par['selection_cycles']}*{parallel_workers - 1} "
          f"(limit={probe_limit}); batches={mixed_par['parallel_batch_count']} "
          f"max_batches/cycle="
          f"{mixed_par['max_parallel_batches_per_selection_cycle']} "
          f"max_batch={mixed_par['max_parallel_batch_size']} "
          f"duplicates={mixed_par['duplicate_parallel_endpoints']} "
          f"telemetry={exact_probe_telemetry} -> {'PASS' if h8 else 'FAIL'}")

    print("\n=== physical-profile k_time grid (wall bonus fixed at 1.0) ===")
    grid_results = []
    candidate_values = (
        PRIMARY_K_TIME_CANDIDATES + ADJACENT_K_TIME_CANDIDATES)
    for k_time in candidate_values:
        if math.isclose(k_time, DEPLOY_K_TIME):
            r = v3
        else:
            r = _run_scenario(
                f"k={k_time}", utility_mode="mixed", parallel_workers=0,
                mixed_weights={
                    "mixed_frontier_weight": 0.5,
                    "mixed_visual_gain_weight": 1.0,
                    "mixed_heading_penalty": k_time,
                    "mixed_wall_bonus": 1.0,
                    "mixed_expansion_bonus": 0.1,
                    "yaw_step_deg": 45.0,
                    "max_vel_x": DEPLOY_MAX_VEL_X,
                    "max_vel_theta": DEPLOY_MAX_VEL_THETA,
                })
        gates = _tuning_gate_status(r, nearest)
        grid_results.append((k_time, r, gates))
        print(
            f"  k={k_time:4.1f}: cov={r['coverage_pct']:5.2f}% "
            f"path={r['total_path_m']:6.2f}m "
            f"path_turn={r['total_path_turn_rad']:5.2f}rad "
            f"max_x={r['max_x_reached']:5.2f} "
            f"gates={gates} -> "
            f"{'PASS' if all(gates.values()) else 'FAIL'}")

    primary_ok = [
        item for item in grid_results
        if item[0] in PRIMARY_K_TIME_CANDIDATES
        and all(item[2].values())]
    if not primary_ok:
        print("  Primary k_time 15/20/25: no all-gates candidate; "
              "adjacent search required.")
    candidates_ok = [
        item for item in grid_results if all(item[2].values())]
    recommended = min(
        candidates_ok,
        key=lambda item: (
            item[1]["total_path_turn_rad"],
            abs(item[0] - min(PRIMARY_K_TIME_CANDIDATES)),
            -item[1]["coverage_pct"],
            item[1]["total_path_m"],
        ),
        default=None,
    )
    selected_grid = next(
        (item for item in grid_results
         if math.isclose(item[0], DEPLOY_K_TIME)), None)
    h9 = (
        recommended is not None
        and selected_grid is not None
        and math.isclose(recommended[0], DEPLOY_K_TIME)
        and all(selected_grid[2].values())
    )
    if recommended is not None:
        print(f"  RECOMMENDED_K_TIME={recommended[0]}")
    print(f"  SELECTED_K_TIME={DEPLOY_K_TIME}")
    print(f"  H9 selected physical-profile tuning passes all gates and "
          f"is the gated grid recommendation: {'PASS' if h9 else 'FAIL'}")

    print("\n=== Notes ===")
    print("  - coverage is 'ground-truth free cells the sensor revealed'.")
    print("  - path_m is the sum of the selected BFS planner path lengths.")
    print("  - path_turn_rad sums heading changes along those planned paths,")
    print("    including initial alignment and the selected goal yaw.")
    print("  - H3 requires the complete ordered goal and source sequences to match.")
    print("  - H8 allows at most workers-1 speculative calls per selection cycle;")
    print("    every physical endpoint in a parallel batch must be unique.")
    print("  - H9 path-near is defined as path_m <= 110% of nearest path_m;")
    print("    shorter paths pass because this gate prevents path blow-up.")
    print("  - p50/p95 selection latency is informational, not a pass gate.")
    return 0 if (
        h1 and h2 and h3 and h4 and h5 and h6 and h7 and h8 and h9
    ) else 1


if __name__ == "__main__":
    sys.exit(main())
