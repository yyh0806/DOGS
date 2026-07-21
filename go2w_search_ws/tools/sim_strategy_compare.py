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
- mixed:               α·size + β·visual_gain − γ·path_cost.
- mixed + parallel:    same scoring + concurrent Nav2 probe fast-path.

Hypotheses the sim must confirm:
  H1. mixed reaches equal-or-higher coverage than nearest (problem 4).
  H2. mixed visits large distant frontiers earlier than nearest (problem 1).
  H3. mixed + parallel produces identical waypoint choices as mixed when
      every candidate is reachable (parallel is a latency optimisation,
      not a behaviour change).
"""
from __future__ import annotations

from collections import deque
import math
import sys
from pathlib import Path
from types import SimpleNamespace

WEB_DIR = Path(__file__).resolve().parent.parent / "web"
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
    )
    if mixed_weights is not None:
        kwargs.update(mixed_weights)

    sim_tracker = _SimTracker(truth, observed, width, height, resolution)
    kwargs["visibility_tracker"] = sim_tracker

    manager = ExplorationManager(**kwargs)

    goals = []
    total_path = 0.0
    total_turn = 0.0
    terminal_reason = None
    for _ in range(max_iterations):
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
        target_yaw = float(target.get("yaw", 0.0))
        total_turn += abs(math.atan2(
            math.sin(target_yaw - pose[2]),
            math.cos(target_yaw - pose[2])))
        step_len = math.hypot(
            target["x"] - pose[0], target["y"] - pose[1])
        total_path += step_len
        pose[:] = [target["x"], target["y"], target["yaw"]]
        _reveal(truth, observed, width, height, resolution,
                pose[0], pose[1], sensor_radius)
        manager.mark_visited(target)

    free_cells = [i for i, v in enumerate(truth) if v == 0]
    known_free = sum(1 for i in free_cells if observed[i] == 0)
    coverage = known_free / len(free_cells) if free_cells else 0.0
    snapshot = manager.snapshot()
    return {
        "label": label,
        "waypoints": len(goals),
        "coverage_pct": round(coverage * 100.0, 2),
        "total_path_m": round(total_path, 2),
        "total_turn_rad": round(total_turn, 2),
        "max_x_reached": round(max((g[0] for g in goals), default=0.0), 2),
        "max_y_reached": round(max((g[1] for g in goals), default=0.0), 2),
        "plan_probes": snapshot["plan_probes"],
        "plan_rejections": snapshot["plan_rejections"],
        "terminal_reason": terminal_reason or "max_iterations",
        "first_5_goals": [(round(x, 1), round(y, 1)) for x, y in goals[:5]],
    }


def _print_table(results):
    headers = ["mode", "waypoints", "coverage%", "path_m", "turn_rad",
               "max_x", "max_y", "probes", "rejects", "terminal"]
    rows = [[r["label"], r["waypoints"], r["coverage_pct"],
             r["total_path_m"], r["total_turn_rad"],
             r["max_x_reached"], r["max_y_reached"],
             r["plan_probes"], r["plan_rejections"], r["terminal_reason"]]
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
            },
        ),
        _run_scenario(
            "v3",
            utility_mode="mixed",
            parallel_workers=0,
            mixed_weights={
                "mixed_frontier_weight": 0.5,
                "mixed_visual_gain_weight": 1.0,
                "mixed_heading_penalty": 1.0,
                "mixed_wall_bonus": 2.0,
                "yaw_step_deg": 45.0,
                "max_vel_x": 1.5, "max_vel_theta": 1.0,
            },
        ),
    ]
    _print_table(results)

    nearest = results[0]
    mixed = results[1]
    mixed_par = results[2]
    v3 = results[3]

    print("\n=== Hypothesis checks ===")
    h1 = mixed["coverage_pct"] >= nearest["coverage_pct"] - 1.0
    print(f"  H1 mixed coverage >= nearest (±1%): "
          f"{mixed['coverage_pct']} vs {nearest['coverage_pct']} -> "
          f"{'PASS' if h1 else 'FAIL'}")
    h2 = mixed["max_x_reached"] >= nearest["max_x_reached"] - 0.5
    print(f"  H2 mixed reaches as far in X as nearest: "
          f"{mixed['max_x_reached']} vs {nearest['max_x_reached']} -> "
          f"{'PASS' if h2 else 'FAIL'}")
    h3 = (mixed_par["waypoints"] == mixed["waypoints"]
          and abs(mixed_par["coverage_pct"] - mixed["coverage_pct"]) < 0.1)
    print(f"  H3 mixed+parallel matches mixed behaviour: "
          f"wp={mixed_par['waypoints']}/{mixed['waypoints']} "
          f"cov={mixed_par['coverage_pct']}/{mixed['coverage_pct']} -> "
          f"{'PASS' if h3 else 'FAIL'}")
    h4 = v3["coverage_pct"] >= nearest["coverage_pct"] - 1.0
    print(f"  H4 v3 coverage >= nearest (±1%): "
          f"{v3['coverage_pct']} vs {nearest['coverage_pct']} -> "
          f"{'PASS' if h4 else 'FAIL'}")
    h5 = v3["total_turn_rad"] < nearest["total_turn_rad"]
    print(f"  H5 v3 turn_rad < nearest: "
          f"{v3['total_turn_rad']} vs {nearest['total_turn_rad']} -> "
          f"{'PASS' if h5 else 'FAIL'}")

    print("\n=== v3 权重网格搜索 (k_time × δ) ===")
    grid_results = []
    for k_time in [0.5, 1.0, 2.0, 5.0]:
        for delta in [0.0, 1.0, 2.0, 5.0]:
            r = _run_scenario(
                f"k={k_time},δ={delta}",
                utility_mode="mixed", parallel_workers=0,
                mixed_weights={
                    "mixed_frontier_weight": 0.5,
                    "mixed_visual_gain_weight": 1.0,
                    "mixed_heading_penalty": k_time,
                    "mixed_wall_bonus": delta,
                    "yaw_step_deg": 45.0,
                })
            grid_results.append((k_time, delta, r))
    base_cov = results[0]["coverage_pct"]
    candidates_ok = [(k, d, r) for k, d, r in grid_results
                     if r["coverage_pct"] >= base_cov - 1.0]
    recommended = None
    if candidates_ok:
        best = min(candidates_ok, key=lambda t: t[2]["total_turn_rad"])
        recommended = (best[0], best[1])
        print(f"  推荐: k_time={best[0]}, δ={best[1]}, "
              f"turn={best[2]['total_turn_rad']}rad, cov={best[2]['coverage_pct']}%")
    else:
        print("  WARN: 所有 (k_time,δ) 组合 coverage 退步 > 1%, 需调参")
    print(f"  网格 coverage 范围: "
          f"{min(r['coverage_pct'] for _, _, r in grid_results)}%"
          f" ~ {max(r['coverage_pct'] for _, _, r in grid_results)}%")
    print(f"  网格 turn_rad 范围: "
          f"{min(r['total_turn_rad'] for _, _, r in grid_results)}"
          f" ~ {max(r['total_turn_rad'] for _, _, r in grid_results)}")
    if recommended is not None:
        print(f"  RECOMMENDED_K_TIME={recommended[0]}")
        print(f"  RECOMMENDED_DELTA={recommended[1]}")

    print("\n=== Notes ===")
    print("  - coverage is 'ground-truth free cells the sensor revealed'.")
    print("  - path_m is the sum of straight-line leg distances (no Nav2 cost).")
    print("  - H3 may legitimately differ if parallel exposes a candidate")
    print("    order the serial path masks; investigate if FAIL.")
    return 0 if (h1 and h2 and h3) else 1


if __name__ == "__main__":
    sys.exit(main())
