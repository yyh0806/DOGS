#!/usr/bin/env python3
"""诊断 D-yaw-optional 是否真触发 (验证 MIN_GAIN=20 与 v3 utility 权重的匹配).

假设检验:
  v3 utility 选大转向的条件: mixed_visual*(vg_best - vg_robot) > k_time*t_turn
  默认 k_time=14.5, max_vel_theta=0.5, 180°转 t_turn=pi/0.5~6.28
  -> vg差 > 14.5*6.28 ~ 91 (v3 才选大转)
  D-yaw-optional MIN_GAIN=20 << 91 -> v3 选大转时 vg差总>91>20 -> D-yaw-optional 不抑制
  -> D-yaw-optional 可能在当前权重下是死代码

跑法: PYTHONPATH=web python3 tools/diag_yaw_optional.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web"))

from nx_exploration_manager import ExplorationManager


class _GainTracker:
    """Mock visibility_tracker: visual_gain_at 由 gain_fn 控制."""
    def __init__(self, gain_fn):
        self._gain_fn = gain_fn
        self.observations = []

    def visual_gain_at(self, _map, x, y, yaw):
        return self._gain_fn(x, y, yaw)

    def snapshot(self, _map=None):
        return {"visual_coverage_ratio": 0.25}


class _DummyNav:
    def compute_path_to_pose(self, *a, **kw):
        return {"ok": True, "path_length": 1.0, "poses": 4}


def _make_manager():
    m = ExplorationManager(
        navigation_port=_DummyNav(),
        mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        reject_map_edge=False,
    )
    m.utility_mode = "mixed"
    return m


def _run(label, yaw_optional, min_gain, gain_fn):
    if yaw_optional:
        os.environ["GO2W_FRONTIER_YAW_OPTIONAL"] = "1"
        os.environ["GO2W_FRONTIER_YAW_MIN_GAIN"] = str(min_gain)
    else:
        os.environ.pop("GO2W_FRONTIER_YAW_OPTIONAL", None)
    tracker = _GainTracker(gain_fn)
    m = _make_manager()
    m.visibility_tracker = tracker
    cands = [{"x": 2.0, "y": 0.0, "size": 100, "adjacent_wall_count": 0}]
    r = m._optimize_yaw_for_candidates(cands, (0.0, 0.0, 0.0), None)[0]
    yaw_deg = math.degrees(r.get("yaw", 0.0))
    hc = r.get("heading_change", 0.0)
    vg = r.get("visual_gain", 0)
    print(f"  {label:50s} yaw={yaw_deg:6.1f}  hc={hc:.2f}rad  vg={vg}")


def _gain_far_high(robot_yaw_vg, far_yaw_vg):
    """robot_yaw(0) -> robot_yaw_vg; |delta|>2.5rad(>143deg) -> far_yaw_vg; else 线性."""
    def fn(_x, _y, yaw):
        d = abs(((yaw + math.pi) % (2 * math.pi)) - math.pi)
        if d > 2.5:
            return far_yaw_vg
        if d < 0.1:
            return robot_yaw_vg
        return int(robot_yaw_vg + (far_yaw_vg - robot_yaw_vg) * (d / 2.5))
    return fn


def main():
    print("=== v3 默认 (YAW_OPTIONAL 不设) ===")
    _run("v3 vg差=200 (远大)", False, 20, _gain_far_high(0, 200))
    _run("v3 vg差=50 (中)",   False, 20, _gain_far_high(0, 50))
    _run("v3 vg差=15 (小)",   False, 20, _gain_far_high(0, 15))

    print("\n=== YAW_OPTIONAL=1, MIN_GAIN=20 ===")
    _run("OPTIONAL vg差=200 (应保持大转, 200>=20)", True, 20, _gain_far_high(0, 200))
    _run("OPTIONAL vg差=50  (应保持大转, 50>=20)",  True, 20, _gain_far_high(0, 50))
    _run("OPTIONAL vg差=15  (应抑制, 15<20)",       True, 20, _gain_far_high(0, 15))

    print("\n=== 解读 ===")
    print("若 v3 vg差=15 已选小转 (yaw~0): v3 heading penalty 主导, D-yaw-optional 冗余")
    print("若 v3 vg差=200 选大转 + OPTIONAL vg差=200 也大转: MIN_GAIN=20 太松, 不抑制 v3 大转")
    print("D-yaw-optional 真正触发区间: v3 选大转(hc>30deg) 且 vg差 in [MIN_GAIN, 91].")
    print("  若该区间空(MIN_GAIN<91 且 v3 平衡点=91), D-yaw-optional 几乎不触发.")


if __name__ == "__main__":
    main()
