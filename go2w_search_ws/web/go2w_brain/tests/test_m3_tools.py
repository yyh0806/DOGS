"""M3 安全层测试: 守卫前置 / 恶意航线拦截 / 审批解除 / 干跑照常校验。"""
from __future__ import annotations

import os
from pathlib import Path

_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

from test_m2_tools import TOOLS, _NullLog, _armed_guard  # noqa: E402

from go2w_brain.platform import MockAdapter  # noqa: E402
from nx_water_guard import WaterGuard  # noqa: E402

# 校园基准点周边小环 (~200m): 用作"禁区"
_RING = [(31.4890, 120.3690), (31.4890, 120.3710),
         (31.4910, 120.3710), (31.4910, 120.3690)]


def _ctx(platform, guard, mission_lock="m3", approval_token=None):
    return {"platform": platform, "log": _NullLog(), "config": None,
            "mission_lock": mission_lock, "guard": guard,
            "plan_store": {}, "approval_token": approval_token}


# ---------- 前置: 守卫不在岗, 航线不受理 ------------------------------------


def test_follow_route_requires_armed_guard():
    platform = MockAdapter()
    guard = WaterGuard(approval_token="t")  # 未布防
    result = TOOLS["follow_route"].execute(
        {"waypoints": [{"lat": 31.5, "lon": 120.4}]},
        _ctx(platform, guard))
    assert result["ok"] is False
    assert result["dispatch_denied"] == "water_guard_not_armed"


def test_arm_then_follow_allowed():
    platform = MockAdapter()
    guard = _armed_guard(_RING)
    result = TOOLS["follow_route"].execute(
        {"waypoints": [{"lat": 31.50, "lon": 120.38},   # 距环 ~1.3km
                       {"lat": 31.51, "lon": 120.39}]},
        _ctx(platform, guard))
    assert result["ok"] is True
    assert platform.calls[-1][0] == "submit_gps_route"


# ---------- 恶意航线: 航点落入禁区 → 受理前拦截 ----------------------------


def test_malicious_waypoint_inside_keepout_rejected():
    platform = MockAdapter()
    guard = _armed_guard(_RING)
    result = TOOLS["follow_route"].execute(
        {"waypoints": [{"lat": 31.50, "lon": 120.38},   # 合法
                       {"lat": 31.4900, "lon": 120.3700},  # 禁区正中!
                       {"lat": 31.51, "lon": 120.39}]},
        _ctx(platform, guard))
    assert result["ok"] is False
    assert result["reason"] == "waypoint_in_keepout"
    assert result["waypoint_index"] == 1
    # 一个航点都没下发
    assert not any(c[0] == "submit_gps_route" for c in platform.calls)


def test_malicious_rejection_survives_dry_run():
    """干跑模式拦截照常执行 (安全校验与下发解耦)。"""

    class Cfg:
        dry_run = True

    platform = MockAdapter()
    guard = _armed_guard(_RING)
    ctx = _ctx(platform, guard)
    ctx["config"] = Cfg()
    result = TOOLS["follow_route"].execute(
        {"waypoints": [{"lat": 31.4905, "lon": 120.3705}]},
        ctx)
    assert result["ok"] is False
    assert result["reason"] == "waypoint_in_keepout"


# ---------- arm / disarm / state 工具 ----------------------------------------


def test_arm_water_guard_from_plan():
    platform = MockAdapter()
    guard = WaterGuard(approval_token="t")
    ctx = _ctx(platform, guard)
    # 先规划 (园区点) 再 from_plan 布防
    plan = TOOLS["plan_lake_loop"].execute({}, ctx)
    assert plan["ok"]
    armed = TOOLS["arm_water_guard"].execute({"from_plan": True}, ctx)
    assert armed["ok"] and armed["vertices"] > 3
    assert guard.state()["armed"] is True
    # mock 平台同步: calls 里应有 arm_water_guard
    assert any(c[0] == "arm_water_guard" for c in platform.calls)


def test_disarm_requires_approval():
    platform = MockAdapter()
    guard = WaterGuard(approval_token="tok-7")
    guard.arm(_RING)
    # 无令牌 → 工具直接拒绝 (dispatcher approve 级; 此处给令牌为空)
    result = TOOLS["disarm_water_guard"].execute({}, _ctx(platform, guard))
    assert not result["ok"]
    assert result["reason"] == "approval_token_mismatch"
    assert guard.state()["armed"] is True
    # 令牌错误 → 拒
    result = TOOLS["disarm_water_guard"].execute(
        {}, _ctx(platform, guard, approval_token="wrong"))
    assert not result["ok"]
    # 令牌正确 → 解除
    result = TOOLS["disarm_water_guard"].execute(
        {}, _ctx(platform, guard, approval_token="tok-7"))
    assert result["ok"]
    assert guard.state()["armed"] is False


def test_get_guard_state_tool():
    platform = MockAdapter()
    guard = WaterGuard(approval_token="t")
    result = TOOLS["get_guard_state"].execute({}, _ctx(platform, guard))
    assert result["ok"] and result["armed"] is False
    guard.arm(_RING)
    result = TOOLS["get_guard_state"].execute({}, _ctx(platform, guard))
    assert result["armed"] is True and result["vertices"] == 4
