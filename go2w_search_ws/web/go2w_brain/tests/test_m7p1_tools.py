"""M7.1 测试: hazard 自动布防 / 几何持久化复用 / 计划失败重规划。"""
from __future__ import annotations

import os
from pathlib import Path

_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

import pytest  # noqa: E402

from go2w_brain.memory import MemoryStore  # noqa: E402
from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402
from nx_water_guard import WaterGuard  # noqa: E402

TOOLS = {t.name: t for t in BUILTIN_TOOLS}


class _NullLog:
    def append(self, kind, **fields):
        return None


def _ctx(platform=None, guard=None, memory=None, plan_store=None,
         mission_lock="m71", config=None):
    return {"platform": platform or MockAdapter(), "log": _NullLog(),
            "config": config, "mission_lock": mission_lock,
            "guard": guard, "memory": memory,
            "plan_store": plan_store if plan_store is not None else {},
            "detector": None, "frame_source": None,
            "approval_token": "tok"}


@pytest.fixture
def memory(tmp_path):
    return MemoryStore(tmp_path / "mem.jsonl")


def _plan_lake(ctx):
    plan = TOOLS["plan_lake_loop"].execute({}, ctx)
    assert plan["ok"], plan
    return plan


# ---------- hazard 自动布防 ---------------------------------------------------

def test_hazard_auto_armed_on_from_plan(memory):
    """记忆中的 hazard → arm_water_guard(from_plan) 确定性自动布防。"""
    ctx = _ctx(guard=WaterGuard(approval_token="tok"), memory=memory)
    _plan_lake(ctx)  # 填 plan_store (规划区域=园区水塘)
    memory.record("hazard", {"lat": 31.4890, "lng": 120.3693},
                  data={"kind_note": "陡岸"}, confidence=0.8)
    result = TOOLS["arm_water_guard"].execute({"from_plan": True}, ctx)
    assert result["ok"]
    assert result["hazards_armed"] == 1
    assert result["rings"] == 2  # 主禁区 + hazard 环
    # hazard 点现在在守卫 veto 带内
    guard = ctx["guard"]
    verdict = guard.evaluate(31.4890, 120.3693)
    assert verdict["verdict"] == "veto"


def test_hazard_ring_blocks_follow_route(memory):
    """记忆 hazard 自动布防后, 穿 hazard 的航线在受理期被拦。"""
    platform = MockAdapter()
    ctx = _ctx(platform=platform, guard=WaterGuard(approval_token="tok"),
               memory=memory)
    plan = _plan_lake(ctx)
    # hazard 放在航点正上方 (环线必经之处)
    wp = plan["first_waypoints"][1]
    memory.record("hazard", {"lat": wp["lat"], "lng": wp["lon"]},
                  data={}, confidence=0.9)
    TOOLS["arm_water_guard"].execute({"from_plan": True}, ctx)
    result = TOOLS["follow_route"].execute({"from_plan": True}, ctx)
    # 航点距 hazard 圈中心 0m < veto(2)+margin(2) → 受理期拦截
    assert result["ok"] is False
    assert result["reason"] == "waypoint_in_keepout"


def test_no_hazard_no_extra_rings(memory):
    ctx = _ctx(guard=WaterGuard(approval_token="tok"), memory=memory)
    _plan_lake(ctx)
    result = TOOLS["arm_water_guard"].execute({"from_plan": True}, ctx)
    assert result["hazards_armed"] == 0
    assert result["rings"] == 1


# ---------- 几何持久化与复用 -------------------------------------------------

def test_geometry_persisted_then_reused(memory):
    """二次规划命中 geometry 记忆: 免瓦片复用, 结果一致。"""
    ctx = _ctx(guard=WaterGuard(approval_token="tok"), memory=memory)
    first = _plan_lake(ctx)
    assert first["ok"]
    entries = memory.query(31.488, 120.369, 2000.0,
                           kinds=("geometry",), min_score=0.0)
    assert entries, "首次规划应持久化 geometry"
    # 第二次: 清空环境变量模拟"完全离线", 应命中记忆复用
    second = TOOLS["plan_lake_loop"].execute({}, ctx)
    assert second["ok"]
    assert second["waypoint_count"] == first["waypoint_count"]
    # 复用事件进轨迹 (用真 log 不便, 这里检查 plan_store 结果的标记)
    reused = ctx["plan_store"]["last_route"].get("memory_reused")
    assert reused is True
    assert ctx["plan_store"]["last_route"]["memory_id"] == entries[0]["id"]


def test_geometry_reuse_respects_kind(memory):
    """绕湖的 geometry 不能喂给绕园区 (plan_kind 校验)。"""
    ctx = _ctx(guard=WaterGuard(approval_token="tok"), memory=memory)
    _plan_lake(ctx)
    ctx["plan_store"]["last_route"] = {}
    campus = TOOLS["plan_campus_loop"].execute({}, ctx)
    assert campus["ok"]
    # campus 走真实 Overpass 缓存路径 (未复用湖的 geometry)
    assert not ctx["plan_store"]["last_route"].get("memory_reused")


# ---------- 多环守卫 ----------------------------------------------------------

def test_multi_ring_min_distance_semantics():
    guard = WaterGuard(approval_token="t")
    far = [(31.4000, 120.5000), (31.4000, 120.5020),
           (31.4020, 120.5020), (31.4020, 120.5000)]
    near = [(31.4890, 120.3690), (31.4890, 120.3710),
            (31.4910, 120.3710), (31.4910, 120.3690)]
    guard.arm(far)
    guard.arm(near)
    assert guard.state()["rings"] == 2
    assert guard.state()["vertices"] == 4  # 主环 (far, 首个) 顶点数
    # 近环内 → veto (min 取负距离)
    assert guard.evaluate(31.4900, 120.3700)["verdict"] == "veto"
    # 远环内但近环外 → 仍 veto (min 跨环)
    assert guard.evaluate(31.4010, 120.5010)["verdict"] == "veto"
    guard.disarm("t")
    assert guard.state()["rings"] == 0
    assert guard.evaluate(31.4010, 120.5010)["verdict"] == "not_armed"
