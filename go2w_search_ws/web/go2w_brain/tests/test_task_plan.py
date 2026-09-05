"""M7 任务计划测试: 强校验器 / 规则组合器 / LLM 草稿解析。"""
from __future__ import annotations

import pytest

from go2w_brain.memory import MemoryStore
from go2w_brain.task_plan import (PlanStep, TaskPlan, detect_plan_kind,
                                  parse_draft, rule_compose)
from go2w_brain.tools import BUILTIN_TOOLS
from go2w_brain.registry import ToolRegistry


@pytest.fixture
def registry():
    reg = ToolRegistry()
    for tool in BUILTIN_TOOLS:
        reg.register(tool)
    return reg


@pytest.fixture
def gate(registry):
    from go2w_brain.dispatcher import (DispatchGate, require_mission_lock,
                                       require_water_guard_armed)
    gate = DispatchGate(registry)
    gate.register_precondition("mission_lock", require_mission_lock)
    gate.register_precondition("water_guard_armed",
                               require_water_guard_armed)
    return gate


@pytest.fixture
def memory(tmp_path):
    return MemoryStore(tmp_path / "memory.jsonl")


# ---------- 校验器: 计划也要过安检 -------------------------------------------

def test_valid_patrol_plan_passes(registry, gate, memory, tmp_path):
    plan = rule_compose("lake")
    ok, errors = plan.validate(registry, gate, memory)
    assert ok, errors
    assert plan.state == "valid"
    order = [s.id for s in plan.ordered_steps()]
    assert order.index("g1") > order.index("p1")   # 依赖在前
    assert order.index("f1") > order.index("g1")
    assert order.index("r1") > order.index("s1")


def test_unknown_verb_rejected(registry, gate, memory):
    plan = TaskPlan([PlanStep("x1", "fly_to_moon", {}, [], [])])
    ok, errors = plan.validate(registry, gate, memory)
    assert not ok and any("unknown_verb" in e for e in errors)


def test_unknown_precondition_rejected(registry, gate, memory):
    plan = TaskPlan([PlanStep("f1", "follow_route", {"from_plan": True},
                              ["mission_lock", "invented_guard"], [], [])])
    ok, errors = plan.validate(registry, gate, memory)
    assert not ok and any("unknown_precondition" in e for e in errors)


def test_unknown_dep_and_cycle_rejected(registry, gate, memory):
    plan = TaskPlan([
        PlanStep("a", "get_battery", {}, [], ["ghost"]),
        PlanStep("b", "get_gps", {}, [], ["a"]),
        PlanStep("c", "get_pose", {}, [], ["b"]),
    ])
    ok, errors = plan.validate(registry, gate, memory)
    assert not ok and any("unknown_dep" in e for e in errors)
    cycle = TaskPlan([
        PlanStep("a", "get_battery", {}, [], ["b"]),
        PlanStep("b", "get_gps", {}, [], ["a"]),
    ])
    ok, errors = cycle.validate(registry, gate, memory)
    assert not ok and "cycle_detected" in errors


def test_unresolved_memory_ref_rejected(registry, gate, memory):
    plan = TaskPlan([PlanStep("s1", "scan_water", {"frames": 3},
                              [], [], ["mem-ghost"])])
    ok, errors = plan.validate(registry, gate, memory)
    assert not ok and any("unresolved_memory_ref" in e for e in errors)


def test_resolved_memory_ref_passes(registry, gate, memory, tmp_path):
    entry = memory.record("fp_zone", {"lat": 31.488, "lng": 120.369})
    plan = TaskPlan([PlanStep("s1", "scan_water", {"frames": 3},
                              [], [], [entry["id"]])])
    ok, errors = plan.validate(registry, gate, memory)
    assert ok, errors


# ---------- 规则组合器与任务识别 ----------------------------------------------

def test_detect_plan_kind():
    assert detect_plan_kind("绕湖一周巡查落水人员") == "lake"
    assert detect_plan_kind("绕园区巡查") == "campus"
    assert detect_plan_kind("报告当前状态") == "status"
    assert detect_plan_kind("把门打开") is None


def test_detect_plan_kind_natural_phrasings():
    """M7.3: 自然说法必须识别 (2026-09-04 实测 "绕着当前园区湖绕行一圈"
    未被识别 → 落入自由循环)。"""
    assert detect_plan_kind("绕着当前园区湖绕行一圈") == "lake"
    assert detect_plan_kind("绕湖巡逻") == "lake"
    assert detect_plan_kind("带我去巡湖") == "lake"
    assert detect_plan_kind("绕着园区转一圈") == "campus"
    assert detect_plan_kind("环湖转一圈") == "lake"
    assert detect_plan_kind("绕水塘一圈") is None  # 无"湖"字不误判


def test_rule_compose_shapes():
    lake = rule_compose("lake")
    # 离线规则保底用通用最近水体; 园区湖语义链走 LLM 计划路径 (plan_campus_lake)
    assert [s.verb for s in lake.steps] == [
        "plan_lake_loop", "arm_water_guard", "follow_route",
        "scan_water", "patrol_report"]
    status = rule_compose("status")
    assert {s.verb for s in status.steps} == {
        "get_battery", "get_gps", "get_pose"}


# ---------- LLM 草稿解析 -------------------------------------------------------

def test_parse_draft_plain_json():
    raw = ('{"steps": [{"id": "p1", "verb": "get_battery", "args": {}, '
           '"depends": [], "preconditions": []}]}')
    plan = parse_draft(raw)
    assert plan is not None and plan.steps[0].verb == "get_battery"


def test_parse_draft_with_code_fence_and_noise():
    raw = ("好的，计划如下：\n```json\n{\"steps\": [{\"id\": \"p1\", "
           "\"verb\": \"plan_lake_loop\", \"args\": {\"offset_m\": 20}}]}\n```")
    plan = parse_draft(raw)
    assert plan is not None
    assert plan.steps[0].args == {"offset_m": 20}


def test_parse_draft_garbage_returns_none():
    assert parse_draft("没有计划") is None
    assert parse_draft('{"steps": [{"verb": "x"}]}') is None  # 缺 id/参数非对象
