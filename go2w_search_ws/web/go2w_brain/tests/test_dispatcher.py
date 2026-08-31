"""派发安检: 未知动词拒绝 / 风险分级 / 前置 fail-closed。

这是整个 M1 最核心的安全性质测试 —— 大脑永远无法幻觉出一个动作。
"""
from __future__ import annotations

import pytest

from go2w_brain.dispatcher import DispatchGate
from go2w_brain.registry import ToolRegistration


def _exec(args, ctx):
    return {"ok": True}


def test_unknown_tool_rejected(gate):
    """没有注册的动词 = 硬边界: 拒绝, 而不是执行。"""
    ok, reason, tool = gate.check("open_door", {}, {})
    assert not ok
    assert reason == "unknown_tool"
    assert tool is None


def test_read_tool_passes(gate):
    ok, reason, tool = gate.check("get_gps", {}, {})
    assert ok and tool is not None


def test_approve_tool_requires_token(gate):
    reg_ = gate._registry
    reg_.register(ToolRegistration("disarm_guard", "", {"type": "object"},
                                   _exec, risk="approve"))
    ok, reason, _ = gate.check("disarm_guard", {}, {})
    assert not ok and reason == "approval_required"
    ok, reason, _ = gate.check("disarm_guard", {},
                               {"approval_token": "t-123"})
    assert ok


def test_missing_precondition_fails_closed(gate):
    """声明了前置但没人实现 → 视为不安全, 拒绝。"""
    reg_ = gate._registry
    reg_.register(ToolRegistration("route_stub_fp", "", {"type": "object"},
                                   _exec, risk="act",
                                   requires=("arm_water_guard",)))
    ok, reason, _ = gate.check("route_stub_fp", {}, {})
    assert not ok and "missing_precondition:arm_water_guard" in reason


def test_precondition_gate(gate):
    reg_ = gate._registry
    reg_.register(ToolRegistration("route_stub_pg", "", {"type": "object"},
                                   _exec, risk="act",
                                   requires=("arm_water_guard",)))
    armed = {"armed": False}

    def arm_precondition(tool, ctx):
        return (True, "") if armed["armed"] else (
            False, "water_guard_not_armed")

    gate.register_precondition("arm_water_guard", arm_precondition)
    ok, reason, _ = gate.check("route_stub_pg", {}, {})
    assert not ok and reason == "water_guard_not_armed"
    armed["armed"] = True
    ok, _, _ = gate.check("route_stub_pg", {}, {})
    assert ok


def test_mission_lock_precondition(gate):
    reg_ = gate._registry
    reg_.register(ToolRegistration("speak_lock_test", "",
                                   {"type": "object"}, _exec, risk="act",
                                   requires=("mission_lock",)))
    ok, reason, _ = gate.check("speak_lock_test", {}, {})
    assert not ok and reason == "mission_lock_required"
    ok, _, _ = gate.check("speak_lock_test", {},
                          {"mission_lock": "m1"})
    assert ok


def test_invalid_args_rejected(gate):
    ok, reason, _ = gate.check("speak", {}, {})
    assert not ok and "missing_required" in reason
