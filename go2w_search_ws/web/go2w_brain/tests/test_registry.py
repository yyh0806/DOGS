"""注册表: 能力硬边界 + scope 遮蔽 + schema 校验。"""
from __future__ import annotations

import pytest

from go2w_brain.registry import ToolRegistration, ToolRegistry


def _dummy_execute(args, ctx):
    return {"ok": True}


def test_register_and_get():
    reg = ToolRegistry()
    tool = ToolRegistration("probe", "探测", {"type": "object"},
                            _dummy_execute)
    reg.register(tool)
    assert reg.get("probe") is tool
    assert reg.names() == ["probe"]


def test_same_layer_duplicate_rejected():
    reg = ToolRegistry()
    reg.register(ToolRegistration("a", "", {"type": "object"},
                                  _dummy_execute))
    with pytest.raises(ValueError, match="同层重名"):
        reg.register(ToolRegistration("a", "", {"type": "object"},
                                      _dummy_execute))


def test_mission_layer_shadows_global():
    """近者遮蔽 (DSH scope 思想): 任务层同名工具压过全局层。"""
    reg = ToolRegistry()
    global_tool = ToolRegistration("a", "global 版", {"type": "object"},
                                   _dummy_execute)
    mission_tool = ToolRegistration("a", "mission 版", {"type": "object"},
                                    _dummy_execute)
    reg.register(global_tool, scope="global")
    reg.register(mission_tool, scope="mission")
    assert reg.get("a") is mission_tool


def test_validate_args_required_and_type():
    reg = ToolRegistry()
    tool = ToolRegistration(
        "move", "", {"type": "object",
                     "properties": {"dist_m": {"type": "number"}},
                     "required": ["dist_m"]}, _dummy_execute)
    ok, err = reg.validate_args(tool, {})
    assert not ok and "missing_required" in err
    ok, err = reg.validate_args(tool, {"dist_m": "x"})
    assert not ok and "type:dist_m" in err
    ok, err = reg.validate_args(tool, {"dist_m": 1.5})
    assert ok


def test_validate_args_enum():
    reg = ToolRegistry()
    tool = ToolRegistration(
        "scan", "", {"type": "object",
                     "properties": {"mode": {"type": "string",
                                             "enum": ["water", "land"]}}},
        _dummy_execute)
    ok, err = reg.validate_args(tool, {"mode": "sky"})
    assert not ok and "enum" in err
    ok, _ = reg.validate_args(tool, {"mode": "water"})
    assert ok


def test_unknown_fields_tolerated():
    reg = ToolRegistry()
    tool = ToolRegistration("t", "", {"type": "object"}, _dummy_execute)
    ok, _ = reg.validate_args(tool, {"extra": 1})
    assert ok


def test_risk_levels_validated():
    with pytest.raises(ValueError):
        ToolRegistration("x", "", {"type": "object"}, _dummy_execute,
                         risk="nuclear")
