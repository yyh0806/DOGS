"""提示词组装: 分节有序 + 严格变量 + 前缀稳定 + 技能目录注入。"""
from __future__ import annotations

import pytest

from go2w_brain import prompt_assembler
from go2w_brain.registry import SkillCatalog


def test_sections_ordered_and_stable():
    out = prompt_assembler.assemble([], "")
    assert out["sections"] == ["identity", "persona", "safety",
                               "tool_guidance", "skills_catalog"]
    out2 = prompt_assembler.assemble([], "")
    assert out2["system"] == out["system"]  # 确定性: 前缀稳定


def test_variables_replaced():
    out = prompt_assembler.assemble([], "catalog-x",
                                    variables={"model": "deepseek-chat"})
    assert "deepseek-chat" not in out["system"]  # model 未在分节引用
    assert "catalog-x" in out["system"]


def test_unknown_variable_raises():
    with pytest.raises(ValueError, match="未注册变量"):
        prompt_assembler.render("hi {{nope}}", {})


def test_unclosed_variable_raises():
    with pytest.raises(ValueError, match="未闭合"):
        prompt_assembler.render("hi {{oops", {})


def test_tools_sorted_and_passed_through():
    schemas = [{"type": "function", "function": {"name": "b"}},
               {"type": "function", "function": {"name": "a"}}]
    out = prompt_assembler.assemble(schemas, "")
    assert out["tools"] == schemas  # 原样透传 (排序由注册表 names() 负责)


def test_snapshot_message_is_append_style():
    msg = prompt_assembler.render_snapshot_message(
        {"battery_soc": 60.0, "gps": {"available": False}})
    assert msg.startswith("[runtime-context]")
    assert "battery_soc" in msg


def test_skill_catalog_text(skills):
    text = skills.catalog_text()
    assert "status-report" in text
    assert "lake-patrol" in text
    # 目录只含摘要, 不含正文细节 (懒加载纪律)
    assert "五级" not in text
