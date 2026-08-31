"""load_skill — 按名加载技能全文 (DSH 懒加载的模型接口)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    skills = ctx.get("skills")
    if skills is None:
        return {"ok": False, "reason": "skill_catalog_unavailable"}
    name = str(args.get("name", ""))
    skill = skills.load(name)
    if skill is None:
        available = [s["name"] for s in skills.summaries()]
        return {"ok": False, "reason": "unknown_skill",
                "available": available}
    return {"ok": True, "name": skill.name,
            "content": skills.render_skill_content(skill)}


TOOL = ToolRegistration(
    name="load_skill",
    description=(
        "按名加载一份技能的完整方法论 (目录见系统提示词)。任务命中某技能"
        "的适用场景时, 先加载全文再按其流程行动。未知名返回可用列表。"),
    parameters={"type": "object",
                "properties": {"name": {"type": "string"}},
                "required": ["name"]},
    execute=execute,
    risk="read",
    requires=(),
)
