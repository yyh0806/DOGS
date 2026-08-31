"""get_guard_state — 读离水守卫状态 (M3)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    guard = ctx.get("guard")
    if guard is None:
        return {"ok": False, "reason": "guard_unavailable"}
    return {"ok": True, **guard.state()}


TOOL = ToolRegistration(
    name="get_guard_state",
    description=("读离水守卫状态: 是否布防/违规计数/最近裁决与距离。"
                 "巡逻期间用它确认守卫在岗。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
