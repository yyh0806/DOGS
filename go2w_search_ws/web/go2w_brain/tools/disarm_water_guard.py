"""disarm_water_guard — 解除禁区守卫 (approve 级: 必须审批令牌)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    guard = ctx.get("guard")
    if guard is None:
        return {"ok": False, "reason": "guard_unavailable"}
    token = ctx.get("approval_token") or ""
    result = guard.disarm(token)
    if result.get("ok"):
        ctx["log"].append("event", event="water_guard_disarmed")
        platform = ctx.get("platform")
        config = ctx.get("config")
        dry = config is not None and getattr(config, "dry_run", False)
        if not dry and hasattr(platform, "disarm_water_guard"):
            try:
                platform.disarm_water_guard(str(token))
            except Exception:  # noqa: BLE001 — 远端失败不回滚本地 (保守)
                pass
    return result


TOOL = ToolRegistration(
    name="disarm_water_guard",
    description=(
        "解除离水守卫。高危操作: 需要操作员审批令牌 (approval_token), "
        "dispatcher 在无令牌时直接拒绝。正常巡逻任务全程不需要解除。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="approve",
    requires=("mission_lock",),
)
