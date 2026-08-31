"""cancel_route — 取消当前 GPS 航线 (安全方向: 停止永远允许, 无需任务锁)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    config = ctx.get("config")
    if config is not None and getattr(config, "dry_run", False):
        return {"ok": True, "dry": True, "note": "干跑模式: 无航线在下发"}
    return ctx["platform"].cancel_gps_route()


TOOL = ToolRegistration(
    name="cancel_route",
    description=("取消当前 GPS 航线并停车。安全方向操作: 不需要任务锁, "
                 "任何时候都允许调用。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="act",
    requires=(),
)
