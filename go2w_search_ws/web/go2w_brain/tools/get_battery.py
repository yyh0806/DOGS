"""get_battery — 读取电量 (SoC 百分比)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    snapshot = ctx["platform"].snapshot()
    soc = snapshot.get("battery_soc")
    if soc is None:
        return {"ok": False, "reason": "battery_soc_unavailable"}
    return {"ok": True, "battery_soc": float(soc)}


TOOL = ToolRegistration(
    name="get_battery",
    description=("读取电池电量 (SoC, 0-100)。未知时返回 "
                 "{\"ok\": false, \"reason\": \"battery_soc_unavailable\"}。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
