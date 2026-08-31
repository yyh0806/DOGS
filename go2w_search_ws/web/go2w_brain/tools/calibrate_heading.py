"""calibrate_heading — 北向标定 (M2: 写入 map 系与真北夹角)。"""
from __future__ import annotations

import math
from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    heading = args["heading_deg"]
    if not isinstance(heading, (int, float)) or not math.isfinite(heading):
        return {"ok": False, "reason": "invalid_heading"}
    config = ctx.get("config")
    if config is not None and getattr(config, "dry_run", False):
        return {"ok": True, "dry": True, "heading_deg": float(heading) % 360.0}
    return ctx["platform"].calibrate_heading(float(heading))


TOOL = ToolRegistration(
    name="calibrate_heading",
    description=(
        "写入北向标定 (map 坐标系 x 轴与真北的夹角, 度)。每场地一次, "
        "标定前服务器会拒绝受理任何航线 (follow_route 会收到 "
        "heading_not_calibrated)。"),
    parameters={"type": "object",
                "properties": {"heading_deg": {"type": "number"}},
                "required": ["heading_deg"]},
    execute=execute,
    risk="act",
    requires=(),
)
