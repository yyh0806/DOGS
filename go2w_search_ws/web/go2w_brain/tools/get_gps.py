"""get_gps — 读取当前 GNSS 定位 (WGS-84)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    snapshot = ctx["platform"].snapshot()
    gps = snapshot.get("gps") or {}
    if not gps.get("available"):
        return {"ok": False, "reason": gps.get("reason", "no_fix"),
                "gps_route": snapshot.get("gps_route") or {}}
    return {"ok": True, "lat": gps["lat"], "lng": gps["lng"],
            "hdop": gps.get("hdop"), "sats": gps.get("sats"),
            "quality": gps.get("quality"), "fix_age_s": gps.get("fix_age_s")}


TOOL = ToolRegistration(
    name="get_gps",
    description=(
        "读取当前 GNSS 定位: WGS-84 经纬度、HDOP、卫星数、质量、数据新鲜度。"
        "无定位时返回 {\"ok\": false, \"reason\": ...}。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
