"""push_alert — 落水告警推送 (M7.2, 经 NX 服务器 WS 广播给操作员)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    lat = args.get("lat")
    lng = args.get("lng")
    if not isinstance(lat, (int, float)) or not isinstance(lng, (int, float)):
        return {"ok": False, "reason": "invalid_alert_position"}
    alert = {
        "lat": float(lat), "lng": float(lng),
        "tier": args.get("tier", "confirmed"),
        "note": str(args.get("note", "")),
    }
    platform = ctx.get("platform")
    if not hasattr(platform, "post_alert"):
        return {"ok": False, "reason": "alert_channel_unavailable"}
    result = platform.post_alert(alert)
    ctx["log"].append("event", event="alert_pushed",
                      tier=alert["tier"], lat=alert["lat"], lng=alert["lng"])
    return {"ok": bool(result.get("ok", False)), **result}


TOOL = ToolRegistration(
    name="push_alert",
    description=(
        "把一条落水告警推送给操作员 (WS 广播到控制台/界面): 传 lat/lng 与"
        "tier (suspect/confirmed)。confirmed 告警应同时用 speak 播报。"),
    parameters={"type": "object",
                "properties": {
                    "lat": {"type": "number"}, "lng": {"type": "number"},
                    "tier": {"type": "string",
                             "enum": ["suspect", "confirmed"]},
                    "note": {"type": "string"}},
                "required": ["lat", "lng"]},
    execute=execute,
    risk="act",
    requires=("mission_lock",),
)
