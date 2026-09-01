"""get_memory — 检索语义记忆 (M7, 经验层)。"""
from __future__ import annotations

from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    memory = ctx.get("memory")
    if memory is None:
        return {"ok": False, "reason": "memory_unavailable"}
    lat = args.get("lat")
    lng = args.get("lng")
    if lat is None or lng is None:
        gps = ctx["platform"].snapshot().get("gps") or {}
        if not gps.get("available"):
            return {"ok": False, "reason": "no_gps_and_no_explicit_center"}
        lat, lng = gps["lat"], gps["lng"]
    radius = min(max(float(args.get("radius_m", 500.0)), 50.0), 5000.0)
    kinds = tuple(args["kinds"]) if args.get("kinds") else None
    entries = memory.query(float(lat), float(lng), radius, kinds=kinds,
                           min_score=float(args.get("min_score", 0.2)))
    return {"ok": True, "count": len(entries),
            "summary": memory.summary(),
            "entries": [{k: e[k] for k in
                         ("id", "kind", "geo", "data", "source",
                          "confidence", "dist_m", "score", "age_days")}
                        for e in entries]}


TOOL = ToolRegistration(
    name="get_memory",
    description=(
        "检索语义记忆 (地图式经验层): 以某点为中心取半径内的历史观测"
        "(可走/堵点/观察点/危险/误报区/检测历史), 按 置信×新鲜度 评分排序。"
        "任务规划前用它了解'这块地方过去什么样'; 默认中心=当前 GNSS。"),
    parameters={"type": "object",
                "properties": {
                    "lat": {"type": "number"}, "lng": {"type": "number"},
                    "radius_m": {"type": "number"},
                    "kinds": {"type": "array"},
                    "min_score": {"type": "number"}},
                "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
