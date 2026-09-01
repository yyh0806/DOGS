"""record_observation — 写入语义记忆 (M7, 经验层)。"""
from __future__ import annotations

from typing import Any

from ..memory import ENTRY_KINDS
from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    memory = ctx.get("memory")
    if memory is None:
        return {"ok": False, "reason": "memory_unavailable"}
    kind = args.get("kind")
    if kind not in ENTRY_KINDS:
        return {"ok": False, "reason": "invalid_kind",
                "allowed": list(ENTRY_KINDS)}
    try:
        entry = memory.record(kind, args["geo"],
                              data=args.get("data") or {},
                              confidence=float(args.get("confidence", 0.6)),
                              source="observation")
    except ValueError as exc:
        return {"ok": False, "reason": f"invalid_geo:{exc}"}
    ctx["log"].append("event", event="memory_recorded",
                      memory_id=entry["id"], mem_kind=kind)
    return {"ok": True, "id": entry["id"], "kind": entry["kind"],
            "confidence": entry["confidence"]}


TOOL = ToolRegistration(
    name="record_observation",
    description=(
        "把一条机器人观测写入语义记忆 (地图式经验层): kind 取值 "
        "passable(可走)/blocked(堵)/vantage(观察点)/hazard(危险)/"
        "fp_zone(误报区)/detection(检测历史); geo 为 {\"lat\",\"lng\"} 点或 "
        "{\"points\":[[lat,lng]...]} 段; confidence 0.05-1.0。"
        "同类同源邻近的新观测自动覆盖旧假设。巡逻中发现的一切值得"
        "下次参考的事实, 都应写入。"),
    parameters={"type": "object",
                "properties": {
                    "kind": {"type": "string", "enum": list(ENTRY_KINDS)},
                    "geo": {"type": "object"},
                    "data": {"type": "object"},
                    "confidence": {"type": "number"}},
                "required": ["kind", "geo"]},
    execute=execute,
    risk="act",
    requires=("mission_lock",),
)
