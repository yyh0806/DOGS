"""scan_water — 湖面扫描检测落水者 (M4, 驱动检测管线跑一组帧; M7 记忆回写)。"""
from __future__ import annotations

import json
from typing import Any

from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    detector = ctx.get("detector")
    source = ctx.get("frame_source")
    if detector is None or source is None:
        return {"ok": False, "reason": "detector_unavailable",
                "hint": "检测引擎未随会话装载 (NX 生产由 nx_ai_node 提供)"}
    frames = args.get("frames", 3)
    frames = min(max(int(frames), 1), 6)
    events: list[dict[str, Any]] = []
    errors = 0
    for _ in range(frames):
        try:
            frame, robot = source()
        except Exception:  # noqa: BLE001 — 帧源抖动容忍, 记错误数
            errors += 1
            continue
        events += detector.process_frame(frame, robot)
    confirmed = [e for e in events if e["tier"] == "confirmed"]
    # M7 回写: confirmed 检测历史入记忆; 读误报区/堵点经验作提示
    memory = ctx.get("memory")
    memory_note = None
    if memory is not None:
        gps = ctx["platform"].snapshot().get("gps") or {}
        if gps.get("available"):
            relevant = memory.query(gps["lat"], gps["lng"], 300.0,
                                    kinds=("fp_zone", "blocked"),
                                    min_score=0.25)
            if relevant:
                memory_note = [
                    f"{e['kind']} score={e['score']} dist={e['dist_m']}m "
                    f"data={json.dumps(e.get('data'), ensure_ascii=False)}"
                    for e in relevant[:3]]
        for event in confirmed:
            try:
                entry = memory.record(
                    "detection", {"lat": event["lat"], "lng": event["lng"]},
                    data={"bearing_deg": event["bearing_deg"],
                          "est_range_m": event["est_range_m"],
                          "confidence": event["confidence"]},
                    confidence=0.8)
                ctx["log"].append("event", event="memory_recorded",
                                  memory_id=entry["id"],
                                  mem_kind="detection")
            except ValueError:
                pass
    ctx["log"].append("event", event="scan_water",
                      frames=frames, new_events=len(events),
                      confirmed=len(confirmed), frame_errors=errors)
    return {
        "ok": True, "frames_scanned": frames, "frame_errors": errors,
        "new_events": [
            {"tier": e["tier"], "lat": round(e["lat"], 6),
             "lng": round(e["lng"], 6), "bearing_deg": e["bearing_deg"],
             "est_range_m": e["est_range_m"],
             "confidence": e["confidence"]} for e in events],
        "confirmed_count": len(confirmed),
        "stats": detector.stats(),
        "memory_note": memory_note,
        "hint": ("发现 confirmed 落水告警! 位置见 new_events, "
                 "可 approach_vantage 接近确认 (M5)" if confirmed else None),
    }


TOOL = ToolRegistration(
    name="scan_water",
    description=(
        "扫描湖面检测落水人员: 连续采样数帧 (默认 3) 跑五级检测管线"
        "(检出→水域上下文→VLM语义→多帧时序→定位), 返回新告警。"
        "两级: suspect(疑似, 不停步) / confirmed(确认, 含 WGS-84 坐标)。"
        "巡逻中每到扫描点调用一次; 发现 confirmed 后按技能方法论处置。"),
    parameters={"type": "object",
                "properties": {"frames": {"type": "number"}},
                "required": []},
    execute=execute,
    risk="act",
    requires=("mission_lock",),
)
