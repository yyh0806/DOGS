"""patrol_report — 汇总巡逻任务报告 (M5)。

数据源全部来自会话状态: 规划 (plan_store) / 航线进度 (platform) /
检测告警 (detector) / 守卫 (guard) / 电量 (platform)。
输出 markdown 摘要并落轨迹 (event=patrol_report)。
"""
from __future__ import annotations

from typing import Any

from ..patrol_math import route_completion
from ..registry import ToolRegistration


def execute(args: dict[str, Any], ctx: dict[str, Any]) -> dict[str, Any]:
    plan_store = ctx.get("plan_store") or {}
    last = plan_store.get("last_route") or {}
    platform = ctx["platform"]
    snapshot = platform.snapshot()
    route_state = snapshot.get("gps_route") or {}
    completion = route_completion(route_state)
    detector = ctx.get("detector")
    events = detector.events() if detector is not None else []
    confirmed = [e for e in events if e["tier"] == "confirmed"]
    suspects = [e for e in events if e["tier"] == "suspect"]
    guard = ctx.get("guard")
    guard_state = guard.state() if guard is not None else {"armed": False}
    waypoints = last.get("waypoints") or []
    scan_points = last.get("scan_points") or []
    stats = last.get("stats") or {}
    kind = last.get("kind") or "none"
    target = last.get("target") or {}

    lines = ["# 巡逻任务报告", ""]
    lines.append(f"- 目标: {kind}"
                 + (f" ({target.get('area_km2', '?')}km², 距离 "
                    f"{target.get('dist_km', '?')}km)" if target else ""))
    lines.append(f"- 环线: {len(waypoints)} 航点 / "
                 f"{stats.get('length_m', 0) and round(stats['length_m'] / 1000, 2)}km"
                 f" / 闭合={'是' if stats.get('closed') else '否'}")
    lines.append(f"- 扫描点: {len(scan_points)} 个 (间距 "
                 f"{stats.get('scan_spacing_m', 150)}m)")
    lines.append(f"- 航线进度: {route_state.get('waypoint_index', 0)}/"
                 f"{route_state.get('waypoint_total', 0)} "
                 f"({completion * 100:.0f}%)")
    lines.append(f"- 守卫: {'在岗' if guard_state.get('armed') else '未布防'}"
                 f", 违规计数 {guard_state.get('violation_count', 0)}")
    lines.append(f"- 电量: {snapshot.get('battery_soc', '?')}%")
    lines.append(f"- 告警: confirmed {len(confirmed)} / suspect "
                 f"{len(suspects)}")
    for e in confirmed:
        lines.append(f"  - [confirmed] ({e['lat']:.6f}, {e['lng']:.6f}) "
                     f"方位 {e['bearing_deg']}° 距 {e['est_range_m']}m "
                     f"置信 {e['confidence']}")
    report = "\n".join(lines)
    ctx["log"].append("event", event="patrol_report",
                      confirmed=len(confirmed), suspect=len(suspects),
                      completion=round(completion, 3))
    return {"ok": True, "report": report,
            "summary": {"completion": round(completion, 3),
                        "confirmed": len(confirmed),
                        "suspect": len(suspects),
                        "scan_points": len(scan_points)}}


TOOL = ToolRegistration(
    name="patrol_report",
    description=(
        "生成巡逻任务报告: 目标/环线/扫描点/航线进度/守卫状态/电量/"
        "两级告警清单 (confirmed 含 WGS-84 坐标)。任务结束必调用。"),
    parameters={"type": "object", "properties": {}, "required": []},
    execute=execute,
    risk="read",
    requires=(),
)
