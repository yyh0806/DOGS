"""plan_memory — 规划结果的记忆持久化与复用 (M7.1, L1 几何层)。

用户架构 "建图转成记忆" 的直接落地:
- persist_plan: 规划成功后把环线/扫描点/多边形/统计写入 geometry 记忆
  (长半衰期 30 天, 同类同源新规划覆盖旧);
- try_reuse: 规划前检索中心附近的 geometry 记忆, 新鲜且高置信时直接
  复用 —— 二次任务零瓦片/零 Overpass, 离线也可规划。

安全边界: 复用只跳过数据获取, 守卫布防/follow_route 禁区校验照常全链
执行 —— 记忆加速的是"建图", 从不替代安全检查。
"""
from __future__ import annotations

import math
from typing import Any, Optional


def try_reuse(memory, kind: str, lat: float, lng: float,
              min_score: float = 0.8,
              radius_m: float = 300.0) -> Optional[dict[str, Any]]:
    """命中 geometry 记忆 → 重建完整规划结果; 否则 None。"""
    if memory is None:
        return None
    entries = memory.query(float(lat), float(lng), radius_m,
                           kinds=("geometry",), min_score=min_score)
    if not entries:
        return None
    entry = entries[0]
    data = entry.get("data") or {}
    if data.get("plan_kind") != kind:
        return None
    ring = entry.get("geo", {}).get("points")
    if not ring or len(ring) < 8:
        return None
    waypoints = [{"lat": p[0], "lon": p[1], "name": f"wp{i:03d}"}
                 for i, p in enumerate(ring)]
    polygon = data.get("polygon")
    if not polygon:
        return None
    result = {
        "ok": True,
        "kind": kind,
        "waypoints": waypoints,
        "scan_points": data.get("scan_points") or [],
        "target": data.get("target") or {},
        "stats": data.get("stats") or {},
        "anchor": data.get("anchor") or {},
        "memory_id": entry["id"],
        "memory_reused": True,
    }
    result["water_polygon" if kind == "water" else "campus_polygon"] = polygon
    return result


def persist_plan(memory, result: dict[str, Any]) -> Optional[str]:
    """规划结果 → geometry 记忆 (返回条目 id)。"""
    if memory is None or not result.get("ok"):
        return None
    waypoints = result.get("waypoints") or []
    if len(waypoints) < 4:
        return None
    ring = [[w["lat"], w["lon"]] for w in waypoints]
    # 环心近似: 取首点与半程点中点 (免全量求均值)
    mid = waypoints[len(waypoints) // 2]
    lat = (waypoints[0]["lat"] + mid["lat"]) / 2
    lng = (waypoints[0]["lon"] + mid["lon"]) / 2
    entry = memory.record(
        "geometry", {"points": ring},
        data={
            "plan_kind": result.get("kind"),
            "scan_points": result.get("scan_points") or [],
            "target": result.get("target") or {},
            "stats": result.get("stats") or {},
            "polygon": (result.get("water_polygon")
                        or result.get("campus_polygon")),
            "anchor": result.get("anchor") or {},
        },
        confidence=0.9, source="task")
    return entry["id"]
