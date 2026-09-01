"""patrol_math — 巡逻语义的纯计算 (M5): 续航模型 / 完成度 / 接近点。

续航模型 (粗, M6 实机标定): Go2 巡航 ~1.0m/s, 能耗 ~4%/km (含云台扫描
停顿的等效里程上浮 25%)。判决: 余量 - 预计消耗 ≥30 → GO; ≥15 → SEGMENT
(需分段+返航点); 否则 REFUSE。
"""
from __future__ import annotations

import math
from typing import Any, Optional

SPEED_MPS = 1.0
ENERGY_PCT_PER_KM = 4.0
STOP_OVERHEAD = 1.25      # 扫描停顿等效里程上浮
RESERVE_GO = 30.0         # 跑完仍应剩余
RESERVE_SEGMENT = 15.0


def endurance_check(length_m: float, battery_soc: Optional[float],
                    ) -> dict[str, Any]:
    """续航判决。电量未知 → unknown (保守放行给人工确认)。"""
    if battery_soc is None:
        return {"verdict": "unknown", "reason": "battery_soc_unavailable",
                "est_pct": None}
    length_km = max(0.0, float(length_m)) / 1000.0
    est_pct = round(length_km * ENERGY_PCT_PER_KM * STOP_OVERHEAD, 1)
    remaining = float(battery_soc) - est_pct
    eta_min = round(length_km * 1000.0 / SPEED_MPS / 60.0
                    * STOP_OVERHEAD, 1)
    if remaining >= RESERVE_GO:
        verdict = "GO"
    elif remaining >= RESERVE_SEGMENT:
        verdict = "SEGMENT"
    else:
        verdict = "REFUSE"
    return {"verdict": verdict, "est_pct": est_pct,
            "battery_soc": float(battery_soc),
            "remaining_after": round(remaining, 1),
            "eta_min": eta_min}


def route_completion(route_state: dict[str, Any]) -> float:
    """航线完成度 0..1 (waypoint_index/total; 首点即 0)。"""
    total = route_state.get("waypoint_total") or 0
    index = route_state.get("waypoint_index") or 0
    if total <= 0:
        return 0.0
    return min(1.0, max(0.0, index / max(1, total - 1)))


def nearest_route_point(waypoints: list[dict[str, Any]], lat: float,
                        lng: float) -> tuple[int, float]:
    """距 (lat,lng) 最近的航点 → (index, 距离米)。"""
    kx = 111320.0 * math.cos(math.radians(lat))
    ky = 110540.0
    best_i, best_d = 0, float("inf")
    for i, wp in enumerate(waypoints):
        d = math.hypot((wp["lon"] - lng) * kx, (wp["lat"] - lat) * ky)
        if d < best_d:
            best_i, best_d = i, d
    return best_i, round(best_d, 1)
