"""nx_water_guard — 离水/地理围栏看门狗 (纯逻辑核心, 无 ROS 依赖)。

M3 安全层的核心: 给定一个 WGS-84 禁区环 (水域多边形或园区凸包),
对任意当前位置给出三档裁决:
    allow  —— 离禁区足够远, 正常行驶;
    limit  —— 进入缓冲带, 限速 (speed_cap);
    veto   —— 距禁区过近或在禁区内, 必须停车/不允许动车。

设计纪律 (对齐 nx_gps_nav 的纯核心模式):
- 本文件零 ROS 依赖, 全部逻辑可单测;
- fail-closed: 未装载环时 evaluate 返回 not_armed (调用方决定语义),
  裁决只在 armed 后给出;
- 解除必须带审批令牌 (与 go2w_brain.dispatcher 的 approve 风险级联动,
  令牌校验在本类做 —— 单一真相源);
- 线程安全 (evaluate 可能被高频位姿回调调用)。

距离采用环质心处的等距圆柱近似 (局部几百米尺度误差 < 0.5%,
对安全阈值无影响); 多边形取外环, 不处理洞。
"""
from __future__ import annotations

import math
import threading
import time
import uuid
from typing import Any, Optional

# 默认阈值 (米): 与禁区的最小允许距离
DEFAULT_VETO_M = 2.0      # 距禁区 < 2m 或在禁区内 → veto (硬停)
DEFAULT_LIMIT_M = 8.0     # 距禁区 < 8m → limit (限速)
DEFAULT_SPEED_CAP = 0.4   # limit 档的速度上限 (m/s)
_HEARTBEAT_STALE_S = 1.5  # 位姿超龄即按最坏情况处理 (fail-closed)


class WaterGuard:
    """单禁区环的守卫。arm → evaluate... → disarm(审批)。"""

    def __init__(self, veto_m: float = DEFAULT_VETO_M,
                 limit_m: float = DEFAULT_LIMIT_M,
                 speed_cap: float = DEFAULT_SPEED_CAP,
                 approval_token: str = "",
                 monotonic=time.monotonic):
        if not 0.0 < veto_m < limit_m:
            raise ValueError("阈值必须满足 0 < veto_m < limit_m")
        self._veto_m = float(veto_m)
        self._limit_m = float(limit_m)
        self._speed_cap = float(speed_cap)
        self._approval_token = approval_token.strip()
        self._monotonic = monotonic
        self._lock = threading.RLock()
        # M7.1: 多环支持 —— 主禁区 (水域/园区) + 记忆危险带 (hazard)
        # 全部参与裁决, 最小有符号距离取胜 (哪个环近就听哪个)。
        self._rings: list[list[tuple[float, float]]] = []
        self._margin_m = 0.0
        self._armed = False
        self._violation_count = 0
        self._last_verdict: Optional[str] = None
        self._last_min_dist: Optional[float] = None
        self._last_eval_ts: Optional[float] = None
        self._armed_at: Optional[float] = None

    # ---------- 布防 / 解除 --------------------------------------------------

    def arm(self, ring: list[tuple[float, float]],
            margin_m: float = 0.0) -> dict[str, Any]:
        """追加一个禁区环 (WGS-84 [(lat,lng)...], ≥4 点)。

        M7.1 起 arm 是追加语义: 主禁区与记忆危险带逐环叠加, 任一环
        命中即 veto; disarm(审批) 一次性清空全部。
        """
        normalized = _normalize_ring(ring)
        if normalized is None:
            return {"ok": False, "reason": "invalid_ring"}
        margin = min(max(float(margin_m), 0.0), 100.0)
        with self._lock:
            self._rings.append(normalized)
            self._margin_m = margin
            self._armed = True
            self._armed_at = self._monotonic()
            self._violation_count = 0
            self._last_verdict = None
            self._last_min_dist = None
        return {"ok": True, "vertices": len(normalized),
                "rings": len(self._rings),
                "margin_m": margin, "veto_m": self._veto_m,
                "limit_m": self._limit_m}

    def disarm(self, approval_token: str) -> dict[str, Any]:
        """解除布防 (清空全部环)。审批令牌不匹配 → 拒绝 (单一真相源在此)。"""
        with self._lock:
            if not self._armed:
                return {"ok": True, "note": "not_armed"}
            token = str(approval_token or "").strip()
            if not self._approval_token:
                return {"ok": False, "reason": "approval_token_not_configured"}
            if token != self._approval_token:
                return {"ok": False, "reason": "approval_token_mismatch"}
            self._armed = False
            self._rings = []
            return {"ok": True}

    # ---------- 裁决 ---------------------------------------------------------

    def evaluate(self, lat: float, lng: float,
                 fix_age_s: Optional[float] = None) -> dict[str, Any]:
        """对当前位置给出裁决。fail-closed: 位姿超龄 → veto(stale)。"""
        with self._lock:
            self._last_eval_ts = self._monotonic()
            if not self._armed or not self._rings:
                return {"verdict": "not_armed", "min_dist_m": None,
                        "speed_cap": None, "reason": "guard_not_armed"}
            if fix_age_s is not None and fix_age_s > _HEARTBEAT_STALE_S:
                self._violation_count += 1
                self._last_verdict = "veto"
                return {"verdict": "veto", "min_dist_m": None,
                        "speed_cap": 0.0, "reason": "fix_stale"}
            min_dist = min(_distance_to_ring_m(lat, lng, ring)
                           for ring in self._rings)
            effective = min_dist - self._margin_m
            if effective < self._veto_m:  # 含禁区内 (有符号距离为负)
                verdict, cap = "veto", 0.0
                self._violation_count += 1
            elif effective < self._limit_m:
                verdict, cap = "limit", self._speed_cap
            else:
                verdict, cap = "allow", None
            self._last_verdict = verdict
            self._last_min_dist = round(min_dist, 2)
            return {"verdict": verdict, "min_dist_m": round(min_dist, 2),
                    "speed_cap": cap, "reason": None}

    # ---------- 状态 ---------------------------------------------------------

    def state(self) -> dict[str, Any]:
        with self._lock:
            primary = self._rings[0] if self._rings else None
            return {
                "armed": self._armed,
                "rings": len(self._rings),
                "vertices": len(primary) if primary else 0,
                "margin_m": self._margin_m,
                "veto_m": self._veto_m,
                "limit_m": self._limit_m,
                "speed_cap": self._speed_cap,
                "violation_count": self._violation_count,
                "last_verdict": self._last_verdict,
                "last_min_dist_m": self._last_min_dist,
                "armed_at": self._armed_at,
                "last_eval_age_s": (
                    None if self._last_eval_ts is None
                    else round(self._monotonic() - self._last_eval_ts, 2)),
            }

    def ring_copy(self) -> Optional[list[tuple[float, float]]]:
        """主环 (第一个装载) 的只读副本 (未布防返回 None)。"""
        with self._lock:
            return list(self._rings[0]) if self._rings else None

    def distance_m(self, lat: float, lng: float) -> Optional[float]:
        """到全部环的最小有符号距离 (环内为负)。未布防返回 None。"""
        with self._lock:
            if not self._rings:
                return None
            return min(_distance_to_ring_m(float(lat), float(lng), ring)
                       for ring in self._rings)


def new_approval_token() -> str:
    """生成一次性审批令牌 (操作员授权解除时使用)。"""
    return uuid.uuid4().hex[:12]


# ---------- 内部几何 ---------------------------------------------------------

def _normalize_ring(ring) -> Optional[list[tuple[float, float]]]:
    if not isinstance(ring, (list, tuple)) or len(ring) < 4:
        return None
    out = []
    for pt in ring:
        if not isinstance(pt, (list, tuple)) or len(pt) != 2:
            return None
        lat, lng = float(pt[0]), float(pt[1])
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lng <= 180.0):
            return None
        if not (math.isfinite(lat) and math.isfinite(lng)):
            return None
        out.append((lat, lng))
    if out[0] == out[-1]:
        out = out[:-1]
    if len(out) < 3:
        return None
    return out


def _point_in_ring(lat: float, lng: float, ring) -> bool:
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        lat_i, lng_i = ring[i]
        lat_j, lng_j = ring[j]
        if (lng_i > lng) != (lng_j > lng):
            cross = (lat_j - lat_i) * (lng - lng_i) / (lng_j - lng_i) + lat_i
            if lat < cross:
                inside = not inside
        j = i
    return inside


def _distance_to_ring_m(lat: float, lng: float, ring) -> float:
    """有符号距离: 环内为负 (深度), 环外为到最近边的距离 (米)。"""
    if _point_in_ring(lat, lng, ring):
        # 环内: 到最近边的距离取负 (进入深度)
        return -_min_edge_distance_m(lat, lng, ring)
    return _min_edge_distance_m(lat, lng, ring)


def _min_edge_distance_m(lat: float, lng: float, ring) -> float:
    ref_lat = sum(p[0] for p in ring) / len(ring)
    kx = 111320.0 * math.cos(math.radians(ref_lat))
    ky = 110540.0
    x = lng * kx
    y = lat * ky
    best = float("inf")
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i][1] * kx, ring[i][0] * ky
        x2, y2 = ring[(i + 1) % n][1] * kx, ring[(i + 1) % n][0] * ky
        dx, dy = x2 - x1, y2 - y1
        seg2 = dx * dx + dy * dy or 1e-9
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg2))
        px, py = x1 + t * dx, y1 + t * dy
        best = min(best, math.hypot(x - px, y - py))
    return best
