"""memory — 语义记忆库 (M7, 经验层 L2): 地图式、geo 键控、可审计。

设计 (对应用户决策: 地图式记忆, 经验层起步):
- 条目 = {id, kind, geo, data, source, confidence, ts};
  geo 为点 {"lat","lng"} 或段 {"points":[[lat,lng]...]};
- 存储 = append-only jsonl + 网格索引 (cell≈0.001°≈110m), 重启重建;
- 冲突消解: 同类、同单元、同源的邻近观测 → 新观测覆盖旧假设
  (新 > 旧); 跨源不覆盖 (OSM 假设 vs 机器人观测分层);
- 检索: 中心+半径 → 网格候选 → 评分 = 置信 × exp(-年龄/半衰期),
  按评分降序; 过龄条目只在显式请求时出现。

半衰期 (天, M7 初值; 实机标定): hazard 7 / blocked 3 /
passable 14 / vantage 14 / fp_zone 3 / detection 7。
"""
from __future__ import annotations

import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Optional

ENTRY_KINDS = ("passable", "blocked", "vantage", "hazard", "fp_zone",
               "detection", "geometry")
HALFLIFE_DAYS = {"passable": 14.0, "blocked": 3.0, "vantage": 14.0,
                 "hazard": 7.0, "fp_zone": 3.0, "detection": 7.0,
                 "geometry": 30.0}  # L1 几何层: 长半衰期, 新规划覆盖旧
_CELL_DEG = 0.001  # ≈110m
_CONFLICT_RADIUS_M = 60.0
_SOURCE_HIERARCHY = {"osd": 0, "osm": 0, "observation": 1, "task": 1}


class MemoryStore:
    def __init__(self, path: Path, monotonic=time.time,
                 clock=time.time):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._monotonic = monotonic
        self._clock = clock
        self._lock = threading.RLock()
        self._entries: dict[str, dict[str, Any]] = {}
        self._grid: dict[tuple[int, int], set[str]] = {}
        self._load()

    # ---------- 写入 ---------------------------------------------------------

    def record(self, kind: str, geo: dict[str, Any], data: Optional[dict] = None,
               confidence: float = 0.6,
               source: str = "observation") -> dict[str, Any]:
        if kind not in ENTRY_KINDS:
            raise ValueError(f"未知记忆种类: {kind}")
        entry_geo = _normalize_geo(geo)
        if entry_geo is None:
            raise ValueError("非法 geo: 需 {\"lat\",\"lng\"} 或 {\"points\":[...]}")
        conf = min(1.0, max(0.05, float(confidence)))
        with self._lock:
            # 冲突消解: 同源同类邻近 → 覆盖 (新观测胜旧假设)
            for eid in list(self._grid.get(self._cell(entry_geo), ())):
                old = self._entries[eid]
                if (old["kind"] == kind
                        and _source_rank(old["source"]) == _source_rank(source)
                        and _geo_dist_m(entry_geo, old["geo"]) < _CONFLICT_RADIUS_M):
                    old.update({"geo": entry_geo, "data": data or old.get("data"),
                                "confidence": conf, "ts": self._clock(),
                                "updated_by": old.get("id"),
                                "source": source})
                    self._flush()
                    return dict(old)
            entry = {"id": uuid.uuid4().hex[:12], "kind": kind,
                     "geo": entry_geo, "data": data or {}, "source": source,
                     "confidence": conf, "ts": self._clock()}
            self._entries[entry["id"]] = entry
            self._grid.setdefault(self._cell(entry_geo), set()).add(entry["id"])
            self._flush()
            return dict(entry)

    # ---------- 检索 ---------------------------------------------------------

    def query(self, lat: float, lng: float, radius_m: float,
              kinds: Optional[tuple[str, ...]] = None,
              min_score: float = 0.2, limit: int = 20) -> list[dict[str, Any]]:
        kinds = tuple(kinds) if kinds else ENTRY_KINDS
        now = self._clock()
        scored = []
        for cell in _cells_in_radius(lat, lng, radius_m):
            for eid in list(self._grid.get(cell, ())):
                entry = self._entries[eid]
                if entry["kind"] not in kinds:
                    continue
                dist = _geo_dist_m({"lat": lat, "lng": lng}, entry["geo"])
                if dist > radius_m:
                    continue
                halflife = HALFLIFE_DAYS.get(entry["kind"], 7.0)
                age_days = (now - entry["ts"]) / 86400.0
                score = entry["confidence"] * math.exp(
                    -age_days * math.log(2) / halflife)
                if score < min_score:
                    continue
                out = dict(entry)
                out.update({"dist_m": round(dist, 1), "score": round(score, 3),
                            "age_days": round(age_days, 1)})
                scored.append(out)
        scored.sort(key=lambda e: -e["score"])
        return scored[:limit]

    def get(self, entry_id: str) -> Optional[dict[str, Any]]:
        with self._lock:
            entry = self._entries.get(entry_id)
            return dict(entry) if entry else None

    def entries(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(e) for e in self._entries.values()]

    def summary(self) -> dict[str, Any]:
        with self._lock:
            counts: dict[str, int] = {}
            for e in self._entries.values():
                counts[e["kind"]] = counts.get(e["kind"], 0) + 1
            return {"total": len(self._entries), "by_kind": counts,
                    "path": str(self.path)}

    # ---------- 持久化 -------------------------------------------------------

    def _flush(self):
        # 规模小 (<1k 条目): 原子重写整个 jsonl (追加语义由日志轨迹另行保留)
        with open(self.path, "w", encoding="utf-8") as fp:
            for entry in self._entries.values():
                fp.write(json.dumps(entry, ensure_ascii=False,
                                    default=str) + "\n")

    def _load(self):
        if not self.path.exists():
            return
        for line in self.path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                continue
            eid = entry.get("id") or uuid.uuid4().hex[:12]
            entry["id"] = eid
            self._entries[eid] = entry
            self._grid.setdefault(self._cell(entry["geo"]), set()).add(eid)

    def _cell(self, geo) -> tuple[int, int]:
        lat, lng = _geo_center(geo)
        return (int(lat / _CELL_DEG), int(lng / _CELL_DEG))


# ---------- 内部几何 ---------------------------------------------------------

def _normalize_geo(geo) -> Optional[dict[str, Any]]:
    if not isinstance(geo, dict):
        return None
    if "lat" in geo and "lng" in geo:
        lat, lng = float(geo["lat"]), float(geo["lng"])
        if not (-90 <= lat <= 90 and -180 <= lng <= 180):
            return None
        return {"lat": lat, "lng": lng}
    if isinstance(geo.get("points"), list) and len(geo["points"]) >= 2:
        pts = []
        for p in geo["points"]:
            if not (isinstance(p, (list, tuple)) and len(p) == 2):
                return None
            lat, lng = float(p[0]), float(p[1])
            if not (-90 <= lat <= 90 and -180 <= lng <= 180):
                return None
            pts.append([lat, lng])
        return {"points": pts}
    return None


def _geo_center(geo) -> tuple[float, float]:
    if "points" in geo:
        pts = geo["points"]
        return (sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts))
    return geo["lat"], geo["lng"]


def _geo_dist_m(a, b) -> float:
    alat, alng = _geo_center(a)
    blat, blng = _geo_center(b)
    kx = 111320.0 * math.cos(math.radians((alat + blat) / 2))
    ky = 110540.0
    return math.hypot((alng - blng) * kx, (alat - blat) * ky)


def _cells_in_radius(lat, lng, radius_m):
    d = max(1, int(radius_m / 100))  # 网格步数
    cells = []
    for dy in range(-d, d + 1):
        for dx in range(-d, d + 1):
            cells.append((int(lat / _CELL_DEG) + dy,
                          int(lng / _CELL_DEG) + dx))
    return cells


def _source_rank(source):
    return _SOURCE_HIERARCHY.get(str(source), 1)


def circle_ring(lat: float, lng: float, radius_m: float,
                n: int = 12) -> list[list[float]]:
    """点 → 圆形禁区环 (正 n 边形, 供守卫/hazard 布防用)。"""
    dlat = radius_m / 110540.0
    dlng = radius_m / (111320.0 * math.cos(math.radians(lat)))
    ring = []
    for i in range(n):
        ang = 2 * math.pi * i / n
        ring.append([lat + dlat * math.cos(ang), lng + dlng * math.sin(ang)])
    return ring
