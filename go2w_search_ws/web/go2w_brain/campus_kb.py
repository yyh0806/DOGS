"""campus_kb — 园区标定知识库 (2026-09-05)。

用户在地图上点选标定园区边界/湖岸线 → 持久化 JSON → plan_campus_lake
优先使用标定真值 (source=calibrated), 不再依赖 OSM/VLM 猜测。

文件: runs/campus_kb.json (GO2W_CAMPUS_KB 可覆盖), 小文件每次执行时重读
(标定随时生效, 无需重启大脑)。结构:
{"<园区名>": {"center": [lat, lng], "boundary": [[lat,lng]...],
              "lake": [[lat,lng]...], "updated": "ISO"}}
"""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any, Optional

_LOCK = threading.RLock()


def kb_path() -> Path:
    return Path(os.environ.get("GO2W_CAMPUS_KB", "runs/campus_kb.json"))


def load() -> dict[str, Any]:
    with _LOCK:
        p = kb_path()
        if not p.exists():
            return {}
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}


def get(campus_name: str) -> Optional[dict[str, Any]]:
    return load().get(campus_name)


def upsert(campus_name: str, kind: str,
           polygon: list[list[float]] | list[list[list[float]]],
           multipoly: bool = False) -> dict[str, Any]:
    """保存某园区某类标定。kind: boundary|lake。

    multipoly=True 时 polygon 为多环 (湖面由多个不连通水体组成,
    2026-09-05 用户实测园区湖为三块), 存为 lake_parts=[ring,ring,...];
    单环湖沿用 lake 字段。返回更新后的条目。
    """
    import datetime
    with _LOCK:
        data = load()
        entry = data.get(campus_name) or {}
        if multipoly:
            entry["lake_parts"] = [[list(p) for p in ring]
                                   for ring in polygon]
            entry.pop("lake", None)  # 多块覆盖旧单环
        else:
            entry[kind] = [list(p) for p in polygon]
        entry["updated"] = datetime.datetime.now().isoformat(timespec="seconds")
        if kind == "boundary" and polygon and not multipoly:
            lats = [p[0] for p in polygon]
            lngs = [p[1] for p in polygon]
            entry.setdefault("center",
                             [sum(lats) / len(lats), sum(lngs) / len(lngs)])
        data[campus_name] = entry
        p = kb_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, ensure_ascii=False, indent=1),
                     encoding="utf-8")
        return dict(entry)


def lake_rings(entry: dict[str, Any]) -> list[list[list[float]]]:
    """从标定条目取湖面环列表 (多块优先, 单环兼容)。"""
    parts = entry.get("lake_parts")
    if parts:
        return parts
    legacy = entry.get("lake")
    return [legacy] if legacy else []


def _valid_ring(polygon) -> bool:
    return (isinstance(polygon, list) and len(polygon) >= 3
            and all(isinstance(p, (list, tuple)) and len(p) == 2
                    and -90 <= float(p[0]) <= 90
                    and -180 <= float(p[1]) <= 180
                    for p in polygon))


def validate(polygon) -> bool:
    return _valid_ring(polygon)


def record_to_memory(memory, campus_name: str, kind: str,
                     rings: list) -> Optional[str]:
    """标定 → 地图式记忆 (2026-09-05 用户要求: 永久记录, 使用记忆)。

    每次标定保存为一条 geometry 条目 (多块湖 = 一条含全部环的条目,
    data.parts), 大脑取同园区同种类中 ts 最新的条目 —— 重新标定
    自然覆盖旧值, 不残留。
    """
    if memory is None:
        return None
    if rings and isinstance(rings[0][0], (int, float)):
        rings = [rings]  # 单环 → 包一层
    entry = memory.record(
        "geometry", {"points": [list(p) for p in rings[0]]},
        data={"plan_kind": "calibration",
              "calibrated_kind": kind,
              "campus": campus_name,
              "parts": [[list(p) for p in ring] for ring in rings]},
        confidence=1.0, source="calibration")
    return entry["id"]
