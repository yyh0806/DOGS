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
           polygon: list[list[float]]) -> dict[str, Any]:
    """保存某园区某类标定 (kind: boundary|lake)。返回更新后的条目。"""
    import datetime
    with _LOCK:
        data = load()
        entry = data.get(campus_name) or {}
        entry[kind] = [list(p) for p in polygon]
        entry["updated"] = datetime.datetime.now().isoformat(timespec="seconds")
        if kind == "boundary" and polygon:
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


def _valid_ring(polygon) -> bool:
    return (isinstance(polygon, list) and len(polygon) >= 3
            and all(isinstance(p, (list, tuple)) and len(p) == 2
                    and -90 <= float(p[0]) <= 90
                    and -180 <= float(p[1]) <= 180
                    for p in polygon))


def validate(polygon) -> bool:
    return _valid_ring(polygon)


def record_to_memory(memory, campus_name: str, kind: str,
                     polygon: list[list[float]]) -> Optional[str]:
    """标定 → 地图式记忆 (2026-09-05 用户要求: 永久记录, 使用记忆)。

    kind=geometry 条目 (append-only jsonl + 网格索引), 数据标记
    calibrated_kind (campus_boundary|lake_shore); 大脑下次任务的
    记忆检索 (指令×记忆) 即能看到, plan_campus_lake 优先取用。
    """
    if memory is None:
        return None
    entry = memory.record(
        "geometry", {"points": [list(p) for p in polygon]},
        data={"plan_kind": "calibration",
              "calibrated_kind": kind,
              "campus": campus_name},
        confidence=1.0, source="calibration")
    return entry["id"]
