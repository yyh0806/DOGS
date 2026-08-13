"""地标注册表 (landmarks.yaml) — 地图选点标记的语义位置。

与 rooms.yaml 并列的轻量地标存储:
  - 用户在 Web 面板地图上选点 → 命名 ("大门") → POST /api/landmarks 注册
  - NLU "去大门" → LandmarkMap.find("大门") → /api/navigate 导航
  - 室内外切换: gps 可选字段 (lat/lon) 预留全局参考层接口 (TASK_PLANNING_SPEC L2),
    当前执行只依赖 map 坐标系的 x/y/yaw (L1)。

YAML 格式:
  version: "1.0"
  landmarks:
    - name: 大门
      aliases: [门口, gate]
      x: 2.5
      y: 1.8
      yaw: 0.0
      gps: null          # 可选: {lat: 30.1, lon: 120.1}
"""
from __future__ import annotations

import logging
import os
from typing import Dict, List, Optional

logger = logging.getLogger("go2w.landmarks")


class LandmarkValidationError(ValueError):
    pass


class Landmark:
    def __init__(self, name: str, x: float, y: float, yaw: float = 0.0,
                 aliases: Optional[List[str]] = None,
                 gps: Optional[Dict[str, float]] = None,
                 frame_id: str = "map"):
        self.name = str(name).strip()
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.aliases = [str(a).strip() for a in (aliases or [])]
        self.gps = gps
        self.frame_id = str(frame_id)

    @classmethod
    def from_dict(cls, data: Dict) -> "Landmark":
        if not isinstance(data, dict):
            raise LandmarkValidationError("landmark 条目必须是 dict")
        name = data.get("name")
        if not name or not str(name).strip():
            raise LandmarkValidationError("landmark.name 必填")
        for key in ("x", "y"):
            v = data.get(key)
            if not isinstance(v, (int, float)):
                raise LandmarkValidationError(f"landmark.{key} 必须是数值")
        yaw = data.get("yaw", 0.0)
        if not isinstance(yaw, (int, float)):
            raise LandmarkValidationError("landmark.yaw 必须是数值")
        gps = data.get("gps")
        if gps is not None:
            if not isinstance(gps, dict) or not all(
                    isinstance(gps.get(k), (int, float)) for k in ("lat", "lon")):
                raise LandmarkValidationError("landmark.gps 必须是 {lat, lon} 数值")
        return cls(
            name=str(name).strip(),
            x=float(data["x"]),
            y=float(data["y"]),
            yaw=float(yaw),
            aliases=data.get("aliases") or [],
            gps=gps,
            frame_id=str(data.get("frame_id", "map")),
        )

    def to_dict(self) -> Dict:
        d = {
            "name": self.name,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 3),
            "frame_id": self.frame_id,
        }
        if self.aliases:
            d["aliases"] = self.aliases
        if self.gps:
            d["gps"] = self.gps
        return d


class LandmarkMap:
    """landmarks.yaml 的内存表示 + 持久化。"""

    def __init__(self, landmarks: List[Landmark], version: str = "1.0"):
        self.landmarks = list(landmarks)
        self.version = str(version)

    @classmethod
    def load(cls, path: str) -> "LandmarkMap":
        import yaml
        if not os.path.exists(path):
            return cls([])
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise LandmarkValidationError("landmarks.yaml 顶层必须是 dict")
        raw = data.get("landmarks", [])
        if not isinstance(raw, list):
            raise LandmarkValidationError("landmarks 必须是 list")
        landmarks, seen = [], set()
        for rd in raw:
            lm = Landmark.from_dict(rd)
            if lm.name in seen:
                raise LandmarkValidationError(f"landmark.name 重复: {lm.name}")
            seen.add(lm.name)
            landmarks.append(lm)
        return cls(landmarks, str(data.get("version", "1.0")))

    def save(self, path: str) -> None:
        import yaml
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        data = {
            "version": self.version,
            "landmarks": [lm.to_dict() for lm in self.landmarks],
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    def find(self, query: str) -> Optional[Landmark]:
        """地标匹配: name 完全相等 > alias 完全相等 > name/alias 子串包含。"""
        q = str(query).strip()
        if not q:
            return None
        for lm in self.landmarks:
            if lm.name == q:
                return lm
        for lm in self.landmarks:
            if q in lm.aliases:
                return lm
        for lm in self.landmarks:
            if q in lm.name or any(q in a for a in lm.aliases):
                return lm
        return None

    def upsert(self, landmark: Landmark) -> "LandmarkMap":
        for i, lm in enumerate(self.landmarks):
            if lm.name == landmark.name:
                self.landmarks[i] = landmark
                return self
        self.landmarks.append(landmark)
        return self
