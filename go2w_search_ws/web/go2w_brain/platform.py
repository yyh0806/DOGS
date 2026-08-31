"""平台适配层: 大脑读世界状态的唯一入口。

设计: 大脑不 import 任何 ROS/业务模块 —— 状态经 HTTP 只读接口进入
(NX 侧 nx_web_server 已暴露 GET /api/status 与 GET /api/gps/route),
开发/测试用 MockAdapter。

快照字段采用「宽容提取」(tolerant lens): 键缺失 → {"available": False,
"reason": ...}, 绝不抛异常打断大脑; M2 起随 NX 侧状态契约逐项收紧。
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Any


class PlatformAdapter:
    """适配器接口: snapshot() 返回大脑每轮看到的遥测快照。"""

    def snapshot(self) -> dict[str, Any]:
        raise NotImplementedError

    def gps_state(self) -> dict[str, Any]:
        raise NotImplementedError


class MockAdapter(PlatformAdapter):
    """测试/干跑用: 固定遥测, 可用 overrides 覆盖。"""

    def __init__(self, **overrides: Any):
        base: dict[str, Any] = {
            "gps": {
                "available": True,
                "lat": 31.5163, "lng": 120.2673,
                "hdop": 0.8, "sats": 18,
                "quality": "fix", "fix_age_s": 0.4,
            },
            "battery_soc": 72.5,
            "pose": {"available": True, "x": 0.0, "y": 0.0, "yaw_deg": 0.0},
            "gps_route": {
                "active": False, "status": "idle",
                "waypoint_index": 0, "waypoint_total": 0, "reason": None,
            },
        }
        base.update(overrides)
        self._base = base

    def snapshot(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._base, default=str))

    def gps_state(self) -> dict[str, Any]:
        return dict(self._base.get("gps_route") or {})


class NxHttpAdapter(PlatformAdapter):
    """NX 生产/仿真接入: 只读 GET, 超时即按不可用处理 (fail-soft)。"""

    def __init__(self, base_url: str, timeout: float = 3.0):
        self._base = base_url.rstrip("/")
        self._timeout = timeout

    def _get(self, path: str) -> dict[str, Any]:
        try:
            with urllib.request.urlopen(
                    self._base + path, timeout=self._timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            return {"_http_error": f"{type(exc).__name__}: {exc}"}

    def gps_state(self) -> dict[str, Any]:
        return self._get("/api/gps/route") or {}

    def snapshot(self) -> dict[str, Any]:
        status = self._get("/api/status") or {}
        gps_state = self.gps_state()
        return {
            "ts": time.time(),
            "gps": _extract_gps(gps_state),
            "battery_soc": _extract_battery(status),
            "pose": _extract_pose(status),
            "gps_route": gps_state,
            "status_http_error": status.get("_http_error"),
        }


# ---------- 宽容提取 (M2 起收紧) ----------

def _extract_gps(gps_state: dict[str, Any]) -> dict[str, Any]:
    for key in ("fix", "latest_fix", "gps_health"):
        value = gps_state.get(key)
        if isinstance(value, dict):
            lat = value.get("lat")
            lng = value.get("lng", value.get("lon"))
            if isinstance(lat, (int, float)) and isinstance(lng, (int, float)):
                return {
                    "available": True,
                    "lat": float(lat), "lng": float(lng),
                    "hdop": value.get("hdop"),
                    "sats": value.get("sats", value.get("satellites")),
                    "quality": value.get("quality", value.get("status")),
                    "fix_age_s": value.get("age_s", value.get("fix_age_s")),
                }
    return {"available": False, "reason": "no_fix_in_state"}


def _extract_battery(status: dict[str, Any]) -> Any:
    for section in ("stats", "perception", "services"):
        value = (status.get(section) or {}).get("battery_soc")
        if isinstance(value, (int, float)):
            return float(value)
    return None


def _extract_pose(status: dict[str, Any]) -> dict[str, Any]:
    for key in ("localization", "odometry", "pose"):
        value = status.get(key)
        if isinstance(value, dict) and ("x" in value or "lat" in value):
            x = value.get("x", value.get("lat"))
            y = value.get("y", value.get("lng"))
            yaw = value.get("yaw_deg", value.get("yaw", value.get("heading")))
            if x is not None and y is not None:
                return {"available": True, "x": float(x), "y": float(y),
                        "yaw_deg": float(yaw) if yaw is not None else None}
    return {"available": False, "reason": "no_pose_in_status"}
