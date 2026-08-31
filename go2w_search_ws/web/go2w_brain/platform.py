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
    """适配器接口: snapshot() 读遥测; 下列为 M2 运动指令写端口。"""

    def snapshot(self) -> dict[str, Any]:
        raise NotImplementedError

    def gps_state(self) -> dict[str, Any]:
        raise NotImplementedError

    def submit_gps_route(self, waypoints: list[dict[str, Any]]) -> dict[str, Any]:
        raise NotImplementedError

    def cancel_gps_route(self) -> dict[str, Any]:
        raise NotImplementedError

    def calibrate_heading(self, heading_deg: float) -> dict[str, Any]:
        raise NotImplementedError


class MockAdapter(PlatformAdapter):
    """测试/干跑用: 固定遥测 + 内存航线状态机 + 调用记录。"""

    def __init__(self, **overrides: Any):
        base: dict[str, Any] = {
            "gps": {
                "available": True,
                # 默认中心: 无锡太科园园区 (M2.1 基准点, 园区内有湖)
                "lat": 31.488192, "lng": 120.369486,
                "hdop": 0.8, "sats": 18,
                "quality": "fix", "fix_age_s": 0.4,
            },
            "battery_soc": 72.5,
            "pose": {"available": True, "x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        }
        base.update(overrides)
        self._base = base
        self.calls: list[tuple[str, Any]] = []
        self._route: dict[str, Any] = {"active": False, "status": "idle",
                                       "waypoint_index": 0,
                                       "waypoint_total": 0, "reason": None}

    def snapshot(self) -> dict[str, Any]:
        out = json.loads(json.dumps(self._base, default=str))
        out["gps_route"] = dict(self._route)
        return out

    def gps_state(self) -> dict[str, Any]:
        return dict(self._route)

    @staticmethod
    def _valid_waypoints(waypoints: Any) -> bool:
        return (isinstance(waypoints, list) and bool(waypoints)
                and all(isinstance(w, dict)
                        and isinstance(w.get("lat"), (int, float))
                        and isinstance(w.get("lon"), (int, float))
                        for w in waypoints))

    def submit_gps_route(self, waypoints):
        self.calls.append(("submit_gps_route", waypoints))
        if not self._valid_waypoints(waypoints):
            return {"ok": False, "reason": "invalid_waypoints"}
        self._route = {"active": True, "status": "navigating",
                       "waypoint_index": 0, "waypoint_total": len(waypoints),
                       "reason": None}
        return {"ok": True, "active": True,
                "waypoint_total": len(waypoints)}

    def cancel_gps_route(self):
        self.calls.append(("cancel_gps_route", None))
        self._route = {"active": False, "status": "idle",
                       "waypoint_index": 0, "waypoint_total": 0,
                       "reason": "operator_cancel"}
        return {"ok": True, "status": "idle"}

    def calibrate_heading(self, heading_deg: float):
        self.calls.append(("calibrate_heading", heading_deg))
        try:
            value = float(heading_deg)
        except (TypeError, ValueError):
            return {"ok": False, "reason": "invalid_heading"}
        if value != value or value in (float("inf"), float("-inf")):
            return {"ok": False, "reason": "invalid_heading"}
        self._base.setdefault("gps", {})["north_heading_deg"] = value % 360.0
        return {"ok": True, "heading_deg": value % 360.0}


class NxHttpAdapter(PlatformAdapter):
    """NX 生产/仿真接入: GET 读状态 fail-soft; POST 运动指令带控制令牌。

    POST 契约 (对齐 nx_web_server):
      POST /api/gps/route         {"waypoints": [{"lat","lon",...}]}
      POST /api/gps/route_cancel  {}
      POST /api/gps/calibrate     {"heading_deg": float}
    拒绝 (409/400/503) 一律返回 {"ok": False, "reason": ..}, 绝不抛出。
    """

    def __init__(self, base_url: str, timeout: float = 3.0,
                 control_token: str = ""):
        self._base = base_url.rstrip("/")
        self._timeout = timeout
        self._token = control_token

    def _get(self, path: str) -> dict[str, Any]:
        try:
            with urllib.request.urlopen(
                    self._base + path, timeout=self._timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            return {"_http_error": f"{type(exc).__name__}: {exc}"}

    def _post(self, path: str, body: dict[str, Any]) -> dict[str, Any]:
        headers = {"Content-Type": "application/json; charset=utf-8"}
        if self._token:
            headers["Authorization"] = f"Bearer {self._token}"
        request = urllib.request.Request(
            self._base + path,
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            headers=headers, method="POST")
        try:
            with urllib.request.urlopen(
                    request, timeout=self._timeout) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
                return payload if isinstance(payload, dict) else {
                    "ok": True, "raw": payload}
        except urllib.error.HTTPError as exc:
            # 服务端拒绝 (409 未标定/已有航线, 400 参数, 503 不可用)
            try:
                payload = json.loads(exc.read().decode("utf-8"))
            except (OSError, json.JSONDecodeError):
                payload = {}
            reason = payload.get("reason") if isinstance(payload, dict) else None
            return {"ok": False, "http_status": exc.code,
                    "reason": reason or f"http_{exc.code}"}
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            return {"ok": False, "reason":
                    f"unreachable:{type(exc).__name__}"}

    # ---- M2 运动指令写端口 --------------------------------------------------

    def submit_gps_route(self, waypoints):
        return self._post("/api/gps/route", {"waypoints": waypoints})

    def cancel_gps_route(self):
        return self._post("/api/gps/route_cancel", {})

    def calibrate_heading(self, heading_deg: float):
        return self._post("/api/gps/calibrate",
                          {"heading_deg": float(heading_deg)})

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
