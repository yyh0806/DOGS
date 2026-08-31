#!/usr/bin/env python3
"""nx_water_guard_node — 离水守卫 ROS 壳 (M3 运行期防线)。

职责: 把 nx_water_guard.WaterGuard (纯逻辑) 接进 ROS:
  订阅  /gps/fix                     (NavSatFix, 同航线控制器数据源)
  订阅  /water_guard/polygon         (String JSON, 布防/解除指令)
  发布  /water_guard/status          (String JSON 心跳, 5Hz)

心跳契约 (nx_motion_node.WaterGuardClient 消费):
  {"armed": bool, "verdict": "allow"|"limit"|"veto"|null,
   "min_dist_m": float|null, "speed_cap": float|null,
   "violation_count": int, "stale": bool}

布防指令 (由 nx_web_server /api/water_guard/arm 转发):
  {"arm": true, "ring": [[lat,lng]...], "margin_m": float}
  {"disarm": true, "approval_token": str}

解除审批令牌来自环境变量 GO2W_WATER_GUARD_TOKEN (与大脑侧
GO2W_BRAIN_APPROVAL_TOKEN 由操作员同源下发)。
"""
from __future__ import annotations

import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nx_water_guard import WaterGuard  # noqa: E402


class WaterGuardNode(Node):
    def __init__(self):
        super().__init__("nx_water_guard")
        token = os.environ.get("GO2W_WATER_GUARD_TOKEN", "").strip()
        self._guard = WaterGuard(approval_token=token)
        self._fix = None  # (lat, lng, age_monotonic)
        self._polygon_sub = self.create_subscription(
            String, "/water_guard/polygon", self._on_polygon, 10)
        self._fix_sub = self.create_subscription(
            NavSatFix,
            os.environ.get("GO2W_GPS_FIX_TOPIC", "/gps/fix").strip()
            or "/gps/fix",
            self._on_fix, qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            String, "/water_guard/status", 10)
        self._status_timer = self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            "water guard node up (approval token %s)" %
            ("configured" if token else "MISSING — disarm will be refused"))

    def _on_fix(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lng = float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lng)):
            return
        import time
        self._fix = (lat, lng, time.monotonic())

    def _on_polygon(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("not an object")
        except (ValueError, TypeError):
            self.get_logger().warning("bad polygon payload ignored")
            return
        if payload.get("disarm"):
            result = self._guard.disarm(
                str(payload.get("approval_token") or ""))
            self.get_logger().info("disarm: %s" % result)
            return
        if payload.get("arm"):
            ring = payload.get("ring")
            result = self._guard.arm(
                [(float(p[0]), float(p[1])) for p in ring],
                margin_m=float(payload.get("margin_m", 0.0) or 0.0))
            self.get_logger().info("arm: %s" % result)
            return
        self.get_logger().warning("polygon payload without arm/disarm")

    def _publish_status(self) -> None:
        import time
        status = self._guard.state()
        verdict, min_dist, cap = None, None, None
        stale = True
        if status["armed"] and self._fix is not None:
            lat, lng, ts = self._fix
            fix_age = time.monotonic() - ts
            stale = fix_age > 1.5
            result = self._guard.evaluate(lat, lng,
                                          fix_age_s=None if stale
                                          else fix_age)
            verdict = result["verdict"]
            min_dist = result["min_dist_m"]
            cap = result["speed_cap"]
        payload = {
            "armed": status["armed"],
            "verdict": verdict,
            "min_dist_m": min_dist,
            "speed_cap": cap,
            "violation_count": status["violation_count"],
            "stale": stale,
        }
        message = String()
        message.data = json.dumps(payload)
        self._status_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = WaterGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
