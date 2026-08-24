#!/usr/bin/env python3
"""室外 GPS 航线导航 —— 薄 ROS 壳 (业务零逻辑, 全部在 nx_gps_nav.py)。

职责边界 (为什么这么薄):
  - 本文件只做 ROS 管道: 订阅 sensor_msgs/NavSatFix → 转纯数据 GpsFix →
    喂给 GpsRouteController; 起一个定时器调 controller.tick()。
  - 所有决策 (质量门禁/坐标变换/航点状态机/停车条件) 都在纯核心里,
    开发机无 ROS2 也能完整单测 —— 对齐 map_odom_fuser.py 的分层范本。

生产部署形态 (集成执行者 eI 对接点):
  - 正式形态: nx_web_server.py 进程内直接构造 GpsRouteController,
    point_port 传 PointNavigationController (或包一层 arbiter 适配器),
    park_hook 接 NavigationArbiter 的停车路径, state_callback 包成
    {"type": "gps_route", "data": ...} WS 消息。
  - 本文件提供一个可独立拉起的 bringup 节点 (main), 用于上车前用
    mock_nav2_action.py + 回放 GPS 数据做链路验证; 平时不随 systemd 常驻。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 纯核心与 rclpy 解耦, 这里显式 import 以暴露唯一依赖方向: 壳 → 核心。
from nx_gps_nav import (
    GpsFix,
    GpsRouteController,
    nav_sat_fix_to_gps_fix,
)


logger = logging.getLogger("go2w.gps_nav_ros")


class GpsNavRosShell(Node):
    """NavSatFix → GpsRouteController 的 ROS 管道 + tick 定时器。"""

    def __init__(
        self,
        controller: GpsRouteController,
        *,
        fix_topic: str = "/gps/fix",
        tick_period_sec: float = 0.2,
    ) -> None:
        super().__init__("gps_nav_shell")
        self._controller = controller
        self._tick_period_sec = float(tick_period_sec)

        # 真实订阅在 _install_fix_subscription 里装 (sensor_msgs 缺失时可降级)。
        self._subscription = None
        self._install_fix_subscription(fix_topic)

        self._tick_timer = self.create_timer(
            self._tick_period_sec, self._on_tick)
        self._last_status: Optional[str] = None

    def _install_fix_subscription(self, topic: str) -> None:
        """尽力安装 sensor_msgs/NavSatFix 订阅; 缺包时记日志不崩溃。"""
        try:
            from sensor_msgs.msg import NavSatFix
        except ImportError:
            logger.warning(
                "sensor_msgs 不可用, GPS 订阅未安装 (shell 仅 tick)")
            return
        # 传感器数据 QoS 选 BEST_EFFORT: GPS 驱动默认 best-effort 发布,
        # RELIABLE 会直接收不到流; 丢一帧没关系, 门禁按"过期"判停,
        # 不会因为 QoS 抖动误判 (fail-closed 靠时间戳不靠重传)。
        self._subscription = self.create_subscription(
            NavSatFix,
            topic,
            self._on_fix,
            QoSProfile(
                depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def _on_fix(self, msg: Any) -> None:
        # 用"到达本机的单调时刻"做新鲜度基准, 不用设备时间戳 (见 GpsFix 注释)。
        fix = nav_sat_fix_to_gps_fix(msg, time.monotonic())
        self._controller.update_fix(fix)

    def _on_tick(self) -> None:
        state = self._controller.tick()
        status = state.get("status")
        if status != self._last_status:
            logger.info(
                "gps route status: %s -> %s (reason=%s)",
                self._last_status, status, state.get("reason"))
            self._last_status = status


def main(args=None) -> None:
    """独立 bringup: 需要外部 point_port 适配器才有意义, 这里只做链路验证。"""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    rclpy.init(args=args)

    # bringup 模式用"只读观察"端口: 不真控狗, 只驱动状态机与日志,
    # 上车验证 GPS 流/门禁/停车判定是否按预期工作。
    class _ObservingPort:
        def get_state(self) -> dict[str, Any]:
            return {
                "status": "idle",
                "generation": 0,
                "drained": True,
                "healthy": True,
                "quarantined": False,
                "stopped": False,
            }

        def submit(self, x: float, y: float, yaw: float = 0.0) -> dict:
            logger.info("observe submit: x=%.2f y=%.2f yaw=%.2f", x, y, yaw)
            return {"ok": True, "generation": 1}

        def cancel(self, reason: str) -> bool:
            logger.info("observe cancel: %s", reason)
            return True

    controller = GpsRouteController(_ObservingPort())
    node = GpsNavRosShell(controller)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
