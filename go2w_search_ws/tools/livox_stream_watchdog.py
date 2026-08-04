#!/usr/bin/env python3
"""Restart the Livox ROS driver when its process is alive but data is stale."""

from __future__ import annotations

import math
import subprocess
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
except ImportError:  # Keep the pure policy importable in non-ROS unit tests.
    rclpy = None
    Imu = object
    Node = object
    qos_profile_sensor_data = None


class StreamWatchdogPolicy:
    def __init__(
        self,
        *,
        startup_grace_sec: float,
        stale_after_sec: float,
        failures_before_restart: int,
        restart_cooldown_sec: float,
        started_at: float | None = None,
    ):
        values = (
            float(startup_grace_sec),
            float(stale_after_sec),
            float(restart_cooldown_sec),
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("watchdog timing values must be finite and non-negative")
        failures_before_restart = int(failures_before_restart)
        if failures_before_restart < 1:
            raise ValueError("failures_before_restart must be positive")
        self.startup_grace_sec = values[0]
        self.stale_after_sec = values[1]
        self.restart_cooldown_sec = values[2]
        self.failures_before_restart = failures_before_restart
        self.started_at = time.monotonic() if started_at is None else float(started_at)
        self.last_message_at: float | None = None
        self.last_restart_at: float | None = None
        self.consecutive_stale_polls = 0

    def note_message(self, now: float | None = None) -> None:
        self.last_message_at = time.monotonic() if now is None else float(now)
        self.consecutive_stale_polls = 0

    def note_restart(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        self.last_restart_at = now
        self.last_message_at = None
        self.consecutive_stale_polls = 0

    def should_restart(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        if now < self.started_at + self.startup_grace_sec:
            return False
        if (
            self.last_restart_at is not None
            and now < self.last_restart_at + self.restart_cooldown_sec
        ):
            return False

        stale = (
            self.last_message_at is None
            or now - self.last_message_at > self.stale_after_sec
        )
        if not stale:
            self.consecutive_stale_polls = 0
            return False
        self.consecutive_stale_polls += 1
        if self.consecutive_stale_polls < self.failures_before_restart:
            return False
        self.consecutive_stale_polls = 0
        return True


class LivoxStreamWatchdog(Node):
    def __init__(self):
        super().__init__("livox_stream_watchdog")
        self.declare_parameter("startup_grace_sec", 20.0)
        self.declare_parameter("stale_after_sec", 3.0)
        self.declare_parameter("failures_before_restart", 3)
        self.declare_parameter("restart_cooldown_sec", 30.0)
        self.policy = StreamWatchdogPolicy(
            startup_grace_sec=self.get_parameter(
                "startup_grace_sec").get_parameter_value().double_value,
            stale_after_sec=self.get_parameter(
                "stale_after_sec").get_parameter_value().double_value,
            failures_before_restart=self.get_parameter(
                "failures_before_restart").get_parameter_value().integer_value,
            restart_cooldown_sec=self.get_parameter(
                "restart_cooldown_sec").get_parameter_value().double_value,
        )
        self._subscription = self.create_subscription(
            Imu, "/livox/imu", self._on_imu, qos_profile_sensor_data
        )
        self._timer = self.create_timer(1.0, self._check_stream)
        self.get_logger().info(
            "watching lightweight /livox/imu heartbeat; stale driver will be "
            "restarted after "
            f"{self.policy.failures_before_restart} confirmed polls"
        )

    def _on_imu(self, _message: Imu) -> None:
        self.policy.note_message()

    def _check_stream(self) -> None:
        now = time.monotonic()
        if not self.policy.should_restart(now):
            return
        self.policy.note_restart(now)
        self.get_logger().error(
            "/livox/imu is stale while the driver service is expected alive; "
            "restarting livox-mid360-driver.service"
        )
        try:
            result = subprocess.run(
                ["/usr/bin/systemctl", "restart", "livox-mid360-driver.service"],
                check=False,
                capture_output=True,
                text=True,
                timeout=30.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f"Livox driver restart failed: {exc}")
            return
        if result.returncode == 0:
            self.get_logger().warning("Livox driver restart completed")
        else:
            detail = (result.stderr or result.stdout or "unknown error").strip()
            self.get_logger().error(
                f"Livox driver restart returned {result.returncode}: {detail}"
            )


def main() -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 and livox_ros_driver2 Python modules are required")
    rclpy.init()
    node = LivoxStreamWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
