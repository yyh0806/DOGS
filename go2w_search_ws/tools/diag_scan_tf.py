#!/usr/bin/env python3
"""Diagnose whether each MID360 scan timestamp is transformable into odom."""

import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanTfDiagnostic(Node):
    def __init__(self):
        super().__init__("diag_scan_tf")
        self._buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._listener = TransformListener(self._buffer, self)
        self._started = time.monotonic()
        self._seen = 0
        self.create_subscription(LaserScan, "/scan_mid360", self._on_scan, 10)

    def _on_scan(self, msg):
        self._seen += 1
        stamp = Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        age = (now.nanoseconds - stamp.nanoseconds) / 1e9
        try:
            transform = self._buffer.lookup_transform(
                "odom", msg.header.frame_id, stamp, Duration(seconds=0.2)
            )
            result = "ok"
            tf_stamp = Time.from_msg(transform.header.stamp)
            tf_age = (now.nanoseconds - tf_stamp.nanoseconds) / 1e9
        except TransformException as exc:
            result = f"FAIL {type(exc).__name__}: {exc}"
            tf_age = float("nan")
        print(
            f"scan={self._seen} frame={msg.header.frame_id} age={age:+.3f}s "
            f"tf_age={tf_age:+.3f}s result={result}",
            flush=True,
        )
        if self._seen >= 12 or time.monotonic() - self._started > 8.0:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = ScanTfDiagnostic()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
