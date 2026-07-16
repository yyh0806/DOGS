#!/usr/bin/env python3
"""Read-only timing and geometry diagnostics for Livox CustomMsg + IMU."""

import argparse
import math
import statistics
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quantiles(values):
    ordered = sorted(values)
    if not ordered:
        return "none"
    picks = []
    for fraction in (0.0, 0.1, 0.5, 0.9, 1.0):
        index = min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))
        picks.append(ordered[index])
    return "/".join(f"{value:.6f}" for value in picks)


class Collector(Node):
    def __init__(self):
        super().__init__("diag_livox_raw")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.lidar_receive = []
        self.lidar_stamps = []
        self.point_counts = []
        self.offset_spans_ms = []
        self.ranges = []
        self.imu_receive = []
        self.imu_stamps = []
        self.accel = []
        self.gyro = []
        self.lidar_sub = self.create_subscription(
            CustomMsg, "/livox/lidar", self.on_lidar, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, "/livox/imu", self.on_imu, qos
        )

    def on_lidar(self, msg):
        self.lidar_receive.append(time.monotonic())
        self.lidar_stamps.append(stamp_seconds(msg.header.stamp))
        self.point_counts.append(len(msg.points))
        if msg.points:
            offsets = [point.offset_time for point in msg.points]
            self.offset_spans_ms.append((max(offsets) - min(offsets)) / 1e6)
            self.ranges.extend(
                math.sqrt(point.x ** 2 + point.y ** 2 + point.z ** 2)
                for point in msg.points
                if all(math.isfinite(v) for v in (point.x, point.y, point.z))
            )

    def on_imu(self, msg):
        self.imu_receive.append(time.monotonic())
        self.imu_stamps.append(stamp_seconds(msg.header.stamp))
        self.accel.append((
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ))
        self.gyro.append((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))


def frequency(receive_times):
    if len(receive_times) < 2:
        return 0.0
    duration = receive_times[-1] - receive_times[0]
    return (len(receive_times) - 1) / duration if duration > 0.0 else 0.0


def vector_mean(values):
    if not values:
        return None
    return tuple(statistics.fmean(value[i] for value in values) for i in range(3))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Collector()
    deadline = time.monotonic() + max(1.0, args.seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(
        f"lidar messages={len(node.lidar_receive)} hz={frequency(node.lidar_receive):.2f} "
        f"points/message min/median/max={min(node.point_counts, default=0)}/"
        f"{statistics.median(node.point_counts) if node.point_counts else 0}/"
        f"{max(node.point_counts, default=0)}"
    )
    print(f"lidar stamp_delta_s={quantiles([b-a for a, b in zip(node.lidar_stamps, node.lidar_stamps[1:])])}")
    print(f"lidar offset_span_ms={quantiles(node.offset_spans_ms)}")
    print(f"raw range_m={quantiles(node.ranges)}")
    print(
        f"imu messages={len(node.imu_receive)} hz={frequency(node.imu_receive):.2f} "
        f"stamp_delta_s={quantiles([b-a for a, b in zip(node.imu_stamps, node.imu_stamps[1:])])}"
    )
    print(f"imu accel_mean={vector_mean(node.accel)} gyro_mean={vector_mean(node.gyro)}")
    if node.lidar_stamps and node.imu_stamps:
        print(f"latest_lidar_minus_imu_s={node.lidar_stamps[-1] - node.imu_stamps[-1]:.6f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
