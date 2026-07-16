#!/usr/bin/env python3
"""Read-only rate and spatial-coverage check for FAST_LIO's map-frame cloud."""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def decode_xyz(message):
    if message.is_bigendian or message.point_step <= 0 or not message.data:
        return np.empty((0, 3), dtype=np.float32)
    fields = {field.name: field for field in message.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    count = len(message.data) // message.point_step
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(count, message.point_step)
    columns = [
        raw[:, fields[name].offset:fields[name].offset + 4]
        .copy().reshape(-1).view("<f4")
        for name in ("x", "y", "z")
    ]
    return np.column_stack(columns)


class Collector(Node):
    def __init__(self, topic):
        super().__init__("diag_fastlio_map")
        self.receive_times = []
        self.frames = []
        self.frame_id = None
        self.create_subscription(PointCloud2, topic, self._cloud, qos_profile_sensor_data)

    def _cloud(self, message):
        points = decode_xyz(message)
        finite = points[np.isfinite(points).all(axis=1)]
        self.receive_times.append(time.monotonic())
        if finite.size:
            self.frames.append(finite)
        self.frame_id = message.header.frame_id


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--seconds", type=float, default=4.0)
    args = parser.parse_args()
    rclpy.init()
    node = Collector(args.topic)
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = (
        node.receive_times[-1] - node.receive_times[0]
        if len(node.receive_times) >= 2 else 0.0
    )
    hz = (len(node.receive_times) - 1) / elapsed if elapsed > 0.0 else 0.0
    points = np.concatenate(node.frames, axis=0) if node.frames else np.empty((0, 3))
    print(f"topic={args.topic} frame={node.frame_id} messages={len(node.receive_times)} hz={hz:.2f}")
    print(f"points_per_frame={[len(frame) for frame in node.frames[:10]]}")
    if points.size:
        minimum = np.min(points, axis=0)
        maximum = np.max(points, axis=0)
        span = maximum - minimum
        print(f"bounds_min={minimum.tolist()} bounds_max={maximum.tolist()} span={span.tolist()}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
