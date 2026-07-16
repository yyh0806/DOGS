#!/usr/bin/env python3
"""Read-only diagnostics for the MID360 Nav2 point filtering chain."""

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def decode_xyz(msg):
    if msg.is_bigendian or msg.point_step <= 0 or not msg.data:
        return np.empty((0, 3), dtype=np.float32)
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    count = len(msg.data) // int(msg.point_step)
    raw = np.frombuffer(
        msg.data, dtype=np.uint8, count=count * int(msg.point_step)
    ).reshape(count, int(msg.point_step))
    columns = [
        raw[:, fields[name].offset:fields[name].offset + 4]
        .copy().reshape(-1).view("<f4")
        for name in ("x", "y", "z")
    ]
    return np.column_stack(columns)


def summarize(label, points):
    finite_points = points[np.isfinite(points).all(axis=1)]
    print(f"{label}: total={len(points)} finite={len(finite_points)}")
    if not len(finite_points):
        return
    for index, axis in enumerate("xyz"):
        values = finite_points[:, index]
        q = np.quantile(values, [0.0, 0.1, 0.5, 0.9, 1.0])
        print(f"  {axis} min/p10/p50/p90/max=" + "/".join(f"{v:.3f}" for v in q))
    radii = np.hypot(finite_points[:, 0], finite_points[:, 1])
    q = np.quantile(radii, [0.0, 0.1, 0.5, 0.9, 1.0])
    print("  xy_radius min/p10/p50/p90/max=" + "/".join(f"{v:.3f}" for v in q))


def filter_counts(points, range_min=0.20, range_max=8.0,
                  min_height=0.05, max_height=1.50):
    finite = np.isfinite(points).all(axis=1)
    radii = np.hypot(points[:, 0], points[:, 1])
    in_range = finite & (radii >= range_min) & (radii <= range_max)
    in_height = in_range & (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    self_return = (
        (points[:, 0] >= -0.30) & (points[:, 0] <= 0.35)
        & (points[:, 1] >= -0.25) & (points[:, 1] <= 0.25)
    )
    return {
        "finite": int(finite.sum()),
        "range": int(in_range.sum()),
        "height": int(in_height.sum()),
        "self_among_height": int((in_height & self_return).sum()),
        "kept": int((in_height & ~self_return).sum()),
    }


def height_band_counts(points, range_min=0.20, range_max=8.0):
    radii = np.hypot(points[:, 0], points[:, 1])
    finite_range = (
        np.isfinite(points).all(axis=1)
        & (radii >= range_min) & (radii <= range_max)
    )
    bands = (-2.0, -0.80, -0.60, -0.45, -0.30, -0.15,
             0.0, 0.05, 0.30, 0.80, 1.50, 2.0)
    results = []
    for low, high in zip(bands, bands[1:]):
        mask = finite_range & (points[:, 2] >= low) & (points[:, 2] < high)
        count = int(mask.sum())
        nearest = float(np.min(radii[mask])) if count else None
        results.append((low, high, count, nearest))
    return results


class Collector(Node):
    def __init__(self, topic="/cloud_registered_body"):
        super().__init__("diag_mid360_filter")
        self.messages = 0
        self.points = []
        self.metadata = None
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.on_cloud,
            qos_profile_sensor_data
        )

    def on_cloud(self, msg):
        self.messages += 1
        points = decode_xyz(msg)
        if points.size:
            self.points.append(points)
        if self.metadata is None:
            self.metadata = {
                "frame": msg.header.frame_id,
                "height": msg.height,
                "width": msg.width,
                "point_step": msg.point_step,
                "row_step": msg.row_step,
                "data_bytes": len(msg.data),
                "fields": [(field.name, field.offset, field.datatype)
                           for field in msg.fields],
            }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--topic", default="/cloud_registered_body")
    args = parser.parse_args()

    rclpy.init()
    node = Collector(args.topic)
    deadline = time.monotonic() + max(0.5, args.seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"messages={node.messages} metadata={node.metadata}")
    if not node.points:
        print("NO_DECODED_POINTS")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    raw = np.concatenate(node.points, axis=0).astype(np.float64, copy=False)
    summarize("raw_body", raw)
    print(f"  current_filter_counts={filter_counts(raw, min_height=-0.60)}")
    print(f"  height_bands(low,high,count,nearest_xy)={height_band_counts(raw)}")
    pitch = math.radians(-20.0)
    candidates = {
        "current_inverse_minus20": rpy_matrix(0.0, pitch, 0.0).T,
        "direct_minus20": rpy_matrix(0.0, pitch, 0.0),
        "identity": np.eye(3),
    }
    for label, rotation in candidates.items():
        transformed = raw @ rotation.T
        summarize(label, transformed)
        print(f"  filter_counts={filter_counts(transformed)}")
        if label == "current_inverse_minus20":
            print(f"  height_bands(low,high,count,nearest_xy)={height_band_counts(transformed)}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
