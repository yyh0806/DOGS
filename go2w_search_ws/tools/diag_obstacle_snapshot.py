#!/usr/bin/env python3
"""Print one synchronized-enough Nav2 obstacle snapshot for field debugging."""

import argparse
import math
import struct
import time
from collections import Counter

import rclpy
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class Snapshot(Node):
    def __init__(
        self,
        scan_topic="/scan_mid360",
        cloud_topic="/mid360/points_nav",
        costmap_topic="/local_costmap/costmap_raw",
    ):
        super().__init__("diag_obstacle_snapshot")
        self.messages = {}
        self.close_scan_hits = []
        self.close_cloud_hits = []
        self.create_subscription(
            LaserScan, scan_topic, self._scan, 10
        )
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Costmap,
            costmap_topic,
            lambda msg: self.messages.__setitem__("costmap", msg),
            costmap_qos,
        )
        self.create_subscription(
            Odometry, "/odom", lambda msg: self._save("odom", msg), 10
        )
        self.create_subscription(PointCloud2, cloud_topic, self._cloud, 10)

    def _save(self, name, message):
        self.messages.setdefault(name, message)

    def _scan(self, message):
        self.messages["scan"] = message
        for index, distance in enumerate(message.ranges):
            if math.isfinite(distance) and distance < 1.0:
                self.close_scan_hits.append(
                    (
                        float(distance),
                        message.angle_min + index * message.angle_increment,
                    )
                )

    def _cloud(self, message):
        if message.point_step < 12:
            return
        for offset in range(0, len(message.data), message.point_step):
            x, y, z = struct.unpack_from("<fff", message.data, offset)
            radius = math.hypot(x, y)
            if radius < 1.0:
                self.close_cloud_hits.append((radius, x, y, z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-topic", default="/scan_mid360")
    parser.add_argument("--cloud-topic", default="/mid360/points_nav")
    parser.add_argument(
        "--costmap-topic", default="/local_costmap/costmap_raw"
    )
    parser.add_argument("--probe-x", type=float, default=None)
    parser.add_argument("--probe-y", type=float, default=None)
    parser.add_argument("--seconds", type=float, default=6.0)
    args = parser.parse_args()
    rclpy.init()
    node = Snapshot(args.scan_topic, args.cloud_topic, args.costmap_topic)
    deadline = time.monotonic() + max(1.0, args.seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    scan = node.messages.get("scan")
    costmap = node.messages.get("costmap")
    odom = node.messages.get("odom")
    if scan:
        values = [
            (scan.angle_min + index * scan.angle_increment, float(distance))
            for index, distance in enumerate(scan.ranges)
            if math.isfinite(distance)
        ]
        print("scan_finite", len(values), "min", min((x for _, x in values), default=None))
        front_hits = sorted(
            (round(angle, 4), round(distance, 3))
            for angle, distance in values
            if abs(angle) <= 0.35
        )
        print("front_hits_angle_rad_distance_m", front_hits)
        for low, high, name in (
            (-0.35, 0.35, "front40"),
            (0.35, 1.2, "left"),
            (-1.2, -0.35, "right"),
            (-math.pi, math.pi, "all"),
        ):
            sector = [distance for angle, distance in values if low <= angle <= high]
            print(name, "min", min(sector, default=None), "lt1.5", sum(x < 1.5 for x in sector))
        print("scan_hits_lt1_over_window", len(node.close_scan_hits))
        print("nearest_scan_hits", sorted(node.close_scan_hits)[:20])
        print("cloud_hits_lt1_over_window", len(node.close_cloud_hits))
        print("nearest_cloud_hits", sorted(node.close_cloud_hits)[:30])
        for center_degrees in range(-180, 180, 30):
            center = math.radians(center_degrees)
            half_width = math.radians(15)
            sector = [
                distance
                for angle, distance in values
                if center - half_width <= angle < center + half_width
            ]
            print(
                "sector",
                center_degrees,
                "min",
                round(min(sector), 3) if sector else None,
                "open_gt1",
                sum(distance > 1.0 for distance in sector),
                "hits",
                len(sector),
            )
    if odom:
        print("odom", odom.pose.pose.position.x, odom.pose.pose.position.y)
    if costmap:
        histogram = Counter(costmap.data)
        print("cost_histogram", sorted(histogram.items()))
        origin_x = costmap.metadata.origin.position.x
        origin_y = costmap.metadata.origin.position.y
        resolution = costmap.metadata.resolution
        width = costmap.metadata.size_x
        height = costmap.metadata.size_y
        if args.probe_x is not None and args.probe_y is not None:
            cell_x = int(math.floor((args.probe_x - origin_x) / resolution))
            cell_y = int(math.floor((args.probe_y - origin_y) / resolution))
            if 0 <= cell_x < width and 0 <= cell_y < height:
                probe_cost = int(costmap.data[cell_y * width + cell_x])
                print(
                    "probe_cost",
                    (args.probe_x, args.probe_y),
                    "cell",
                    (cell_x, cell_y),
                    "value",
                    probe_cost,
                )
                nearby = []
                search_cells = int(math.ceil(1.0 / resolution))
                for dy in range(-search_cells, search_cells + 1):
                    for dx in range(-search_cells, search_cells + 1):
                        nx = cell_x + dx
                        ny = cell_y + dy
                        if not (0 <= nx < width and 0 <= ny < height):
                            continue
                        value = int(costmap.data[ny * width + nx])
                        if value < 253:
                            nearby.append(
                                (
                                    math.hypot(dx, dy) * resolution,
                                    origin_x + (nx + 0.5) * resolution,
                                    origin_y + (ny + 0.5) * resolution,
                                    value,
                                )
                            )
                print("nearest_traversable", sorted(nearby)[:5])
            else:
                print(
                    "probe_cost",
                    (args.probe_x, args.probe_y),
                    "outside_costmap",
                )
        lethal = []
        for index, value in enumerate(costmap.data):
            if value >= 253:
                cell_x = index % width
                cell_y = index // width
                lethal.append(
                    (
                        origin_x + (cell_x + 0.5) * resolution,
                        origin_y + (cell_y + 0.5) * resolution,
                        value,
                    )
                )
        bounds = (
            min((point[0] for point in lethal), default=None),
            max((point[0] for point in lethal), default=None),
            min((point[1] for point in lethal), default=None),
            max((point[1] for point in lethal), default=None),
        )
        print("cost_lethal", len(lethal), "bounds", bounds)
        if odom and lethal:
            robot_x = odom.pose.pose.position.x
            robot_y = odom.pose.pose.position.y
            nearest = sorted(
                (
                    math.hypot(x - robot_x, y - robot_y),
                    x,
                    y,
                    value,
                )
                for x, y, value in lethal
            )[:10]
            print("nearest_lethal", nearest)
            for value in (253, 254, 255):
                matching = [point for point in lethal if point[2] == value]
                distances = sorted(
                    (math.hypot(x - robot_x, y - robot_y), x, y)
                    for x, y, _ in matching
                )
                print(
                    "cost_value",
                    value,
                    "count",
                    len(matching),
                    "nearest",
                    distances[:5],
                )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
