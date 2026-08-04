#!/usr/bin/env python3
"""Read-only audit of SLAM and Nav2 grids at the robot pose."""

import math
import time

import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _world_cell(info, x, y):
    dx = x - float(info.origin.position.x)
    dy = y - float(info.origin.position.y)
    angle = -_yaw(info.origin.orientation)
    local_x = math.cos(angle) * dx - math.sin(angle) * dy
    local_y = math.sin(angle) * dx + math.cos(angle) * dy
    return (
        int(math.floor(local_x / float(info.resolution))),
        int(math.floor(local_y / float(info.resolution))),
    )


def _summarize(label, info, data, x, y, occupied):
    cell_x, cell_y = _world_cell(info, x, y)
    width = int(getattr(info, "width", getattr(info, "size_x", 0)))
    height = int(getattr(info, "height", getattr(info, "size_y", 0)))
    if not (0 <= cell_x < width and 0 <= cell_y < height):
        print(f"{label}: robot=outside grid cell=({cell_x},{cell_y})")
        return
    robot_value = int(data[cell_y * width + cell_x])
    resolution = float(info.resolution)
    radius_cells = int(math.ceil(2.0 / resolution))
    nearest = None
    nearest_cell = None
    counts = {0.3: 0, 0.6: 0, 0.75: 0, 1.0: 0, 2.0: 0}
    for grid_y in range(max(0, cell_y - radius_cells), min(height, cell_y + radius_cells + 1)):
        for grid_x in range(max(0, cell_x - radius_cells), min(width, cell_x + radius_cells + 1)):
            value = int(data[grid_y * width + grid_x])
            if not occupied(value):
                continue
            distance = math.hypot(grid_x - cell_x, grid_y - cell_y) * resolution
            if nearest is None or distance < nearest:
                nearest = distance
                nearest_cell = (grid_x, grid_y)
            for radius in counts:
                if distance <= radius:
                    counts[radius] += 1
    nearest_text = "none<=2m" if nearest is None else f"{nearest:.3f}m"
    print(
        f"{label}: robot_cell=({cell_x},{cell_y}) value={robot_value} "
        f"nearest_occupied={nearest_text} counts={counts} "
        f"shape={width}x{height} resolution={resolution:.3f}"
    )
    if nearest_cell is None:
        return None
    local_x = (nearest_cell[0] + 0.5) * resolution
    local_y = (nearest_cell[1] + 0.5) * resolution
    origin_yaw = _yaw(info.origin.orientation)
    world_x = (
        float(info.origin.position.x)
        + math.cos(origin_yaw) * local_x
        - math.sin(origin_yaw) * local_y
    )
    world_y = (
        float(info.origin.position.y)
        + math.sin(origin_yaw) * local_x
        + math.cos(origin_yaw) * local_y
    )
    return world_x, world_y


class LayerAudit(Node):
    def __init__(self):
        super().__init__("diag_occupancy_layers")
        self.pose = None
        self.messages = {}
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/localization_pose", self._pose, 10)
        for topic in ("/map_frontier_raw", "/map_frontier"):
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda message, name=topic: self.messages.__setitem__(name, message),
                map_qos,
            )
        for topic in ("/global_costmap/costmap_raw", "/local_costmap/costmap_raw"):
            self.create_subscription(
                Costmap,
                topic,
                lambda message, name=topic: self.messages.__setitem__(name, message),
                costmap_qos,
            )
        self.create_subscription(
            LaserScan,
            "/scan_mid360",
            lambda message: self.messages.__setitem__("/scan_mid360", message),
            10,
        )

    def _pose(self, message):
        self.pose = message


def main():
    rclpy.init()
    node = LayerAudit()
    deadline = time.monotonic() + 12.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.pose is not None and len(node.messages) == 5:
                break
        if node.pose is None:
            raise RuntimeError("timed out waiting for /localization_pose")
        position = node.pose.pose.pose.position
        robot_yaw = _yaw(node.pose.pose.pose.orientation)
        print(
            f"pose=({position.x:.4f},{position.y:.4f}) yaw={robot_yaw:.4f} "
            f"frame={node.pose.header.frame_id}"
        )
        nearest_static = None
        for topic in ("/map_frontier_raw", "/map_frontier"):
            message = node.messages.get(topic)
            if message is None:
                print(f"{topic}: timeout")
            else:
                nearest = _summarize(
                    topic,
                    message.info,
                    message.data,
                    position.x,
                    position.y,
                    lambda value: value >= 65,
                )
                if topic == "/map_frontier_raw":
                    nearest_static = nearest
        for topic in ("/global_costmap/costmap_raw", "/local_costmap/costmap_raw"):
            message = node.messages.get(topic)
            if message is None:
                print(f"{topic}: timeout")
            else:
                _summarize(
                    topic,
                    message.metadata,
                    message.data,
                    position.x,
                    position.y,
                    lambda value: 253 <= value <= 254,
                )
        scan = node.messages.get("/scan_mid360")
        if scan is not None and nearest_static is not None:
            bearing = math.atan2(
                nearest_static[1] - position.y,
                nearest_static[0] - position.x,
            ) - robot_yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            index = int(round((bearing - scan.angle_min) / scan.angle_increment))
            neighbors = []
            for offset in range(-5, 6):
                sample = index + offset
                if 0 <= sample < len(scan.ranges):
                    value = float(scan.ranges[sample])
                    neighbors.append("inf" if math.isinf(value) else (
                        "nan" if math.isnan(value) else f"{value:.2f}"
                    ))
            print(
                f"nearest_static_world=({nearest_static[0]:.3f},{nearest_static[1]:.3f}) "
                f"relative_bearing={bearing:.3f}rad scan_index={index} "
                f"scan_neighbors={neighbors}"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
