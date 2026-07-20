#!/usr/bin/env python3
"""Prepare the live SLAM grid for persistent, movable-obstacle navigation.

SLAM Toolbox owns ``/map_frontier_raw`` and its map-to-odom transform. This
node adds unknown cells around the grid and publishes ``/map_frontier`` for
Nav2.  In that derived output only, fresh laser evidence removes stale occupied
cells left by moved objects.  It never alters TF, the raw SLAM map, or unknown
space.
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def clear_obstacles_visible_in_scan(
    data,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    origin_yaw,
    robot_x,
    robot_y,
    robot_yaw,
    angle_min,
    angle_increment,
    ranges,
    range_min,
    range_max,
    endpoint_margin=0.2,
    no_return_min_run=7,
    no_return_clear_range=1.0,
    visibility_neighbor_bins=1,
    occupied_threshold=65,
):
    """Clear stale occupied cells that a supported laser sector sees through.

    Unknown cells stay unknown and a margin before every measured endpoint is
    preserved.  A healthy scan may also clear a short range inside a broad,
    contiguous positive-infinity sector.  Isolated no-return bins and an
    all-infinite/unhealthy scan can never erase persistent occupancy.  Each
    cell is checked against neighbouring beams so a single beam through a
    narrow gap cannot erase a wall, while scan-smear between ray centrelines
    is removed completely.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError("occupancy data shape is invalid")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map resolution must be finite and positive")
    if not math.isfinite(endpoint_margin) or endpoint_margin < 0.0:
        raise ValueError("endpoint margin must be finite and non-negative")
    if int(no_return_min_run) < 1:
        raise ValueError("no_return_min_run must be positive")
    if not math.isfinite(no_return_clear_range) or no_return_clear_range <= 0.0:
        raise ValueError("no_return_clear_range must be finite and positive")
    if int(visibility_neighbor_bins) < 0:
        raise ValueError("visibility_neighbor_bins must be non-negative")
    if not ranges or not math.isfinite(angle_increment) or angle_increment == 0.0:
        return list(data), 0
    cleaned = list(data)
    cleared = set()
    map_cosine = math.cos(origin_yaw)
    map_sine = math.sin(origin_yaw)
    measurements = [float(value) for value in ranges]
    scan_is_healthy = any(
        math.isfinite(value) and range_min <= value <= range_max
        for value in measurements
    )
    no_return_indices = set()
    if scan_is_healthy:
        runs = []
        run_start = None
        for index in range(len(measurements) + 1):
            is_no_return = (
                index < len(measurements)
                and math.isinf(measurements[index])
                and measurements[index] > 0.0
            )
            if is_no_return and run_start is None:
                run_start = index
            elif not is_no_return and run_start is not None:
                runs.append((run_start, index))
                run_start = None
        minimum_run = int(no_return_min_run)
        for start, end in runs:
            if end - start >= minimum_run:
                no_return_indices.update(range(start, end))
        if len(runs) >= 2 and runs[0][0] == 0 and runs[-1][1] == len(measurements):
            if runs[0][1] + len(measurements) - runs[-1][0] >= minimum_run:
                no_return_indices.update(range(runs[0][0], runs[0][1]))
                no_return_indices.update(range(runs[-1][0], runs[-1][1]))
    clear_limits = []
    for index, measured_range in enumerate(measurements):
        if math.isfinite(measured_range):
            if measured_range < range_min or measured_range > range_max:
                clear_limits.append(None)
                continue
            clear_limit = measured_range - endpoint_margin
        elif index in no_return_indices:
            clear_limit = min(float(range_max), float(no_return_clear_range))
        else:
            clear_limits.append(None)
            continue
        if clear_limit <= range_min:
            clear_limits.append(None)
            continue
        clear_limits.append(clear_limit)

    beam_count = len(measurements)
    angular_span = abs(angle_increment) * beam_count
    wraps = angular_span >= 2.0 * math.pi - abs(angle_increment) * 2.0
    neighbor_bins = int(visibility_neighbor_bins)
    half_cell_diagonal = resolution / math.sqrt(2.0)
    for cell, occupancy in enumerate(data):
        if int(occupancy) < occupied_threshold:
            continue
        cell_x = cell % width
        cell_y = cell // width
        local_x = (cell_x + 0.5) * resolution
        local_y = (cell_y + 0.5) * resolution
        world_x = origin_x + map_cosine * local_x - map_sine * local_y
        world_y = origin_y + map_sine * local_x + map_cosine * local_y
        dx = world_x - robot_x
        dy = world_y - robot_y
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance) or distance < range_min:
            continue
        relative_angle = math.atan2(dy, dx) - robot_yaw
        relative_angle = math.atan2(
            math.sin(relative_angle), math.cos(relative_angle))
        raw_index = (relative_angle - angle_min) / angle_increment
        nearest = int(round(raw_index))
        if wraps:
            nearest %= beam_count
        else:
            nearest = min(beam_count - 1, max(0, nearest))
        beam_angle = angle_min + nearest * angle_increment
        angular_error = abs(math.atan2(
            math.sin(relative_angle - beam_angle),
            math.cos(relative_angle - beam_angle),
        ))
        cell_angle = math.asin(min(1.0, half_cell_diagonal / distance))
        if angular_error > abs(angle_increment) * 0.5 + cell_angle + 1e-9:
            continue
        support = []
        for offset in range(-neighbor_bins, neighbor_bins + 1):
            support_index = nearest + offset
            if wraps:
                support_index %= beam_count
            elif support_index < 0 or support_index >= beam_count:
                continue
            limit = clear_limits[support_index]
            if limit is None:
                support = []
                break
            support.append(float(limit))
        if support and distance <= min(support) + 1e-9:
            cleaned[cell] = 0
            cleared.add(cell)
    return cleaned, len(cleared)


def pad_occupancy_data(data, width, height, resolution, padding_m):
    """Return a centred, unknown-padded row-major occupancy grid."""
    if width <= 0 or height <= 0:
        raise ValueError("map width and height must be positive")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map resolution must be finite and positive")
    if not math.isfinite(padding_m) or padding_m < 0.0:
        raise ValueError("padding_m must be finite and non-negative")
    if len(data) != width * height:
        raise ValueError(
            f"occupancy data length {len(data)} != {width} * {height}"
        )

    pad_cells = int(math.ceil(padding_m / resolution))
    padded_width = width + 2 * pad_cells
    padded_height = height + 2 * pad_cells
    padded = [-1] * (padded_width * padded_height)
    for source_row in range(height):
        source_start = source_row * width
        target_start = (source_row + pad_cells) * padded_width + pad_cells
        padded[target_start : target_start + width] = data[
            source_start : source_start + width
        ]
    return padded, padded_width, padded_height, pad_cells


def shift_grid_origin(origin_x, origin_y, origin_yaw, padding_distance):
    """Shift an OccupancyGrid origin by (-padding, -padding) in grid axes."""
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        origin_x - padding_distance * cosine + padding_distance * sine,
        origin_y - padding_distance * sine - padding_distance * cosine,
    )


def point_boundary_margin(
    point_x,
    point_y,
    origin_x,
    origin_y,
    origin_yaw,
    width,
    height,
    resolution,
):
    """Return signed minimum distance from a world point to the grid boundary."""
    dx = point_x - origin_x
    dy = point_y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return min(
        local_x,
        local_y,
        width * resolution - local_x,
        height * resolution - local_y,
    )


def _quaternion_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _transient_map_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MapPaddingBridge(Node):
    def __init__(self):
        super().__init__("map_padding_bridge")
        self.declare_parameter("input_topic", "/map_frontier_raw")
        self.declare_parameter("output_topic", "/map_frontier")
        self.declare_parameter("padding_m", 2.0)
        self.declare_parameter("scan_topic", "/scan_mid360")
        self.declare_parameter("pose_topic", "/localization_pose")
        self.declare_parameter("dynamic_clearing_enabled", True)
        self.declare_parameter("dynamic_clearing_max_age", 1.0)
        self.declare_parameter("dynamic_clearing_endpoint_margin", 0.2)
        self.declare_parameter("dynamic_clearing_no_return_min_run", 7)
        self.declare_parameter("dynamic_clearing_no_return_range", 1.0)
        self.declare_parameter("dynamic_clearing_visibility_neighbor_bins", 1)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        self._padding_m = float(self.get_parameter("padding_m").value)
        self._dynamic_clearing_enabled = bool(
            self.get_parameter("dynamic_clearing_enabled").value
        )
        self._dynamic_clearing_max_age = float(
            self.get_parameter("dynamic_clearing_max_age").value
        )
        self._dynamic_clearing_endpoint_margin = float(
            self.get_parameter("dynamic_clearing_endpoint_margin").value
        )
        self._dynamic_clearing_no_return_min_run = int(
            self.get_parameter("dynamic_clearing_no_return_min_run").value
        )
        self._dynamic_clearing_no_return_range = float(
            self.get_parameter("dynamic_clearing_no_return_range").value
        )
        self._dynamic_clearing_visibility_neighbor_bins = int(
            self.get_parameter(
                "dynamic_clearing_visibility_neighbor_bins").value
        )
        if not math.isfinite(self._padding_m) or self._padding_m <= 0.0:
            raise ValueError("padding_m must be finite and greater than zero")
        if (
            not math.isfinite(self._dynamic_clearing_max_age)
            or self._dynamic_clearing_max_age <= 0.0
        ):
            raise ValueError("dynamic_clearing_max_age must be positive")
        if (
            not math.isfinite(self._dynamic_clearing_endpoint_margin)
            or self._dynamic_clearing_endpoint_margin < 0.0
        ):
            raise ValueError("dynamic_clearing_endpoint_margin must be non-negative")
        if self._dynamic_clearing_no_return_min_run < 1:
            raise ValueError("dynamic_clearing_no_return_min_run must be positive")
        if (
            not math.isfinite(self._dynamic_clearing_no_return_range)
            or self._dynamic_clearing_no_return_range <= 0.0
        ):
            raise ValueError("dynamic_clearing_no_return_range must be positive")
        if self._dynamic_clearing_visibility_neighbor_bins < 0:
            raise ValueError(
                "dynamic_clearing_visibility_neighbor_bins must be non-negative")

        qos = _transient_map_qos()
        self._publisher = self.create_publisher(OccupancyGrid, output_topic, qos)
        self._subscription = self.create_subscription(
            OccupancyGrid, input_topic, self._on_map, qos
        )
        self._pose_subscription = self.create_subscription(
            Odometry, pose_topic, self._on_pose, 10
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 10
        )
        self._latest_pose = None
        self._latest_pose_received = float("-inf")
        self._latest_scan = None
        self._latest_scan_received = float("-inf")
        self._last_shape = None
        self.get_logger().info(
            f"padding {input_topic} -> {output_topic} by {self._padding_m:.2f}m; "
            f"finite-ray stale-obstacle clearing="
            f"{self._dynamic_clearing_enabled}"
        )

    def _on_pose(self, message):
        self._latest_pose = message
        self._latest_pose_received = time.monotonic()

    def _on_scan(self, message):
        self._latest_scan = message
        self._latest_scan_received = time.monotonic()

    def _dynamically_cleaned_data(self, source):
        if not self._dynamic_clearing_enabled:
            return list(source.data), 0
        now = time.monotonic()
        if (
            self._latest_pose is None
            or self._latest_scan is None
            or now - self._latest_pose_received > self._dynamic_clearing_max_age
            or now - self._latest_scan_received > self._dynamic_clearing_max_age
        ):
            return list(source.data), 0
        pose = self._latest_pose
        scan = self._latest_scan
        map_frame = str(source.header.frame_id).lstrip("/")
        pose_frame = str(pose.header.frame_id).lstrip("/")
        scan_frame = str(scan.header.frame_id).lstrip("/")
        if (map_frame and pose_frame and map_frame != pose_frame) or scan_frame != "base_link":
            return list(source.data), 0
        position = pose.pose.pose.position
        return clear_obstacles_visible_in_scan(
            data=source.data,
            width=int(source.info.width),
            height=int(source.info.height),
            resolution=float(source.info.resolution),
            origin_x=float(source.info.origin.position.x),
            origin_y=float(source.info.origin.position.y),
            origin_yaw=_quaternion_yaw(source.info.origin.orientation),
            robot_x=float(position.x),
            robot_y=float(position.y),
            robot_yaw=_quaternion_yaw(pose.pose.pose.orientation),
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            ranges=scan.ranges,
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            endpoint_margin=self._dynamic_clearing_endpoint_margin,
            no_return_min_run=self._dynamic_clearing_no_return_min_run,
            no_return_clear_range=self._dynamic_clearing_no_return_range,
            visibility_neighbor_bins=(
                self._dynamic_clearing_visibility_neighbor_bins),
        )

    def _on_map(self, source):
        try:
            cleaned, cleared_count = self._dynamically_cleaned_data(source)
            data, width, height, pad_cells = pad_occupancy_data(
                cleaned,
                int(source.info.width),
                int(source.info.height),
                float(source.info.resolution),
                self._padding_m,
            )
        except ValueError as exc:
            self.get_logger().error(f"rejecting malformed raw map: {exc}")
            return

        padding_distance = pad_cells * float(source.info.resolution)
        yaw = _quaternion_yaw(source.info.origin.orientation)
        origin_x, origin_y = shift_grid_origin(
            float(source.info.origin.position.x),
            float(source.info.origin.position.y),
            yaw,
            padding_distance,
        )

        output = OccupancyGrid()
        output.header = source.header
        output.info.map_load_time = source.info.map_load_time
        output.info.resolution = source.info.resolution
        output.info.width = width
        output.info.height = height
        output.info.origin.position.x = origin_x
        output.info.origin.position.y = origin_y
        output.info.origin.position.z = source.info.origin.position.z
        output.info.origin.orientation = source.info.origin.orientation
        output.data = data
        self._publisher.publish(output)
        if cleared_count:
            self.get_logger().info(
                f"cleared {cleared_count} stale occupied map cells from current scan",
                throttle_duration_sec=2.0,
            )

        shape = (source.info.width, source.info.height, width, height, pad_cells)
        if shape != self._last_shape:
            self.get_logger().info(
                f"map {shape[0]}x{shape[1]} -> {width}x{height}; "
                f"effective padding {padding_distance:.2f}m"
            )
            self._last_shape = shape


class MapMarginGate(Node):
    def __init__(self):
        super().__init__("map_margin_gate")
        self._map = None
        self._pose = None
        self.result = None
        self._map_subscription = self.create_subscription(
            OccupancyGrid, "/map_frontier", self._on_map, _transient_map_qos()
        )
        self._pose_subscription = self.create_subscription(
            Odometry, "/localization_pose", self._on_pose, 10
        )

    def _on_map(self, message):
        self._map = message
        self._evaluate()

    def _on_pose(self, message):
        self._pose = message
        self._evaluate()

    def _evaluate(self):
        if self._map is None or self._pose is None:
            return
        map_frame = self._map.header.frame_id.lstrip("/")
        pose_frame = self._pose.header.frame_id.lstrip("/")
        if map_frame and pose_frame and map_frame != pose_frame:
            self.result = (False, float("-inf"), map_frame, pose_frame)
            return
        info = self._map.info
        position = self._pose.pose.pose.position
        margin = point_boundary_margin(
            float(position.x),
            float(position.y),
            float(info.origin.position.x),
            float(info.origin.position.y),
            _quaternion_yaw(info.origin.orientation),
            int(info.width),
            int(info.height),
            float(info.resolution),
        )
        self.result = (True, margin, map_frame, pose_frame)


def _run_margin_gate(minimum_margin, timeout_s, ros_args):
    rclpy.init(args=ros_args)
    node = MapMarginGate()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is None:
            print("map margin gate timed out waiting for map and localization", file=sys.stderr)
            return 1
        compatible, margin, map_frame, pose_frame = node.result
        if not compatible:
            print(
                f"map margin gate frame mismatch: map={map_frame} pose={pose_frame}",
                file=sys.stderr,
            )
            return 1
        if margin < minimum_margin:
            print(
                f"map margin gate failed: {margin:.3f}m < {minimum_margin:.3f}m",
                file=sys.stderr,
            )
            return 1
        print(f"map margin gate passed: {margin:.3f}m >= {minimum_margin:.3f}m")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--check-margin",
        type=float,
        default=None,
        metavar="METERS",
        help="check /localization_pose margin inside /map_frontier and exit",
    )
    parser.add_argument("--timeout", type=float, default=20.0)
    known, ros_args = parser.parse_known_args(args)
    if known.check_margin is not None:
        return _run_margin_gate(known.check_margin, known.timeout, ros_args)

    rclpy.init(args=ros_args)
    node = MapPaddingBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
