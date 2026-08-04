"""Convert raw MID360 returns into Nav2 observations.

The safety scan defaults to the driver's raw ``/livox/lidar`` CustomMsg rather
than FAST_LIO output.  Localization estimators may diverge, restart, or reject a
frame without removing Nav2's live obstacle source.  The optional legacy
``/cloud_registered_body`` input remains available for diagnostics.

The node levels the measured 20 degree sensor mount, filters floor/self returns,
and publishes both a PointCloud2 obstacle source and a LaserScan safety/panel
view.  No Unitree built-in sensor topic is consumed.
"""

import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

try:
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
except ImportError:  # Unit tests and developer workstations need no Livox SDK.
    LivoxCustomMsg = None


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def _transform_points(points, rotation, translation):
    """Apply ``p_target = R_target_source p_source + t`` to row vectors."""
    values = np.asarray(points, dtype=np.float64)
    return values @ np.asarray(rotation, dtype=np.float64).T + np.asarray(
        translation, dtype=np.float64
    )


def _scale_points(points, scale):
    """Convert the source XYZ unit to metres before geometric filtering."""
    value = float(scale)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("input XYZ scale must be finite and positive")
    return np.asarray(points, dtype=np.float64) * value


def _points_to_scan(points, bins, range_min, range_max):
    """Project XYZ points into a nearest-return 360 degree LaserScan."""
    ranges = np.full(int(bins), np.inf, dtype=np.float32)
    values = np.asarray(points)
    if values.size == 0:
        return ranges.tolist()
    radii = np.hypot(values[:, 0], values[:, 1])
    valid = np.isfinite(radii) & (radii >= range_min) & (radii <= range_max)
    if not np.any(valid):
        return ranges.tolist()
    radii = radii[valid]
    angles = np.arctan2(values[valid, 1], values[valid, 0])
    indices = np.floor((angles + math.pi) * bins / (2.0 * math.pi)).astype(np.int64)
    indices %= bins
    np.minimum.at(ranges, indices, radii.astype(np.float32))
    return ranges.tolist()


def _merge_scan_ranges(history):
    """Merge recent sparse scans while preserving the nearest return per bin."""
    if not history:
        return []
    return np.min(np.asarray(history, dtype=np.float32), axis=0).tolist()


def _advance_sample_deadline(now_monotonic, next_allowed_monotonic, period_sec):
    """Bound expensive Python point decoding while always taking a new frame."""
    now = float(now_monotonic)
    deadline = float(next_allowed_monotonic)
    period = max(0.0, float(period_sec))
    if now < deadline:
        return False, deadline
    return True, now + period


def _decode_xyz(msg):
    """Decode little-endian float32 x/y/z fields without a per-point loop."""
    if msg.is_bigendian or msg.point_step <= 0 or not msg.data:
        return np.empty((0, 3), dtype=np.float32)
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    count = len(msg.data) // int(msg.point_step)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * int(msg.point_step))
    raw = raw.reshape(count, int(msg.point_step))
    columns = [
        raw[:, fields[name].offset:fields[name].offset + 4]
        .copy().reshape(-1).view("<f4")
        for name in ("x", "y", "z")
    ]
    return np.column_stack(columns)


def _decode_livox_xyz(msg):
    """Decode Livox CustomMsg points without depending on FAST_LIO output."""
    points = getattr(msg, "points", None)
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    try:
        count = min(len(points), max(0, int(getattr(msg, "point_num", len(points)))))
    except (TypeError, ValueError):
        count = len(points)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float32)
    flat = np.fromiter(
        (
            value
            for point in points[:count]
            for value in (point.x, point.y, point.z)
        ),
        dtype=np.float32,
        count=count * 3,
    )
    if flat.size != count * 3:
        return np.empty((0, 3), dtype=np.float32)
    return flat.reshape((-1, 3))


def _filter_nav_points(points, range_min, range_max, min_height, max_height):
    """Keep obstacle surfaces while rejecting the measured floor and chassis."""
    values = np.asarray(points)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    finite = np.isfinite(values).all(axis=1)
    radii = np.hypot(values[:, 0], values[:, 1])
    keep = (
        finite
        & (radii >= range_min)
        & (radii <= range_max)
        & (values[:, 2] >= min_height)
        & (values[:, 2] <= max_height)
    )
    # Remove every return inside the same padded 0.90 x 0.64 m footprint used
    # by Nav2.  The narrower historical box leaked wheel/chassis returns when
    # BalanceStand changed posture; those false ~0.28 m hits latched the
    # pure-turn clearance guard even in an otherwise empty room.
    self_return = (
        (values[:, 0] >= -0.45) & (values[:, 0] <= 0.45)
        & (values[:, 1] >= -0.32) & (values[:, 1] <= 0.32)
    )
    return values[keep & ~self_return].astype(np.float32, copy=False)


class Mid360NavBridge(Node):
    def __init__(self):
        super().__init__("mid360_nav_bridge")
        self.declare_parameter("input_source", "livox_custom")
        self.declare_parameter("body_to_base_x", 0.0)
        self.declare_parameter("body_to_base_y", 0.0)
        self.declare_parameter("body_to_base_z", 0.0)
        self.declare_parameter("body_to_base_roll", 0.0)
        self.declare_parameter("body_to_base_pitch", -0.3490658504)
        self.declare_parameter("body_to_base_yaw", 0.0)
        # FAST_LIO mid360.yaml mapping.extrinsic_* (body/IMU <- lidar).
        self.declare_parameter("lidar_to_body_x", -0.011)
        self.declare_parameter("lidar_to_body_y", -0.02329)
        self.declare_parameter("lidar_to_body_z", 0.04412)
        self.declare_parameter("lidar_to_body_roll", 0.0)
        self.declare_parameter("lidar_to_body_pitch", 0.0)
        self.declare_parameter("lidar_to_body_yaw", 0.0)
        # Livox CustomMsg and FAST_LIO use metres.  Keep the scale explicit so
        # another source can be integrated without weakening sanity filters;
        # a diverged estimator must be rejected, not silently shrunk.
        self.declare_parameter("input_xyz_scale", 1.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("input_hz", 5.0)
        self.declare_parameter("range_min", 0.20)
        self.declare_parameter("range_max", 8.0)
        # Field captures put the leveled floor in z=[-0.60, -0.50] m.  Keep
        # only returns at least about 15 cm above that plane so floor jitter
        # cannot accumulate into a rolling-costmap obstacle carpet.
        self.declare_parameter("min_height", -0.45)
        self.declare_parameter("max_height", 1.50)
        self.declare_parameter("scan_bins", 360)
        self.declare_parameter("scan_memory_sec", 0.5)

        xyz = np.array([
            float(self.get_parameter("body_to_base_x").value),
            float(self.get_parameter("body_to_base_y").value),
            float(self.get_parameter("body_to_base_z").value),
        ])
        r_body_base = _rpy_matrix(
            float(self.get_parameter("body_to_base_roll").value),
            float(self.get_parameter("body_to_base_pitch").value),
            float(self.get_parameter("body_to_base_yaw").value),
        )
        # Config is E=body<-base; incoming points need base<-body=inv(E).
        self._rotation = r_body_base.T
        self._translation = -self._rotation @ xyz
        lidar_to_body_xyz = np.array([
            float(self.get_parameter("lidar_to_body_x").value),
            float(self.get_parameter("lidar_to_body_y").value),
            float(self.get_parameter("lidar_to_body_z").value),
        ])
        lidar_to_body_rotation = _rpy_matrix(
            float(self.get_parameter("lidar_to_body_roll").value),
            float(self.get_parameter("lidar_to_body_pitch").value),
            float(self.get_parameter("lidar_to_body_yaw").value),
        )
        self._raw_rotation = self._rotation @ lidar_to_body_rotation
        self._raw_translation = (
            self._rotation @ lidar_to_body_xyz + self._translation
        )
        self._input_xyz_scale = float(
            self.get_parameter("input_xyz_scale").value
        )
        if (not math.isfinite(self._input_xyz_scale)
                or self._input_xyz_scale <= 0.0):
            raise ValueError("input_xyz_scale must be finite and positive")
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._min_height = float(self.get_parameter("min_height").value)
        self._max_height = float(self.get_parameter("max_height").value)
        self._bins = int(self.get_parameter("scan_bins").value)
        self._scan_memory_sec = float(self.get_parameter("scan_memory_sec").value)
        if not math.isfinite(self._scan_memory_sec) or self._scan_memory_sec <= 0.0:
            raise ValueError("scan_memory_sec must be finite and positive")
        input_hz = float(self.get_parameter("input_hz").value)
        if not math.isfinite(input_hz) or input_hz <= 0.0:
            raise ValueError("input_hz must be finite and positive")
        self._input_period_sec = 1.0 / input_hz
        self._next_input_t = 0.0

        self._lock = threading.Lock()
        self._pending = []
        self._scan_history = deque()
        self._last_stamp = None
        self._first_publish_logged = False
        self._cloud_pub = self.create_publisher(PointCloud2, "/mid360/points_nav", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan_mid360", 10)
        self._cloud_group = MutuallyExclusiveCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()
        latest_sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._input_source = str(
            self.get_parameter("input_source").value
        ).strip().lower()
        if self._input_source == "livox_custom":
            if LivoxCustomMsg is None:
                raise RuntimeError(
                    "livox_ros_driver2.msg.CustomMsg is required for livox_custom input"
                )
            self._sub = self.create_subscription(
                LivoxCustomMsg, "/livox/lidar", self._on_livox,
                latest_sensor_qos, callback_group=self._cloud_group
            )
            input_description = "/livox/lidar (raw CustomMsg)"
        elif self._input_source == "fastlio_body":
            self._sub = self.create_subscription(
                PointCloud2, "/cloud_registered_body", self._on_fastlio_cloud,
                latest_sensor_qos, callback_group=self._cloud_group
            )
            input_description = "/cloud_registered_body (FAST_LIO)"
        else:
            raise ValueError(
                "input_source must be 'livox_custom' or 'fastlio_body'"
            )
        hz = float(self.get_parameter("publish_hz").value)
        if not math.isfinite(hz) or hz <= 0.0:
            raise ValueError("publish_hz must be finite and positive")
        self._publish_timer = self.create_timer(
            1.0 / hz, self._publish, callback_group=self._timer_group
        )
        self.get_logger().info(
            f"MID360 Nav bridge: {input_description} -> "
            f"/mid360/points_nav + /scan_mid360 @ {hz:.1f}Hz"
        )

    def _queue_source_points(self, points, stamp, rotation, translation):
        points = _scale_points(points, self._input_xyz_scale)
        if points.size == 0:
            return
        points = _transform_points(points, rotation, translation)
        filtered = _filter_nav_points(
            points,
            self._range_min,
            self._range_max,
            self._min_height,
            self._max_height,
        )
        with self._lock:
            # The costmap needs the latest obstacle geometry, not a lossless
            # history which can recreate sensor backlog under CPU pressure.
            self._pending = [filtered]
            self._last_stamp = stamp

    def _input_is_due(self):
        accept, self._next_input_t = _advance_sample_deadline(
            time.monotonic(), self._next_input_t, self._input_period_sec
        )
        return accept

    def _on_livox(self, msg):
        if not self._input_is_due():
            return
        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", ""))
        if frame_id and frame_id != "livox_frame":
            self.get_logger().warning(
                f"accept raw MID360 frame {frame_id!r}; calibrated for 'livox_frame'",
                throttle_duration_sec=5.0,
            )
        self._queue_source_points(
            _decode_livox_xyz(msg), msg.header.stamp,
            self._raw_rotation, self._raw_translation,
        )

    def _on_fastlio_cloud(self, msg):
        if not self._input_is_due():
            return
        if msg.header.frame_id != "body":
            self.get_logger().warning(
                f"ignore MID360 cloud frame {msg.header.frame_id!r}, expected 'body'",
                throttle_duration_sec=5.0,
            )
            return
        self._queue_source_points(
            _decode_xyz(msg), msg.header.stamp,
            self._rotation, self._translation,
        )

    def _publish(self):
        with self._lock:
            if self._last_stamp is None or not self._pending:
                return
            batches, stamp = self._pending, self._last_stamp
            self._pending = []
        nonempty = [batch for batch in batches if batch.size]
        points = np.concatenate(nonempty, axis=0) if nonempty else np.empty(
            (0, 3), dtype=np.float32
        )
        cloud = PointCloud2()
        cloud.header.frame_id = "base_link"
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = np.ascontiguousarray(points, dtype="<f4").tobytes()
        # Stamp after all potentially preempted serialization work.  The
        # MID360 sensor clock trails wall time, and stamping earlier here made
        # Nav2 discard observations after a long scheduler pause.
        output_stamp = self.get_clock().now().to_msg()
        cloud.header.stamp = output_stamp
        self._cloud_pub.publish(cloud)
        if not self._first_publish_logged:
            self.get_logger().info(f"first nav cloud published ({cloud.width} points)")

        scan = LaserScan()
        scan.header = cloud.header
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self._bins
        scan.angle_max = scan.angle_min + (self._bins - 1) * scan.angle_increment
        scan.scan_time = 0.1
        scan.time_increment = 0.0
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        current_ranges = _points_to_scan(
            points, self._bins, self._range_min, self._range_max
        )
        now_monotonic = time.monotonic()
        self._scan_history.append((now_monotonic, current_ranges))
        while (
            self._scan_history
            and now_monotonic - self._scan_history[0][0] > self._scan_memory_sec
        ):
            self._scan_history.popleft()
        scan.ranges = _merge_scan_ranges(
            [ranges for _, ranges in self._scan_history]
        )
        scan.header.stamp = self.get_clock().now().to_msg()
        if not self._first_publish_logged:
            self.get_logger().info("first MID360 LaserScan built; publishing")
        self._scan_pub.publish(scan)
        if not self._first_publish_logged:
            self._first_publish_logged = True
            self.get_logger().info("first MID360 LaserScan published")


def main(args=None):
    rclpy.init(args=args)
    node = Mid360NavBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
