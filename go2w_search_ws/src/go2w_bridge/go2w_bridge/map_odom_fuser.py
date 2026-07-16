"""Publish a Nav2 TF/odometry chain with FAST_LIO as the pose backbone.

Go2W wheels rotate to balance and can slip without chassis displacement, so
wheel integration is not a valid navigation pose source.  A wheel-first fuser
can therefore report progress and even goal success while the robot has not
moved.  Valid, leveled FAST_LIO motion exclusively owns ``odom -> base_link``;
wheel odometry remains subscribed for diagnostics but never moves the TF tree.
If LIO stops, the TF/localization stream becomes stale and the existing Nav2
health gates fail closed instead of dead-reckoning through wheel slip.
"""

import math
import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def _tf_to_mat(transform):
    t = transform
    qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        rotation = np.eye(3)
    else:
        s = 2.0 / n
        rotation = np.array([
            [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw),
             s * (qx * qz + qy * qw)],
            [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz),
             s * (qy * qz - qx * qw)],
            [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw),
             1 - s * (qx * qx + qy * qy)],
        ])
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return result


def _rpy_to_mat(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _build_static_tf(x, y, z, roll, pitch, yaw):
    result = np.eye(4)
    result[:3, :3] = _rpy_to_mat(roll, pitch, yaw)
    result[:3, 3] = [x, y, z]
    return result


def _conjugate_pose(T_camera_body, T_body_base):
    return np.linalg.inv(T_body_base) @ T_camera_body @ T_body_base


def _relative_planar_pose(initial_leveled, current_leveled):
    """Anchor FAST_LIO's arbitrary initial frame and expose SE(2) to Nav2."""
    relative = np.linalg.inv(initial_leveled) @ current_leveled
    yaw = math.atan2(float(relative[1, 0]), float(relative[0, 0]))
    return _build_static_tf(
        float(relative[0, 3]), float(relative[1, 3]), 0.0,
        0.0, 0.0, yaw,
    )


def _propagate_map_pose(observed_map_base, observed_odom_base, current_odom_base):
    """Propagate a SLAM map observation with continuous wheel odometry."""
    map_to_odom = (
        np.asarray(observed_map_base, dtype=np.float64)
        @ np.linalg.inv(np.asarray(observed_odom_base, dtype=np.float64))
    )
    current_map_base = map_to_odom @ np.asarray(
        current_odom_base, dtype=np.float64
    )
    return map_to_odom, current_map_base


def _lio_pose_is_plausible(previous_position, previous_stamp_ns, position,
                           stamp_ns, max_speed, jump_slack, max_abs_position):
    """Reject non-finite, globally absurd, or physically impossible LIO poses."""
    current = np.asarray(position, dtype=np.float64)
    if current.shape != (3,) or not np.isfinite(current).all():
        return False
    if float(np.linalg.norm(current)) > float(max_abs_position):
        return False
    if previous_position is None or previous_stamp_ns is None:
        return stamp_ns > 0
    previous = np.asarray(previous_position, dtype=np.float64)
    dt = (int(stamp_ns) - int(previous_stamp_ns)) / 1e9
    if not math.isfinite(dt) or dt <= 0.0:
        return False
    allowed = float(jump_slack) + float(max_speed) * dt
    return float(np.linalg.norm(current - previous)) <= allowed


def _lio_message_age_is_fresh(
    stamp_ns,
    now_ns,
    *,
    max_age_sec,
    future_tolerance_sec=0.05,
):
    """Accept only finite, positive LIO stamps close to the ROS wall clock."""
    try:
        stamp_ns = int(stamp_ns)
        now_ns = int(now_ns)
        maximum = float(max_age_sec)
        future_tolerance = float(future_tolerance_sec)
    except (TypeError, ValueError, OverflowError):
        return False
    if stamp_ns <= 0 or now_ns <= 0:
        return False
    if (
        not math.isfinite(maximum)
        or not math.isfinite(future_tolerance)
        or maximum < 0.0
        or future_tolerance < 0.0
    ):
        return False
    age_sec = (now_ns - stamp_ns) / 1e9
    return -future_tolerance <= age_sec <= maximum


def _mat_to_tf(matrix):
    tx, ty, tz = (float(matrix[0, 3]), float(matrix[1, 3]),
                  float(matrix[2, 3]))
    r = matrix[:3, :3]
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r[2, 1] - r[1, 2]) / scale
        qy = (r[0, 2] - r[2, 0]) / scale
        qz = (r[1, 0] - r[0, 1]) / scale
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / scale
        qx = 0.25 * scale
        qy = (r[0, 1] + r[1, 0]) / scale
        qz = (r[0, 2] + r[2, 0]) / scale
    elif r[1, 1] > r[2, 2]:
        scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / scale
        qx = (r[0, 1] + r[1, 0]) / scale
        qy = 0.25 * scale
        qz = (r[1, 2] + r[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / scale
        qx = (r[0, 2] + r[2, 0]) / scale
        qy = (r[1, 2] + r[2, 1]) / scale
        qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    return tx, ty, tz, qx / norm, qy / norm, qz / norm, qw / norm


def _pose_to_mat(pose):
    transform = type("TransformLike", (), {})()
    transform.translation = pose.position
    transform.rotation = pose.orientation
    return _tf_to_mat(transform)


class MapOdomFuser(Node):
    def __init__(self):
        super().__init__("map_odom_fuser")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("fastlio_world", "camera_init")
        self.declare_parameter("fastlio_body", "body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("body_to_base_x", 0.0)
        self.declare_parameter("body_to_base_y", 0.0)
        self.declare_parameter("body_to_base_z", 0.0)
        self.declare_parameter("body_to_base_roll", 0.0)
        self.declare_parameter("body_to_base_pitch", -0.3490658504)
        self.declare_parameter("body_to_base_yaw", 0.0)
        self.declare_parameter("max_lio_speed", 3.0)
        self.declare_parameter("lio_jump_slack", 0.5)
        self.declare_parameter("max_abs_position", 10000.0)
        self.declare_parameter("max_lio_age_sec", 2.0)  # 放宽容差: livox 已改 host time (偏移~0), 留余量容忍 FastLIO 处理抖动; 见 memory livox-host-time-clock-rootcause
        self.declare_parameter("max_lio_future_skew_sec", 0.05)
        self.declare_parameter("max_slam_tf_age_sec", 0.5)
        self.declare_parameter("max_slam_tf_future_skew_sec", 0.3)
        self.declare_parameter("publish_map_to_odom", True)
        self.declare_parameter("use_slam_pose", False)

        self._world = str(self.get_parameter("world_frame").value)
        self._fl_world = str(self.get_parameter("fastlio_world").value)
        self._fl_body = str(self.get_parameter("fastlio_body").value)
        self._odom = str(self.get_parameter("odom_frame").value)
        self._base = str(self.get_parameter("base_frame").value)
        self._body_to_base = _build_static_tf(
            *[
                float(self.get_parameter(name).value)
                for name in (
                    "body_to_base_x", "body_to_base_y", "body_to_base_z",
                    "body_to_base_roll", "body_to_base_pitch", "body_to_base_yaw",
                )
            ]
        )
        self._base_from_body_rotation = np.linalg.inv(self._body_to_base)[:3, :3]
        self._last_stamp_ns = None
        self._last_position = None
        self._initial_leveled = None
        self._latest_lio_planar = None
        self._max_lio_speed = float(self.get_parameter("max_lio_speed").value)
        self._lio_jump_slack = float(self.get_parameter("lio_jump_slack").value)
        self._max_abs_position = float(self.get_parameter("max_abs_position").value)
        self._max_lio_age_sec = float(
            self.get_parameter("max_lio_age_sec").value
        )
        self._max_lio_future_skew_sec = float(
            self.get_parameter("max_lio_future_skew_sec").value
        )
        self._max_slam_tf_age_sec = float(
            self.get_parameter("max_slam_tf_age_sec").value
        )
        self._max_slam_tf_future_skew_sec = float(
            self.get_parameter("max_slam_tf_future_skew_sec").value
        )
        self._publish_map_to_odom = bool(
            self.get_parameter("publish_map_to_odom").value
        )
        self._use_slam_pose = bool(self.get_parameter("use_slam_pose").value)
        self._latest_twist = None
        self._latest_wheel_odom = None
        self._latest_odom_planar = None
        self._slam_map_to_odom = None
        self._last_slam_tf_wall_ns = None
        self._pending_slam_map_base = None
        self._slam_pose_covariance = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        # Dynamic TF is perishable.  Humble's Python broadcaster defaults to
        # reliable depth=100, which amplifies acknowledgement/backpressure
        # when all Nav2 processes join the UDP-only DDS graph at once.  Keep
        # reliability for the stock TF listeners but retain only the newest
        # transform for every reader.
        latest_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._broadcaster = TransformBroadcaster(self, qos=latest_tf_qos)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._pose_pub = self.create_publisher(Odometry, '/localization_pose', 10)
        # Physical pose is perishable sensor data.  A reliable depth-N reader
        # replays old poses after transient DDS/CPU backpressure (for example
        # while both Nav2 costmaps activate), keeping TF seconds behind even
        # after FAST_LIO itself has caught up.  Consume only the newest pose;
        # the age gate below still fails closed if fresh data actually stops.
        # Keep RELIABLE to match FAST_LIO's writer: this FastDDS build exhibits
        # severe delivery latency when its reliable writer is matched to a
        # best-effort Python reader, while a reliable latest-only probe stays
        # below the commissioned latency bound.
        latest_lio_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._lio_sub = self.create_subscription(Odometry, '/Odometry',
                                                 self._on_lio,
                                                 latest_lio_qos)
        self._wheel_sub = self.create_subscription(Odometry, '/wheel_odom', self._on_wheel, 20)
        self._slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/slam_pose', self._on_slam_pose, 10
        )
        self._slam_tf_timer = self.create_timer(
            0.1, self._refresh_map_to_odom
        )
        self.get_logger().info(
            "FAST_LIO pose backbone: /Odometry -> odom/base_link; "
            "/wheel_odom is diagnostic-only; "
            f"map_tf_owner={'fuser' if self._publish_map_to_odom else 'slam'}; "
            f"localization_pose={'slam+lio' if self._use_slam_pose else 'lio'}"
        )

    def _refresh_map_to_odom(self):
        """Consume SLAM Toolbox's canonical map->odom TF correction."""
        if not self._use_slam_pose or self._publish_map_to_odom:
            return
        now = self.get_clock().now()
        try:
            transform = self._tf_buffer.lookup_transform(self._world, self._odom, Time())
        except TransformException:
            if (
                self._last_slam_tf_wall_ns is not None
                and (now.nanoseconds - self._last_slam_tf_wall_ns) / 1e9
                > self._max_slam_tf_age_sec
            ):
                self._slam_map_to_odom = None
            return
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        if not _lio_message_age_is_fresh(
            stamp_ns,
            now.nanoseconds,
            max_age_sec=self._max_slam_tf_age_sec,
            future_tolerance_sec=self._max_slam_tf_future_skew_sec,
        ):
            self._slam_map_to_odom = None
            return
        self._slam_map_to_odom = _tf_to_mat(transform.transform)
        self._last_slam_tf_wall_ns = now.nanoseconds

    def _on_slam_pose(self, msg):
        if not self._use_slam_pose:
            return
        if msg.header.frame_id != self._world:
            self.get_logger().warning(
                f"ignore SLAM pose in frame {msg.header.frame_id!r}",
                throttle_duration_sec=5.0,
            )
            return
        observed_map_base = _pose_to_mat(msg.pose.pose)
        self._slam_pose_covariance = msg.pose.covariance
        if self._latest_odom_planar is None:
            self._pending_slam_map_base = observed_map_base
            return
        self._slam_map_to_odom, current_map_base = _propagate_map_pose(
            observed_map_base,
            self._latest_odom_planar,
            self._latest_odom_planar,
        )
        self._pending_slam_map_base = None
        self._publish_map_localization(current_map_base, msg.header.stamp)

    def _publish_map_localization(self, map_base, stamp):
        localization = Odometry()
        localization.header.stamp = stamp
        localization.header.frame_id = self._world
        localization.child_frame_id = self._base
        self._fill_pose(localization, _mat_to_tf(map_base))
        if self._slam_pose_covariance is not None:
            localization.pose.covariance = self._slam_pose_covariance
        if self._latest_twist is not None:
            localization.twist = self._latest_twist
        self._pose_pub.publish(localization)

    def _on_lio(self, msg):
        if msg.header.frame_id != self._fl_world or msg.child_frame_id != self._fl_body:
            self.get_logger().warning(
                f"ignore FAST_LIO odometry {msg.header.frame_id!r} -> "
                f"{msg.child_frame_id!r}", throttle_duration_sec=5.0
            )
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        now = self.get_clock().now()
        if not _lio_message_age_is_fresh(
            stamp_ns,
            now.nanoseconds,
            max_age_sec=self._max_lio_age_sec,
            future_tolerance_sec=self._max_lio_future_skew_sec,
        ):
            age_sec = (now.nanoseconds - stamp_ns) / 1e9
            self.get_logger().error(
                f"reject stale FAST_LIO pose age={age_sec:.3f}s "
                f"limit={self._max_lio_age_sec:.3f}s",
                throttle_duration_sec=1.0,
            )
            return
        if stamp_ns <= 0 or (
            self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns
        ):
            return
        raw_pose = _pose_to_mat(msg.pose.pose)
        position = raw_pose[:3, 3]
        if not _lio_pose_is_plausible(
            self._last_position, self._last_stamp_ns, position, stamp_ns,
            self._max_lio_speed, self._lio_jump_slack, self._max_abs_position,
        ):
            self.get_logger().error(
                f"reject impossible FAST_LIO pose at {position.tolist()}",
                throttle_duration_sec=1.0,
            )
            return
        self._last_stamp_ns = stamp_ns
        self._last_position = position.copy()

        leveled = _conjugate_pose(raw_pose, self._body_to_base)
        if self._initial_leveled is None:
            self._initial_leveled = leveled.copy()
        planar = _relative_planar_pose(self._initial_leveled, leveled)
        self._latest_lio_planar = planar
        self._latest_odom_planar = planar.copy()
        if self._pending_slam_map_base is not None:
            self._slam_map_to_odom, _ = _propagate_map_pose(
                self._pending_slam_map_base, planar, planar
            )
            self._pending_slam_map_base = None
        values = _mat_to_tf(planar)
        # The age gate above proves this is a current physical estimate.  Stamp
        # it at callback time so tf2 can combine it with wall-clock Nav2 goals.
        output_stamp = now.to_msg()
        identity = TransformStamped()
        identity.header.stamp = output_stamp
        identity.header.frame_id = self._world
        identity.child_frame_id = self._odom
        identity.transform.rotation.w = 1.0

        base_tf = TransformStamped()
        base_tf.header.stamp = output_stamp
        base_tf.header.frame_id = self._odom
        base_tf.child_frame_id = self._base
        self._fill_transform(base_tf, values)
        if self._publish_map_to_odom:
            self._broadcaster.sendTransform([identity, base_tf])
        else:
            self._broadcaster.sendTransform(base_tf)

        odom = Odometry()
        odom.header.stamp = output_stamp
        odom.header.frame_id = self._odom
        odom.child_frame_id = self._base
        self._fill_pose(odom, values)
        odom.pose.covariance = msg.pose.covariance
        linear = self._base_from_body_rotation @ np.array([
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        angular = self._base_from_body_rotation @ np.array([
            msg.twist.twist.angular.x, msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular
        odom.twist.covariance = msg.twist.covariance
        self._latest_twist = odom.twist
        self._odom_pub.publish(odom)

        if self._use_slam_pose:
            if self._slam_map_to_odom is not None:
                self._publish_map_localization(
                    self._slam_map_to_odom @ planar, output_stamp
                )
        else:
            localization = Odometry()
            localization.header.stamp = output_stamp
            localization.header.frame_id = self._world
            localization.child_frame_id = self._base
            localization.pose = odom.pose
            localization.twist = odom.twist
            self._pose_pub.publish(localization)

    def _on_wheel(self, msg):
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            return
        # Keep the message available for diagnostics only.  BalanceStand wheel
        # rotation is not chassis translation and must never affect TF, /odom,
        # /localization_pose, or Nav2 progress/goal checking.
        self._latest_wheel_odom = msg

    @staticmethod
    def _fill_transform(msg, values):
        tx, ty, tz, qx, qy, qz, qw = values
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

    @staticmethod
    def _fill_pose(msg, values):
        tx, ty, tz, qx, qy, qz, qw = values
        msg.pose.pose.position.x = tx
        msg.pose.pose.position.y = ty
        msg.pose.pose.position.z = tz
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomFuser()
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
