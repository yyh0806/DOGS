"""SimTelemetryBridge: /odom_planar → /wheel_feedback (String JSON).

真机 nx_sensor_node 发 /wheel_feedback (LowState 反推); 仿真缺, 致 nx_motion_node
ScanFreshnessWatchdog/drive_session 卡 BOOT_HOLD (waiting_for_sdk_and_feedback).
本桥订阅 /odom_planar 反推 Telemetry, 复用 build_wheel_feedback_payload 发
/wheel_feedback, 让 motion 状态机真跑 (BOOT_HOLD→PARKED→NAV_ACTIVE).

sport_mode 判定 (仿真核心难题: controller_server 无目标也 20Hz 发布零速 /cmd_vel,
简单超时永不触发 → sport_mode 永不切 JOINT_LOCK → PARKING 卡死):
  - /cmd_vel 任意发布 → sport_mode=3 WHEEL (激活期, 含 balance 零速)
  - /dog_state drive_session 含 "active" → sport_mode=3 WHEEL (维持激活,
    避免 balance 稳态无 /cmd_vel 时误切 JOINT_LOCK)
  - 切换 JOINT_LOCK: /cmd_vel_nav 停发 >2s (Nav2 无 goal) AND
    /dog_state drive_session 不含 "active" AND odom 静止 >60 帧

零改 motion_machine (信任状态机代码), 仅让仿真 feedback 跟真机 LowState 同形态.

spec 2026-07-25-real-fidelity §5.
"""
import json

from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist

try:
    from go2w_bridge.motion_protocol import build_wheel_feedback_payload
except ImportError:  # pytest / direct-file
    from motion_protocol import build_wheel_feedback_payload


_WHEEL_LOCOMOTION_MODE = 3   # WHEEL (ACTIVATING→NAV_ACTIVE 推进)
_JOINT_LOCK_MODE = 6         # JOINT_LOCK (BOOT_HOLD→PARKED 推进)
_MOVING_VX_THRESHOLD = 0.02


class SimTelemetryBridge(Node):
    """planar_move /odom_planar → /wheel_feedback JSON 让 motion 状态机跑."""

    def __init__(self):
        super().__init__('sim_telemetry_bridge')
        self._sample_id = 0
        self._last_odom = None
        self._pub = self.create_publisher(String, '/wheel_feedback', 10)
        self.create_subscription(Odometry, '/odom_planar', self._on_odom, 10)

        # /cmd_vel: 非零速度才标记激活 (Move); 零速不续期 (Nav2 controller
        # 无目标也 20Hz 发布零速 Twist → 需要忽略以免 sport_mode 永不回 JOINT_LOCK).
        # BalanceStand 零速 → 不触发; 依赖 /dog_state drive_session="active" 维持.
        self._cmd_vel_seen = False
        self._last_cmd_vel_sec = 0.0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        # /cmd_vel_nav: 仅 Nav2 active goal 时发布; 用于降级判定
        self._cmd_vel_nav_seen = False
        self._last_cmd_vel_nav_sec = 0.0
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_cmd_vel_nav, 10)

        # /dog_state: drive_session 含 "active" → 运动持续激活 (balance/manual/nav)
        self._drive_session_active = False
        self.create_subscription(String, '/dog_state', self._on_dog_state, 10)

        self._odom_stopped_frames = 0
        self.create_timer(0.05, self._publish)
        self.get_logger().info(
            'SimTelemetryBridge: /odom_planar → /wheel_feedback '
            '(20Hz, sport_mode: cmd/cmd_nav/active→3/WHEEL, all_quiet→6/JOINT_LOCK)')

    # ── subscriptions ──────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        # 仅非零速度标记激活; 零速忽略 (Nav2 controller 无目标也 20Hz 零速)
        vx = abs(float(getattr(msg.linear, 'x', 0.0) or 0.0))
        vy = abs(float(getattr(msg.linear, 'y', 0.0) or 0.0))
        wz = abs(float(getattr(msg.angular, 'z', 0.0) or 0.0))
        if max(vx, vy, wz) < 0.001:
            return
        self._cmd_vel_seen = True
        self._last_cmd_vel_sec = self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel_nav(self, msg: Twist) -> None:
        self._cmd_vel_nav_seen = True
        self._last_cmd_vel_nav_sec = self.get_clock().now().nanoseconds * 1e-9

    def _on_dog_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            ds = str(data.get("drive_session", "") or "")
        except (json.JSONDecodeError, TypeError):
            return
        self._drive_session_active = "active" in ds

    # ── publish ────────────────────────────────────────────────────────

    def _publish(self) -> None:
        self._sample_id += 1
        vx = 0.0
        if self._last_odom is not None:
            vx = float(self._last_odom.twist.twist.linear.x)
        wheel_dq = [vx, vx, vx, vx]

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # /cmd_vel 单帧续期 (需持续发布才保持)
        cmd_vel_fresh = (
            self._cmd_vel_seen
            and (now_sec - self._last_cmd_vel_sec) < 0.1
        )
        self._cmd_vel_seen = False

        # /cmd_vel_nav 超时检测
        if (self._cmd_vel_nav_seen
                and (now_sec - self._last_cmd_vel_nav_sec) > 2.0):
            self._cmd_vel_nav_seen = False

        # odom 静止计数
        if abs(vx) < _MOVING_VX_THRESHOLD:
            self._odom_stopped_frames += 1
        else:
            self._odom_stopped_frames = 0

        # sport_mode 判定:
        #   保持 WHEEL: /cmd_vel 活跃 或 drive_session active 或 /cmd_vel_nav 活跃
        #   切换 JOINT_LOCK: 以上全不满足 + odom 静止 >60 帧 (3s)
        keep_wheel = (
            cmd_vel_fresh
            or self._drive_session_active
            or self._cmd_vel_nav_seen
        )
        allow_joint_lock = (
            not keep_wheel
            and self._odom_stopped_frames >= 60
        )
        sport_mode = (_JOINT_LOCK_MODE if allow_joint_lock
                      else _WHEEL_LOCOMOTION_MODE)

        payload = build_wheel_feedback_payload(
            sample_id=self._sample_id,
            source_stamp=self.get_clock().now().nanoseconds * 1e-9,
            wheel_dq=wheel_dq,
            battery_soc=80.0,
            bms_status=0,
            sport_mode=sport_mode,
            sport_error_code=0,
            roll=0.0,
            pitch=0.0,
            motor_lost=[0, 0, 0, 0],
            extras={'sport_progress': 0, 'gait_type': 0},
        )
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main() -> None:
    import rclpy
    rclpy.init()
    node = SimTelemetryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
