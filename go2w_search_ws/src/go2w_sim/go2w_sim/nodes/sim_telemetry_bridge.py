"""SimTelemetryBridge: /odom_planar → /wheel_feedback (String JSON).

真机 nx_sensor_node 发 /wheel_feedback (LowState 反推); 仿真缺, 致 nx_motion_node
ScanFreshnessWatchdog/drive_session 卡 BOOT_HOLD (waiting_for_sdk_and_feedback).
本桥订阅 /odom_planar 反推 Telemetry, 复用 build_wheel_feedback_payload 发
/wheel_feedback, 让 motion 状态机真跑 (BOOT_HOLD→PARKED→NAV_ACTIVE).

sport_mode 按 odom 速度推断, 复现真机 boot/nav 行为 (motion_types.Go2WModeProfile):
  - 停 (vx≈0) → mode 6 (JOINT_LOCK): boot_hold 观测 joint_lock+stopped → PARKED
  - 动 (vx>阈值) → mode 3 (WHEEL_LOCOMOTION): activating 观测 wheel_mode → NAV_ACTIVE
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


# sport_mode → PhysicalMode (motion_types.Go2WModeProfile._MAP)
_WHEEL_LOCOMOTION_MODE = 3   # 狗在动: WHEEL (ACTIVATING→NAV_ACTIVE 推进)
_JOINT_LOCK_MODE = 6         # 狗趴下: JOINT_LOCK (BOOT_HOLD→PARKED 推进)
# 与 motion_machine wheels_stopped 阈值同量级 (运动/静止分界)
_MOVING_VX_THRESHOLD = 0.02


class SimTelemetryBridge(Node):
    """planar_move /odom_planar → /wheel_feedback JSON 让 motion 状态机跑.

    planar_move 不俯仰 (roll/pitch ≈ 0); wheel_dq 从 odom twist.linear.x 反推
    (4 轮简化同速); sport_mode 按速度推断 (停=JOINT_LOCK/动=WHEEL);
    battery/error/motor_lost 静态健康值.
    """

    def __init__(self):
        super().__init__('sim_telemetry_bridge')
        self._sample_id = 0
        self._last_odom = None
        self._pub = self.create_publisher(String, '/wheel_feedback', 10)
        self.create_subscription(Odometry, '/odom_planar', self._on_odom, 10)
        # 订 /cmd_vel: motion BalanceStand/MoveTo 发布 → sport_mode=3 (WHEEL_LOCOMOTION).
        # planar_move 不模拟 BalanceStand 状态转换 (/cmd_vel=0 不动), 真机 SDK 激活后
        # wheel_balance 即使 vx=0. 用 /cmd_vel 发布 (非 vx 值) 判定激活 → NAV_ACTIVE.
        self._cmd_vel_active = False
        self._last_cmd_vel_sec = 0.0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        # 20Hz feedback (实机 ~50Hz; 20Hz 够 ScanFreshnessWatchdog/DriveWatchdog)
        self.create_timer(0.05, self._publish)
        self.get_logger().info(
            'SimTelemetryBridge: /odom_planar → /wheel_feedback '
            '(20Hz, sport_mode: stop→6/JOINT_LOCK, move→3/WHEEL)')

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        # motion 任何 /cmd_vel 发布 (BalanceStand vx=0 或 MoveTo vx>0) → 狗激活
        self._cmd_vel_active = True
        self._last_cmd_vel_sec = self.get_clock().now().nanoseconds * 1e-9

    def _publish(self) -> None:
        self._sample_id += 1
        vx = 0.0
        if self._last_odom is not None:
            vx = float(self._last_odom.twist.twist.linear.x)
        # planar_move 4 轮简化: 同速 (wheel_dq 不影响状态机判断, 只要有值)
        wheel_dq = [vx, vx, vx, vx]
        # sport_mode: /cmd_vel 发布 (motion 激活) → 3 WHEEL; 超时 2s 无 cmd_vel (狗停) → 6 JOINT_LOCK
        # (回 parked 让后续导航可激活; 否则永远 3 卡 parking 不回 parked, 第二次导航拒).
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._cmd_vel_active and (now_sec - self._last_cmd_vel_sec) > 2.0:
            self._cmd_vel_active = False
        sport_mode = (_WHEEL_LOCOMOTION_MODE if self._cmd_vel_active
                      else _JOINT_LOCK_MODE)
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
