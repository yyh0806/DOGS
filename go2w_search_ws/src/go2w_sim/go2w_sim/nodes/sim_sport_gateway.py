"""SimSportGateway: GO2W_SIM adapter, Effect -> Twist -> /cmd_vel -> planar_move.

替代 SportGatewayClient (Unix socket -> sport lease gateway -> 真狗四足步态).
上层 MotionController / Go2WMotionMachine 零感知 — 仅 nx_motion_node:284
在 GO2W_SIM=1 时换此 adapter; 状态机/watchdog/owner/drive_session 全真跑,
这是"仿真栈跟真机一致, 仅简化运动模型"的切点 (spec
2026-07-25-real-fidelity-simulation-design.md §3-4).

契约对齐 SportGatewayClient (sport_gateway_client.py):
  initialize()           -> InitializationResult(code=0, motion_service="ai-w")
  check_motion_service() -> InitializationResult
  execute(effect: Effect) -> CommandReceipt(code=0)
  close()                -> None

receipt 流向: controller._execute -> adapter.execute(effect) -> machine.record_receipt.
因此返回真 CommandReceipt (type compat), 不能 duck-type.
"""
from __future__ import annotations

from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from go2w_bridge.motion_types import CommandReceipt, Effect, InitializationResult
except ImportError:  # Direct-file / pytest without colcon install
    from motion_types import CommandReceipt, Effect, InitializationResult


class SimSportGateway:
    """Sim adapter: translate motion Effect into /cmd_vel Twist for planar_move.

    真机 SportGatewayClient 把 Effect 经 socket 发给 sport lease gateway 进程;
    仿真直接 publish /cmd_vel, planar_move (libgazebo_ros_planar_move.so) 消费.
    返回 CommandReceipt(code=0) — planar_move 永不"传输失败", 所以 transport_ok 恒真.
    """

    # motion_machine 实际产生的 Effect operation (grep motion_machine.py 确认):
    #   "Move"     — 速度控制, arguments=(vx, vy, wz)  (motion_machine.py:388-391)
    #   "MoveZero" — 零速止动 (estop/停车), arguments=()  (motion_machine.py:155)
    _MOVE = "Move"
    _MOVE_ZERO = "MoveZero"

    def __init__(self, node: Node) -> None:
        self._node = node
        # planar_move 硬编码订阅 /cmd_vel (不读 SDF commandTopic/ros remapping).
        # 回环由 motion tick (~2Hz) 限制, 不爆炸; nx_motion_node 收 SimSportGateway
        # 输出只 enqueue 同值 velocity, 不放大.
        self._pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self._node.get_logger().info(
            "SimSportGateway active: Effect -> /cmd_vel -> planar_move (GO2W_SIM)")

    def initialize(self) -> InitializationResult:
        """planar_move 永远就绪; 返回 ai-w 通过 nx_motion_node 门禁 (code==0, ai-w)."""
        return InitializationResult(code=0, motion_service="ai-w", raw_mode="ai-w")

    def check_motion_service(self) -> InitializationResult:
        return self.initialize()

    def execute(self, effect: Effect) -> CommandReceipt:
        twist = Twist()
        if effect.operation == self._MOVE and effect.arguments:
            args = effect.arguments
            twist.linear.x = float(args[0]) if len(args) > 0 else 0.0
            twist.linear.y = float(args[1]) if len(args) > 1 else 0.0
            twist.angular.z = float(args[2]) if len(args) > 2 else 0.0
        # MoveZero / 未知 operation -> 零速 Twist (安全止动, 同 SportGatewayClient MoveZero)
        self._pub.publish(twist)
        return CommandReceipt(
            operation=effect.operation,
            code=0,
            sequence=effect.sequence,
            physical_confirmed=False,
        )

    def close(self) -> None:
        """停车: 发零速 Twist 再退出 (同真机 MoveZero 收尾语义)."""
        try:
            self._pub.publish(Twist())
        except Exception:  # noqa: BLE001 - 关闭路径 best-effort
            pass
