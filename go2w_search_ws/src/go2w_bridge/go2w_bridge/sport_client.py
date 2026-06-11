"""Go2W Sport Client - 封装 unitree_sdk2py 的运动控制接口。

该模块封装了 Go2W 机器狗的高层运动控制 API，
包括: 站立、坐下、速度控制、移动到目标点、步态切换、运动模式切换。

依赖: unitree_sdk2py (pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git)
"""

import time
import logging
import threading
from typing import Optional, Callable

logger = logging.getLogger(__name__)

# 尝试导入 Unitree SDK
try:
    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    SDK_AVAILABLE = True
    logger.info("unitree_sdk2py 已加载")
except ImportError:
    SDK_AVAILABLE = False
    logger.warning("unitree_sdk2py 未安装，将使用模拟模式")


class Go2WSportClient:
    """Go2W 运动控制客户端。

    封装 unitree_sdk2py 的 SportClient，提供:
    - 连接管理
    - 基本姿态控制 (站立/坐下/平衡)
    - 速度控制
    - 位置移动
    - 运动模式切换 (行走/轮式)
    """

    # 运动模式
    MODE_WALK = 0
    MODE_DRIVE = 1  # Go2W 轮式模式

    # 步态类型
    GAIT_TROT = 1

    def __init__(self, network_interface: str = "", timeout: float = 10.0):
        """初始化运动客户端。

        Args:
            network_interface: 网卡名称 (空=自动检测, 如 "enx001e06300000")
            timeout: 通信超时 (秒)
        """
        self._interface = network_interface
        self._timeout = timeout
        self._client: Optional[SportClient] = None
        self._connected = False
        self._lock = threading.Lock()
        self._state_callback: Optional[Callable] = None

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        """连接到 Go2W 机器狗。

        Returns:
            True 连接成功，False 连接失败
        """
        if not SDK_AVAILABLE:
            logger.info("[模拟模式] 连接到 Go2W")
            self._connected = True
            return True

        try:
            # 初始化 DDS 通道
            factory = ChannelFactory()
            factory.Init(0, self._interface)

            # 创建运动客户端
            self._client = SportClient()
            self._client.SetTimeout(self._timeout)
            self._client.Init()

            self._connected = True
            logger.info("已连接到 Go2W (接口: %s)", self._interface or "auto")
            return True

        except Exception as e:
            logger.error("连接 Go2W 失败: %s", e)
            self._connected = False
            return False

    def disconnect(self):
        """断开连接。"""
        self._connected = False
        self._client = None
        logger.info("已断开 Go2W 连接")

    # ---- 姿态控制 ----

    def damp_stand(self) -> bool:
        """阻尼站立（节能站立模式）。"""
        return self._call("Damp", lambda: self._client.Damp())

    def balance_stand(self) -> bool:
        """平衡站立。"""
        return self._call("BalanceStand", lambda: self._client.BalanceStand())

    def stand_up(self) -> bool:
        """站立。"""
        return self._call("StandUp", lambda: self._client.StandUp())

    def sit_down(self) -> bool:
        """坐下。"""
        return self._call("Sit", lambda: self._client.Sit())

    # ---- 运动控制 ----

    def stop_move(self) -> bool:
        """停止移动。"""
        return self._call("StopMove", lambda: self._client.StopMove())

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> bool:
        """设置速度指令。

        Args:
            vx: 前进速度 (m/s)，正值前进
            vy: 侧移速度 (m/s)，正值左移
            vyaw: 旋转角速度 (rad/s)，正值逆时针
        """
        def cmd():
            self._client.Move(vx, vy, vyaw)
        return self._call("Move", cmd)

    def move_to(self, x: float, y: float, yaw: float) -> bool:
        """移动到相对目标位置。

        Args:
            x: X方向相对位移 (米)
            y: Y方向相对位移 (米)
            yaw: 相对旋转 (弧度)
        """
        def cmd():
            self._client.MoveTo(x, y, yaw)
        return self._call("MoveTo", cmd)

    def continuous_move(self, vx: float, vy: float, vyaw: float, duration: float) -> bool:
        """持续移动指定时间。

        Args:
            vx, vy, vyaw: 速度
            duration: 持续时间 (秒)
        """
        if not self.set_velocity(vx, vy, vyaw):
            return False
        time.sleep(duration)
        return self.stop_move()

    # ---- 模式切换 ----

    def switch_to_drive_mode(self) -> bool:
        """切换到轮式驱动模式 (Go2W 专属)。"""
        return self._call("SwitchMoveMode(DRIVE)",
                          lambda: self._client.SwitchMoveMode(self.MODE_DRIVE))

    def switch_to_walk_mode(self) -> bool:
        """切换到步行模式。"""
        return self._call("SwitchMoveMode(WALK)",
                          lambda: self._client.SwitchMoveMode(self.MODE_WALK))

    def switch_gait(self, gait: int = GAIT_TROT) -> bool:
        """切换步态。

        Args:
            gait: 步态类型 (1=Trot, 2=...)
        """
        return self._call("SwitchGait", lambda: self._client.SwitchGait(gait))

    # ---- 状态订阅 ----

    def subscribe_state(self, callback: Callable):
        """订阅机器人运动状态。

        Args:
            callback: 状态回调函数，接收 SportModeState 消息
        """
        if not SDK_AVAILABLE:
            logger.info("[模拟模式] 状态订阅已注册")
            self._state_callback = callback
            return

        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            sub = ChannelSubscriber("rt SportModeState", SportModeState_)
            sub.Init(callback, 1)
            logger.info("已订阅 Go2W 运动状态")
        except Exception as e:
            logger.error("订阅状态失败: %s", e)

    # ---- 内部方法 ----

    def _call(self, name: str, func: Callable) -> bool:
        """安全调用 SDK 方法。"""
        if not self._connected:
            logger.warning("[未连接] 忽略 %s 调用", name)
            return False

        if not SDK_AVAILABLE:
            logger.debug("[模拟] %s", name)
            return True

        try:
            with self._lock:
                func()
            logger.debug("执行 %s 成功", name)
            return True
        except Exception as e:
            logger.error("执行 %s 失败: %s", name, e)
            return False


def main():
    """独立测试: 连接 Go2W 并执行基本动作。"""
    import argparse

    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')

    parser = argparse.ArgumentParser(description="Go2W Sport Client 测试工具")
    parser.add_argument("--interface", "-i", default="", help="网卡名称")
    parser.add_argument("--action", "-a", default="stand",
                        choices=["stand", "sit", "drive", "walk", "stop", "forward"],
                        help="执行的动作")
    args = parser.parse_args()

    client = Go2WSportClient(network_interface=args.interface)

    if not client.connect():
        print("连接失败!")
        return

    print(f"已连接，执行动作: {args.action}")

    if args.action == "stand":
        client.balance_stand()
    elif args.action == "sit":
        client.sit_down()
    elif args.action == "drive":
        client.switch_to_drive_mode()
    elif args.action == "walk":
        client.switch_to_walk_mode()
    elif args.action == "stop":
        client.stop_move()
    elif args.action == "forward":
        client.continuous_move(0.5, 0.0, 0.0, 2.0)

    print("动作完成")
    client.disconnect()


if __name__ == "__main__":
    main()
