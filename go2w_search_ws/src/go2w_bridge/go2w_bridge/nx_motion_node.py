"""载荷NX 运动控制节点 — 订阅 /cmd_vel + /cmd_pose → 控狗。

go2w_bridge 的运动控制职责从"PC直连狗"迁移到"载荷NX"。
这样:
1. 控狗走网线直连狗主控(不依赖热点), 可靠
2. 持续持有 lease → 压制狗主控里的残留乱跑程序(它抢不到lease)
3. 看门狗在NX上 → 即使笔记本↔NX热点断了, NX也会自动停狗

ROS2 接口:
  订阅 /cmd_vel  (geometry_msgs/Twist)  - 速度指令 vx vy vyaw
  订阅 /cmd_pose (std_msgs/String)      - "stand"/"sit"/"estop"
  发布 /dog_state (std_msgs/String JSON) - 狗当前状态 (供监控)

状态机 (复用 panel.py 验证过的逻辑):
  DISCONNECTED → STANDING → STOPPED (BalanceStand静止, 每0.5s发零速保lease)
  STOPPED → MOVING (Move@20Hz)
  MOVING → STOPPED (StopMove + BalanceStand, 看门狗1s超时自动停)
  任意 → EMERGENCY (Damp趴下)

运行 (载荷NX):
  export LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$LD_LIBRARY_PATH
  source /opt/ros/humble/setup.bash
  ros2 run go2w_bridge nx_motion_node
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from unitree_sdk2py.core.channel import ChannelFactory
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    SDK_OK = True
except Exception as _e:
    SDK_OK = False
    _SDK_ERR = str(_e)


# 状态
DISCONNECTED, STANDING, STOPPED, MOVING, SITTING, SEATED, EMERGENCY = range(7)


class NxMotionNode(Node):
    def __init__(self):
        super().__init__('nx_motion_node')

        # 连狗网卡: 优先用 DOG_INTERFACE 环境变量 (service 部署时传入),
        # 兜底硬编码默认值 (老 NX 的网卡名, 新 NX 部署时会被覆盖)
        _default_iface = os.environ.get('DOG_INTERFACE', 'enxc8a362616c4c')
        self.declare_parameter('dog_interface', _default_iface)
        self.declare_parameter('stand_on_start', True)
        self.declare_parameter('cmd_timeout', 1.0)   # 看门狗超时
        self.declare_parameter('move_rate', 20.0)     # MOVING发Move频率
        self._cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value

        # 控制状态 (锁保护) — 提前初始化, 保证 _publish_state 在 SDK 未就绪时也安全
        self._lock = threading.Lock()
        self._state = DISCONNECTED
        self._vx = self._vy = self._vyaw = 0.0
        self._last_cmd_time = 0.0
        self._pose_cmd = None  # 'stand'/'sit'/'estop'
        self._cmd_vel_count = 0  # 可观测性: 收到的 /cmd_vel 累计(排查 subscription 静默失效)

        # SDK 句柄 — _init_sdk 填充; None 表示狗主控未就绪 (狗没上电/链路 DOWN)
        self._factory = None
        self._sport = None
        self._SWITCHGAIT_API_ID = 1011
        self._sdk_ready = False
        self._sdk_retry_sec = 5.0   # 狗没电时 SDK 重试间隔 (比原崩溃循环 2s 省 60% CPU/日志)

        # ROS2 接口
        # ⚠️ rclpy 坑根治 (2026-07-02): publisher/subscription 必须存 self, 否则 GC 回收静默失效。
        # subscription 延迟到 SDK 就绪后创建: SDK 没起来时收 /cmd_vel 会误触发状态机但控不了狗。
        self._cmd_vel_sub = None
        self._cmd_pose_sub = None
        self._state_pub = self.create_publisher(String, '/dog_state', 10)

        # SDK 初始化在后台线程重试 — 狗主控没上电时: 不崩进程、不爆 journal、不 CPU 空转,
        # 狗上电后 _sdk_init_loop 自动检测到并恢复。原版 SportClient 构造直接访问
        # CycloneDDS participant._ref, 网卡 DOWN 时 participant=None → AttributeError 崩 →
        # systemd Restart=always 每 2s 一个循环 (实测 restart counter 998 次)。
        if not SDK_OK:
            self.get_logger().error(f"unitree_sdk2py 不可用: {_SDK_ERR}; 节点空转, 控狗功能禁用")
        else:
            threading.Thread(target=self._sdk_init_loop, daemon=True).start()

        # 状态发布始终起 (SDK 未就绪时发 "DISCONNECTED + sdk_ready=false",
        # web 能区分"motion 在等狗"还是"motion 失联")
        self.create_timer(0.5, self._publish_state)

    # ---- SDK 初始化 (狗主控没上电时不崩, 后台重试到狗上电自动恢复) ----
    def _init_sdk(self) -> bool:
        """初始化 unitree SDK (ChannelFactory + SportClient). 成功 True, 失败 False (不抛).

        狗主控没上电时: 连狗网卡链路 DOWN → CycloneDDS domain participant=None →
        SportClient 构造访问 None._ref 抛 AttributeError. catch 住返回 False,
        _sdk_init_loop 定期重试, 狗上电自动恢复。
        """
        iface = self.get_parameter('dog_interface').get_parameter_value().string_value
        try:
            factory = ChannelFactory()
            try:
                factory.Init(0, iface)
            except Exception as e:
                # 网卡名不对时 fallback 自动检测 (与原逻辑一致)
                self.get_logger().warning(f"网卡 {iface} Init 失败 {e}, 自动检测")
                factory.Init(0, None)
            self._factory = factory

            # 关键检测点: SportClient 构造触发 DDS topic 创建, participant=None 时
            # 在 cyclonedds/topic.py 访问 None._ref 抛 AttributeError — catch 住即"狗没就绪"。
            sport = SportClient(enableLease=True)
            sport.SetTimeout(10.0)
            sport.Init()
            # ⚠️ 历史遗留: 手动注册 SwitchGait API 1011。实测 SwitchGait(1)=trot 让 Go2W
            # 摔倒 (docs/TECH_DECISIONS.md 第一节), 已停用, 注册保留待清理。
            sport._RegistApi(self._SWITCHGAIT_API_ID, 0)
            time.sleep(2)  # 等 lease 激活
            self._sport = sport
            self.get_logger().info(f"SportClient lease 已激活 (网卡={iface}, 持续持有压制残留程序)")
            return True
        except Exception as e:
            self.get_logger().warning(
                f"SDK 初始化失败 (狗主控没上电? 网卡 {iface} 链路 DOWN?): "
                f"{type(e).__name__}: {e}; {self._sdk_retry_sec:.0f}s 后重试")
            self._sport = None
            self._factory = None
            return False

    def _sdk_init_loop(self):
        """后台线程: 反复重试 SDK 初始化直到成功 (狗上电自动恢复). 独立线程, sleep 不阻塞 executor."""
        while not self._sdk_ready and rclpy.ok():
            if self._init_sdk():
                self._on_sdk_ready()
            else:
                time.sleep(self._sdk_retry_sec)

    def _on_sdk_ready(self):
        """SDK 就绪后: 创建 /cmd_vel /cmd_pose 订阅 + 启动控制/看门狗线程 + 自动站立."""
        self._sdk_ready = True
        self.get_logger().info("SDK 就绪, 启用 /cmd_vel /cmd_pose 订阅 + 控制/看门狗线程")
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._cmd_pose_sub = self.create_subscription(String, '/cmd_pose', self._on_cmd_pose, 10)
        # 启动控制线程 (独立线程, 避免 SDK 调用阻塞 ROS2 executor)
        threading.Thread(target=self._ctrl_loop, daemon=True).start()
        # 看门狗线程
        threading.Thread(target=self._watchdog, daemon=True).start()
        # 自动站立
        if self.get_parameter('stand_on_start').get_parameter_value().bool_value:
            with self._lock: self._pose_cmd = 'stand'

    # ---- ROS2 回调 ----
    def _on_cmd_vel(self, msg):
        self._cmd_vel_count += 1  # 可观测性: 累计收到的指令数(_publish_state 带入 /dog_state)
        with self._lock:
            # 坐标系: vx前后 vy左右 vyaw旋转(正=左转)
            # 实测 Go2W SDK Move(x,y,z): z正=左转 (与cmd_vel angular.z约定一致, 无需反转)
            self._vx = msg.linear.x
            self._vy = msg.linear.y
            self._vyaw = msg.angular.z
            self._last_cmd_time = time.time()
            if self._state in (STOPPED, MOVING):
                self._state = MOVING

    def _on_cmd_pose(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('stand', 'sit', 'estop'):
            with self._lock: self._pose_cmd = cmd
            self.get_logger().info(f"收到姿态指令: {cmd}")

    def _watchdog(self):
        """看门狗: MOVING 状态超过 cmd_timeout 无新指令 → 自动停。"""
        while True:
            with self._lock:
                state = self._state
                last = self._last_cmd_time
            if state == MOVING and last > 0 and time.time() - last > self._cmd_timeout:
                with self._lock:
                    if self._state == MOVING:
                        self._state = STOPPED
                        self._vx = self._vy = self._vyaw = 0.0
                self.get_logger().info(f"看门狗: {self._cmd_timeout}s无指令, 自动停")
            time.sleep(0.2)

    def _ctrl_loop(self):
        """控制循环 (复用 panel.py 验证过的状态机)。所有 SDK 调用只在此线程。"""
        self.get_logger().info("控制线程启动")
        last_zero_move = 0.0
        while True:
            try:
                # 消费姿态指令 (优先)
                cmd = None
                with self._lock:
                    cmd = self._pose_cmd; self._pose_cmd = None
                if cmd == 'stand':
                    self._do_stand(); last_zero_move = 0; continue
                if cmd == 'sit':
                    self._do_sit(); last_zero_move = 0; continue
                if cmd == 'estop':
                    self._do_estop(); last_zero_move = 0; continue

                # 速度控制
                state, vx, vy, vyaw = STOPPED, 0.0, 0.0, 0.0
                with self._lock:
                    state, vx, vy, vyaw = self._state, self._vx, self._vy, self._vyaw
                if state == STOPPED:
                    # STOPPED: 高频(20Hz)发Move(0,0,0)钉住瞬时速度,
                    # 且每0.5s补一次StopMove清除运动控制器内部残留目标速度。
                    # Go2W ai-w 轮式模式: 仅Move(0,0,0)无法停轮子(实测轮子仍
                    # 以~1rad/s转), 必须StopMove()才能真正刹住。
                    # ⚠️ 与 SDK_CAPABILITIES.md"StopMove对Go2W无效"冲突, 待 LowState 轮速
                    # 实测裁决(见 TECH_DECISIONS §一 实车 TODO)。
                    now = time.time()
                    if now - last_zero_move > 0.5:
                        self._sport.StopMove()
                        last_zero_move = now
                    self._sport.Move(0, 0, 0)
                elif state == MOVING:
                    self._sport.Move(vx, vy, vyaw)
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"控制循环异常: {e}")
                time.sleep(0.5)

    def _switch_gait(self, gait_type):
        """⚠️ 已禁用(当前无调用方)。SwitchGait(1)=trot 实测让 Go2W 摔倒,
        见 docs/TECH_DECISIONS.md 第一节。保留仅供后续研究轮式步态切换参考, 勿盲调。"""
        import json as _json
        p = {"data": gait_type}
        code, _ = self._sport._Call(self._SWITCHGAIT_API_ID, _json.dumps(p))
        if code != 0:
            self.get_logger().warning(f"SwitchGait({gait_type}) 返回 code={code}")
        return code

    def _do_stand(self):
        try:
            with self._lock:
                self._state = STANDING
                self._vx = self._vy = self._vyaw = 0.0
            self.get_logger().info("STANDING: StandUp → BalanceStand → Move(0,0,0)")
            # 站立序列对齐 web/panel.py:RobotConnection._do_stand (SDK_CAPABILITIES 实测能动版)。
            # 关键: BalanceStand 让狗进入主动平衡态, 轮子才接受 Move 速度指令; 早期版本误删
            # 此步(误判"后滑危险"), 导致 Move 返回 code=0 但轮子不转。后滑是 BalanceStand
            # 进入瞬间轮子调平衡的副作用, 紧接 Move(0,0,0) 可压住。
            # ⚠️ 待实车验证(硬件装完后): vx=0.1 短按, 手放 Damp 急停。
            self._sport.StandUp(); time.sleep(2)
            self._sport.BalanceStand(); time.sleep(0.5)
            self._sport.Move(0, 0, 0)
            with self._lock:
                self._state = STOPPED
                self._vx = self._vy = self._vyaw = 0.0
                self._last_cmd_time = 0.0
            self.get_logger().info("STANDING → STOPPED")
        except Exception as e:
            self.get_logger().error(f"站立失败: {e}")
            with self._lock: self._state = STOPPED

    def _do_sit(self):
        try:
            with self._lock: self._state = SITTING
            self.get_logger().info("SITTING: StopMove → Damp")
            self._sport.Move(0, 0, 0); time.sleep(0.05)
            self._sport.StopMove(); time.sleep(0.3)
            self._sport.Damp()
            with self._lock:
                self._state = SEATED
                self._last_cmd_time = 0.0
            self.get_logger().info("SITTING → SEATED")
        except Exception as e:
            self.get_logger().error(f"坐下失败: {e}")
            with self._lock: self._state = STOPPED

    def _do_estop(self):
        try:
            with self._lock: self._state = EMERGENCY
            self._sport.Damp()
            self.get_logger().warn("⚠️ EMERGENCY: Damp 趴下")
            with self._lock: self._last_cmd_time = 0.0
        except Exception as e:
            self.get_logger().error(f"急停失败: {e}")

    def _publish_state(self):
        with self._lock:
            state, vx, vy, vyaw = self._state, self._vx, self._vy, self._vyaw
        names = ['DISCONNECTED','STANDING','STOPPED','MOVING','SITTING','SEATED','EMERGENCY']
        msg = String()
        msg.data = json.dumps({
            'state': names[state] if state < len(names) else str(state),
            'sdk_ready': self._sdk_ready,  # False = 狗没上电/SDK 未就绪, motion 在等 (web 可显"等待狗")
            'vx': round(vx, 3), 'vy': round(vy, 3), 'vyaw': round(vyaw, 3),
            'cmd_vel_n': self._cmd_vel_count,  # 卡在 0 = subscription 又静默失效了
        })
        self._state_pub.publish(msg)

    def destroy_node(self):
        # SDK 未就绪时 self._sport=None, 跳过趴下 (没句柄也没必要)
        if self._sport is not None:
            try:
                self._sport.Move(0, 0, 0); time.sleep(0.05)
                self._sport.StopMove(); time.sleep(0.2)
                self._sport.Damp()
                self.get_logger().info("退出: 已趴下释放")
            except Exception: pass
        else:
            self.get_logger().info("退出: SDK 未就绪, 无需趴下")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NxMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
