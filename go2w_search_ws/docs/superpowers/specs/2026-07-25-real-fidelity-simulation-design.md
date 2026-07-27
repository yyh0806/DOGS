# Real-Fidelity Simulation Design (真机一致仿真)

> **Status:** 2026-07-25 方向转变 —— 推翻 2026-07-24 plan 的 Task 2 极简决策。
> **Supersedes (partial):** `2026-07-24-gazebo-nav2-simulation-design.md` 中 Task 2(M1b 极简 odom_planar) 与 Task 3(M1c-1 简化双订阅 mock)。Task 1(碰撞)、Task 4-6 框架保留,内容按本 spec 重写。

## 1. 决策记录

- **用户指令(2026-07-25):** "先保证仿真里跑的跟真机一致,不要极简方案,要技术方案一模一样,唯一可以简化就是狗的运动模型。"
- **含义:** 仿真栈必须复刻真机完整业务链(Livox CustomMsg → FastLIO → fuser → nav2-3d costmap → Nav2 → motion_machine → nx_motion_node → adapter → 运动模型)。所有真实业务代码一行不改,**仅**在两处切换:
  1. **运动模型** —— 四足步态固件(Unitree sport lease) → Gazebo kinematic-base(`libgazebo_ros_planar_move.so`)。
  2. **adapter 传输** —— `SportGatewayClient`(Unix socket → sport gateway 进程 → 真狗) → `SimSportGateway`(`/cmd_vel` Twist → planar_move)。
- **地基突破(2026-07-25):** `livox_ros_driver2` 在 ROS2 Humble **build 成功**(colcon exit 0)。2026-07-24 记为"深坑"实为误判 —— 真根因是 `package.xml` 缺 `<member_of_group>rosidl_interface_packages</member_of_group>`(已补) + 诊断记错变量名(`LIVOX_INTERFACE_INCLUDE_DIR` 在当前 CMakeLists 不存在,实际是 `LIVOX_LIDAR_SDK_INCLUDE_DIR`,且 `/usr/local/include/livox_lidar_api.h` 已就位)。SDK2(`liblivox_lidar_sdk_shared.so` 5MB + static 26MB + headers)早已 install 到 `/usr/local`。

## 2. 真机栈 → 仿真映射

| 层 | 真机(NX) | 仿真(WSL2 Gazebo Classic 11) | 改动 |
|---|---|---|---|
| LiDAR 硬件 | Livox MID360 | Gazebo + `ros2_livox_simulation` 插件(`libros2_livox.so`,玫瑰扫描) | URDF 挂载 + pin commit |
| LiDAR 驱动 msg | `livox_ros_driver2`(`livox_ros_driver2/msg/CustomMsg`) | **同(只取 msg 类型,不跑 driver 运行时)** | ✅ build 成功 |
| LIO | `fast_lio` 包 | **同,零改** | 待 build + `mid360.yaml` 话题对齐 |
| TF fuser | `map_odom_fuser` | **同,零改** | 切输入源 `/Odometry` |
| IMU | MID360 内置 | `libgazebo_ros_imu_sensor.so`(原生 m/s²,**无 ×9.80665**,见决策#5) | URDF 挂载 |
| costmap | nav2-3d + scan_probe(rolling + 去 static_layer,方案A) | **同,零改** | config 继承 |
| Nav2 | nav2-3d(DWB + BT navigator) | **同,零改 config** | `nav2_sim_params.yaml` 仅切 scan/odom 源 |
| motion 核心域 | `motion_machine` + `motion_controller` + `motion_safety` | **同,零改**(纯 Python,无 ROS/SDK) | 无 |
| nx_motion_node | 真机 | **+`GO2W_SIM` 分支换 adapter**(nx_motion_node.py:285) | 1 处接口适配 |
| SportGateway → 狗 | `SportGatewayClient`(socket→sport lease→腿) | **`SimSportGateway`**(`/cmd_vel`→planar_move) | 新 adapter(本 spec §4) |
| Telemetry 反馈 | `LowState` from SDK | `/odom_planar` → `Telemetry` 桥(roll/pitch≈0,wheel_dq 反推) | 新 feedback bridge(§5) |
| web 栈 | `nx_web_server` + `nx_exploration_manager` + `nx_navigation_arbiter` + `nx_room_orchestrator` + `nx_person_localizer` + `nx_visibility_coverage` | **同,零改 + `GO2W_SIM` 跳硬件门禁** | 接口层 §6 |
| **运动模型** | **四足步态固件** | **kinematic-base(planar_move)** | ✅ 唯一简化 |

## 3. 核心架构:adapter 注入切点

真机 cmd_vel 链:
```
web/Nav2 goal → velocity_smoother → /cmd_vel_nav
  → nx_motion_node(订阅, enqueue) → actor thread
  → MotionController.update_velocity(owner, velocity)
    → Go2WMotionMachine(纯逻辑, 无 ROS/SDK) 产 Effect
    → MotionController._execute(effects) → adapter.execute(effect)
    → SportGatewayClient.execute() → Unix socket /run/go2w-sport-gateway/sport.sock
      → sport lease → 真狗四足步态
```

`MotionController.attach_adapter(adapter, motion_service)`(`motion_controller.py:97`)是**依赖注入点**。`nx_motion_node.py:285` 构造 `SportGatewayClient`。仿真在该点分支:

```python
# nx_motion_node.py:~285 (示意, 实施时按当前行号)
if os.environ.get('GO2W_SIM'):
    from go2w_sim.nodes.sim_sport_gateway import SimSportGateway
    adapter = SimSportGateway(self)            # /cmd_vel → planar_move
else:
    adapter = SportGatewayClient(self._gateway_socket, ...)
```

上层(订阅/actor/watchdog/owner/drive_session/SportModeState 回报)一行不改,真机状态机真跑。

### Effect / CommandReceipt 契约(`motion_types.py`, 不改)

```python
@dataclass(frozen=True)
class Effect:
    operation: str                         # "Move" | "MoveZero" | (posture/estop 等, 实施时 grep motion_machine.py 补全)
    sequence: int
    arguments: Tuple[float, ...] = ()      # Move: (vx, vy, wz)
    transition_id: Optional[int] = None
    reason: Optional[str] = None

@dataclass(frozen=True)
class CommandReceipt:
    operation: str
    code: int                              # 0 == transport_ok
    sequence: int
    physical_confirmed: bool = False
```

确认的 operation 取值(grep `motion_machine.py`):
- `Effect(operation="Move", arguments=(vx,vy,wz))` —— 速度控制(line 388-391)
- `Effect(operation="MoveZero", ...)` —— 零速/estop 止动(line 155)

## 4. SimSportGateway 契约(新文件 `go2w_sim/nodes/sim_sport_gateway.py`)

复刻 `SportGatewayClient` 的 4 方法接口,把 socket 传输换成 `/cmd_vel` publish:

```python
class SimSportGateway:
    """GO2W_SIM adapter: Effect → Twist → /cmd_vel → planar_move.

    替代 SportGatewayClient(socket → sport lease → 真狗).
    上层 MotionController / Go2WMotionMachine 零感知.
    """
    _EFFECT_OPERATIONS = {"Move", "MoveZero"}   # 实施时按 motion_machine 实际全集补

    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def initialize(self) -> InitializationResult:
        return InitializationResult(success=True, motion_service="ai-w", mode_healthy=True)

    def check_motion_service(self) -> InitializationResult:
        return self.initialize()                 # planar_move 永远就绪

    def execute(self, effect: Effect) -> CommandReceipt:
        twist = Twist()
        if effect.operation == "Move" and effect.arguments:
            vx, vy, wz = effect.arguments[:3]
            twist.linear.x, twist.linear.y, twist.angular.z = float(vx), float(vy), float(wz)
        # MoveZero / 其他 → 零速 Twist
        self._pub.publish(twist)
        return CommandReceipt(operation=effect.operation, code=0,
                              sequence=effect.sequence, physical_confirmed=False)

    def close(self) -> None:
        pass
```

**测试 pin:** `test_sim_sport_gateway.py` 发 `Effect("Move", seq, (0.5,0,0))` 断言 `/cmd_vel` 收到 `linear.x≈0.5`;发 `Effect("MoveZero", seq)` 断言收到零速。

## 5. SimTelemetryBridge(新文件,`/odom_planar` → `Telemetry`)

真机 `LowState` → `Telemetry(sample_id, raw_mode, wheel_dq, battery_soc, error_code, roll, pitch, motor_fault, ...)`。仿真缺 LowState,需桥:

```python
class SimTelemetryBridge:
    """订阅 /odom_planar, 反推 Telemetry 喂 controller.observe_feedback.

    让 ScanFreshnessWatchdog / drive_session 状态机真跑(否则卡 BOOT_HOLD).
    """
    # roll/pitch ≈ 0(planar_move 不俯仰); battery_soc=80; error_code=0;
    # motor_fault=False; wheel_dq 从 odom velocity 反推;
    # raw_mode 按当前 session phase 映射(PARKED/NAV_ACTIVE → 对应 sport_mode int).
```

**关键:** 没有 Telemetry 反馈,`ScanFreshnessWatchdog.is_fresh()` 永远 False → `nav_guard_reason()` 拒绝 velocity → 狗不动。这是真机一致必须补的桥(Task 3 极简版没碰,所以状态机没真跑)。

## 6. web 栈 GO2W_SIM 落点

`nx_web_server.py` 多处判 `sdk_ready`(line 738/741/1151/1186/1252/1258/1268/1300)。真机 `sdk_ready` 来自 Unitree SDK 连接;仿真由 `SimSportGateway.initialize()` 返回 success → `MotionController.sdk_ready=True` → nx_motion_node status snapshot `sdk_ready:True`(line 375)。**web 代码零改**,只要 nx_motion_node 在仿真里跑通,sdk_ready 自然 True。

需新加的 GO2W_SIM 落点(实施时 grep 确认):
- `nx_web_server.py` 顶部 import 后 `if os.environ.get('GO2W_SIM'): ` 跳过任何硬编码硬件探测(如直连相机/YOLO 的 import guard)。
- `nx_navigation_arbiter.py` 若有直连 SDK 探测,同样跳过。

目标:web 栈在 `GO2W_SIM=1` 下直接起,订阅仿真话题(`/map` `/Odometry` `/scan` `/cmd_vel`),`:8000` 暴露前端,Windows 浏览器经 WSL2 端口访问。

## 7. 验收标准(真机一致度量)

| 维度 | 标准 |
|---|---|
| **话题同名** | `/livox/lidar`(CustomMsg) `/livox/imu` `/Odometry` `/odom` `/scan` `/map` `/cmd_vel` `/cmd_vel_nav` —— 仿真与真机完全一致 |
| **节点同代码** | fast_lio / map_odom_fuser / motion_machine / motion_controller / motion_safety / nx_motion_node(除 adapter 分支)/ nx_web_server / nx_exploration_manager / nx_navigation_arbiter —— `git diff` 仅 adapter 切换 + GO2W_SIM guard,无业务逻辑改动 |
| **静止原点** | FastLIO 静止 30s,`/Odometry` 三轴 max(|·|) < 0.02m |
| **TF age** | `map→odom→base_link` 全链 age < 0.1s,无 TF_ERROR |
| **点选导航** | 发 Nav2 goal (3,3),狗到达欧氏误差 < 0.3m,无 hard contact(/contacts force>1N 计数=0) |
| **状态机真跑** | nx_motion_node log 见 `drive_session` phase 迁移(BOOT_HOLD→PARKED→ACTIVATING→NAV_ACTIVE),非 mock 占位 |
| **前端可见** | `localhost:8000`(WSL2 端口转发) 显示地图(FastLIO 建图)+ 状态灯 + 可发导航 goal |
| **frontier 探索** | 多房间 world 覆盖率 ≥ 70%,frontier 耗尽终止,步数 > 8 |

## 8. 风险 + 缓解

| 风险 | 缓解 |
|---|---|
| `fast_lio` ROS2 fork Humble 兼容 | 已 clone `FAST_LIO_ROS2`;build 在跑(bb947gjfn)。若失败,回退到 `LihanChen2004/pb_rm_simulation` 同款 fork |
| Gazebo IMU 重力分量致 FastLIO 静止漂移 | `<orientation_reference_frame>` 设 `<identity>`;acc_cov/gyr_cov 沿用实机;无 ×9.80665(决策#5) |
| `ros2_livox_simulation` 插件 Gazebo 11 兼容 | pin commit;若 `mid360.csv` 扫描模式路径错,launch 传绝对路径 |
| Telemetry 桥 wheel_dq 反推不准致 watchdog 误判 | 反推用 odom `twist.linear/angular`,加噪声容差;watchdog threshold 沿用实机 |
| web 栈 Python 依赖(WSL2 缺) | `pip install -r web/requirements.txt`(实施时确认) |
| WSL2 :8000 端口转发 | mirrored 网络模式(`.wslconfig`)自动;否则 `netsh interface portproxy` |

## 9. 执行顺序(替代 2026-07-24 plan Task 2-6)

1. ✅ `livox_ros_driver2` build(地基,已完成)
2. ⏳ `ros2_livox_simulation` + `fast_lio` build(bb947gjfn 进行中)
3. URDF 加 Livox(`libros2_livox.so`)+ IMU(`libgazebo_ros_imu_sensor.so`)插件
4. `config/fastlio_sim.yaml`(`mid360.yaml` 话题对齐,无 ×9.80665)
5. `SimSportGateway` + `SimTelemetryBridge` + nx_motion_node GO2W_SIM 分支
6. `sim_fastlio_bringup.launch.py` + `test_fastlio_static_origin.py`(静止原点 <0.02m)
7. `sim_nav_bringup.launch.py`(nav2-3d)+ `test_sim_nav_goal.py`(点选闭环)
8. web 栈 GO2W_SIM 接入 + `:8000` 前端可见
9. multiroom world + frontier explore(Task 5)
10. headless fixture + TF stamp 抖动(Task 6)
