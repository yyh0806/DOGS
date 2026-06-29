# 技术决策与调研结论（2026-06-29）

> 本轮并行调研的结论固化。作为后续开发的依据，避免重复踩坑。

## 一、阶段 0 移动控制：根因与突破方向

> 🔧 **2026-06-29 根因定位（推翻下方"业界未解难题"结论）**：
> 真因 = `nx_motion_node._do_stand` **误删了 `BalanceStand`**。对比 `web/panel.py:RobotConnection._do_stand`
> （`SDK_CAPABILITIES.md` 实测能动）的站立序列 `StandUp → BalanceStand → Move(0,0,0)`，
> nx_motion_node 此前只有 `StandUp → StopMove → Move(0,0,0)`。缺 BalanceStand → 狗停在 idle 态 →
> Move 返回 code=0 但轮子不转（精确匹配下方实测症状）。
> **已修复**：`nx_motion_node._do_stand` 加回 BalanceStand，对齐 panel.py。**待实车验证**（硬件装完后 `vx=0.1` 短按）。
> 下方旧排查记录保留作过程留档，其中"业界未解难题"结论已过时。
>
> **实车 TODO**：① `vx=0.1` 短按测 BalanceStand 后能否移动；② STOPPED 态 `StopMove` 三方矛盾裁决
> （`nx_motion_node` 注释说必需 / `SDK_CAPABILITIES` 说无效 / `panel.py` 不用）——用 LowState `motor_state`
> 轮速实测哪种真能刹住，二选一统一 `nx_motion_node` 与 `panel.py`。

### 已排除的根因（实测）
- ❌ 不是 SDK 缺 Move_Wheel（1.0.1 版本 Go2W 就用 Move，无 Move_Wheel）
- ❌ 不是缺 enableLease（nx_motion_node 第73行已 enableLease=True）
- ❌ 不是运动模式错（CheckMode 确认 ai-w 轮式模式）
- ❌ 不是 SwitchGait 值（官方枚举 0-4 全是足式，无轮式值；SwitchGait(1)=trot 实测让狗摔倒，禁用）

### 最可能的根因（待验证）
**狗未进入 locomotion 模式（mode 3）。** 官方文档 V2.0 说明：
- mode 0 = idle（默认站立）
- mode 1 = balanceStand
- mode 3 = locomotion（运动）

StandUp 让狗处于 idle/balance 态，直接 Move 可能不触发轮式运动。

### 待验证的突破方向（按风险从低到高）
1. **SwitchJoystick(True)**（API 1027）：SDK 里有，仓库从未测过。比 SwitchGait 风险低，可能是进入 locomotion 的开关。**优先试这个。**
2. 持续发非零 Move 自行触发 locomotion（部分机器人这样设计）
3. **绝对不再盲试 SwitchGait 数值**（已证明 trot 会摔）

### 安全测试原则
- 实车前先用 LowState 读轮速确认（只读不控）
- 用极小速度（vx=0.1）短时（2秒）测试
- NX 上随时准备 Damp 急停
- 一次只改一个变量

### 2026-06-29 实测突破（关键证据）
- `Move(0.1,0,0)` **返回 code=0（指令被接受）**，但**轮速持续 ≈0（轮子不转）**
- → 排除"指令没发出去"，根因是**狗运动控制器接受了 Move 但不驱动轮子**
- `SwitchJoystick(True)` 返回 **3203**（被拒绝），不是 locomotion 入口
- `sportmodestate` 话题**无数据**（运动服务状态不可读）
- 外部无权威解决方案（unitree_rl_lab #122 Go2W 不动，无人解答）

**结论**（⚠️ 已过时，见文首根因定位）：Go2W 轮式狗驱动轮子需要某种未知的模式激活，SDK 的 Move/StandUp/SwitchJoystick 都不触发。这是业界未解难题，非我们代码 bug。

## 二、阶段 2 建图：FAST_LIO + MID360 部署方案

### 关键决策
- **用 MID360 自带 IMU**（ICM40609, 200Hz），不用狗的 IMU。FAST_LIO 要求 LiDAR+IMU 同源时钟，MID360 自带最省心。
- 用 **Ericsii/FAST_LIO_ROS2**（Humble 移植版，维护活跃）
- 用 **livox_ros_driver2**（第二代驱动，支持 MID360）

### 部署步骤（NX 上）
```
1. Livox-SDK2 源码编译安装
2. livox_ros_driver2 (改 package.xml format 2→3, 配 MID360_config.json)
3. FAST_LIO_ROS2 (--recurse-submodules, CMakeLists: pcl_ros→tf2_ros, 加 -Wno-deprecated-copy)
4. 同一 ws 编译 (FAST_LIO 依赖 livox_ros_driver2 的 CustomMsg)
```

### 关键坑
- Sophus 用 third_party 的旧版（系统新版会编译错）
- TF：FAST_LIO 发 camera_init→body，需桥接到 map→odom→base_link
- 编译可能 OOM，限并发或加 swap
- Jetson 上开 MAXN 功率模式

### FAST_LIO 输出
- /Odometry (~100Hz, camera_init→body)
- /cloud_registered (全局点云)
- /Laser_map (累积地图)
- TF: camera_init→body

## 三、阶段 3 Nav2：3D 点云配置

### 关键决策
- **路线 A（推荐）**：MID360 点云 → pointcloud_to_laserscan → /scan → 现有 costmap。改动最小。
- local_costmap 额外加 **VoxelLayer** 直接吃 PointCloud2（双保险）
- **不用 amcl**（FAST_LIO 已替代定位职能，amcl 会抢 map→odom 发布权）
- Go2W 室内用**差速模式起步**（max_vel_y=0.0），跑通再开全向

### Go2W 室内参数
- max_vel_x: 0.6 m/s, max_vel_theta: 1.0 rad/s
- footprint: [ [0.30, 0.20], [0.30, -0.20], [-0.25, -0.20], [-0.25, 0.20] ]
- xy_goal_tolerance: 0.20, yaw_goal_tolerance: 0.15

### TF 架构
- map→odom：slam_toolbox 或 FAST_LIO（二选一，不能都发）
- odom→base_link：FAST_LIO（通过 frame 改名或 TF 桥）

## 四、并行开发分工建议

| 阶段 | 依赖 | 可并行性 |
|------|------|----------|
| 0 移动控制 | 无 | **必须最先**，阻塞所有 |
| 1 NX本机闭环 | 阶段0 | 可与2并行（不依赖雷达） |
| 2 FAST_LIO建图 | 无（独立于控狗） | **可与0并行**（建图只需雷达，不需狗动） |
| 3 Nav2导航 | 阶段0+2 | 需 0 和 2 都完成 |
| 4 AI迁移 | 无 | 完全独立，随时可做 |

**重要**：阶段 2（建图）其实**可以和阶段 0（移动控制）并行**——建图只需要雷达转起来，不需要狗动（推着狗走也能建图）。这是并行开发的关键突破口。
