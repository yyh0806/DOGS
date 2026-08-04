# FastLIO 定位、Nav2 避障与 Panel 点选导航设计

## 目标与完成判据

在 Unitree Go2W 的 Jetson NX 上恢复一条可长期运行的自主导航链：MID360 与 IMU 驱动 FastLIO 持续定位和建图，Nav2 使用实时激光扫描规划并避障，操作者在 Panel 地图空白处单击任意可达点即可发送 `NavigateToPose` 目标并看到执行结果。

完成必须同时满足：

- `/livox/lidar` 约 10 Hz、`/livox/imu` 约 200 Hz，FastLIO 持续发布 `/Odometry` 与 `camera_init -> body`。
- TF 为单链 `map -> odom -> base_link`，不存在 `base_link` 双 parent；静止与低速转向时定位不发生米级跳变。
- MID360 模组整体向下俯仰 20° 被显式补偿，Panel 与 Nav2 使用的 `map` 坐标系保持水平。
- Nav2 的 planner、controller、behavior、BT navigator 与 velocity smoother 均为 active，局部和全局 costmap 能标记、清除并膨胀障碍。
- CLI 目标与 Panel 点选目标都能产生正向 `/cmd_vel`，机器狗绕开障碍后到达目标，结果为 `SUCCEEDED`。
- NX 重启后定位和导航栈能自动恢复，无需手工运行 bringup。

## 已确认根因

2026-07-10 的 NX 实况采集确认：

1. `fastlio`、`map-odom-fuser`、`nav2-3d` 是 `systemd-run` 创建的 transient unit，NX 重启后 unit 消失，因此 `/Odometry`、`map` TF 和 `/navigate_to_pose` 都不存在。
2. `bringup_slam_nav2.sh` 的 `wait_tf()` 在全局 `set -o pipefail` 下执行 `tf2_echo | grep -q`。`grep -q` 命中后提前关闭管道，`tf2_echo` 收到 SIGPIPE；`pipefail` 将本应成功的检查判为失败。实测 `/Odometry` 和 `camera_init -> body` 均持续存在，脚本仍在等待后报失败，后续 fuser/Nav2 从未启动。
3. 当前 Panel 只实现拖框搜索，没有地图点选到 `/navigate_to_pose` 的 API 或前端交互。
4. 当前 Nav2 voxel layer 将 `/livox/lidar` 声明为 `PointCloud2`，但驱动实际发布 `livox_ros_driver2/msg/CustomMsg`；该观测源类型不匹配。`pointcloud_to_laserscan` 节点虽被定义，却未加入 launch，且同样不能直接消费 CustomMsg。
5. Panel 当前广播 `/odom` 轮速积分位姿，而 Nav2 目标使用 `map`；长期运行后二者会因 `map -> odom` 校正产生坐标偏差。

## 方案选择

采用分层最小修复，保留现有 FastLIO、手写 fuser、Nav2 与 web 架构。相比把全栈重写成单一 launch，或改用 `robot_localization` 重新标定，该方案复用已经实机跑通过的控制链，变更边界清晰，并能逐层回归。

## 架构与数据流

```text
MID360 CustomMsg + IMU
        |
        v
     FastLIO --------------------> 3D cloud/map
        | /Odometry + camera_init->body
        v
 map_odom_fuser <---- odom->base_link (nx_sensor wheel/IMU)
        | map->odom + /localization_pose(map, base_link)
        +-------------------------------> Panel map pose
        |
        v
map->odom->base_link
        |
        +--> Nav2 planner/controller/BT --> /cmd_vel_nav
        |                                  |
horizontal /scan --> costmaps              v
                                     velocity_smoother
                                            |
                                            v
                                       /cmd_vel
                                            |
                                            v
                                       nx_motion

Panel click -> POST /api/navigate -> point navigation controller
                                      -> NavigateToPose(map)
                                      -> WS nav_goal status
```

## 20° 俯角与定位坐标

MID360 内部 LiDAR 与 IMU 随模组一起倾斜，两者相对外参不因整机安装角改变，因此 FastLIO 的 LiDAR-to-IMU 外参保持当前标定，不把 20° 重复写入 `extrinsic_R`。

定义 `T_body_base` 为倾斜传感器 body 到水平底盘 `base_link` 的安装变换。按当前安装方向使用：

```text
body_to_base_pitch = -20° = -0.3490658504 rad
```

fuser 在每个新的 FastLIO 时间戳 `t_s` 上使用双侧共轭换基，并查询同一时刻的轮速里程计：

```text
T_map_base_slam(t_s) = inverse(T_body_base) * T_camera_body(t_s) * T_body_base
C_map_odom = T_map_base_slam(t_s) * inverse(T_odom_base(t_s))
```

静止时 `T_camera_body` 近似单位阵，左右安装变换严格相消；运动时传感器坐标中的旋转被转换为水平底盘坐标。`C_map_odom` 是慢变化的 SLAM 校正，在下一帧 FastLIO 到来前保持；每个 fuser 周期用最新里程计 `T_odom_base(t)` 预测当前位姿：

```text
T_map_base(t) = C_map_odom * T_odom_base(t)
```

`map -> odom` TF 的矩阵为 `C_map_odom`，时间戳使用最新里程计时间 `t`，避免把约 1.5 秒前的 FastLIO stamp 伪装成可供 Nav2 当前查询的 TF，也不依赖 `transform_tolerance` 做 tf2 不支持的动态外推。fuser 同时发布 `/localization_pose`（`nav_msgs/Odometry`，frame=`map`，child=`base_link`，pose=`T_map_base(t)`），供 Panel 使用与 Nav2 完全一致的当前世界坐标。同一个 FastLIO stamp 只更新一次校正。

## 启动与恢复

修复 `wait_tf()`：在局部关闭 `pipefail`，对 `tf2_echo` 使用行缓冲，并以 `grep` 的结果作为判据；健康检查应在首次收到 `At time` 时立即返回，而不是用循环嵌套多个 3 秒超时。

增加持久 `go2w-slam-nav.service`：

- `After`/`Requires` 指向 Livox driver、go2w sensor 与 motion 服务。
- `ExecStart` 调用已修复的 bringup；失败时重试。
- FastLIO、fuser 与 Nav2 子 unit 使用 `Restart=on-failure`，并继承统一 FastDDS/RMW 环境。
- FastDDS profile 使用 NX 实际存在的 `/home/nx/go2w_ws/fastdds_udp.xml`。
- 服务成功的最终门禁是 `/Odometry`、`map -> base_link`、三个核心 lifecycle node active 和 `/navigate_to_pose` 可见。

## Nav2 避障

第一版稳定避障使用狗自带、已验证为水平且持续约 10 Hz 的 `/scan`：

- local costmap：rolling window，ObstacleLayer + InflationLayer。
- global costmap：rolling window，ObstacleLayer + InflationLayer；FastLIO 不提供 2D OccupancyGrid，因此不启 StaticLayer。
- 删除类型错误的 `/livox/lidar` `PointCloud2` voxel observation，避免“配置存在但实际无数据”的假双保险。
- marking 与 clearing 同时启用，障碍距离与膨胀范围按 Go2W footprint 验证。
- 控制链固定为 `controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> nx_motion`，不增加方向反转 remap。

MID360 继续负责 FastLIO 3D 建图。若后续需要 MID360 直接进入 costmap，应增加经过测试的 CustomMsg-to-PointCloud2 转换节点和独立 `base_link` 安装外参；本次不把错误类型订阅保留为表面功能。

## Panel 点选导航

### 前端交互

- 地图空白处短按/单击转换为 `map` 世界坐标；拖动超过 8 px 仍用于搜索区域，点击人物标记仍打开截图。
- 单击后以当前定位点到目标点的方位计算默认 yaw，显示目标标记和 `pending/active/succeeded/failed/canceled` 状态。
- 前端向 `POST /api/navigate` 发送 JSON：`{"x": number, "y": number, "yaw": number, "frame_id": "map"}`。
- 新目标替换旧目标；急停和 `/api/stop` 必须取消当前 Nav2 goal 并发布零速度。

### 后端控制

新增单点导航控制器，复用已有 `Nav2ActionClient` 协议但不依赖房间搜索状态机：

- HTTP handler 只校验参数并提交后台任务，不阻塞 Web 主线程。
- 只允许有限浮点数且 frame 固定为 `map`。
- 同一时刻只保留一个 goal；通过 generation/token 防止旧线程覆盖新目标状态。
- 通过 WebSocket 广播 `type=nav_goal`，包含目标、状态、Nav2 结果和错误原因。
- action server 不在线、goal 被拒绝、超时或 aborted 时返回明确状态，不伪报到达。

## 错误处理与安全停止

- 任一 TF、lifecycle 或 action 门禁失败时，不启动后续运动链，并在 systemd 日志中给出具体层级。
- 导航测试期间监控 FastLIO 位姿、`map` TF、`/cmd_vel` 和轮速里程计；发生定位米级跳变、TF 消失或服务退出时立即取消 goal 并发布零速度。
- Panel 重载只发送停止/取消，不让浏览器连接丢失后继续残留目标。
- 不把 Nav2 返回 `SUCCEEDED` 单独视为成功；还必须核对真实位移和定位连续性。

## 测试与验证

实现采用 TDD，依次建立以下回归：

1. shell contract 复现 `pipefail + grep -q` 假失败，修复后对真实/模拟 TF 输出正确返回。
2. fuser 矩阵测试验证 -20° 参数、旋转矩阵正交性、静止严格相消、纯 yaw 不引入明显 roll/pitch，以及同一 FastLIO/odom 时间求出的校正能正确预测较新的 odom 位姿。
3. fuser pose topic 测试验证 frame、child、最新 odom 时间戳、位置与四元数来自 `C_map_odom * T_odom_base(t)`；重复 FastLIO stamp 不重复计算不同校正。
4. Nav2 配置 contract 确认不存在 CustomMsg-as-PointCloud2，`/scan` marking/clearing 与速度平滑链完整。
5. systemd contract 确认持久 unit、依赖、重启策略和实际 profile 路径。
6. 单点导航控制器测试覆盖接受、替换、取消、成功、拒绝、超时与 aborted。
7. Panel JS contract 覆盖屏幕到世界坐标、短按目标、拖框不发目标、marker 点击不发目标和 API payload。
8. 本地全量测试、Python 编译和 `git diff --check`。
9. NX 静态门禁：传感器频率、FastLIO 频率、TF 单链、roll/pitch、fuser CPU、Nav2 lifecycle、costmap 非空。
10. 实机门禁：0.1 rad/s 低速转向、1 m CLI 目标、带障碍目标、Panel 点选目标；每次均核对 action、速度、真实位移与定位连续性。

## 回滚

所有部署文件在 NX 上按时间戳备份。若 -20° 共轭导致静止 roll/pitch 超过 5°或定位跳变，停止运动并回滚 fuser 与 pitch 参数；若新 Panel 控制器异常，只禁用 `/api/navigate`，不影响手动急停；若持久服务启动失败，可停止并禁用 `go2w-slam-nav.service`，恢复手工 bringup 进行诊断。
