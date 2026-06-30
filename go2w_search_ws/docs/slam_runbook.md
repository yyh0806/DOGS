# 阶段F Runbook — slam_toolbox 2D 建图/定位 + Nav2 slim 联调

> 降级路径 (slam_toolbox 替代未就绪的 FAST_LIO)。本文件是 NX 上实跑 SOP, 配置见
> `src/go2w_nav/config/slam_toolbox.yaml` + `nav2_params_slim.yaml`, launch 见
> `src/go2w_nav/launch/slam.launch.py` + `nav2_slim.launch.py`。规格见
> `gan-harness/spec-slam.md`。
>
> 适用条件: MID360/FAST_LIO 未就绪, 用狗自带 LiDAR 投影的 `/scan` 2D 建图。
> 硬件依赖: NX 上线 + go2w-sensor.service 运行 + 阶段0 移动控制就绪 (推狗走 / 遥控)。

---

## 0. 前置检查 (每次实跑前)

```bash
# 1. nx_sensor 运行中 (阶段A systemd, 发 /scan /imu /odom + odom→base_link TF)
sudo systemctl status go2w-sensor.service    # 应 active (running)

# 2. /scan 正常 (狗自带 LiDAR 投影, 10Hz, frame=base_link)
ros2 topic hz /scan                          # 应 ≈10Hz
ros2 topic echo /scan --once | grep frame_id # 应 "base_link"

# 3. /odom 正常 (50Hz, frame=odom/child=base_link)
ros2 topic hz /odom                          # 应 ≈50Hz

# 4. ⚠️ 头号风险: /scan QoS 确认 (RELIABLE)
#    nx_sensor /scan 发布端 QoS = RELIABLE (nx_sensor_node.py:104-107)
ros2 topic info /scan -v                     # 看 Publisher 的 Reliability = RELIABLE

# 5. TF: odom→base_link 由 nx_sensor 发 (50Hz)
ros2 run tf2_ros tf2_echo odom base_link     # 应有输出, 随狗动更新

# 6. map→odom 此时应无 (slam_toolbox 还没起)
ros2 run tf2_ros tf2_echo map odom           # 启 slam_toolbox 前无输出, 正常
```

---

## 1. 建图流程 (mode=mapping)

### 1.1 启动 (5 终端, 见 spec-slam.md 决策 5)

```bash
# 终端1: nx_sensor (阶段A, systemd 自启或手动)
sudo systemctl start go2w-sensor.service
# 验证: ros2 topic hz /scan  (应 ~10Hz)

# 终端2: slam_toolbox 建图模式
ros2 launch go2w_nav slam.launch.py mode:=mapping
# 验证 (开建图后):
#   ros2 topic hz /map              # 应 ≈0.5Hz (map_update_interval=2.0)
#   ros2 run tf2_ros tf2_echo map odom  # 应有输出 (slam_toolbox 发)

# 终端3 (可选, 推荐): rviz2 看 /map 实时生长
rviz2
# add display: OccupancyGrid topic=/map; TF; LaserScan topic=/scan

# 终端4 (可选, 建图阶段不导航): 先不开 Nav2

# 终端5: 阶段0 移动控制 (推狗走或遥控, 见阶段0 文档)
```

### 1.2 推狗走遍房间

- 慢推 (< 0.5 m/s), 让 slam_toolbox scan-matching 跟得上 (minimum_travel_distance=0.3m 加节点)。
- 走遍所有要搜索的房间, 包括门口/转角 (loop closing 需要重访)。
- rviz 看 `/map` 实时生长, 覆盖完整后停下。

### 1.3 建图完成, 存图 (slam_toolbox 原生序列化)

```bash
# slam_toolbox /save_map service, 存 .posegraph + .data (不是 nav2_map_server 的 pgm/yaml)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: /home/nx/maps/room, timeout: 5000}"
# 产物: /home/nx/maps/room.posegraph + /home/nx/maps/room.data
# (slam_toolbox 自带序列化格式, localization 模式直接载入, 比 amcl 的 pgm 精确)

# 建议把建图起始点设为 map 原点 (狗从某固定位置出发建图, 该位置 = map [0,0,0])
# 这样 localization 的 map_start_pose=[0,0,0] 就是 identity, 不需要重定位
```

### 1.4 Ctrl-C 停建图 (终端2)

---

## 2. 定位流程 (mode=localization, 替代 amcl)

```bash
# 终端2 (替换建图终端): slam_toolbox localization 模式
ros2 launch go2w_nav slam.launch.py mode:=localization map_file:=/home/nx/maps/room
# map_file 不含扩展名, slam_toolbox 自动找 .posegraph + .data

# 验证:
#   ros2 topic echo /map --once           # 应有载入的静态地图 (OccupancyGrid)
#   ros2 run tf2_ros tf2_echo map odom    # 应有输出, scan-matching 持续校正
#   ros2 topic info /map -v               # /map publisher = slam_toolbox (TRANSIENT_LOCAL latched)

# 把狗放到建图起始位置 (map 原点), 让 scan-matching 初值 = identity 收敛快
```

定位模式下 slam_toolbox:
- 载入 `.posegraph` + `.data`, 发**静态** `/map` (OccupancyGrid, global_costmap static_layer 终于有数据)。
- 持续 scan-matching 发 `map→odom` (替代 amcl 粒子滤波, 更精确)。

---

## 3. Nav2 slim 联调 (mode=localization 时)

```bash
# 终端4: Nav2 slim (吃 slam_toolbox /map + map→odom→base_link TF)
ros2 launch go2w_nav nav2_slim.launch.py

# 终端5: nx_motion_node (阶段A, 控狗)
ros2 run go2w_bridge nx_motion_node

# 终端6 (可选): web (阶段A/B/E, Nav2 action client 在此)
python3 web/nx_web_server.py
```

### 3.1 联调验证

```bash
# 1. action 暴露
ros2 action list                          # 应含 /navigate_to_pose

# 2. TF 链完整 + 单源 (Critical: 不能多源)
ros2 topic info /tf -v
#   map→odom publisher: 只 slam_toolbox (不能有 amcl/FAST_LIO/static_transform)
#   odom→base_link publisher: 只 nx_sensor

# 3. costmap 有障碍物 (obstacle_layer 收到 /scan)
ros2 topic echo /local_costmap/costmap --once   # 应有非 0 cells

# 4. 单点导航测试 (frame=map, 与建图坐标系一致)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
   orientation: {w: 1.0}}}}"
# 狗应走到 (1,0), 方向正确 (无乱转, /cmd_vel 零反转)

# 5. 浏览器端到端 (阶段E 编排)
# 打开 web, 点"搜索客厅", 狗应走到客厅入口 + 房间内覆盖搜索
```

---

## 4. 常见故障排查

### 4.1 `/map` 一直空 (建图模式) → QoS 不匹配 (头号风险)

**症状**: slam_toolbox 启动正常无报错, `ros2 topic hz /scan` ≈10Hz 正常, 但
`ros2 topic hz /map` = 0, rviz /map 空, `tf2_echo map odom` 无输出。

**根因**: nx_sensor `/scan` 发 RELIABLE QoS (nx_sensor_node.py:107), slam_toolbox 默认
订阅 BEST_EFFORT (sensor_data QoS), 不兼容 → 静默收不到 scan, 无报错。

**修复**: 确认 `slam_toolbox.yaml` 含 `use_sensor_data_qos: false` (已配, 决策 4)。
```bash
grep use_sensor_data_qos src/go2w_nav/config/slam_toolbox.yaml   # 应 false
ros2 topic info /scan -v   # slam_toolbox 订阅者 Reliability 应 = RELIABLE (匹配发布端)
```

### 4.2 Nav2 costmap 无障碍 (撞墙) → obstacle_layer QoS 不匹配 (第二个 QoS 坑)

**症状**: Nav2 起来正常, 但 local/global costmap 无障碍物, 狗规划穿墙。

**根因**: Nav2 costmap obstacle_layer 默认订 scan 用 sensor_data QoS (BEST_EFFORT),
与 nx_sensor RELIABLE `/scan` 不匹配 → 静默收不到。

**修复**: 确认 `nav2_params_slim.yaml` 的 local + global costmap obstacle_layer scan 段都含
`reliability: reliable` (已配, slim diff ③)。
```bash
grep -A1 reliability src/go2w_nav/config/nav2_params_slim.yaml   # 应 2 处 reliable
```

### 4.3 TF `map→odom` 断 → slam_toolbox 没起或 mode 错

```bash
ros2 node list | grep slam_toolbox        # 应有 /slam_toolbox
ros2 param get /slam_toolbox use_sim_time # 检查参数加载
```
若 localization 模式 `map_file` 路径错或 `.posegraph` 缺失, slam_toolbox 启动失败。

### 4.4 TF `odom→base_link` 断 → nx_sensor 没起

```bash
sudo systemctl status go2w-sensor.service   # 应 active
ros2 topic hz /odom                         # 应 ≈50Hz
```

### 4.5 Nav2 global_costmap 全空 → static_layer 没收到 /map

slam_toolbox 必须在 **localization 模式** (或 mapping 模式建图中) 才发 /map。
确认 `slam.launch.py mode:=localization` 已起, `ros2 topic echo /map --once` 有数据。
static_layer 的 `map_subscribe_transient_local: true` 匹配 slam_toolbox /map 的 TRANSIENT_LOCAL。

### 4.6 狗乱转 → /cmd_vel remap (Critical)

**症状**: 狗原地乱转或方向反。

**根因**: nav2_slim.launch.py 误加了 `/cmd_vel` remap (反转)。

**修复**: nav2_slim.launch.py **禁止** `/cmd_vel` remap (继承阶段D 决策 5, 零反转)。
```bash
grep cmd_vel src/go2w_nav/launch/nav2_slim.launch.py   # 应无 remap 行 (只有 yaml 里 cmd_vel_topic 声明)
```

### 4.7 建图扭曲 → 推太快或 /scan 掉帧

推狗走 < 0.5 m/s, 检查 `ros2 topic hz /scan` 稳定 ≈10Hz。转弯慢, 让 scan-matching 收敛。

### 4.8 多源 TF 跳变 (狗乱动) → 误混起 3d / slim launch

slim 路线 (slam_toolbox) 和 3d 路线 (FAST_LIO + TF 桥) 都依赖 map→odom, **不可混起**。
确认只起一套: 要么 `slam.launch.py + nav2_slim.launch.py`, 要么 `nav2_3d.launch.py + FAST_LIO`。

---

## 5. 故障注入 (验证鲁棒性, 可选)

```bash
# 1. kill slam_toolbox → Nav2 报 TF 断链 (map→base_link lookup 失败)
ros2 node kill /slam_toolbox
ros2 topic echo /tf   # map→odom 停止发布, Nav2 planner 报错

# 2. kill nx_sensor → odom→base_link 断, slam_toolbox scan-matching 失去里程计先验
sudo systemctl stop go2w-sensor.service
# slam_toolbox /map 停止生长, Nav2 local_costmap rolling window 卡住
```

---

## 6. 切回 FAST_LIO 路线 (MID360 装好后)

slim 是降级方案, MID360 装好 + FAST_LIO 就绪后切回阶段D 原路线:

```bash
# 1. 停 slim 路线
# Ctrl-C 关 slam.launch.py + nav2_slim.launch.py

# 2. 启 3d 路线 (阶段D, commit 9b0c397 已 gan 收敛)
ros2 launch go2w_nav nav2_3d.launch.py
# + 阶段C FAST_LIO launch (camera_init→body TF + /Odometry)
# + livox_ros_driver2 (MID360 → /livox/lidar)
```

**两路线互斥**, 不可混起 (都发 map→odom 会 TF 冲突)。切换通过选不同 launch 实现。

---

## 7. mock /scan 建图 (NX 未恢复时调试, 可选)

NX 未恢复时, 可在 PC 上 mock 验证 QoS + launch 结构 (不能替代真实建图):

```bash
# 自写 mock node 发 RELIABLE /scan (固定或缓慢旋转的 LaserScan) + mock /odom + odom→base_link TF
# 关键: mock /scan 必须用 RELIABLE QoS (匹配 slam_toolbox use_sensor_data_qos=false),
#        否则复现不了真实 QoS 链路, 验证无效。
# 然后:
ros2 launch go2w_nav slam.launch.py mode:=mapping
# 验证: slam_toolbox 收到 scan (ros2 topic info /scan -v 订阅者 RELIABLE), /map 生长, map→odom 发布
```

**注意**: mock scan 无真实环境特征, scan-matching 可能失败。mock 仅验证 QoS + launch 结构,
真实建图验收必须等 NX 恢复 + 推狗走 (Sprint 4)。

---

## 附: 文件速查

| 文件 | 职责 | 关键参数 |
|---|---|---|
| `src/go2w_nav/config/slam_toolbox.yaml` | slam_toolbox 参数 (mapping+localization 共用) | `use_sensor_data_qos: false`, `max_laser_range: 10.0`, `base_frame: base_link` |
| `src/go2w_nav/launch/slam.launch.py` | 启 slam_toolbox (mode arg 切 executable) | `mode:=mapping\|localization`, `map_file:=` |
| `src/go2w_nav/config/nav2_params_slim.yaml` | Nav2 slim 参数 (slam_toolbox 适配) | `odom_topic: /odom`, 删 voxel_layer, obstacle_layer `reliability: reliable` |
| `src/go2w_nav/launch/nav2_slim.launch.py` | 启 Nav2 slim (无 TF 桥/p2l) | params 默认 slim yaml, lifecycle 不含 amcl/map_server |
