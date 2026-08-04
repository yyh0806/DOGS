# Fast-LIVO2 + ROS2 + C13 上机 Runbook

> **目标**: 在 Go2W NX 上跑通 **Fast-LIVO2** (LiDAR-Inertial-Visual Odometry 2),
> 用 Livox MID360 雷达 (向下倾斜 20° 安装) + Skydroid C13 云台可见光相机做视觉融合定位,
> 替代现有纯 LiDAR-Inertial 的 FastLIO, 下游 Nav2 3D 导航栈不动。
>
> **嫁接点**: Fast-LIVO2 替换 `bringup_slam_nav2.sh` 第 3 步的 FastLIO。LIVO 同样发
> `/Odometry` + `camera_init→body` TF, 下游 `map_odom_fuser` / nav2 / p2l 完全不动。
> 唯一新依赖: 一路 ROS Image (由 `nx_c13_image_node.py` 从 C13 RTSP 桥入)。

---

## §0 离线准备自检 (连不上 NX 时也能跑)

仓库内已备齐 7 产物, 跑契约测试验证完整性 + 外参自洽:

```bash
cd DOGS
python -m pytest go2w_search_ws/docker/test_fastlivo2_contract.py -v
```

**期望全绿** (含以下数值不变量, 机器校验防手滑):
- `test_lc_rotation_is_orthogonal`: T_lc 旋转矩阵 det≈1, R'R≈I (正交)
- `test_lc_rotation_encodes_20_degree_tilt`: 旋转角 arccos((trace-1)/2) ∈ [17°, 23°]
- `test_livo_yaml_topics_align`: lid/imu/img topic 对齐本仓库实际
- `test_livo_yaml_lidar_imu_extrinsic_is_identity`: LiDAR→IMU 单位阵 (倾斜不改!)
- `test_bringup_livo_inherits_rmw_env`: systemd-run 带 User=nx + RMW (坑6 防治)

**产物清单**:

| 产物 | 路径 | 作用 |
|------|------|------|
| C13 Image 桥 | `web/nx_c13_image_node.py` | C13 RTSP → `/c13/image_raw` + `/c13/camera_info` |
| 部署脚本 | `docker/deploy_fastlivo2.sh` | 编译 Fast-LIVO2 + 拷节点/配置 |
| bringup | `docker/bringup_livo.sh` | 编排 c13→LIVO→fuser→nav2 |
| LIVO 配置 | `src/go2w_nav/config/fastlivo2_mid360_c13.yaml` | 主配置 (含 20° 倾斜) |
| 内参模板 | `src/go2w_nav/config/c13_intrinsic.yaml` | 手册标称 FOV 基线 (仍待现场标定) |
| fuser | `src/go2w_bridge/go2w_bridge/map_odom_fuser.py` | 加 body_to_base_* 倾斜补偿 |
| 契约测试 | `docker/test_fastlivo2_contract.py` | 离线自检 |

---

## §1 Fast-LIVO2 ROS2 移植选型 (决策 3, 连通后第一件事)

**关键背景**: Fast-LIVO2 官方仓库 `HKU-MARS/FAST_LIVO2` 是 **ROS1**, 在 ROS2 Humble 上
**编译不过**。必须用社区 ROS2 移植。

**离线不锁死**: `deploy_fastlivo2.sh` 的 `FASTLIVO2_REPO` 参数化, 默认指向官方 (仅看提示),
连通 NX 后选定一个能编译过的 ROS2 移植再跑。

**连通后选型步骤**:
1. 在 NX 上 `git clone` 候选移植, `colcon build --packages-select fast_livo`
2. 选**编译过 + launch 文件名最接近官方** (`mapping.launch.py` + `config_path:=`) 的
3. 若字段名跟本配置 yaml 不一致 (如 `img_topic` vs `image_topic`,
   `extrinsic_R_LiDAR2CAM` vs `extrinsic_T_cam`), 连通后用 `sed` 对齐:
   ```bash
   sed -i 's/img_topic/image_topic/g' ~/ws_livox/src/FAST_LIVO2/config/fastlivo2_mid360_c13.yaml
   ```
4. 改完重跑契约测试 (本仓库的 yaml 模板字段名是基线)

> ⚠️ 不在离线阶段给具体 fork URL — 移植版本参差, 连通后实测选定才靠谱。
>    仓库选型列表见团队 wiki 或 GitHub 搜 `FAST_LIVO2 ROS2`。

---

## §2 连通 NX 后 bringup 顺序

**前提**: FastLIO 已在 NX 跑通 (`deploy_fastlio.sh` + `bringup_slam_nav2.sh` 验证过),
即 Livox-SDK2 + livox_ros_driver2 + `~/ws_livox` 已就绪, MID360_config.json 已配。

### 2.1 部署 LIVO 栈

```bash
# 在 NX 上 (或 PC 经 SSH)
export FASTLIVO2_REPO=https://github.com/<选定的 ROS2 移植>.git
bash docker/deploy_fastlivo2.sh
```

脚本做: ① colcon build fast_livo ② 拷 nx_c13_image_node.py → `~/go2w_ws/web/`
③ 拷两 yaml → `~/ws_livox/src/FAST_LIVO2/config/` ④ 配置指纹自检。

### 2.2 起 LIVO bringup

```bash
# NX 上 (nx 用户!)
bash docker/bringup_livo.sh
```

编排 (每步 health gate, 失败即停):
```
0. SHM 治理 (清 fastrtps_* 残留)
1. systemd 永久服务健康 (livox/sensor/motion/web)
2. /livox/lidar + /livox/imu 前置 (wait_hz)
3. ★ C13 Image 桥 → /c13/image_raw (wait_hz, ~19-30fps)
4. Fast-LIVO2 → /Odometry + camera_init→body TF (wait_hz + wait_tf)
5. map_odom_fuser (带 body_to_base_pitch=-0.349 倾斜补偿) → map→odom TF
6. Nav2 3D (p2l + navigation_launch + lifecycle)
```

### 2.3 调倾斜补偿 (若模组非 -20° pitch)

```bash
# 水平装 (无倾斜)
BODY_TO_BASE_PITCH=0 bash docker/bringup_livo.sh

# +20° (反向倾斜)
BODY_TO_BASE_PITCH=0.349 bash docker/bringup_livo.sh

# 完整 6 自由度 (平移+旋转, 实测模组在底盘上的位置)
BODY_TO_BASE_X=0.1 BODY_TO_BASE_Y=0 BODY_TO_BASE_Z=0.18 \
BODY_TO_BASE_PITCH=-0.349 bash docker/bringup_livo.sh
```

---

## §3 标定 (首次连通必做, 否则 LIVO 建图粗)

### 3.1 相机内参 (覆盖 c13_intrinsic.yaml)

当前 `c13_intrinsic.yaml` 按 C13 手册 HFOV=77.4°、VFOV=48.8°反推
`fx≈798.85, fy≈793.62`，只是标称基线。**连通后仍必须实测**:

```bash
# NX 上, 起了 c13_image_node 后
# ⚠️ --no-service-check 必须加: nx_c13_image_node 只发 image_raw+camera_info,
#    不实现 set_camera_info service, 不加此参数 cameracalibrator 启动即 crash
#    "Waiting for service camera/set_camera_info ... Service not found"
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --no-service-check \
  image:=/c13/image_raw camera:=/c13

# 拿着标定板在 C13 前后左右上下挥, 采够后点 Calibrate
# 工具输出 ost.yaml 格式 → 转成本仓库 c13_intrinsic.yaml 格式 (字段名一致, 直接替换 data 区)
# 覆盖 ~/ws_livox/src/FAST_LIVO2/config/c13_intrinsic.yaml + 本仓库 src/go2w_nav/config/
```

**判据**: 标定后重投影误差 < 0.5 像素。

### 3.2 LiDAR↔Camera 外参 T_lc (实测, 改 yaml)

Fast-LIVO2 **不支持在线估计** T_lc, 必须离线标定后填进 `fastlivo2_mid360_c13.yaml`
的 `extrinsic_R_LiDAR2CAM` + `extrinsic_T_LiDAR2CAM`。

**方法 A (推荐, 用标定板)**:
1. 打印/拿一块 ArUco 或棋盘格标定板
2. 用 `kalibr` 或 `livox_camera_calibration` 工具联合标定 (LiDAR 点云 + 相机图像)
3. 输出 T (LiDAR→Camera) 4×4 矩阵 → 拆成 R (3×3, 行优先 9 数) + t (3 数) 填 yaml

**方法 B (粗略, 目视)**:
1. 测相机相对 LiDAR 的物理位置 (卷尺, m) → `extrinsic_T_LiDAR2CAM`
2. 测倾斜角 (量角器) → 算旋转矩阵填 `extrinsic_R_LiDAR2CAM`

**填后必跑契约测试验证正交性**:
```bash
python -m pytest go2w_search_ws/docker/test_fastlivo2_contract.py::test_lc_rotation_is_orthogonal -v
```

---

## §4 20° 倾斜专项说明 (本任务核心约束)

### 4.1 倾斜编码在哪 (三个外参, 别改错)

| 外参 | 在哪 | 倾斜改不改 | 说明 |
|------|------|-----------|------|
| LiDAR→IMU (`extrinsic_R/T`) | `fastlivo2_mid360_c13.yaml` | **不改** | MID360 模组内出厂值, LiDAR 与 BMI088 轴系对齐 → R=I。整个模组倾斜时 LiDAR 和 IMU 一起斜, 相对关系不变 |
| Camera↔LiDAR (`T_lc`) | `fastlivo2_mid360_c13.yaml` `extrinsic_R/T_LiDAR2CAM` | **改** | 相机水平装、雷达斜 20°, T_lc 含这 20° 相对转角。标定后覆盖 |
| body→base_link | `map_odom_fuser.py` `body_to_base_*` 参数 | **改** | body(IMU, 跟模组斜)≠base_link(底盘, 水平)。fuser 公式插 T(body→base_link) 项补偿 |

### 4.2 默认假设 (待你确认, 决策 1)

`fastlivo2_mid360_c13.yaml` 的 `extrinsic_R_LiDAR2CAM` 预填 **Ry(+20°)** (= inv(雷达低头 -20°)):
```
T_lc = T(LiDAR→Camera) = inv(Ry(-20°)) = Ry(+20°)
Ry(+20°) = [ 0.9397  0  0.3420]
           [   0     1    0   ]
           [-0.3420  0  0.9397]
```
> ⚠️ 此值适用 T_lc=LiDAR→Camera 约定 + 雷达 pitch=-20° + 相机水平。
>    若 LIVO 移植把 T_lc 定义成 Camera→LiDAR, 翻转 sin 符号成 Ry(-20°)。
>    详见 yaml 贡献位注释。

**若实际是别的姿态, 改这里**:
- roll (绕 X, 雷达侧倾): 见 yaml 贡献位注释的 R_roll 公式
- 方向相反: 翻转 sin 项符号

改完跑契约测试:
```bash
python -m pytest go2w_search_ws/docker/test_fastlivo2_contract.py \
  -k "lc_rotation" -v
```
正交性 + 角度测试通过 = 矩阵合法。

### 4.3 body→base_link 倾斜补偿 (fuser)

`map_odom_fuser.py` 默认参数全零 (identity = body==base_link, 向后兼容老 FastLIO)。
倾斜时 `bringup_livo.sh` 从 env 注入:
```
BODY_TO_BASE_PITCH=-0.349  # -20° 弧度 (默认)
BODY_TO_BASE_Z=0.15        # 模组装底盘上方 15cm
```

fuser 公式变为 (单链 TF, 不双 parent):
```
map→odom = T(camera_init→body) × T(body→base_link) × inv(T(odom→base_link))
```

**验证 TF 单链**:
```bash
ros2 run tf2_tools view_frames
# 期望: map→odom→base_link 单链, camera_init→body (FastLIO/LIVO 内部, fuser 消费不查)
```

---

## §5 验证清单 (LIVO 跑通判据)

### 5.1 基础健康
```bash
ros2 topic hz /c13/image_raw        # ~19-30fps (NVDEC 硬解)
ros2 topic hz /Odometry             # ~100Hz (LIVO)
ros2 topic hz /livox/lidar          # 10Hz
ros2 topic hz /livox/imu            # 200Hz
ros2 run tf2_ros tf2_echo map base_link  # 可查 (fuser 正常)
```

### 5.2 视觉约束生效 (LIVO 相对纯 LIO 的增益)
```bash
# 静止 30s, 看 /Odometry 漂移
ros2 topic echo /Odometry --once  # 记 (x,y,z)

# 对比: 关掉图像 (C13_ENABLE=0 重启 c13-image, 退化为纯 LIO), 再静止 30s 看 /Odometry
# 期望: LIVO (开图) 漂移明显 < 纯 LIO (关图)
# 若无明显差异 → 视觉约束没生效, 查:
#   - T_lc 是否标定 (占位内参会让视觉约束过弱)
#   - /c13/image_raw 时间戳是否对齐 IMU 时钟域 (nx_c13_image_node 用 clock.now())
#   - LIVO 日志有无 "visual residual" 信息
```

### 5.3 导航功能 (跟 FastLIO 版同判据)
```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0, z: 0}}}}"
# 期望: 狗平滑移动到 (1,0), 不撞墙, costmap 障碍正常
```

### 5.4 长跑稳定性
```bash
# 30min 长跑, 看 /Odometry 是否持续发 (IMU 断流坑5)
ros2 topic hz /Odometry  # 跑 30min, 不应断
# 若断: livox driver restart (bringup_slam_nav2.sh 的 --watch-imu 模式)
```

---

## §6 回退 (LIVO 跑不通时)

LIVO 栈出问题 (视觉约束发散 / T_lc 标不上 / 移植编译不过) 时, 回退到已验证的 FastLIO:
```bash
# 停 LIVO 栈
sudo systemctl stop fastlivo2 c13-image 2>/dev/null

# 回 FastLIO (已验证 commit ecf9c87)
bash docker/bringup_slam_nav2.sh
# 注意: fuser 已加倾斜补偿 (body_to_base_*), FastLIO 版若要带倾斜补偿,
#       用同样参数起 fuser: 见 bringup_slam_nav2.sh step 3.5 加 -p body_to_base_pitch:=-0.349
```

---

## §7 已知坑 (memory 沉淀)

- **NVDEC 硬解必须 (c13-nvdec-hwdecode)**: 软解延迟堆积, `start_go2w_web.sh` 剥离 ws_livox
  LD 污染 + `C13_BACKEND=gst` 才流畅。`nx_c13_image_node.py` 默认 gst, 同套。
- **systemd ROS2 RMW 坑 (ros2-systemd-rmw-env)**: systemd-run 起 ROS 节点必须
  `-p User=nx` 继承 `RMW_IMPLEMENTATION`, 否则 root 缺 RMW → DDS 隐形。
  `bringup_livo.sh` 的 `start_transient` 已注入。
- **SHM 盲区 (slam-nav2-bringup-gotchas 坑2)**: costmap 报 TF two-trees 时重跑加 `--no-shm`。
- **livox CustomMsg 反序列化 (livox-spin-independent-context)**: nx_lidar_node 必须独立
  context + 自 spin。C13 Image 桥是标准 sensor_msgs, 无此坑 (独立进程, 自带 context)。
