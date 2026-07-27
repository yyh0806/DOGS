# QUICKSTART — 另一台电脑快速上手

> 目标：在一台干净的 Linux 机器上 30 分钟内把项目跑起来。
> **路径 A（仿真）为主** —— 无需任何硬件，Gazebo 里跑完整 Nav2 + FastLIO + 搜索栈。
> **路径 B（真狗）为辅** —— 连 Go2W 实车的部署指针。

---

## 0. 先决条件对照表

| 路径 | 操作系统 | 关键软件 | 硬件 |
|------|----------|----------|------|
| **A 仿真**（推荐） | Ubuntu 22.04（原生双启动最稳，WSL2 可用但有 SIGFPE 坑） | ROS2 Humble + Gazebo Classic 11 + fast_lio | 无 |
| **B 真狗** | Ubuntu 22.04 (Jetson Orin NX, aarch64) | ROS2 Humble + 全套 systemd 服务 | Go2W + MID360 + USB 网口 |

> ⚠️ 当前活跃分支是 `codex/product-room-person-search`（领先 master 7 万+行）。clone 后**务必切到此分支**，否则拿不到仿真栈。

---

## 路径 A：仿真模式（无需硬件）

### Step 1 — Clone & 切分支

```bash
git clone https://github.com/yyh0806/DOGS.git
cd DOGS
git checkout codex/product-room-person-search
```

### Step 2 — 装系统依赖（一次性）

假设 ROS2 Humble 已装（[官方安装指引](https://docs.ros.org/en/humble/Installation.html)）。再装仿真栈需要的 apt 包：

```bash
sudo apt update && sudo apt install -y \
  ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-topic-tools \
  ros-humble-tf2-ros ros-humble-tf2-tools \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  gazebo \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  libwebsockets-dev
```

> 如果 `ros-humble-gazebo-ros-pkgs` 找不到，先确认 `ros-humble-desktop-full` 已装，或单独装 `ros-humble-gazebo-ros` + `ros-humble-gazebo-plugins` + `gazebo11`。

### Step 3 — 装 fast_lio（第三方 LIO，仿真栈依赖）

仿真用社区 ROS2 fork（本项目在实机改过 `laserMapping.cpp` 的 IMU 单位，但**仿真 IMU 是 Gazebo 原生 m/s²**，无需那个 patch，原版即可）：

```bash
mkdir -p ~/go2w_ws/src && cd ~/go2w_ws/src
git clone -b ros2 https://github.com/hku-mars/FAST_LIO.git fast_lio
# Livox CustomMsg 插件 (仿真 URDF 用 ray sensor 出 PointCloud2, 此包非必需但建议装)
git clone -b ros2 https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/go2w_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4 — 把本项目软链进 workspace，编译

```bash
# 把 DOGS 里的 ROS2 包软链到 colcon workspace (避免重复 clone)
ln -s /path/to/DOGS/go2w_search_ws/src/go2w_bridge ~/go2w_ws/src/go2w_bridge
ln -s /path/to/DOGS/go2w_search_ws/src/go2w_nav    ~/go2w_ws/src/go2w_nav
ln -s /path/to/DOGS/go2w_search_ws/src/go2w_sim    ~/go2w_ws/src/go2w_sim

cd ~/go2w_ws
colcon build --symlink-install
source install/setup.bash
```

> 把 `/path/to/DOGS` 换成你 Step 1 clone 的实际绝对路径。

### Step 5 — 编译 SIGFPE workaround（WSL2 必须，原生 Linux 可选但建议）

WSL2 的 glibc 2.35 在 `cos(0)` 会 SIGFPE 把 gzserver 打挂（见 `tools/mycos.c` 头注释）。原生 Linux 双启动一般无此问题，但 launch 无条件 `LD_PRELOAD=/root/mycos.so`，所以**任何机器都要放一份**避免告警：

```bash
cd /path/to/DOGS/go2w_search_ws/tools
gcc -shared -fPIC -O2 -Wl,--version-script=mycos.map -o mycos.so mycos.c -lm
sudo mkdir -p /root && sudo cp mycos.so /root/mycos.so
```

### Step 6 — 启动仿真全栈

```bash
cd /path/to/DOGS/go2w_search_ws
export GO2W_WEB_DIR="$PWD/web"     # 覆盖 launch 里默认的 WSL2 /mnt/c 路径
ros2 launch go2w_sim sim_full_bringup.launch.py
```

这一条命令会拉起：gzserver + go2 URDF + FastLIO + Nav2 + slam_toolbox + motion_sdk_mock + sim_telemetry_bridge + nx_web_server + nx_sim_video_node。

### Step 7 — 打开浏览器

```
http://localhost:8000
```

应看到：雷达 scan 点云（绿）+ 狗 marker（FastLIO 定位）+ 视频画面（URDF 相机）+ 11 个服务状态灯。

### 验证清单（开另一个终端）

```bash
source ~/go2w_ws/install/setup.bash
ros2 topic list | grep -E '/scan|/Odometry|/map|/cmd_vel|/tf'   # 应有这些话题
ros2 topic hz /scan                                              # 应 ~3-10Hz
curl -s http://localhost:8000/api/status | head -c 200          # 应返回 JSON
```

**功能验证**（在浏览器里）：
- 点地图某处 → 狗通过 Nav2 自主导航过去（看狗 marker 移动）
- 点"搜索房间"按钮 → frontier 探索开始（地图逐渐被覆盖）
- 键盘 W/S 或前进/后退按钮 → `cmd_vel` 链路控狗

### 常用运维脚本

```bash
bash go2w_search_ws/tools/kill_sim.sh      # 杀掉所有仿真进程（清场）
bash go2w_search_ws/tools/restart_sim.sh   # 清场 + rebuild go2w_sim + 重启（⚠️ 路径硬编码 /mnt/c，需手动改）
bash go2w_search_ws/tools/diag_sim.sh      # 诊断仿真栈健康（话题频率/TF/状态灯）
```

> ⚠️ `restart_sim.sh` / `kill_sim.sh` 内部路径硬编码了原作者的 `/mnt/c/Users/ROG/...`，在新机器上需要 `sed -i "s|/mnt/c/Users/ROG/yangyuhui/DOGS|/path/to/DOGS|g"` 改一下，或直接用上面的 `ros2 launch` 命令。

---

## 路径 B：真狗模式（NX + Go2W）

> 仿真栈就是真机栈，唯一区别是**不设 `GO2W_SIM` 环境变量**，`nx_motion_node` 走真机 `SportGatewayClient` socket。

### Step 1 — 硬件接线

| 链路 | 接口 | IP |
|------|------|----|
| NX → 狗主控 | USB-Ethernet (AX88179) | NX 端 192.168.123.100/24，狗主控 192.168.123.161 |
| MID360 LiDAR | NX 网口 | 192.168.1.160 |
| PC ↔ NX | 手机热点 Wi-Fi | NX DHCP 动态（用 ARP 扫描找） |

详细见 `go2w_search_ws/hardware/SETUP_GUIDE.md`。

### Step 2 — NX 部署（一次性）

```bash
# 在 PC 上对 NX 部署 (假设 NX_IP 已知)
NX_HOST=<NX_IP> bash go2w_search_ws/docker/deploy_nx.sh        # go2w-motion (控狗, lease 持有)
NX_HOST=<NX_IP> bash go2w_search_ws/docker/deploy_nx_web.sh    # go2w-web (HTTP:8000 + WS:8001)
```

两者开机自启（systemd `enabled`）。`go2w-web` 依赖 `go2w-motion`（`After=`）。

### Step 3 — PC 端

每次开机只需打开浏览器：

```
http://<NX_IP>:8000
```

---

## 故障排查

| 症状 | 原因 | 解决 |
|------|------|------|
| `gzserver ... SIGFPE` 崩 | WSL2 glibc cos(0) 除零 | Step 5 编译 `mycos.so` 放到 `/root/mycos.so` |
| `object '/root/mycos.so' from LD_PRELOAD cannot be preloaded` | .so 不存在或架构不对 | 同上；aarch64 用 `gcc` 默认即可，x86_64 也一样 |
| web 打不开 / 500 | `GO2W_WEB_DIR` 没设或路径错 | `export GO2W_WEB_DIR=<绝对路径>/go2w_search_ws/web` |
| 点击导航不动 | `cmd_vel` 链路 / nav2 没激活 | `ros2 topic echo /cmd_vel` 看是否有速度；看 11 个状态灯哪盏红 |
| `Package 'go2w_sim' not found` | colcon build 没过 / setup.bash 没 source | `source ~/go2w_ws/install/setup.bash` |
| `map` frame 缺失 / TF 报错 | static_transform_publisher 没起 | 看 `ros2 run tf2_tools view_frames` |
| 真狗模式狗不动 | 时钟/NTP/lease/costmap 8 层坑之一 | 见 `docs/TROUBLESHOOTING.md` 与项目记忆 |

更多踩坑见 `go2w_search_ws/docs/TROUBLESHOOTING.md`、`docs/TECH_DECISIONS.md`。

---

## 项目结构速查

| 路径 | 作用 |
|------|------|
| `go2w_search_ws/src/go2w_sim/` | 仿真包（nodes + launch + worlds + URDF） |
| `go2w_search_ws/src/go2w_bridge/` | 控狗 + 传感器 + TF 桥（`nx_motion_node` / `nx_sensor_node`） |
| `go2w_search_ws/src/go2w_nav/` | Nav2 配置 + slam_online launch |
| `go2w_search_ws/web/` | web 服务（`nx_web_server.py`）+ 前端（`static/`）+ 搜索逻辑 |
| `go2w_search_ws/tools/` | 运维脚本（仿真/部署/诊断）+ `mycos.c` SIGFPE workaround |
| `go2w_search_ws/docker/` | NX 部署脚本 + systemd service 文件 |

权威文档：`go2w_search_ws/README.md` + `docs/PROJECT_STRUCTURE.md`。
