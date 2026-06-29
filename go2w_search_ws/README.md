# Go2W Search & Discover

通过前端（及未来的语音）让 Unitree Go2W 轮足机器狗自动搜索区域、发现并报告目标。

> ⚠️ **权威文档声明**：本文件仅作快速入口。
> - 项目**真实结构与文件状态** → [`docs/PROJECT_STRUCTURE.md`](docs/PROJECT_STRUCTURE.md)
> - **技术决策与实测结论** → [`docs/TECH_DECISIONS.md`](docs/TECH_DECISIONS.md)
> - 其余 `docs/*.md` 多为阶段性记录，阅读时注意时效（部分已被推翻，见各文首注明）。

---

## 系统架构（NX 中心化 — 迁移进行中）

```
┌── PC (仅前端 UI + 高层指令) ────────────────────┐
│  浏览器 → web/panel.py (HTTP:8000 / WS:8001)     │
│  [迁移目标: panel 重活迁 NX, PC 退化为瘦客户端]   │
└────────────────────┬────────────────────────────┘
                     │ ROS2 Humble DDS (手机热点, 只传低频状态/指令)
                     ▼
┌── 载荷 NX (Orin NX 16GB, Ubuntu 22.04, Humble) ──┐  ← 所有重活在此
│  nx_motion_node   持 lease 控狗 (systemd 自启)     │
│  nx_sensor_node   读狗 IMU/雷达 → /imu /scan /odom │
│  [迁移中] FAST_LIO + Nav2 + YOLO/VLM              │
└────────────────────┬────────────────────────────┘
                     │ USB 转网口 (192.168.123.100/24)
                     │ unitree_sdk2py / CycloneDDS
                     ▼
          狗主控 192.168.123.161 (出厂系统, 只收 SDK 指令)
```

**核心原则**（详见 [`docs/REFACTOR_NX_CENTRIC.md`](docs/REFACTOR_NX_CENTRIC.md)）：
- **NX 本机闭环**：感知 → 建图 → 规划 → 控狗 全在 NX，零跨网延迟
- **lease 钉在 NX**：压制狗主控残留乱跑程序；PC↔NX 热点断了，NX 看门狗仍会自动停狗
- **热点只传低频数据**：状态/指令（KB 级），不传点云/视频流

> 技术栈：**ROS2 Humble + Python**（非 Rust/Galactic；早期 README 的 Rust 描述已废弃）。

---

## 硬件

| 组件 | 说明 |
|------|------|
| Unitree Go2W | 轮足版机器狗（主控 192.168.123.161） |
| Jetson Orin NX 16GB | 载荷，跑 ROS2 Humble + 全部重活 |
| USB-Ethernet (AX88179) | NX → 狗主控，192.168.123.100/24（nmcli 持久化, con-name: go2-dog） |
| MID360 LiDAR (USB 版) | 接 NX，建图用（⚠️ USB 供电问题排查中，见 TROUBLESHOOTING 问题7） |

接线/IP/网卡清单见 [`hardware/SETUP_GUIDE.md`](hardware/SETUP_GUIDE.md) 与 [`docs/REFACTOR.md`](docs/REFACTOR.md) 第五节。

---

## 当前能力状态

| 能力 | 状态 | 说明 |
|------|------|------|
| Web 前端（键盘/按钮） | ✅ | `web/panel.py` + `static/panel.html` |
| 站立 / 坐下 / 急停 | ✅ | `nx_motion_node` |
| 狗轮式移动 | ⏳ 待实车 | _do_stand 已加回 BalanceStand（对齐 panel.py），硬件装完验证 |
| 乱跑 / 后滑防护 | ✅ | systemd 崩溃自启 + 看门狗超时停狗 |
| YOLO 检测 | ✅ | `ai/detector.py`（当前在 PC，待迁 NX） |
| 地图/雷达显示 | ⚠️ | `nx_sensor_node` 数据流 |
| VLM 指令解析 | ⚠️ | `ai/vlm.py`，模型加载失败时降级关键词匹配 |
| 语音控制 | ⏳ | `ai/voice.py` 待修（暂后置） |
| FAST_LIO 建图 | ⏳ | 待 MID360 供电解决后落地 |
| Nav2 自主导航 | ⏳ | 依赖建图 + 移动 |
| 自动搜索/跟踪 | ⏳ | 依赖 Nav2 |

---

## 项目结构（精简版）

> 完整逐文件状态见 [`docs/PROJECT_STRUCTURE.md`](docs/PROJECT_STRUCTURE.md)。

| 路径 | 状态 | 说明 |
|------|------|------|
| `web/panel.py` + `static/panel.html` + `static/map.js` | ✅ 活跃 | 当前前端后端 + 页面 |
| `ai/`（detector / vlm / tracker / config） | ✅ 活跃 | AI 推理（待迁 NX） |
| `src/go2w_bridge/nx_motion_node.py` | ✅ 活跃 | NX 控狗（systemd 服务） |
| `src/go2w_bridge/nx_sensor_node.py` | ✅ 活跃 | NX 读狗传感器 |
| `docker/`（deploy_nx.sh, go2w-motion.service） | ✅ 活跃 | NX 部署 + systemd |
| `src/go2w_interfaces/` | 💤 休眠 | msg/srv，NX 多节点通信将启用 |
| `src/go2w_nav/` | 💤 休眠 | Nav2 配置，迁移目标载体 |
| `src/go2w_orchestrator/` | 💤 休眠 | 任务编排，迁移目标载体 |
| `src/go2w_detector/` | 💤 休眠 | YOLO ROS 节点，迁移目标载体 |
| `src/go2w_bringup/` | 💤 休眠 | launch，迁移目标载体 |
| `web/server.py`、`static/index.html` | ❌ 废弃 | panel.py 的前身（老单体） |
| `src/go2w_bridge/bridge_node.py`、`sport_client.py` | ❌ 废弃 | 老 PC 直连狗桥（被 NX 架构取代） |

> - 💤 **休眠** = 当前不在运行链路，但是 NX 中心化迁移的**目标载体**，**勿删**。
> - ❌ **废弃** = 已被取代，待清理（清理前会再次确认无引用）。

---

## 快速开始（阶段A：web 通信层上移 NX）

> 阶段A 起，web 服务（`web/nx_web_server.py`，内嵌 rclpy）跑在载荷 NX 上，PC 摆脱 `go2w_humble` Docker 容器，浏览器直连 `http://<NX_IP>:8000`。PC 端不再需要 rclpy / 容器 / `dog_state.json` 文件桥。详见 [`gan-harness/spec.md`](gan-harness/spec.md)。

**NX 端**（一次性部署）：
```bash
NX_HOST=<NX_IP> bash docker/deploy_nx.sh        # 控狗服务 go2w-motion (lease 持有)
NX_HOST=<NX_IP> bash docker/deploy_nx_web.sh    # web 服务 go2w-web (HTTP:8000 + WS:8001)
```
两者开机自启（systemd `enabled`）。web 服务依赖控狗服务（`go2w-web.service` 设 `After=go2w-motion.service`）。

**PC 端**（每次开机，只开浏览器）：
```bash
cd go2w_search_ws
bash web/start_pc_browser.sh    # 只提示浏览器打开 http://<NX_IP>:8000
```

**验证**（NX 上跑，8 项全 PASS，不依赖狗硬件）：
```bash
bash web/verify_nx_web.sh       # 启 nx_web + mock，跑 curl + WS 断言
```

> 退役链路（PC fallback，可回滚）：`web/start_ros2.sh.legacy`（原 PC 容器 + panel.py 路径）、`web/cmd_publisher.py`、`web/ros_to_json.py`、`web/panel.py` 均保留不删。

---

## 架构演进方向

正在从「PC 跑重活」迁移到「NX 跑所有重活，PC 仅 UI」。分阶段路线与并行分工见
[`docs/REFACTOR_NX_CENTRIC.md`](docs/REFACTOR_NX_CENTRIC.md) 与
[`docs/TECH_DECISIONS.md`](docs/TECH_DECISIONS.md) 第四节。

## 关键决策与踩坑

- [`docs/DECISIONS.md`](docs/DECISIONS.md) — 架构/部署决策
- [`docs/TECH_DECISIONS.md`](docs/TECH_DECISIONS.md) — 技术调研结论（移动控制 / FAST_LIO / Nav2）
- [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) — 实测踩坑（网卡 / DDS 版本 / USB 供电等）
