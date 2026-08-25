# Go2W Search & Discover

通过前端（及语音）让 Unitree Go2W 轮足机器狗自动搜索区域、发现并报告目标。

> ⚠️ **权威文档声明**：本文件仅作快速入口。
> - 项目**真实结构与文件状态** → [`docs/PROJECT_STRUCTURE.md`](docs/PROJECT_STRUCTURE.md)
> - **技术决策与实测结论** → [`docs/TECH_DECISIONS.md`](docs/TECH_DECISIONS.md)
> - **整体架构** → [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
> - 其余 `docs/*.md` 多为阶段性记录，阅读时注意时效（部分已被推翻，见各文首注明）。

---

## 系统架构（NX 中心化）

```
┌── PC (仅浏览器瘦客户端) ─────────────────────┐
│  访问 http://<NX_IP>:8000 (HTTP) / :8001 (WS) │
└────────────────────┬─────────────────────────┘
                     │ 手机热点 (只传低频状态/指令)
                     ▼
┌── 载荷 NX (Orin NX 16GB, Ubuntu 22.04, ROS2 Humble) ──┐  ← 所有重活在此，本机闭环
│  go2w-web         nx_web_server.py  HTTP:8000 + WS:8001 │
│                   (内嵌感知/云台/雷达/任务编排组件)        │
│  go2w-motion      nx_motion_node.py  持 lease 控狗       │
│  go2w-sport-gateway  nx_sport_gateway.py  Unix socket 网关│
│  go2w-sensor      nx_sensor_node.py  读狗传感器          │
│  go2w-safety-observer   nx_safety_observer.py  只读看门狗 │
│  go2w-slam-nav    bringup_slam_nav2.sh  (transient units) │
│                   FAST_LIO + map_odom_fuser + Nav2 + slam │
│  costmap-bridge   写 /tmp/*.json 供 web 转发              │
│  livox-*          MID360 雷达驱动 + 看门狗               │
└────────────────────┬────────────────────────────────────┘
                     │ USB 转网口 (192.168.123.100/24)
                     │ unitree_sdk2py / FastDDS
                     ▼
          狗主控 192.168.123.161 (出厂系统, 只收 SDK 指令)
```

**核心原则**（详见 [`docs/REFACTOR_NX_CENTRIC.md`](docs/REFACTOR_NX_CENTRIC.md)）：
- **NX 本机闭环**：感知 → 建图 → 规划 → 控狗 全在 NX，零跨网延迟
- **lease 钉在 NX 网关**：`nx_sport_gateway.py` 是全系统唯一 `SportClient(enableLease=True)` 持有者，运动策略进程重启期间租约不断、永不重放速度
- **热点只传低频数据**：状态/指令（KB 级），不传点云/视频流

> 技术栈：**ROS2 Humble + Python**（非 Rust/Galactic；早期 README 的 Rust 描述已废弃）。
> 部署方式：**原子发布**（`docker/build_release.sh` + `docker/deploy_release.sh`，content-addressed 归档 + 软链切换）。

---

## 硬件

| 组件 | 说明 |
|------|------|
| Unitree Go2W | 轮足版机器狗（主控 192.168.123.161） |
| Jetson Orin NX 16GB | 载荷，跑 ROS2 Humble + 全部重活 |
| USB-Ethernet (AX88179) | NX → 狗主控，192.168.123.100/24（nmcli 持久化, con-name: go2-dog） |
| MID360 LiDAR (网口版) | 接 NX（192.168.1.160，host 192.168.1.200/32），`livox-mid360-net.service` 持久化，建图用 |

接线/IP/网卡清单见 [`hardware/SETUP_GUIDE.md`](hardware/SETUP_GUIDE.md)。

---

## 当前能力状态

| 能力 | 状态 | 说明 |
|------|------|------|
| Web 前端（键盘/按钮/地图/视频） | ✅ | `web/nx_web_server.py` + `static/panel.html`（HTTP:8000 + WS:8001） |
| 站立 / 坐下 / 急停 / 手动移动 | ✅ | `nx_motion_node.py` 状态机（9 态）经 `nx_sport_gateway.py` 控狗 |
| 自主导航（点选 / 任务） | ✅ | Nav2 + `nx_navigation_arbiter.py` 运动所有权仲裁（point/tasks/manual） |
| 房间搜索（frontier 探索） | ✅ | `nx_room_orchestrator.py` + `nx_exploration_manager.py` + `nx_frontier_planner.py` |
| 产品搜索（next-best-view） | ✅ | `nx_active_search.py` 视锥覆盖 NBV 规划 |
| 目标物证（照片/去重/报告） | ✅ | `nx_person_mission.py`（空间+外观融合去重，report.json 落盘） |
| 目标跟踪 / 跟随 | ✅ | `ai/tracker.py`（VLM + SDK 闭环） |
| 取物（fetch） | ✅ | `nx_fetch_action.py` 插件（S1-S5 状态机） |
| YOLO 检测 / VLM 定位 | ✅ | `ai/detector.py` / `ai/vlm.py` / `ai/locate_anything.py`（注入 `nx_web_server` 同进程） |
| LLM 指令解析 | ✅ | `nx_llm_planner.py`（DeepSeek propose-verify，动作白名单，fail-closed） |
| 语音控制（PC 端） | ✅ | `tools/voice_console.py`（Vosk 离线 STT → 确定性解析 → 可选本地 LLM → NX `/api/command`） |
| 仿真全栈 | ✅ | `src/go2w_sim/`（Gazebo + FastLIO + Nav2 + web，`GO2W_SIM=1`） |
| 地图/雷达显示 | ✅ | `/scan_mid360` + `/mid360/points_nav` → 前端渲染 |
| 室外 GPS 航线导航 | ✅代码就绪⏳待实机标定 | `web/nx_gps_nav.py`（NMEA/NavSatFix→质量门→ENU+北向→逐航点）+ `/api/gps/route`；北向标定流程见 `docs/GPS_NAV.md` §4 |
| UWB 钥匙扣跟随 | ✅代码就绪⏳待实机标定 | `nlink_uwb` 协议桥（Nooploop AOA）+ `web/nx_uwb_follow.py` 状态机 + `/api/uwb_follow/*`；AOA 角度零偏标定见 `docs/UWB_FOLLOW.md` |

> ⏳ 遗留：`ai/voice.py`（Audio-Interaction 语音方案）无生产调用方且 import 即 NameError（引用 config 未定义的常量），实际语音走 `tools/voice_console.py`；`config/rooms.yaml` 三房间仍为占位坐标（`calibrated: false`），需实车标定。

---

## 项目结构（精简版）

> 完整逐文件状态见 [`docs/PROJECT_STRUCTURE.md`](docs/PROJECT_STRUCTURE.md)。

| 路径 | 状态 | 说明 |
|------|------|------|
| `web/nx_web_server.py` | ✅ 活跃 | **唯一 Web 主程序**（HTTP+WS+rclpy），装配所有组件（感知/云台/雷达/任务编排） |
| `web/nx_*.py`（30+ 模块） | ✅ 活跃 | 任务编排 / 导航仲裁 / 产品命令 / 物证 / 探索 / 前端桥等，见 PROJECT_STRUCTURE |
| `web/static/panel.html` + `map.js` | ✅ 活跃 | 前端页面 + 地图 Canvas 渲染 |
| `src/go2w_bridge/` | ✅ 活跃 | 运动控制层：`nx_motion_node` / `nx_sensor_node` / `nx_sport_gateway` / `nx_safety_observer` + 纯策略状态机（`motion_*.py`） |
| `src/go2w_nav/` | ✅ 活跃 | Nav2 launch + 参数（`nav2_3d` 真机主力） |
| `src/go2w_sim/` | ✅ 活跃 | Gazebo 仿真包（7 自定义节点 + 4 launch） |
| `ai/` | ✅ 活跃 | detector / vlm / locate_anything / tracker / cloud_llm / config（voice.py 遗留待清理） |
| `tools/` | ✅ 活跃 | 诊断 `diag_*` / 安全门控 `*_gate` / 发布验证 `verify_*` / 语音 `voice_console.py` / 运维脚本 |
| `docker/` | ✅ 活跃 | 12 个 systemd service + `build_release.sh` / `deploy_release.sh`（原子发布唯一入口） |
| `config/rooms.yaml` | ⏳ 占位 | 房间标定坐标（需实车标定后 `calibrated: true`） |

> 旧 PC 中心化架构产物（`panel.py` / `cmd_publisher.py` / `ros_to_json.py` / PC 容器）**已全部删除**，勿再参考旧文档中的路径。

---

## 快速开始

### 路径 A：仿真（无需硬件，30 分钟内跑通）

见顶层 [`QUICKSTART.md`](../QUICKSTART.md)。核心一条命令：

```bash
cd go2w_search_ws
export GO2W_WEB_DIR="$PWD/web"
ros2 launch go2w_sim sim_full_bringup.launch.py   # Gazebo + FastLIO + Nav2 + web
# 浏览器打开 http://localhost:8000
```

### 路径 B：真狗（NX 原子发布）

**NX 端**（一次性部署，全部 systemd 自启）：

```bash
python tools/verify_release.py                       # 离线发布门禁
bash docker/build_release.sh all                     # 构建 content-addressed 归档
NX_HOST=<NX_IP> NX_USER=nx bash docker/deploy_release.sh \
  dist/<artifact>-all.tar.gz --allow-motion-restart \
  --control-token-file control-token.txt             # 原子部署（软链切换 current）
```

**PC 端**：每次开机只开浏览器 `http://<NX_IP>:8000`（`bash web/start_pc_browser.sh`）。

> 旧的 `deploy_nx.sh` / `deploy_nx_web.sh` / `deploy_nav2_bprime.sh` 仅作历史排障参考，不再是生产发布入口（见 `docs/PROJECT_STRUCTURE.md` 与 `docs/NX_REDEPLOY.md`）。

**验证**（NX 上跑）：
```bash
bash web/scripts/verify_nx_web.sh    # 启 nx_web + mock，跑 curl + WS 断言
bash web/scripts/verify_nx_ai.sh     # AI 感知链验证
bash web/scripts/verify_stage_e.sh   # 端到端阶段验证
```

---

## PC 本地语音指令

`tools/voice_console.py` 的本地流程为：Vosk 离线识别 → 确定性产品指令解析 →（仅在需要时）本地 LLM 归一化 → 再次确定性校验 → NX `/api/command`。先用文本 dry-run 检查配置：

```bash
python tools/voice_console.py --text "帮我找一下椅子并标出来" --no-auto-send \
  --llm-url http://127.0.0.1:11434/api/chat \
  --llm-model qwen2.5:3b --llm-mode fallback --llm-timeout 5
```

Ollama 使用 `/api/chat`；其他本地 OpenAI 兼容服务使用完整的 `/v1/chat/completions` 地址。对应环境变量为 `GO2W_LOCAL_LLM_URL`、`GO2W_LOCAL_LLM_MODEL`、`GO2W_LOCAL_LLM_MODE` 和 `GO2W_LOCAL_LLM_TIMEOUT`。模式可选 `off`、`fallback`（默认）和 `always`；URL 留空会彻底禁用 LLM 请求。

安全边界：本地模型没有直接控制权，只能提出一个规范中文移动或"当前房间搜索"指令；任何输出都必须重新通过 `validate_voice_command`，并继续接受 NX 端解析与任务准入检查。

> ⚠️ 鉴权说明：NX 端 `nx_control_auth.authorize_request` 自 2026-07-16 起被用户要求短路为 `auth_disabled`（局域网内所有 `/api` 请求免 Bearer Token）。恢复 Token 限制需重新启用 `nx_control_auth.py` 中已被短路的下半段校验逻辑。

---

## 架构演进方向

正在从「PC 跑重活」迁移到「NX 跑所有重活，PC 仅 UI」——**已完成**：当前全部重活（感知/建图/规划/控狗/搜索）均在 NX，PC 只开浏览器。历史分阶段路线见
[`docs/REFACTOR_NX_CENTRIC.md`](docs/REFACTOR_NX_CENTRIC.md) 与
[`docs/TECH_DECISIONS.md`](docs/TECH_DECISIONS.md) 第四节。

## 关键决策与踩坑

- [`docs/DECISIONS.md`](docs/DECISIONS.md) — 架构/部署决策
- [`docs/TECH_DECISIONS.md`](docs/TECH_DECISIONS.md) — 技术调研结论（移动控制 / FAST_LIO / Nav2）
- [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) — 实测踩坑（网卡 / DDS 版本 / USB 供电等）
- [`docs/OPTIMIZATION_PLAN.md`](docs/OPTIMIZATION_PLAN.md) — 代码级优化建议与方案（2026-08 代码审计产出）
