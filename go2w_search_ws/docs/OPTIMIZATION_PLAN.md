# Go2W 代码审计 — 优化建议与方案

> 2026-08-14 产出。基于**源码通读**（`web/` 30+ 模块、`src/go2w_bridge/` 全量 18 源文件 + 14 测试、`ai/`、`tools/`、`src/go2w_sim/`、`src/go2w_nav/`、`docker/`、`config/`），非文档复述。
> 优先级：**P0** 正确性/必现失败 → **P1** 质量/可维护性 → **P2** 架构/演进。工作量：XS < 0.5d，S ≤ 1d，M 2-5d，L > 5d。

---

## P0 — 正确性 / 必现失败（建议尽快处理）

### P0-1 `ai/voice.py` import 必现失败 ⚠️ 已实证
- **位置**：`ai/voice.py:16-17`（`from ai.config import VOICE_MODEL_NAME, VOICE_QUANT, ...`）
- **问题**：`ai/config.py` 未定义 `VOICE_MODEL_NAME` / `VOICE_QUANT`（AST 全量扫描证实：config 只有 `AUDIO_*` / `WHISPER_*` 常量）。`import ai.voice` 必然 `NameError`；且**无任何生产调用方**（实际语音走 `tools/voice_console.py` 的 Vosk 方案）。
- **方案 A（推荐）**：删除 `ai/voice.py` + `requirements-voice.txt`，语音入口收敛为 `tools/voice_console.py` 单一路径。
- **方案 B**：若保留 Audio-Interaction 方案，在 `ai/config.py` 补 `VOICE_MODEL_NAME` / `VOICE_QUANT` 常量并接通调用链（工作量更大，且与 voice_console 重复）。
- **影响**：仅拖累 `ai/` 包可导入性；不影响运行链路。

### P0-2 运动契约测试必现失败 ⚠️ 已实证
- **位置**：`src/go2w_bridge/test/test_motion_node_v2_contract.py:139`（`test_pure_motion_modules_support_nx_direct_file_deployment`）
- **问题**：测试的 `-c "import ... unitree_sport_adapter, ..."` 清单仍含**已删除**的 `unitree_sport_adapter`（源码确认无此文件，仅 __pycache__ 残留 .pyc）。该测试断言 `returncode == 0`，**必然失败**。另有 `tools/verify_release.py:144` 也引用 `src/go2w_bridge/go2w_bridge/unitree_sport_adapter.py`（发布门禁会报缺文件）。
- **方案**：从 `:139` 的 import 清单删除 `unitree_sport_adapter`（清单共 8 个模块，删后剩 7 个）；同步从 `tools/verify_release.py:144` 的打包清单删除该路径。两处各一行，XS。

### P0-3 sim launch 引用不存在的 map ⚠️ 已实证
- **位置**：`src/go2w_sim/launch/sim_full_bringup.launch.py:141`（`maps/warehouse_map.yaml`）、`src/go2w_sim/launch/sim_nav_bringup.launch.py:78`（`maps/indoor_empty_map.yaml`）
- **问题**：全库（含 build/install）**不存在**这两个地图文件，`setup.py` 的 data_files 也未打包 → Nav2 `map:=` 参数断链，仿真 Nav2 点选闭环不可用。
- **方案 A（推荐）**：两个 launch 改为走 `slam_online` 在线建图（与真机一致，无预置地图依赖），删除静态 map 参数。
- **方案 B**：用 `map_saver` 从 worlds 生成两张 pgm/yaml 并加入 `setup.py` data_files。
- **影响**：`sim_full_bringup` / `sim_nav_bringup` 的 Nav2 导航部分在仿真中失效。

### P0-4 传感器自屏蔽 footprint 两套且不一致
- **位置**：`nx_sensor_node.py`（self_box `(-0.25,0.30,-0.20,0.20)`）vs `mid360_nav_bridge.py`（`(-0.45,0.45,-0.32,0.32)`）
- **问题**：`nx_sensor_node.py:231` 注释点名"与 nav2_params_3d.yaml 一致"，但数值实际对齐 `nav2_params_slim`（±0.30/±0.20），与 `nav2_params_3d`（±0.45/±0.32）不符；`mid360_nav_bridge` 才与 3d 一致。三处若不同，会出现"代价层把狗自身当障碍"或"狗身盲区"。
- **方案**：定义单一 `FOOTPRINT` 常量（建议放 `go2w_bridge/nx_config.py` 或沿用 `ai/config.py` 风格），sensor / mid360_bridge / `nav2_params_3d.yaml` 三处引用同一来源，并加跨包契约测试断言一致。
- **影响**：安全相关（障碍滤波），需实车确认。

---

## P1 — 质量 / 可维护性

### P1-5 三个建图节点零测试
- `map_odom_fuser.py`（位姿主干）、`map_padding_bridge.py`（动态障碍清除）、`mid360_nav_bridge.py`（安全扫描）**无任何运行时测试**；`nx_sensor_node.py` 仅 2 个 AST 契约测试。
- **方案**：为 `map_odom_fuser` 写确定性单测（注入固定 `/Odometry` + `/wheel_odom`，断言 `/odom`、`/localization_pose`、TF 输出与时间戳门）；`map_padding_bridge` 用合成 OccupancyGrid + 扫描消息验证 padding 与障碍清除；`mid360_nav_bridge` 验证 CustomMsg→LaserScan 映射。M。
- **理由**：三者是建图→导航链路的承重墙，目前行为完全无回归保护。

### P1-6 规划器三处重复
- `nx_web_server.plan_lawnmower/plan_spiral`（L438/459）≈ `nx_room_orchestrator._fallback_planner`（L3510，等价公式）≈ `nx_active_search` 栅格 NBV 规划。
- **方案**：抽 `web/nx_mission_planner.py`（或 `planner_utils.py`）：统一 lawnmower/spiral/栅格覆盖 API；`room_orchestrator` 顶层导入（消除 `nx_room_orchestrator.py:3131` 的懒 import 循环依赖）。M。

### P1-7 死代码清理（低风险，可批量）
| 项 | 位置 | 处置 |
|---|---|---|
| `battery_allows_drive()` | `motion_safety.py:39` | 无调用者，删 |
| `DriveExecutionWatchdog.observe_low_state()` | `motion_safety.py:321` | 无运行时调用者，删（连同 `WHEEL_INDICES`） |
| `ScanFreshnessWatchdog.nav_must_stop()` | `motion_safety.py:264` | 仅测试引用，删或改私有 |
| token 校验主体 | `nx_control_auth.py:54-70` | `auth_disabled` 短路后不可达；见 P2-11 |
| `_wheel_radius` | `nx_sensor_node.py:215,286` | 读入未用，删或接 telemetry |

### P1-8 样板/重复代码
- `go2w_bridge` 10 处 `try: from .x import … except ImportError: from x import …` 双导入样板 → 抽 `go2w_bridge/_compat.py`。
- `sport_gateway_server._read_frame`（:269-284）与 `sport_gateway_client._read_frame`（:154-168）逐字重复 → 提到 `sport_gateway_protocol.py`。
- RPY 矩阵构建三处重复（`nx_sensor_node._transform_lidar_to_base` / `map_odom_fuser._rpy_to_mat` / `mid360_nav_bridge._rpy_matrix`）→ 抽 `go2w_bridge/_geometry.py`。
- `WHEEL_MODES` 在 `motion_controller.py` 与 `motion_safety.py` 各定义一次 → 收敛到 `motion_types.py`。

### P1-9 硬编码 / 魔数
- 网卡默认 `enxc8a362616c4c`（nx_motion_node / nx_sensor_node / nx_sport_gateway / nx_safety_observer 四处）→ 环境变量统一（部署已用 `/etc/go2w/hardware.env`）。
- `body_to_base_pitch=-0.3490658504`（-20°）在 `map_odom_fuser.py:197` 与 `mid360_nav_bridge.py:174` 硬编码重复 → 常量化并注释来源。
- `nx_sensor_node._on_lidar` 的 `range_max=10.0`、`bin_count=360`、`range_min=0.15` 字面量 → 参数化（其余字段已参数化）。
- 状态机魔数（`wheel_stop_threshold=0.20`、`moving_fault_samples=15`、`park_zero_settle=0.50`、姿态 0.70、电池 20.0）已有注释但散落 → 收敛为模块常量块。

### P1-10 docstring 与实现不一致（文档债）
- `nx_lidar_node.py` docstring 写订阅 `/livox/lidar`（CustomMsg），实际订阅 `PointCloud2 /mid360/points_nav`（两条分支都是）。
- `nx_web_server.py:8` docstring 写订阅 `/imu /scan`，实际订阅 `/livox/imu` + `/scan_mid360`。
- `nx_ai_node.py` docstring 线程编号与实际不符。
- 修复：改 docstring 即可，XS；已同步修正到 `docs/PROJECT_STRUCTURE.md` 与 `docs/ARCHITECTURE.md`。

---

## P2 — 架构 / 演进

### P2-11 鉴权状态需要决策
- `nx_control_auth.authorize_request` 自 **2026-07-16** 用户要求短路为 `auth_disabled`（局域网全放行），token 校验为不可达死代码，`GO2W_CONTROL_TOKEN` / `control-token.txt` / `generate_control_token.py` 相关链路已空转。
- **方案 A**：确认局域网可信 → 删除不可达校验代码 + 收敛 token 工具链（清理最彻底）。
- **方案 B**：恢复鉴权 → 删除短路，重新启用校验；前端 panel.html 需补 Bearer 注入。
- **建议**：至少保留 CORS `GO2W_PANEL_ORIGINS` 白名单（已在用）；该决策已在文档中标注。

### P2-12 旧栈文档/脚本与现状脱节
- `setup_jetson.sh`（JetPack 4.6 / Ubuntu 20.04 / ROS2 **Galactic** / **CycloneDDS** 一键部署）与 `hardware/SETUP_GUIDE.md`（旧网段/RTSP/JetPack 4.6）均描述已废弃的安装路径；当前是 humble + FastDDS + systemd 原子发布。
- **方案**：`setup_jetson.sh` 顶部加 deprecated 横幅并指向 `docs/NX_REDEPLOY.md`（或重写为 humble 版）；`hardware/SETUP_GUIDE.md` 同步。S-M。

### P2-13 Nav2 配置三份并存
- `nav2_params{,_3d,_slim}.yaml` + 3 个 launch 变体（`nav2.launch.py` 为早期独立版）。`nav2_3d` 是真机主力，`nav2_slim` 是降级路径，`nav2.launch.py` 已无引用价值。
- **方案**：删除 `nav2.launch.py` + `nav2_params.yaml`（先 grep 确认无引用）；`slim` 保留并注明触发条件。

### P2-14 语音双方案收敛（与 P0-1 合并）
- `ai/voice.py`（Audio-Interaction-3B，无调用方且坏 import）vs `tools/voice_console.py`（Vosk，在用）。收敛为后者。

### P2-15 `map_odom_fuser` 频率瓶颈（待实车数据）
- Python 实现发布仅 ~0.6Hz，bringup 已跳过 5Hz 检查（`bringup_slam_nav2.sh`）。若实车出现定位/导航抖动：
  - 短期：记录基准 + 提高 fuser 发布频率（看 FAST_LIO `/Odometry` 输入频率上限）；
  - 中期：将位姿变换热路径（`_conjugate_pose` + SE(2) 投影）C++ 化或向量化。L。

### P2-16 导航双通道 preflight 重复
- `/api/navigate`（point 通道：`arbiter.start_point_goal`）与任务内 `_navigate_blocking`（tasks 通道：`MissionNavigationPort`）各自独立 preflight（定位健康 / 距离上限 / scan 新鲜）。
- **方案**：抽公共 `NavigationPreflight`，两通道共用；统一"定位健康"判定口径（`get_localization_health`）。M。

### P2-17 测试基建统一
- 现状：`web/tests/`（pytest 契约）、`src/go2w_bridge/test/`（pytest，14 文件）、`tools/test_*.py`（脚本式，`conftest.py` 让 pytest 跳过）。
- **方案**：`tools/` 脚本式测试改为 pytest 函数式（或保留但统一 `pytest -m` 标记）；CI 接 GitHub Actions（master 已是活跃分支），跑 `go2w_bridge` 测试 + `web/tests` + 死引用静态检查。M。

---

## 汇总

| 优先级 | 数量 | 主题 | 建议顺序 |
|---|---|---|---|
| P0 | 4 | voice.py 坏 import / 契约测试死引用 / sim map 断链 / footprint 不一致 | 先 P0-2（1 行）、P0-1、P0-3 |
| P1 | 6 | 建图节点测试 / 规划器去重 / 死代码 / 样板 / 硬编码 / docstring | 与日常开发穿插 |
| P2 | 7 | 鉴权决策 / 旧栈清理 / Nav2 收敛 / 语音收敛 / fuser 性能 / preflight 统一 / 测试基建 | 规划迭代 |

> 文档同步：本计划与 `docs/PROJECT_STRUCTURE.md` §6（已知遗留/死代码）互为索引；代码修复后请同步更新两处。
