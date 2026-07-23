# Go2W 控制测试方案 v2

## 状态机 (v2 - 含机器人反馈)

```
DISCONNECTED
     │ connect() + stand()
     ▼
 STANDING ──RiseSit+BalanceStand──→ STOPPED ←──stop_move()── MOVING
                                        │                      │
                                        │ sit()                │ move()
                                        ▼                      │
                                     SITTING ──Sit()──→ SEATED
                                        │
                                        │ estop() (任意状态)
                                        ▼
                                    EMERGENCY (Damp)
```

| 状态 | 控制线程行为 | 机器人反馈 (SportModeState.mode) |
|------|-------------|------|
| STOPPED | BalanceStand + Move(0,0,0) @ 2Hz | mode=0 (NORMAL) |
| MOVING | BalanceStand + Move(vx,vy,vyaw) @ 20Hz | mode=1 (GAIT) |
| SITTING | Move(0,0,0)→StopMove→Sit | mode=2, progress 0→100 |
| SEATED | 不发送任何命令 | mode=2 |
| STANDING | RiseSit→BalanceStand→Move(0,0,0) | mode=3, progress 0→100 |
| EMERGENCY | Damp | mode=4 |

### 关键改进 (v1 → v2)

1. **订阅 `rt/sportmodestate`** — 获取机器人真实 mode/velocity/progress，不再盲猜状态
2. **STOPPED 显式 Move(0,0,0)** — 每 0.5s 发送一次，防止机器人速度指令超时后漂移
3. **MOVING→STOPPED 显式停止** — 控制线程检测到 STOPPED 后发送 Move(0,0,0)，不再仅靠"不发 Move"
4. **Sit/RiseSit 替代 StandDown/StandUp** — 使用专用坐下/起立 API
5. **StopMove 作为坐下前置** — 先取消速度再执行 Sit
6. **stats API 返回 robot_mode** — 前端可实时看到机器人状态

---

## 测试 1: 启动静止验证

| 项目 | 内容 |
|------|------|
| 前提 | panel.py 刚启动，站立序列完成 |
| 操作 | 不做任何操作，观察 30 秒 |
| 期望 | 机器人保持静止，不前进/后退/旋转 |
| 状态 | STOPPED |
| 日志验证 | `CTRL 启动` → `STANDING: RiseSit → BalanceStand` → `→ STOPPED` |
| 反馈验证 | `stats.robot_mode` = 0 (NORMAL), `stats.robot_velocity` ≈ [0,0,0] |
| 传感器 | IMU yaw 变化 < ±0.03 rad，视频画面静止 |

---

## 测试 2: 键盘 ↑ 前进

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED，狗静止 |
| 操作 | 按住 ↑ 键 2 秒 → 松开 |
| 期望 | 狗以 ~0.5 m/s 前进，松开后立即停止，不滑动 |
| 状态转换 | STOPPED → MOVING(vx=0.5) → STOPPED |
| 反馈验证 | MOVING 时 robot_mode=1, 停止后 robot_mode=0 |
| 日志 | `API: move` → `API: stop → STOPPED` |

---

## 测试 3: 键盘 ↓ 后退

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED，狗静止 |
| 操作 | 按住 ↓ 键 2 秒 → 松开 |
| 期望 | 狗以 -0.5 m/s 后退，松开后立即停止，不继续后滑 |
| 状态转换 | STOPPED → MOVING(vx=-0.5) → STOPPED |
| 注意 | 这是之前最容易出 bug 的操作，重点验证 |
| 反馈验证 | 停止后 robot_velocity ≈ [0,0,0] |

---

## 测试 4: 键盘 ← 左转

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 按住 ← 键 2 秒 → 松开 |
| 期望 | 狗逆时针旋转，松开后停 |
| 传感器 | IMU yaw 单调增加 |

---

## 测试 5: 键盘 → 右转

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 按住 → 键 2 秒 → 松开 |
| 期望 | 狗顺时针旋转，松开后停 |
| 传感器 | IMU yaw 单调减小 |

---

## 测试 6: 坐下按钮

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED，狗站立 |
| 操作 | 点击"坐下" |
| 期望 | 狗坐下（Sit，非完全趴下） |
| 状态转换 | STOPPED → SITTING → SEATED |
| 日志 | `API: sit 入队` → `SITTING: Move(0,0,0) → StopMove → Sit` → `→ SEATED` |
| 反馈验证 | robot_mode=2, robot_progress 从 0→100 |
| 后续验证 | 按下任意方向键 → **不应响应**（SEATED 忽略 move） |
| 后续验证 | 30 秒内不自行站起 |

---

## 测试 7: 坐下 → 站立 → 前进

| 项目 | 内容 |
|------|------|
| 前提 | SEATED |
| 操作 | 1) 点"站立"等 7 秒 2) 按 ↑ |
| 期望 | 狗站起后静止 → 按键后前进 |
| 状态转换 | SEATED → STANDING → STOPPED → MOVING |
| 日志 | `API: stand 入队` → `STANDING: RiseSit → BalanceStand` → `→ STOPPED` |

---

## 测试 8: 急停按钮

| 项目 | 内容 |
|------|------|
| 前提 | MOVING，狗正在前进 |
| 操作 | 点击"急停" |
| 期望 | 狗立即趴下（Damp），不管之前什么状态 |
| 状态转换 | MOVING → EMERGENCY |
| 后续 | 按方向键 → 不响应（需先点"站立"） |

---

## 测试 9: 快速交替按键

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 快速交替按 ↑/↓，每次按 0.2 秒，重复 10 次 |
| 期望 | 每次按键狗有对应方向动作，松开即停，不出现滑动 |
| 重点 | 验证状态机竞态保护 |

---

## 测试 10: 长时间静止

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 静置 2 分钟 |
| 期望 | 狗始终保持静止，不漂移 |
| 验证 | stats.robot_velocity 始终 ≈ [0,0,0] |

---

## 测试 11: 看门狗自动停止

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 快速点按 ↑（只按一下不持续）|
| 期望 | 狗可能短暂动一下，0.3 秒后自动停止 |
| 日志 | `看门狗: 0.3s 无指令, 自动停止` |

---

## 测试 12: 浏览器关闭重连

| 项目 | 内容 |
|------|------|
| 前提 | 正常使用中 |
| 操作 | 关浏览器 → 等 10 秒 → 重新打开 |
| 期望 | 视频恢复，狗保持静止，按钮可正常控制 |

---

## 测试 13: 进程重启

| 项目 | 内容 |
|------|------|
| 前提 | 正常使用中 |
| 操作 | `pkill -9 -f panel.py` → 等待 → 重新启动 |
| 期望 | 狗短暂执行站立序列 → 静止在 STOPPED 状态 |
| 重点 | 确保旧进程 DDS 速度指令不会残留 |

---

## 测试 14: 鼠标点击虚拟方向键

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 鼠标按住 ↑ 按钮 → 松开 |
| 期望 | 同键盘 ↑，前进后停止 |

---

## 测试 15: 文字指令 (VLM)

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 输入"前进两米" → 发送 |
| 期望 | VLM 解析 → move 任务 → 前进 → stop |

---

## 测试 16: robot_mode 反馈验证 (新增)

| 项目 | 内容 |
|------|------|
| 前提 | 正常使用中 |
| 操作 | 在不同状态间切换，监控 `/api/status` 返回的 `stats.robot_mode` |
| 期望 | STOPPED→0, MOVING→1, SEATED→2, EMERGENCY→4 |
| 用途 | 确保机器人反馈与我们状态机一致 |

---

## 测试 17: 前进中突然坐下 (新增)

| 项目 | 内容 |
|------|------|
| 前提 | MOVING，狗正在前进 |
| 操作 | 点击"坐下" |
| 期望 | 狗先停止前进，然后坐下 |
| 状态转换 | MOVING → SITTING → SEATED |
| 日志 | `API: sit 入队` → `SITTING: Move(0,0,0) → StopMove → Sit` |

---

## 2026-07-22 语音 / 前端 / 探索 / 视锥联合验收

记录时间：2026-07-22 21:22 +08:00。本节为当前代码树的新鲜证据，不用旧的局部模拟结果替代 NX 实机验收。

### 离线命令与结果

| 门禁 | 命令 | 结果 |
|---|---|---|
| Focused Python | `python -m pytest -q web/test_product_command.py web/test_move_executor.py web/test_task_manager_move.py tools/test_local_llm_nlu.py tools/test_voice_console.py web/test_ws_latest.py web/test_visibility_coverage.py web/test_exploration_manager.py web/test_frontier_explore.py web/test_unknown_room_exploration_sim.py docker/test_global_planner_contract.py` | `368 passed in 19.34s` |
| 地图 JS | `node web/test_map_contract.js` | `map contract tests passed` |
| Panel JS | `node web/test_panel_nav_state.js` | `panel navigation state tests passed` |
| 策略仿真 | `python tools/sim_strategy_compare.py` | H1–H9 全部 PASS；选定 `k_time=14.5`；v3 覆盖率 `98.07%`、BFS 路径 `148.25m`、路径转向 `206.67rad`、`67` probes |
| 行为合同对齐 | `python -m pytest -q web/test_scan_snapshot_contract.py::test_scan_snapshot_stores_laserscan_metadata_timestamp_and_copied_ranges tools/test_locate_anything.py::test_panel_has_locate_overlay_for_boxes_and_explanations web/test_frontier_explore.py::test_send_goal_timeout_uses_configurable_90s_default` | `4 passed in 0.15s` |
| Verifier TDD | 5 个针对 auth-disabled、MID360 restart order、odom/TF owner、Nav reverse boundary 的用例 | RED `5 failed`；GREEN `5 passed`；相关模块 `83 passed` |
| Broad | `python -m pytest -q web tools src/go2w_bridge/test docker -k "not test_lidar_topics"` | `1200 passed in 34.18s` |
| 正式离线发布门禁 | `python tools/verify_release.py` | `architecture: PASS`；`compileall` PASS；`1200 passed in 36.07s`；两套 Node PASS；`offline release gate: PASS` |

本轮对齐的三个陈旧合同：

- 单 frontier 目标不再检查源码字面量 `40.0`；现用真实 `NavigationGateway` 虚拟时钟验证 `GO2W_FRONTIER_NAV_TIMEOUT` 默认 `90.0s`、覆盖值 `12.5s` 生效，且超时实际发送 `navigation_timeout` 取消。
- scan freshness 已以 `time.monotonic()` 判定，测试同步改为单调时钟，不再用可回跳的墙钟推导 age。
- Locate WS 框不再直接绘制旧帧，测试锁定 `scheduleLocateOverlay`、`record: true` 和 C13 帧 generation，与 latest-frame 调度一致。

旧基线的 17 个残余失败经逐项核对后均确认为陈旧合同，并按当前权威行为对齐，而不是修改生产参数迎合测试：

- 认证：用户已明确禁用 Token；仍保留 HTTP 先授权决策后读 body 和非通配 CORS 合同，Panel 验证直接 `fetch`。
- 部署顺序：verifier 解析 `restart_units`，锁定 `livox net → driver → watchdog → go2w-sensor → slam`，不再要求缺少 sensor 的旧字符串。
- odom/TF：`go2w-sensor` 只发 `/wheel_odom` 且 `publish_odom_tf=false`，`map_odom_fuser` 独占 `/odom` 和 `odom→base_link`。
- 自主倒车边界：verifier 解析 YAML `min_velocity` 的线速度分量 `0.0`，并锁定 SDK owner=`nav` 时 `max(0.0, velocity[0])`；角速度下限 `-0.5` 不被误判为线性倒车。
- 地图：持久 SLAM 墙体保持 display-only；Nav 的 50m rolling global costmap 以实时 MID360 obstacle/inflation layer 为规划权威，避免 latched static ghost。
- 语音：旧 30m 房间和“运动指令返回 None”断言对齐当前 120m 大房间和 `move_relative` 产品合同。

### 发布制品（已构建，未部署）

```text
path: dist/go2w-16b10d98ce5b-dirty-9c5969fad66d-all.tar.gz
release_id: 16b10d98ce5b-dirty-9c5969fad66d
payload_digest: 9c5969fad66dd29f62c2fd6a49874735087039048d3350d286b11efa4f19bade
SHA-256: 88528CB086120FCCC0B28808E116D19CA77FA45052CD1BA4984A94DE72EBA98E
files: 114
archive_bytes: 526481
unpacked_bytes: 1680788
```

`python tools/verify_release_artifact.py <artifact>` 严格校验通过。归档目录清单中没有 `test-artifacts/`。

### NX 实机验收（未完成）

目标机已由用户确认为 `192.168.1.105`，但本节不授权部署。后续只能在现场确认急停、场地和狗的姿态后，用同一 release 逐项记录：

1. 语音“往前走”、“往后退”、“左转”、“右转”、“向前走2米”的 canonical task、实际位移/航向、超时与最终停车。
2. “搜索整个房间，标注人”和“搜索房间，标注所有椅子”的人/椅子框、照片、地图 marker 和去重 ID。
3. frontier 日志与真实未知格/墙体的一致性，以及“连续 3 轮无可达未知 frontier”的终止证据。
4. 选点 mean/p95 latency、probes、总路径/转向、控制器实际速度，以及 Panel FPS、WS replacement/reliable depth、断线重连恢复。

### 2026-07-23 实机语音验收更新

用户已完成本机 Vosk + `qwen3:8b` + NX 直发语音验收，并确认语音阶段结束。NX 现场问题修复包括：

- “搜索房间”默认搜索并标注人；“扭头”表示整只狗转向，左右方向短语均进入 `move_relative`。
- 所有语音任务由 navigation arbiter 的 `nav` owner 接管；后退和原地转向均发布 `/cmd_vel_nav`，不再误走 manual owner。
- 闭环原地转向按 50ms 周期刷新 nav 速度心跳，避免运动节点 0.3s watchdog 将单次角速度提前归零；结束和超时仍强制发送零速。

上节第 1 项据用户现场确认关闭。第 2–4 项仍属于搜索算法、感知和前端实机验收，不随语音验收自动关闭。
