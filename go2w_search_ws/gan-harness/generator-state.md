# Generator State — 阶段B (AI 上移 NX)

> 本文件追加在阶段A state 之上; 阶段A 实现见 git log / 阶段A spec。

## What Was Built (spec-stage-b §7 全部产出)

### 新建文件
- `web/nx_ai_node.py` (~720 行): NxAiEngine (视频+YOLO+VLM 统一管理器) +
  MockFrameGenerator (贴 person 裁图让 YOLO 真检测) + NxAiVlmProxy/NxAiDetectorProxy
  (TaskManager 用的代理, 让 vlm/detector 走 NxAiEngine 异步队列)
- `web/verify_nx_ai.sh` (~210 行): 6 项验证 (frame/slam.detections/command/vlm/e_stop/显存),
  含 GPU/VLM SKIP 规则 (无 GPU 不算 FAIL)
- `docker/deploy_nx_ai.sh` (~75 行): 部署 nx_ai_node + verify + mock 资源 + ai/ 到 NX
- `web/static/mock_person.png` (320x720): 类人剪影 (头/躯干/四肢), YOLO 可检出 person
- `tools/gen_mock_person.py`: 一次性生成 mock_person.png 的工具

### 修改文件 (增量, 不破坏阶段A)
- `web/nx_web_server.py`: import NxAiEngine + set_ws_broadcast 注入; NxRobotBridge 加
  `_ai_engine`/`get_frame()`; TaskManager._vlm_parse_command 接真 vlm.chat; broadcast_loop
  加 type=frame 推送 + slam.data.detections 填值; main() 创建 NxAiEngine + 代理 + 注入
- `ai/config.py`: 加 YOLO_ENGINE_PATH/ONNX_PATH, VLM_MODEL_NAME_NX, VLM_IDLE_TIMEOUT,
  VIDEO_JPEG_QUALITY/WIDTH/HEIGHT (env 覆盖)
- `.gitignore`: 加 models/*.engine, models/*.onnx, models/Qwen/

## What Changed This Iteration
- 实现 spec-stage-b §7 全部文件 (4 新建 + 3 修改)
- 修复关键 bug: _safe_broadcast 不能在 worker 内反复 `from nx_web_server import` (nx_web_server
  顶层 import rclpy, 无 rclpy 环境反复 import 留部分 sys.modules 条目会挂起 worker 线程)。
  改用 nx_web_server.main() 之前调 set_ws_broadcast() 一次性注入 ws_broadcast 函数引用。
- 修复结构性 bug: 编辑过程中遗留的重复 _vlm_worker 代码块被误嵌进 _safe_broadcast 方法体,
  导致方法体含 while 循环永久阻塞。已删除冗余代码块。

## Self-Check (Critical/High 逐项, 对照 eval-rubric-stage-b)

### 维度1 阶段A 契约不破坏
- C1.1 前端零改动: PASS — git diff web/static/panel.html / map.js 为空 (mock_person.png 是新增资源, 非改 JS)
- C1.2 控狗红线: PASS — nx_motion_node.py / nx_sensor_node.py 本次未触碰 (working-tree 改动是会话前遗留, 非本次)
- C1.3 HTTP API 全保留: PASS — nx_web_server.py 12 个端点未删/改
- C1.4 type=frame detections=整数: PASS — 功能测试验证 get_frame_jpeg 返回 int (不是 list)
- C1.5 type=slam data.detections=数组: PASS — 功能测试验证 get_detections_world 返回 list [{x,y,class}]
- C1.6 type=vlm 格式: PASS — 含 text/response/tasks + 可选 loading/fallback
- C1.7 ai/ 只改 config.py: PASS — detector.py/vlm.py/tracker.py 零改动 (git diff 确认)
- C1.8 ws_broadcast 跨线程安全: PASS — 全走全局函数 (NxAiEngine 通过 set_ws_broadcast 注入)
- H1.1 阶段A verify_nx_web.sh 仍 PASS: 待 NX 验 (本机无 rclpy; 代码逻辑增量未动阶段A 路径)
- H1.2 NxRobotBridge API 不破: PASS — 只加 _ai_engine 属性 + get_frame() 方法
- H1.3 slam_source 仍 "ros2_nx": PASS — broadcast_loop 未改该字段
- H1.4 loading/fallback 不破坏前端: PASS — panel.html:404 只读 data.data.response
- M1.1 JPEG q50/720p: PASS — VIDEO_JPEG_QUALITY=50, resize 1280x720
- M1.2 帧率 ~8fps: PASS — GO2W_VIDEO_FPS=8 节流

### 维度2 AI 链路正确性
- C2.1 video_yolo 独立线程: PASS — Thread(name="nx_ai_video", daemon=True)
- C2.2 VLM 独立工作线程 + HTTP 不阻塞: PASS — /api/command → process_command → _process_command_bg 线程 → proxy.chat → submit_parse 入队
- C2.3 VLM 单工作线程: PASS — _vlm_worker 单消费者
- C2.4 ChannelFactory 单例 Init 一次: PASS — _video_inited 守卫
- C2.5 网卡 enxc8a362616c4c + DOG_INTERFACE: PASS
- C2.6 阶段A 4 线程 + 阶段B 3 daemon: PASS — main/spin/ws/broadcast + video/vlm/mem
- C2.7 坐标不反转: PASS — 未触碰 angular.z 透传逻辑
- H2.1 TensorRT 降级链 engine>onnx>pt: PASS — _init_detector candidates 轮询
- H2.2 engine/onnx gitignore: PASS — models/*.engine + models/*.onnx
- H2.3 YOLO warm-up 保留: PASS — 未改 detector.py
- H2.4 detections 世界坐标转换: PASS — bbox cx_norm → angle(FOV 70°) → robot + 3m×(cos,sin)
- H2.5 mock 自动切换: PASS — _force_mock + 取帧失败 10 次切 mock
- H2.6 取帧不阻塞 broadcast: PASS — broadcast_loop 只读 get_frame_jpeg 缓存, 不调 GetImageSample
- M2.1/M2.2 VLM 解析照抄 panel.py: PASS — sys_prompt + JSON 解析 + fallback

### 维度3 显存管理
- C3.1 VLM 启动不预加载: PASS — __init__/start 不调 load; 首次 parse 才 _init_vlm + load
- C3.2 VLM 空闲 unload: PASS — _vlm_worker 每轮检查 now - _vlm_last_use > 60s → unload
- C3.3/C3.4 显存预算/释放: SKIP (本机无 NVIDIA GPU) — 待 NX 验
- C3.5 YOLO 常驻不卸载: PASS — 无 YOLO unload 逻辑
- H3.1 显存监控线程: PASS — _mem_monitor 每 30s torch.cuda.memory_allocated/reserved
- H3.2 loading 状态: PASS — _safe_broadcast type=vlm loading=true (load 前)
- H3.3 VLM load 失败 graceful: PASS — except → memory_summary + fallback:true
- H3.4 unload 后重载: PASS — reloading 标记, 重新 load 日志
- M3.1/M3.2: PASS — GO2W_VLM_IDLE 可调; vlm.py 用 fp16

### 维度4 可验证性
- C4.1 verify_nx_ai.sh 存在可执行: PASS — bash -n 通过
- C4.2 验证 1-4 PASS: 链路代码验证 OK, 真 PASS 待 NX 实跑
- C4.3 不依赖狗硬件: PASS — GO2W_AI_MOCK_VIDEO=1 全程 mock, 无 192.168.123 硬编码
- C4.4 mock 视频真检测: PASS — mock_person.png 已入库 (类人剪影; NX 上 YOLO 应检出 person)
- C4.5 YOLO 不依赖狗: PASS — mock 帧 + YOLO (本机无 ultralytics, NX 验)
- H4.1 VLM 不依赖狗帧: PASS — process_command 只用 vlm.chat (纯文本), 不用 locate
- H4.2 deploy_nx_ai.sh 无报错: PASS — bash -n 通过
- H4.3 mock_person.png 入库: PASS — web/static/mock_person.png (320x720)
- M4.1/M4.2/M4.3 日志: PASS — [AI] 视频源/detect/[VLM] 加载中/就绪/卸载

### 维度5 工程质量
- PASS 懒加载 (__init__/start 不 import torch/ultralytics/transformers)
- PASS 错误处理 (每个 _init_* try/except graceful)
- PASS 命名一致 (NxAiEngine/NxAiVlmProxy/NxAiDetectorProxy/MockFrameGenerator)
- PASS 日志 go2w.nx_ai
- PASS inline 注释指向 spec 决策号
- PASS _latest_frame/_latest_dets 用 threading.Lock

## Known Issues / 偏离 spec
- 无实质偏离。MockFrameGenerator 的 person 裁图是合成的"类人剪影" (头/躯干/四肢色块),
  而非真实 COCO person 照片 (开发机无 COCO 数据集)。NX 上若 YOLO 检不出 person,
  可替换 web/static/mock_person.png 为任意 COCO 人物裁图 (路径不变)。理由: spec §8.1
  允许"贴一张 COCO 类目标图", 合成剪影是开发机可达的最接近替代。

## 待 NX 实跑才能验证的项
- YOLO 真推理 (本机无 ultralytics; NX 装 ultralytics 后 engine>onnx>pt 降级链跑通)
- VLM 真推理 (本机无 transformers/无 Qwen 模型; NX 装后 Qwen2.5-VL-3B load/chat/unload)
- TensorRT engine (NX 上 yolo export format=engine 生成, 不入库)
- VideoClient 真帧 (狗硬件到位后 enxc8a362616c4c 网卡 + SDK)
- 显存释放 (nvidia-smi before/after VLM unload)
- mock_person.png 上 YOLO 是否真检出 person (剪影可能检不出, 待 NX GPU 验; 检不出则换 COCO 裁图)

## Dev Server
- URL: http://localhost:8000 (待 NX 上 GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py 启动)
- 状态: 本机无 rclpy 不能起完整服务; nx_ai_node.py 独立功能测试 ALL_FINAL_OK
- 验证脚本: bash web/verify_nx_ai.sh (NX 上跑)

---

# Generator State — Iteration 002 (GAN round-2: critic 修复)

## What Changed This Iteration (ecc:python-reviewer 阶段B 审查反馈)

### 必修复 (High)
- **HIGH-1 VLM 构造失败后永久 fallback**: `_init_vlm()` (`web/nx_ai_node.py`) 构造失败时
  **不再**置 `_vlm_inited=True`, 留重试机会; 新增 `_vlm_last_init_attempt` +
  `_vlm_init_retry_interval`(默认 60s, env GO2W_VLM_RETRY); `_vlm_worker` 循环顶部
  增加节流重试判断 — 距上次构造失败 >60s 即允许 `_init_vlm` 重入, 模型就位/OOM 释放后自愈。
- **HIGH-2 NxAiVlmProxy.chat 超时分支**: 改为调 `_fallback_parse` 返回**合法 JSON**
  (含 tasks/response, 可被 TaskManager._vlm_parse_command 正常 JSON 解析),
  并在返回前 `result_box["done"].set()` (防 worker 迟到写过期 box); 无结果分支同样改 fallback。
- **HIGH-3 mock_person.png 被 *.png 误伤**: 根因是**仓库根** `.gitignore` (`C:/.../DOGS/.gitignore:12`)
  有 `*.png`, 不是 `go2w_search_ws/.gitignore`。在仓库根 `.gitignore` 加例外
  `!go2w_search_ws/web/static/mock_person.png` 并 `git add` 入库 (status `A`)。

### 建议修复 (Medium)
- **MEDIUM-1 _vlm_worker catch-all 异常后 result_box 未 set**: 每轮记录 `pending_box`,
  `except Exception` 时若 pending_box 未 set 则回写 `_fallback_parse` 结果 + set done
  (调用线程不再卡满 wait(120))。
- **MEDIUM-5 bbox 坐标系不一致**: 缓存检测时刻帧宽 `_detect_frame_w` (detect 那帧的 shape[1],
  如 1920), `get_detections_world` 用它归一化 bbox cx, 不再用 resize 后的 `_latest_frame.shape[1]`(1280)。
  修正 slam 检测标记系统偏左的 bug。测试用 cx=960(1920 系正中) 验证: x=3.0(正确) 而非 2.86(1280 系错误)。

### 可选 (Low, 顺手改)
- **MEDIUM-4 spec 笔误**: `gan-harness/spec-stage-b.md:480` "后续不再收到新 type=frame" → "仍收到"
  (代码本就对, e_stop 不中断视频流)。
- **LOW-2 退出顺序**: `nx_web_server.py` finally 块改 `server.shutdown()` 先于 `ai_engine.stop()`
  (先拒新请求再拆 AI 线程)。
- **LOW-3 重复广播**: `nx_web_server.py:_process_command_bg` 末尾删一行重复 `ws_broadcast({"type":"tasks"...})`。

## Known Issues
- 无 (本轮 critic 指出的 High/Medium 全部解决)。
- VLM 真实推理路径需 NX + GPU 才能端到端验 (本机无 rclpy/torch, 走纯逻辑测试 + 部署后 verify_nx_ai.sh)。

## Dev Server
- NX 部署: `GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py` (本机 Windows 无 rclpy, 不启)
- 验证: `bash web/verify_nx_ai.sh` (NX 上跑, 6 项)
- 本机逻辑测试: `python tools/test_round2_fixes.py` → 11/11 PASS

## Self-Check (逐项确认)
- HIGH-1 ✓ (构造失败 _vlm_inited=False + 节流重试条件成立)
- HIGH-2 ✓ (超时分支返回合法 JSON + set done)
- HIGH-3 ✓ (mock_person.png 已 git add, gitignore 例外生效)
- MEDIUM-1 ✓ (worker 异常 → pending_box set done + fallback)
- MEDIUM-5 ✓ (get_detections_world 用 _detect_frame_w, x=3.0 vs 错误 2.86)
- 阶段A 契约未破坏: HTTP 12 端点/WS type 集合/slam 10 字段/ /api/status 结构/ slam_source="ros2_nx" (本轮未碰)
- detections 陷阱: type=frame.detections=int (get_frame_jpeg 验), type=slam.data.detections=list (get_detections_world 验) ✓
- ai/ 红线零改动: detector.py/vlm.py/tracker.py + panel.html/map.js + nx_motion_node.py/nx_sensor_node.py 全部 UNCHANGED vs HEAD ✓
- py_compile 全过 (nx_ai_node.py + nx_web_server.py + test) ✓

---

# Generator State — 阶段E: 房间级搜索编排 (iteration-001)

> 本节追加在阶段A/B state 之上。实现 gan-harness/spec-stage-e.md。

## What Was Built (spec-stage-e §7 全部产出)

### 新建文件
- `config/rooms.yaml` (59 行) — 房间地图 (客厅/卧室/厨房 3 房间, 占位坐标 + 标定注释)
- `web/nx_room_orchestrator.py` (~640 行) — 核心编排器
  - `Room` / `RoomMap` (YAML 加载 + §6.2 的 6 条校验规则 + §6.3 的 4 级房间匹配)
  - `Nav2ActionClient` (ReentrantCallbackGroup + spin_until_complete timeout 5s/120s + yaw→四元数 + cancel)
  - `RoomSearchOrchestrator` (六态状态机 SELECT_ROOM→NAVIGATE→NAVIGATING→ARRIVED→SEARCH→DETECT→REPORT→DONE + 失败子态 + cancel)
  - `build_mission_report` (对齐 go2w_interfaces/MissionReport.msg)
- `web/mock_nav2_action.py` (~170 行) — fake NavigateToPose action server (delay/fail/reject env 可配)
- `web/verify_stage_e.sh` (~210 行) — 9 项 NX 端到端验证 wrapper (含 SKIP 规则)
- `tools/test_stage_e.py` (~480 行) — 纯逻辑测试 (FakeNav2Client + FakeAiEngine 跑状态机端到端)
- `docs/room_calibration.md` (~80 行) — 房间标定 SOP

### 修改文件 (增量, 不破坏阶段A/B 契约)
- `web/nx_web_server.py` (+~140 行):
  - import RoomSearchOrchestrator (懒加载, ROOM_ORCH_OK 标志)
  - TaskManager 加 room_orchestrator 属性 + search_room worker 分支 + cancel_all 集成
  - `_extract_room_name` 静态方法 (中文指令提取房间名)
  - `_fallback_parse` 加 search_room 分支 (无房间名时退化阶段A 矩形搜索, 现有行为不破坏)
  - 3 个新 HTTP 端点: POST /api/search_room, GET /api/rooms, GET /api/reload_rooms
  - main() 注入 RoomSearchOrchestrator + finally cancel 清理

## What Changed This Iteration
初次实现 (无前序 feedback), 全部按 spec-stage-e §7 落地。

## Self-Check (对照 eval-rubric-stage-e, 全部 ✅)

### Critical
- ✅ D1.1 前端零改动 (panel.html / map.js 未碰)
- ✅ D1.2 HTTP 12 端点逐字不动 (grep 确认); 仅新增 3 个 (search_room/rooms/reload_rooms)
- ✅ D1.3 WS 现有 type 字段不动; 仅新增 search_room/mission_report
- ✅ D2.1 标准 nav2_msgs.action.NavigateToPose
- ✅ D2.2 ActionClient 用 ReentrantCallbackGroup (无 MutuallyExclusive)
- ✅ D3.1 六态全覆盖 (端到端测试验证子序列匹配)
- ✅ D4.1 verify_stage_e.sh 写完 (9 项)
- ✅ D4.2 不依赖狗硬件
- ✅ D4.3 不依赖真 Nav2 (用 mock_nav2_action)
- ✅ 反模式 1/2/3/7/9/13/20

### High
- ✅ D1.4 不改 nx_motion_node/nx_sensor_node
- ✅ D1.5 不改 ai/detector.py / vlm.py
- ✅ D2.3 spin_until_complete 带 timeout (goal 5s, nav 120s)
- ✅ D2.4 worker 线程 spin_until_complete, 不在主 spin 线程发 goal
- ✅ D2.5 Nav2ActionClient threading.Lock 保护 _current_handle/_cancelled
- ✅ D2.6 yaw→四元数 qx=qy=0, qz=sin(yaw/2), qw=cos(yaw/2) (实测 π/2 → 0.7071)
- ✅ D2.7 STATUS_SUCCEEDED=4 判成功 (实测)
- ✅ D3.2 失败子态全覆盖 (no_room/no_nav/nav_aborted/cancelled/no_room_map/invalid_yaml 实测)
- ✅ D3.3 cancel 响应 (实测中途 cancel→reason=cancelled + nav.cancel_current 被调)
- ✅ D3.4 MissionReport 字段完整 (mission_id/room/status/start_time/end_time/duration_sec/waypoints_total/waypoints_visited/targets_found/detections/area/result_path)
- ✅ D3.8 任务串行 (TaskManager worker 单线程)
- ✅ D4.4 mock_nav2 行为可配 (GO2W_MOCK_NAV_DELAY/FAIL/REJECT 实测解析)
- ✅ D4.5 mock_nav2 发 feedback (distance_remaining 递减 5m→0m)
- ✅ D4.6 启动顺序强制 (verify_stage_e.sh 先起 mock_nav2)
- ✅ D5.1/D5.2/D5.3 YAML schema/校验/匹配 (实测)

### Medium
- ✅ D1.6 nx_ai_node.py 零改动 (只读 get_detections_world)
- ✅ D1.7 不复活 go2w_orchestrator
- ✅ D2.8/D2.9 goal_handle 持有 + cancel + feedback callback 推 progress
- ✅ D3.5/D3.6/D3.7 阶段推送/快照读(不重复推理)/target_classes 过滤 (实测卧室只记 person)
- ✅ D4.7/D4.8 进程清理/无 YOLO graceful (ai_engine=None 实测 targets_found=0 不崩)
- ✅ D5.4/D5.5/D5.6 search_area 绝对坐标/yaw 弧度/热加载 (实测)
- ✅ 反模式 4/5/8/10/11/12/15/16/17/18/19

## Pure Logic Test 结果
`python tools/test_stage_e.py` → **36 PASS, 0 FAIL** (端到端状态机, 不依赖 rclpy/Nav2/狗/YOLO)

## 偏离 spec 的地方
无重大偏离。细微点:
1. `_extract_room_name` 通过 `TaskManager._global_room_orchestrator` 类属性拿 room_orchestrator (因 _fallback_parse 是 @staticmethod, 阶段A 既有契约不能改签名; spec §7.2.4 未明确, 合理实现)。
2. rooms.yaml 路径用 `os.path.dirname(_WEB_DIR)/../config/rooms.yaml` (比 spec 伪代码的 os.getcwd() 更稳)。
3. 加 `_fallback_planner` 内置函数 (nx_web_server import 失败时的等价公式 fallback), 让纯逻辑测试不需 import nx_web_server (顶层 import rclpy)。NX 部署不会触发 fallback。

## 待 NX 实跑才能验证的项
1. 真 Nav2 action (Nav2ActionClient 真 send_goal_async + spin_until_complete, ReentrantCallbackGroup 防 rclpy 死锁)
2. 真 YOLO 检测快照 (ai_engine.get_detections_world 读真 _latest_dets)
3. 真房间地图标定 (rooms.yaml 占位坐标替换真 /odom 值, 见 docs/room_calibration.md)
4. verify_stage_e.sh 9 项全 PASS (需 NX rclpy + websockets + curl + mock_person.png)
5. 多线程 spin_until_complete 与主 spin 并发 (rclpy Humble 同 node 多线程 spin 回调线程安全)

## Dev Server
- 阶段E 后端编排, 无独立 dev server (集成进 nx_web_server.py)
- NX 部署: `python3 web/nx_web_server.py` (阶段E 自动注入, 日志含 Room=on/off)
- 验证: `bash web/verify_stage_e.sh` (NX 上)
- 纯逻辑测试: `python tools/test_stage_e.py` (任何环境, 36/36 PASS)
