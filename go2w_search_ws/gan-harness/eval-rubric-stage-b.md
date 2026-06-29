# Evaluation Rubric: Go2W 阶段B — AI 上移 NX（YOLO + VLM + 视频流）

> Consumer: GAN Critic / Verifier
> Source of truth: `gan-harness/spec-stage-b.md`（本文件是可执行检查清单）
> 项目根: `go2w_search_ws/`
> 验证前提: 不实车（硬件在装）；用 mock 视频帧验证 YOLO 真检测 + WS 推送；VLM 在有 GPU 的 NX 上验，无 GPU 环境部分项 SKIP
> 前置: 阶段A 已通过 critic 审查（`web/nx_web_server.py` + `verify_nx_web.sh` 8/8 PASS）

## 总分计算

总分 = 阶段A契约不破坏×0.25 + AI链路正确性×0.25 + 显存管理×0.20 + 可验证性×0.20 + 工程质量×0.10
每维 0-100 分。**任一 Critical 项 = 0 分则整体不通过（GAN 协商继续）**。

---

## 维度 1: 阶段A 契约不破坏（权重 0.25）— 前端无感 + 阶段A 功能不退化

### Critical（任一失败 = 整体不通过）

- [ ] **C1.1 前端零改动**：`git diff web/static/panel.html` 和 `git diff web/static/map.js` 必须为空（红线）。
  - 验证：`git diff --name-only web/static/` 输出为空
- [ ] **C1.2 控狗红线不破**：`git diff src/go2w_bridge/go2w_bridge/nx_motion_node.py` 和 `nx_sensor_node.py` 必须为空（阶段A 红线继续生效）。
  - 验证：`git diff --name-only src/go2w_bridge/`
- [ ] **C1.3 阶段A HTTP API 全保留**：nx_web_server.py 的 12 个 HTTP 端点（阶段A C1.1）一字不改。阶段B 只新增 AI 注入，不删/改现有端点。
  - 验证：grep `p.path ==` / `parsed.path ==`，对照阶段A spec §5.1 表
- [ ] **C1.4 type=frame 格式对齐 panel.py:847-849**：`{"type":"frame","data":"<base64>","detections":<int>}`。`detections` 是**整数计数**（panel.html:389 `(data.detections||0)+' 目标'`），不是数组。
  - 验证：grep `"type": "frame"` 或 `"type":"frame"`，确认 detections 是 int 不是 list
  - **若 detections 写成数组 = Critical 失败**（前端显示"NaN 目标"）
- [ ] **C1.5 type=slam 的 data.detections 是数组**：`[{x,y,class}]` 格式（map.js:52 `detMarks = data.detections`）。与 type=frame 的整数 detections **相反**，Generator 极易混淆。
  - 验证：人工核对 broadcast_loop 的 slam dict，detections 是 list of dict
  - **若 slam 的 detections 写成整数 = Critical 失败**（地图标记消失）
- [ ] **C1.6 type=vlm 格式对齐 panel.py:466**：`{"type":"vlm","data":{"text","response","tasks"}}`。阶段B 新增的 `loading`/`fallback` 是可选字段，前端 panel.html:404 忽略未知字段不报错。
  - 验证：grep `"type": "vlm"`，确认 data 含 text/response/tasks
- [ ] **C1.7 不改 ai/detector.py / ai/vlm.py / ai/tracker.py**：`git diff ai/` 仅含 config.py 的路径常量新增（spec §7.3），detector/vlm/tracker 三个核心文件零改动。
  - 验证：`git diff --name-only ai/` 仅 `ai/config.py`
- [ ] **C1.8 ws_broadcast 跨线程安全**：阶段A 的 `ws_broadcast()` + `asyncio.run_coroutine_threadsafe`（nx_web_server.py:64-68）不改。阶段B 的 VLM worker / video 线程调 ws_broadcast 走同一通道。
  - 验证：grep `ws_broadcast` 调用点，确认都用全局函数，不自建 WS 通道

### High

- [ ] **H1.1 阶段A verify_nx_web.sh 仍 PASS**：阶段B 改动后，重跑 `bash web/verify_nx_web.sh` 仍 8/8 PASS（阶段A 功能不退化）。
- [ ] **H1.2 NxRobotBridge 公共 API 不破**：move/stop_move/stand/sit/e_stop/connected/imu_yaw/robot_state/stats/_lock/_vx/_vy/_vyaw 全保留（阶段A spec §6.1）。阶段B 只**新增** `get_frame()` 和 `_ai_engine` 属性。
  - 验证：grep NxRobotBridge 的 `def ` 和 `@property`，对照阶段A
- [ ] **H1.3 broadcast_loop 的 slam_source 仍 "ros2_nx"**：阶段B 不改成 "ros2_ai" 或其他（前端地图右上角标识不变）。
- [ ] **H1.4 type=vlm 的 loading/fallback 不破坏前端**：前端 panel.html:404 只读 `data.data.response`，新增字段被忽略。Critic 在浏览器 F12 Console 确认无 JS 报错。

### Medium

- [ ] M1.1 JPEG 质量 = 50（env 可调），分辨率 720p（spec 决策 3）。
- [ ] M1.2 帧率节流 ~8fps（对齐 VideoClient GetImageSample 5-8fps）。

---

## 维度 2: AI 链路正确性（权重 0.25）— 线程模型 / TensorRT / 坐标

### Critical

- [ ] **C2.1 视频/YOLO 独立线程**：`NxAiEngine._video_yolo_loop` 在独立 daemon 线程跑，**不在 broadcast_loop 里同步取帧+检测**（会阻塞 status/slam 推送）。
  - 验证：grep `Thread(target=.*video_yolo` 或 `Thread(target=.*_video`，确认独立线程
- [ ] **C2.2 VLM 独立工作线程**：`NxAiEngine._vlm_worker` 在独立 daemon 线程跑，**HTTP handler 不同步调 VLM.chat**（会 HTTP 超时）。`/api/command` 立即返回，parse 入队异步处理。
  - 验证：grep `submit_parse` + `_vlm_queue.put`，确认异步；grep HTTP handler 无 `vlm.chat` 同步调用
- [ ] **C2.3 VLM 单工作线程**：所有 parse 请求经 `queue.Queue` 串行消费，**不并发推理**（GPU 串行 + 抢显存）。
  - 验证：grep `_vlm_worker`，确认单线程消费队列
- [ ] **C2.4 ChannelFactory 单例 Init 一次**：NxAiEngine 在本进程只 Init 一次 ChannelFactory（SDK_CAPABILITIES §3.3）。重复 Init 会报错/泄漏。
  - 验证：grep `ChannelFactory()` + `.Init(`，确认有 `if self._video_inited: return` 守卫
- [ ] **C2.5 网卡名 enxc8a362616c4c**：与 nx_sensor_node.py:57 / nx_motion_node.py:56 一致，env `DOG_INTERFACE` 可覆盖。
  - 验证：grep `enxc8a362616c4c` 或 `DOG_INTERFACE`
- [ ] **C2.6 不改阶段A 4 线程模型**：main(HTTP) + spin(rclpy) + WS(asyncio) + broadcast 四线程结构不动，阶段B 只**新增** video_yolo / vlm_worker / mem_monitor 三个 daemon 线程。
  - 验证：grep `threading.Thread`，确认阶段A 的 4 个 + 阶段B 的 ≤3 个新增
- [ ] **C2.7 坐标不反转（阶段A C2.4 继续生效）**：NxRobotBridge/NxWebNode 的 /cmd_vel angular.z 仍直接透传 vyaw，阶段B 不引入反转。

### High

- [ ] **H2.1 TensorRT 降级链 engine>onnx>pt**：NxAiEngine._init_detector 按 candidates 顺序尝试，任一成功即用，全失败 `_detector=None`。
  - 验证：grep `_init_detector`，确认 candidates 列表 + 轮询
- [ ] **H2.2 engine/onnx 不入库**：`.gitignore` 含 `models/*.engine` `models/*.onnx`。
  - 验证：`git check-ignore models/yolov8n.engine` 返回该路径
- [ ] **H2.3 YOLO warm-up 保留**：detector.py:28-30 的 dummy 推理 warm-up 不删（首次实时推理会 ctypes 崩溃）。Generator 若改 Detector 需保留，但 spec §7.4 禁止改 detector.py。
- [ ] **H2.4 detections 世界坐标转换合理**：bbox 中心 x 归一化 → 方位角（FOV 假设 70°），距离假设 3m，世界坐标 = robot + 3m×(cos(yaw+ang), sin(yaw+ang))。
  - 验证：人工核对 get_detections_world 公式
- [ ] **H2.5 mock 模式自动切换**：VideoClient Init 失败 / 取帧连续失败 10 次 → `_mock_mode=True`，日志告警，浏览器看 mock 帧（不黑屏）。
  - 验证：grep `_mock_mode`，确认切换逻辑
- [ ] **H2.6 VideoClient 取帧不阻塞主循环**：GetImageSample 的 200-500ms 延迟在视频线程内消化，broadcast_loop 只读缓存快照（不调 GetImageSample）。
  - 验证：grep broadcast_loop 内无 `GetImageSample` / `_get_frame` 同步调用，只有 `get_frame_jpeg` 读缓存

### Medium

- [ ] M2.1 VLM 推理用 panel.py:472-495 的 sys_prompt（指令解析 JSON 任务格式），保持解析行为一致。
- [ ] M2.2 VLM chat 走 `_vlm_parse_command`（panel.py:472-518 的 JSON 解析 + fallback 逻辑），不自创解析。
- [ ] M2.3 NxAiVlmProxy 的同步等待用 threading.Event，不用 busy-wait（CPU 浪费）。

---

## 维度 3: 显存管理（权重 0.20）— VLM 按需加载是红线

### Critical

- [ ] **C3.1 VLM 启动不预加载**：nx_web 启动时 VLM 状态 = unloaded（`_vlm=None` 或 `_vlm.loaded=False`）。首次 `/api/command` 才触发 load。
  - 验证：grep NxAiEngine.__init__ 和 start()，确认不调 `_vlm.load()`；grep `_init_vlm`，确认首次 parse 时才调
- [ ] **C3.2 VLM 空闲 unload**：`_vlm_worker` 检查 `now - _vlm_last_use > 60s` → `vlm.unload()` + `torch.cuda.empty_cache()`（vlm.py:163-175 已实现 unload）。
  - 验证：grep `_vlm_idle_timeout` + `unload`，确认卸载逻辑
- [ ] **C3.3 显存预算不超红线**：YOLO 常驻 + VLM 加载 ≤ 8GB（NX 16GB 的一半）。Critic 在 NX 跑 `nvidia-smi` 确认 VLM 加载时显存 ≤ 8GB。
  - 验证（有 GPU 才验）：`nvidia-smi --query-gpu=memory.used --format=csv`，VLM 加载时 used ≤ 8000MB
- [ ] **C3.4 VLM unload 后显存释放**：unload + empty_cache 后，`nvidia-smi` 的 reserved 显存下降 ~6GB（VLM 权重回 OS）。
  - 验证（有 GPU 才验）：对比 unload 前后 reserved 值
- [ ] **C3.5 YOLO 常驻不卸载**：YOLO 是高频检测，全程常驻（spec 决策 2）。**不能为了腾显存卸载 YOLO**（重载 engine 慢 + 检测中断）。
  - 验证：grep 确认无 YOLO unload 逻辑

### High

- [ ] **H3.1 显存监控线程**：NxAiEngine._mem_monitor 每 30s 日志 `torch.cuda.memory_allocated/reserved`（决策 2）。
  - 验证：grep `_mem_monitor` + `memory_allocated`
- [ ] **H3.2 VLM load 期间发 loading 状态**：前端看到 `type=vlm` 的 `loading:true`（spec §6.3），不是无声卡顿。
- [ ] **H3.3 VLM load 失败 graceful**：模型路径错/显存 OOM → 日志 error + memory_summary，发 `type=vlm` 的 `fallback:true`，走关键词匹配（不崩进程）。
- [ ] **H3.4 unload 后新请求触发重载**：VLM unload 后又来 `/api/command`，重新 load（用户感知 10-30s），日志打印"[VLM] 重新加载"。

### Medium

- [ ] M3.1 VLM 空闲超时 env 可调（`GO2W_VLM_IDLE`，默认 60s）。
- [ ] M3.2 VLM 推理用 fp16（vlm.py:65 `torch_dtype=torch.float16`，CUDA 可用时）。

---

## 维度 4: 可验证性（权重 0.20）— 不依赖狗硬件

### Critical

- [ ] **C4.1 verify_nx_ai.sh 存在且可执行**：`test -x web/verify_nx_ai.sh`。
- [ ] **C4.2 验证项 1-4 必须 PASS**（spec §7.1）：
  1. WS 收到 `type=frame`，base64 解码合法 JPEG
  2. `type=slam` 的 `data.detections` 字段存在（数组）
  3. `/api/command` 触发 `type=vlm`（response 非空）
  4. `/api/e_stop` 不中断视频流
- [ ] **C4.3 不需要狗硬件**：`GO2W_AI_MOCK_VIDEO=1` 模式下，verify_nx_ai.sh 全程 0 处依赖 unitree_sdk2py / 狗 IP / USB 网卡。
  - 验证：grep verify_nx_ai.sh 无 `192.168.123` / `enxc8a362616c4c` 硬编码（除非 VideoClient 失败自动切 mock）
- [ ] **C4.4 mock 视频真检测**：MockFrameGenerator 贴 person 裁图，YOLO 在 mock 帧上真检出 person（detections 非空）。
  - 验证：日志打印 `detect: person 0.xx`；WS 抓包 type=frame 的 detections ≥ 1
- [ ] **C4.5 YOLO 不依赖狗**：YOLO 在 mock 帧跑检测，不需要狗摄像头。Critic 在无狗环境（任意装 torch+ultralytics 的 Linux，最好有 GPU）跑 verify_nx_ai.sh 也能验视频+检测链路。

### High

- [ ] **H4.1 VLM 不依赖狗帧**：阶段B 的 process_command 只用 VLM.chat（纯文本，vlm.py:110），不用 locate（需图像）。verify_nx_ai.sh 的 `/api/command` 在 mock 模式能触发 VLM 真解析。
- [ ] **H4.2 deploy_nx_ai.sh 部署流程无报错**：scp nx_ai_node.py + verify_nx_ai.sh + mock 裁图到 NX，可在 mock SSH 环境验。
- [ ] **H4.3 mock 帧 person 裁图随仓库入库**：`web/static/mock_person.png`（或类似）存在，让 mock 检测可复现。
  - 验证：`test -f web/static/mock_person.png`（但**不**计入 web/static/ 的"前端零改动"红线——这是新增资源，非改 panel.html/map.js）

### Medium

- [ ] M4.1 日志清晰区分视频源：`[AI] 视频源: mock` / `[AI] 视频源: unitree VideoClient`。
- [ ] M4.2 日志打印 YOLO 检测：`detect: person 0.85, chair 0.72`（每帧或抽样）。
- [ ] M4.3 日志打印 VLM 状态：`[VLM] 加载中...` / `[VLM] 就绪` / `[VLM] 卸载 (空闲 60s)`。

---

## 维度 5: 工程质量 / Craft（权重 0.10）

非硬性，但影响 Critic 是否判 "High" 问题：

- [ ] **懒加载**：NxAiEngine.__init__ 和 start() 不 import torch/ultralytics/transformers。首次 _init_video/_init_detector/_init_vlm 时才 import。nx_web 启动仍是秒级。
  - 验证：grep `import torch` / `from ultralytics` / `from transformers`，确认都在 `_init_*` 方法内，不在模块顶部或 __init__
- [ ] **错误处理**：每个 _init_* 方法 try/except，失败 graceful 退化（mock/None/fallback），不崩进程。
- [ ] **命名一致**：NxAiEngine / NxAiVlmProxy / NxAiDetectorProxy / MockFrameGenerator 与阶段A 的 NxWebNode / NxRobotBridge 风格一致。
- [ ] **日志格式**：`logging.getLogger("go2w.nx_ai")`，与阶段A 的 `"go2w.nx_web"` 一致前缀。
- [ ] **注释**：关键决策（ChannelFactory 单例、VLM 按需加载、坐标转换简化、JPEG 质量）有 inline 注释指向 spec 决策号。
- [ ] **线程安全**：_latest_frame / _latest_dets 用 threading.Lock 保护（视频线程写，broadcast 读）。

---

## 协商收敛标准（GAN 循环退出条件）

- 0 个 Critical 未通过
- 0 个 High 未通过（或 High 已有明确修复 plan 且 Critic 接受）
- verify_nx_ai.sh 验证项 1-4 PASS（Critic 实跑证据）
- 阶段A verify_nx_web.sh 重跑仍 8/8 PASS（回归不破坏）
- git diff 确认 `web/static/panel.html`、`web/static/map.js`、`nx_motion_node.py`、`nx_sensor_node.py`、`ai/detector.py`、`ai/vlm.py`、`ai/tracker.py` 零改动
- `.gitignore` 含 `models/*.engine` `models/*.onnx`

未满足 → Critic 退回 Generator 继续协商，无轮次上限。

---

## Critic 验证环境说明

| 验证项 | 无 GPU 环境 | 有 GPU（NX/带卡 Linux） | 有狗硬件 |
|---|---|---|---|
| C1.x 契约一致性 | ✅ 可验（grep + git diff） | ✅ | ✅ |
| C2.1-C2.3, C2.6-C2.7 线程模型 | ✅ 可验（grep） | ✅ | ✅ |
| C2.4-C2.5 ChannelFactory/网卡 | ⚠️ grep 验（不实跑） | ✅ | ✅ |
| H2.1 TensorRT 降级链 | ⚠️ grep 验 | ✅（跑 PT 降级） | ✅（跑 engine） |
| C3.x 显存管理 | ❌ SKIP（无 GPU） | ✅ | ✅ |
| C4.x 可验证性 | ⚠️ mock 视频帧链路可验，YOLO 用 PT 慢但通 | ✅ | ✅ |
| H4.1 VLM 解析 | ❌ SKIP（无 GPU 跑 VLM） | ✅ | ✅ |
| C4.5 YOLO mock 检测 | ⚠️ PT 降级可验（慢） | ✅ | ✅ |

Critic 无 NX 时，至少在装了 torch+ultralytics 的 Linux（CPU 也行，YOLO PT 降级）跑 verify_nx_ai.sh 的验证项 1-2（frame + slam.detections 字段），确认链路通；GPU 项标 SKIP 不算 FAIL。
