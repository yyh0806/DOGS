# Product Specification: Go2W 阶段B — AI 上移 NX（YOLO + Qwen2.5-VL + 视频流闭环）

> Generated from brief: "为 Go2W 阶段B：AI 上移 NX 产出完整实现规格（GAN Planner 角色）"
> 主管角色: GAN Planner（架构/规格，不含代码实现）
> 状态: 待 Generator 实现待 Critic 审查
> 范围: 纯软件先落地（硬件在装），用 **mock 视频帧** 验证 YOLO 真检测 + WS 推送 + VLM 按需加载
> 前置: 阶段A 已完成并通过 critic 审查（`web/nx_web_server.py` 在 NX 跑 HTTP:8000/WS:8001，内嵌 rclpy，PC 浏览器直连，0 Critical/0 High）

---

## 0. 规格阅读约定

- 所有路径均为**相对仓库根** `go2w_search_ws/`（Generator 实现时拼绝对路径）。
- 每个文件给三段：**职责 / 关键签名 / 实现要点**。签名是契约，实现要点是约束。
- **阶段A 红线继续生效**：禁止改 `web/static/panel.html`、`web/static/map.js` 的 JS 业务逻辑与 WS 消息字段名；禁止改 `nx_motion_node.py` / `nx_sensor_node.py` 的现有逻辑。
- **阶段B 新红线**：禁止把 `.engine`/`.onnx` 模型文件入库（gitignore）；VLM 必须**按需加载**，不能常驻显存。

---

## 1. Vision（阶段B 目标态）

载荷 Orin NX 16GB 在跑阶段A 的 web 服务（HTTP:8000 + WS:8001）基础上，**额外承载 AI 推理**：
1. 本机用 unitree `VideoClient.GetImageSample()` 取狗摄像头 1080p 帧（狗硬件到位后），或用 **mock 视频源**（不依赖狗）；
2. YOLOv8（**TensorRT FP16 engine，无则降级 ONNX/PT**）实时检测画框，~25ms/帧；
3. 检测后 JPEG 压缩 → broadcast_loop 发 `type=frame`（base64）给 PC 前端，复用阶段A 的 WS 通道；
4. 检测结果 detections 经 `type=slam` 的 `data.detections` 字段推前端地图标记；
5. TaskManager 接**真 VLM**（Qwen2.5-VL-3B）做指令解析，**按需 load/unload**（用户发指令时 load，空闲 N 秒 unload，腾显存给后续 FAST_LIO/Nav2）。

一句话验收：**PC 浏览器访问 `http://<NX_IP>:8000`，能在"第一视角"画面看到带 YOLO 检测框的视频流（mock 帧或真狗帧），地图上看到 detections 标记点；在指令框输入"前进两米"，看到 `type=vlm` 的 VLM 解析结果（而非 fallback 关键词匹配）；空闲一段时间后 `nvidia-smi` 显示 VLM 已 unload。**

---

## 2. 阶段A → 阶段B 链路对比（Generator 必读）

### 阶段A（已交付，故意不跑 AI）
```
浏览器(PC) ──HTTP/WS──> nx_web_server.py(NX,HTTP8000/WS8001, 内嵌 rclpy)
                          ├─ 发布 /cmd_vel /cmd_pose (本机) → nx_motion_node
                          ├─ 订阅 /dog_state /imu /scan /odom (本机) → 推 WS
                          ├─ broadcast_loop: 发 status/slam(tasks/vlm/search), 但:
                          │    - **不发 type=frame** (视频帧)
                          │    - slam.data.detections 恒为 []
                          │    - TaskManager 的 detector=vlm=None, 走 _fallback_parse
                          └─ 4 线程: main(HTTP) + spin(rclpy) + WS(asyncio) + broadcast
```

### 阶段B（本次实现，补 AI 层）
```
浏览器(PC) ──HTTP/WS──> nx_web_server.py(NX, 阶段A 4 线程 + 阶段B 新增 3 线程)
                          ├─ [新增] 视频/YOLO 线程: VideoClient.GetImageSample() / mock 帧
                          │      → YOLO TensorRT 检测 → 画框 → 缓存最新标注帧 + detections
                          ├─ [新增] VLM 线程: 按需 load/unload Qwen2.5-VL, 处理 process_command
                          ├─ [改] broadcast_loop: 读视频线程缓存的标注帧 → 发 type=frame;
                          │      detections 填入 slam.data.detections (不再恒空)
                          ├─ [改] TaskManager: detector=真 Detector, vlm=真 VLMEngine(懒加载代理)
                          └─ [新增] 内存/显存监控: 周期日志 + VLM 空闲 unload 计时
```

收益：PC 不再跑任何 AI（之前 PC 端 panel.py 加载 YOLO/VLM 占 PC 显存）；视频帧在 NX 本机取（省跨热点带宽：原来 panel.py 在 PC 经 docker 桥接拉狗帧，现在 NX 本机拉）；AI 与移动控制同在 NX，端到端延迟最小化。

---

## 3. 关键设计决策（已拍板，给推荐 + 理由）

### 决策 1：AI 代码组织 → **推荐 (b) 新建 `web/nx_ai_node.py` 作为 AI 适配层，与 nx_web_server.py 同进程（不是独立 rclpy 节点）**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 全部塞进 `nx_web_server.py` | 单文件部署 | nx_web_server.py 从 ~400 行膨胀到 ~1000+ 行；AI 的 import（torch/ultralytics/transformers）拖慢 web 启动；Critic 必挂"单文件巨怪" | ❌ |
| **(b) 新建 `web/nx_ai_node.py`，作为同进程的 AI 适配层（class NxAiEngine），nx_web_server.py import 它** | **职责分离：web 协议层（nx_web_server）+ AI 推理层（nx_ai_node）清晰边界；阶段A 的 4 线程模型不动，AI 作为"组件"注入；VLM 懒加载/卸载逻辑内聚在 NxAiEngine**；nx_web_server 改动小（main 里把 detector/vlm 从 None 换成 NxAiEngine 提供的实例） | 需新写一个文件 ~300 行；同进程意味着 AI import 仍拖慢启动（用懒加载解决：NxAiEngine 在 __init__ 不 import torch，首次 detect/parse 时才 import） | ✅ **推荐** |
| (c) 独立 rclpy 节点 `nx_ai_node.py`，订阅视频发 detections | 符合 ROS2 节点规范；AI 进程崩溃不影响 web | **视频流不走 ROS2**（狗帧来自 VideoClient.GetImageSample 是 SDK 调用，不是 ROS2 话题）；跨进程传 1080p 帧（用 ROS2 image topic）开销大且引入 cv_bridge 依赖；detections 经 ROS2 话题再到 web 进程，多一跳延迟；ChannelFactory 进程级单例意味着两个进程各 Init 一次（可行但浪费） | ⚠️ 不选，过度工程 |

**理由总结**：(b) 是"组件注入而非进程拆分"——AI 是 nx_web 的一个能力插件，不是独立服务。视频帧在 web 进程内由 VideoClient 取（不走 ROS2），YOLO 在 web 进程内推理，标注帧直接进 broadcast_loop 缓存，零跨进程拷贝。VLM 懒加载代理让 web 启动时不 import torch（秒级启动），首次解析指令时才加载。

> ⚠️ **ChannelFactory 单例注意**：`nx_web_server.py` 进程目前**不初始化 ChannelFactory**（阶段A 红线：web 不直连狗 SDK）。阶段B 要加 VideoClient，**必须在本进程初始化 ChannelFactory**。这与 `nx_sensor_node` / `nx_motion_node` **不冲突**——它们是**独立进程**，各自 Init 各自的 ChannelFactory（SDK_CAPABILITIES §3.3 明确："多进程各自初始化不冲突"）。但**同进程内 ChannelFactory 只能 Init 一次**，所以 NxAiEngine 初始化 VideoClient 时要复用同一个 factory 句柄。

### 决策 2：VLM 按需加载的线程模型 + 显存管理 → **推荐"懒加载代理 + 单工作线程 + 空闲超时卸载"**

**Orin NX 16GB 显存预算（红线，必须守）**：

| 组件 | fp16 显存 | 占用方式 | 阶段B 策略 |
|---|---|---|---|
| YOLO TensorRT FP16 engine | ~150MB（engine）+ ~200MB（推理上下文/激活） | **常驻** | 全程常驻（检测是高频，加载卸载开销大） |
| Qwen2.5-VL-3B fp16 | ~6GB（模型权重）+ ~1GB（KV cache/激活） | **按需** | 用户发指令时 load，空闲 60s 后 unload |
| CUDA runtime + CUDA context | ~400MB | 常驻 | 不可省 |
| **预留给 FAST_LIO/Nav2** | ~3-4GB | 阶段C+ | VLM unload 后腾出 6-7GB |
| **合计（VLM 加载时）** | ~7.5GB / 16GB | | 安全（剩余 ~8GB 给系统/其他） |
| **合计（VLM 卸载时）** | ~1.5GB / 16GB | | FAST_LIO/Nav2 有 ~14GB 可用 |

**线程模型**：
```
VLM 工作线程 (单线程, daemon):
  - 队列接收 parse 请求 (text, frame 可选)
  - 首次请求: VLMEngine.load() (~10-30s 加载 + warm-up)
  - 推理: VLMEngine.chat() 或 locate()
  - 推理完: 更新 _last_use_time
  - 空闲循环: 每 5s 检查 now - _last_use_time > 60s → VLMEngine.unload()
  - unload 后 torch.cuda.empty_cache() (vlm.py:173 已实现)
```

**关键约束**：
1. **VLM 单工作线程**：VLM 推理是 GPU 串行的，多线程并发推理无收益且抢显存。所有 parse 请求入队，单线程消费。
2. **load/unload 互斥**：vlm.py 已有 `self._lock = threading.Lock()`（vlm.py:44），load/unload/chat/locate 都在锁内。Generator 不要破坏这个锁。
3. **unload 后的请求处理**：若 unload 后又来新请求，重新 load（用户感知 ~10-30s 延迟，前端应显示"VLM 加载中..."）。spec §6 的 `_vlm_parse_command` 要发 `type=vlm` 的 `{"loading":true}` 中间状态。
4. **启动不预加载**：nx_web 启动时 VLM 状态 = unloaded（节省启动时间 + 启动时不占显存）。首次 `/api/command` 才触发 load。
5. **YOLO 与 VLM 共存**：YOLO 常驻 ~350MB，VLM 加载时 +7GB，合计 ~7.5GB < 16GB，安全。**不需要 YOLO 让显存**。

### 决策 3：视频帧 WS 推送的带宽控制 → **推荐"JPEG 质量 50 + 帧率 8fps 节流 + 分辨率降至 720p"**

**热点带宽估算（PC ↔ NX 走 cp1 热点 192.168.43.x）**：

| 参数 | 1080p JPEG q60 | 720p JPEG q50 | 480p JPEG q50 |
|---|---|---|---|
| 单帧大小 | ~120KB | ~50KB | ~20KB |
| 8fps 带宽 | ~960KB/s = 7.7Mbps | ~400KB/s = 3.2Mbps | ~160KB/s = 1.3Mbps |
| 热点典型带宽 | ~20-50Mbps（实测 cp1） | 同左 | 同左 |
| 结论 | 可行但偏紧 | **推荐（留余量给 slam/status/tasks）** | 太糊，丢失检测细节 |

**推荐参数**：
- **JPEG 质量 = 50**（panel.py:847 用的是 60，阶段B 降到 50 省带宽，前端画面略有压缩但够看检测框）
- **帧率节流 = 8fps**（VideoClient.GetImageSample 实测 5-8fps，SDK_CAPABILITIES §1.1；推送频率对齐取帧频率，不浪费）
- **分辨率 = 720p（1280×720）**：取到 1080p 帧后 `cv2.resize` 到 720p 再编码（YOLO 在 640×640 推理，输入 resize 不影响检测；推前端用 720p 平衡清晰度与带宽）
- **detections 字段轻量化**：`type=slam` 的 detections 只发 `[{x,y,class}]`（map.js:16 格式），不发 bbox 像素坐标（前端地图用世界坐标，不用像素框）。像素框信息在 `type=frame` 的画面里（画在图上）+ `type=frame` 的 `detections` 整数计数。

> ⚠️ **type=frame 的 detections 字段是整数计数**（panel.html:389 `(data.detections || 0) + ' 目标'`），不是数组。**type=slam 的 data.detections 才是数组**（map.js:52 `detMarks = data.detections`）。Generator 切勿混淆——这是阶段B 最易踩的契约坑。

### 决策 4：VideoClient 在 NX 的初始化 → **推荐"NxAiEngine 持有 ChannelFactory 单例句柄 + VideoClient，与 nx_sensor_node 隔进程"**

**冲突分析**：
- `nx_sensor_node.py`（独立进程）在它自己的进程里 `ChannelFactory().Init(0, 'enxc8a362616c4c')`。
- `nx_web_server.py`（阶段B 新增 VideoClient）在本进程也要 `ChannelFactory().Init(0, iface)`。
- SDK_CAPABILITIES §3.3 + 决策 1：**跨进程各自 Init 不冲突**，**同进程只能 Init 一次**。

**推荐方案**：
```python
# nx_ai_node.py 的 NxAiEngine.__init__ (懒初始化, 首次 detect/取帧时才做)
def _init_video(self):
    if self._video_inited:
        return
    from unitree_sdk2py.core.channel import ChannelFactory
    from unitree_sdk2py.go2.video.video_client import VideoClient
    iface = os.environ.get('DOG_INTERFACE', 'enxc8a362616c4c')
    self._factory = ChannelFactory()
    try:
        self._factory.Init(0, iface)
    except Exception:
        self._factory.Init(0, None)  # 自动检测兜底
    self._video = VideoClient()
    self._video.SetTimeout(10.0)
    self._video.Init()
    self._video_inited = True
```

**关键约束**：
1. **懒初始化**：nx_web 启动时不连狗（保持阶段A"启动不依赖狗"特性）。首次 broadcast_loop 要取帧时才 Init VideoClient。Init 失败（狗没连/SDK 没装）→ graceful 退化到 mock 帧（spec §8）。
2. **网卡名**：`enxc8a362616c4c`（与 nx_sensor_node:57 / nx_motion_node:56 一致，env `DOG_INTERFACE` 覆盖）。
3. **不与 nx_sensor_node 共享 ChannelFactory**：它们是独立进程，各自 Init。**不要尝试跨进程共享 factory 句柄**（不可行，SDK 设计如此）。
4. **VideoClient 取帧不阻塞 web 主循环**：取帧在独立视频线程（spec §5 线程模型），GetImageSample 的 200-500ms 延迟（SDK_CAPABILITIES §1.1）不阻塞 HTTP/WS/rclpy spin。

---

## 4. YOLO TensorRT 部署方案（engine 生成 / 加载 / 降级）

### 4.1 部署流程（不在本阶段代码里，作为运维 SOP 写入 docs）

**前提**：NX 上已装 JetPack（含 TensorRT）+ ultralytics（`pip install ultralytics`）。

```bash
# 在 NX 上一次性生成 engine (不入库!)
# 方式1: ultralytics CLI (推荐, 自动导出 + 优化)
yolo export model=yolov8n.pt format=engine half=True device=0
# 产出 yolov8n.engine (TensorRT FP16)

# 方式2: Python API
from ultralytics import YOLO
YOLO("yolov8n.pt").export(format="engine", half=True, device=0)
```

**engine 存放路径**：`models/yolov8n.engine`（不入 git，加 .gitignore）。
**降级链**：`yolov8n.engine`（TensorRT FP16，~25ms/帧，~40fps）→ `yolov8n.onnx`（ONNX Runtime，~80ms/帧，~12fps）→ `yolov8n.pt`（PyTorch，~150ms/帧，~6fps）。

### 4.2 加载与降级（Generator 实现要点）

`ai/detector.py` 的 `Detector.__init__` **已支持自动降级**（detector.py:20-34）：
```python
from ultralytics import YOLO
self._model = YOLO(model_path)  # model_path 传 .engine / .onnx / .pt 都能加载
```

**阶段B 改动**（在 NxAiEngine 里，不改 ai/detector.py 本身）：
```python
# nx_ai_node.py 的 NxAiEngine._init_detector (懒初始化)
def _init_detector(self):
    from ai.detector import Detector
    # 降级链: engine > onnx > pt
    candidates = [
        os.environ.get('GO2W_YOLO_ENGINE', 'models/yolov8n.engine'),
        os.environ.get('GO2W_YOLO_ONNX', 'models/yolov8n.onnx'),
        os.environ.get('GO2W_YOLO_MODEL', 'yolov8n.pt'),  # ai/config.py 默认
    ]
    for path in candidates:
        if os.path.exists(path):
            try:
                self._detector = Detector(model_path=path)
                if self._detector.available:
                    logger.info(f"YOLO 加载成功: {path}")
                    return
            except Exception as e:
                logger.warning(f"YOLO 加载失败 {path}: {e}, 尝试下一个")
    logger.error("所有 YOLO 模型加载失败, 检测降级为禁用")
    self._detector = None
```

**关键约束**：
1. **engine 不入库**：`.gitignore` 加 `models/*.engine` `models/*.onnx`（spec §6.3）。
2. **优雅降级**：Detector 内部 try/except（detector.py:32-34）已处理加载失败 → `_model=None` → `detect()` 返回 `[]`。NxAiEngine 层面再补一层 candidates 轮询。
3. **warm-up**：detector.py:28-30 已有 dummy 推理 warm-up，避免首次实时推理 ctypes 崩溃。**不要删 warm-up**。
4. **不改 ai/detector.py**：Detector 类保持平台无关。NX 特定的 TensorRT 配置通过传 `model_path=.engine` 实现，不改 Detector 内部。

---

## 5. 线程模型更新图（阶段A 4 线程 + 阶段B 新增 3 线程）

```
nx_web_server.py 进程 (阶段A + 阶段B)
═══════════════════════════════════════════════════════════════════
[阶段A, 保留不动]
  主线程 (main)
    └─ HTTPServer.serve_forever()                    # HTTP:8000
  线程1 (daemon) = rclpy.spin(NxWebNode)             # ROS2 回调
  线程2 (daemon) = run_ws asyncio loop               # WS:8001
  线程3 (daemon) = broadcast_loop                    # 推 status/slam/frame

[阶段B, 新增]
  线程4 (daemon) = NxAiEngine._video_yolo_loop       # 视频取帧 + YOLO 检测 + 画框
                                                      #   - VideoClient.GetImageSample() 或 mock 帧
                                                      #   - self._detector.detect(frame)
                                                      #   - detector.annotate(frame, dets)
                                                      #   - 缓存 _latest_frame (带框 jpeg) + _latest_dets
                                                      #   - 频率: 对齐取帧 ~8fps, YOLO 跟随
  线程5 (daemon) = NxAiEngine._vlm_worker             # VLM 单工作线程
                                                      #   - 队列消费 parse 请求
                                                      #   - 懒 load/unload + 空闲 60s 卸载
                                                      #   - 推理: chat() / locate()
  线程6 (daemon) = NxAiEngine._mem_monitor (可选)     # 每 30s 日志显存用量
═══════════════════════════════════════════════════════════════════
线程间通信:
  - 线程4 → 线程3: _latest_frame / _latest_dets (threading.Lock 保护, 广播线程读最新值)
  - HTTP /api/command → 线程5: parse 队列 (queue.Queue)
  - 线程5 → 线程3: ws_broadcast({"type":"vlm",...}) (直接调全局 ws_broadcast, 跨线程安全)
```

**关键约束**：
1. **视频/YOLO 独立线程（线程4）**：绝不能在 broadcast_loop 里同步取帧+检测（会阻塞 status/slam 推送，导致地图卡顿）。线程4 持续刷新缓存，broadcast_loop 只读缓存快照。
2. **VLM 独立线程（线程5）**：VLM 推理慢（1-5s/次），绝不能在 HTTP handler 线程同步推理（HTTP 会超时）。`/api/command` 立即返回，parse 入队异步处理（panel.py:459 已是这个模式：`threading.Thread(target=self._process_command_bg)`）。
3. **跨线程数据共享用 Lock**：_latest_frame / _latest_dets 用 `threading.Lock`（线程4 写，线程3 读），参考 nx_sensor_node.py:82 的 _lock 模式。
4. **不引入 MultiThreadedExecutor**：阶段A 决策 2 已定，阶段B 不改。

---

## 6. WS 消息协议增量（阶段B 相对阶段A 的新增/改动）

### 6.1 `type=frame`（阶段A 不发，阶段B 新增）

**格式对齐 panel.py:847-849**（前端 panel.html:384-390 解析）：
```jsonc
{
  "type": "frame",
  "data": "<base64 jpeg string>",     // 720p JPEG, 质量50, base64 编码
  "detections": 3                       // 整数: 本帧检测到的目标数 (不是数组!)
}
```
**关键**：`detections` 是**整数计数**（panel.html:389 `(data.detections || 0) + ' 目标'`）。像素级 bbox 已画在 jpeg 里（`detector.annotate`），无需再传。

### 6.2 `type=slam` 的 `data.detections`（阶段A 恒 `[]`，阶段B 填真值）

**格式对齐 map.js:16 / panel.py:882**：
```jsonc
{
  "type": "slam",
  "data": {
    "x": 0.5, "y": 0.2, "yaw": 1.23,
    "trail": [...],
    "map": [],
    "scan": [...],
    "detections": [                      // 阶段B: 不再恒空, 填检测目标的世界坐标
      {"x": 2.5, "y": 1.0, "class": "person"}
    ],
    "waypoints": [], "currentWP": -1,
    "slam_source": "ros2_nx"
  }
}
```

**detections 坐标转换**（关键，Generator 必读）：
- YOLO 输出像素 bbox `[x1,y1,x2,y2]`（图像坐标系）。
- 前端地图要**世界坐标** `{x,y,class}`（map.js:16 注释明确）。
- 转换需要：目标相对狗的方位角 = 像素中心 x / 图像宽度 × 相机水平 FOV；距离 = 用 bbox 面积估算（粗略）或固定假设（如 3m）。
- **阶段B 简化策略**：bbox 中心 x 归一化（0-1）→ 方位角（假设 FOV=70°，`angle = (cx-0.5)*70°`）；距离假设固定 3m（YOLO 无深度，无法精确定位）；世界坐标 = 狗位姿 + 3m×(cos(yaw+angle), sin(yaw+angle))。
- 这是**近似**（后续阶段可接 LiDAR 融合精确定位），spec §11 标注此简化。

### 6.3 `type=vlm`（阶段A 发 fallback，阶段B 发 VLM 真解析）

**格式对齐 panel.py:466**（前端 panel.html:403-404 解析）：
```jsonc
// 正常解析
{"type":"vlm", "data":{"text":"前进两米","response":"前进","tasks":[{"type":"move",...}]}}

// VLM 加载中 (阶段B 新增中间状态, 前端 console.log 不报错)
{"type":"vlm", "data":{"text":"前进两米","response":"(VLM 加载中...)","tasks":[],"loading":true}}

// VLM 加载失败/超时, fallback 到关键词
{"type":"vlm", "data":{"text":"前进两米","response":"前进(fallback)","tasks":[{"type":"move",...}],"fallback":true}}
```
**关键**：`loading` / `fallback` 是阶段B 新增的可选字段，前端 panel.html:404 只读 `data.data.response`（console.log），**忽略未知字段**，所以新增字段不破坏前端。

---

## 7. 新建/修改文件清单（文件级 + 函数级，Generator 直接实现）

### 7.1 新建文件

#### `web/nx_ai_node.py`（核心，约 300 行）
**职责**：NX 的 AI 适配层。封装 VideoClient 取帧、YOLO 检测、VLM 懒加载/卸载，作为"组件"注入 nx_web_server.py。**不直接处理 HTTP/WS**（那是 nx_web_server 的职责），只暴露数据/方法给后者调用。

**关键签名**：
```python
import logging, os, threading, time, queue
import numpy as np

logger = logging.getLogger("go2w.nx_ai")


class NxAiEngine:
    """NX AI 推理引擎: 视频+YOLO+VLM 的统一管理器。

    生命周期: nx_web_server.main() 创建一个实例, 注入 TaskManager (作 detector+vlm)
    和 broadcast_loop (读 _latest_frame/_latest_dets)。
    """

    def __init__(self):
        self._lock = threading.Lock()
        # 视频源
        self._video = None              # unitree VideoClient (懒初始化)
        self._factory = None            # ChannelFactory 单例 (本进程)
        self._video_inited = False
        self._mock_mode = False         # True=用 mock 帧 (狗没连/SDK 没装)
        self._mock_frame_gen = None     # MockFrameGenerator 实例
        # YOLO
        self._detector = None           # ai.detector.Detector (懒初始化)
        self._detector_inited = False
        # 缓存 (视频/YOLO 线程写, broadcast_loop 读)
        self._latest_frame = None       # numpy BGR (带检测框, 720p)
        self._latest_dets = []          # [{class, confidence, bbox}]
        self._frame_count = 0
        # VLM
        self._vlm = None                # ai.vlm.VLMEngine (懒初始化)
        self._vlm_inited = False
        self._vlm_queue = queue.Queue() # parse 请求队列
        self._vlm_last_use = 0.0
        self._vlm_idle_timeout = float(os.environ.get('GO2W_VLM_IDLE', '60'))
        # 控制
        self._running = False
        self._threads = []

    # ---- 启动/停止 ----
    def start(self):
        """启动视频/YOLO 线程 + VLM 工作线程 + 显存监控。"""
        # 关键: 启动时不 import torch/ultralytics (懒加载)
        self._running = True
        t1 = threading.Thread(target=self._video_yolo_loop, daemon=True)
        t2 = threading.Thread(target=self._vlm_worker, daemon=True)
        t3 = threading.Thread(target=self._mem_monitor, daemon=True)
        t1.start(); t2.start(); t3.start()
        self._threads = [t1, t2, t3]

    def stop(self):
        self._running = False
        if self._vlm and self._vlm.loaded:
            self._vlm.unload()

    # ---- 视频 + YOLO 线程 (线程4) ----
    def _init_video(self):
        """懒初始化 VideoClient (首次取帧时)。失败则切 mock 模式。"""
        # 决策 4: ChannelFactory 单例 + 网卡 enxc8a362616c4c
        # 失败 (SDK 没装/狗没连) → self._mock_mode = True

    def _init_detector(self):
        """懒初始化 YOLO (降级链 engine>onnx>pt)。"""
        # 决策 4.2: candidates 轮询, detector.py 已支持各格式

    def _get_frame(self) -> "np.ndarray | None":
        """取一帧: VideoClient.GetImageSample() 或 mock。返回 BGR ndarray。"""
        # SDK_CAPABILITIES §1.1: code, data = video.GetImageSample()
        # frame = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
        # mock 模式: self._mock_frame_gen.next_frame()

    def _video_yolo_loop(self):
        """视频+YOLO 主循环 (线程4)。
        - 取帧 (~8fps, GetImageSample 200-500ms)
        - YOLO detect + annotate
        - 缓存 _latest_frame (720p resize 后) + _latest_dets
        - 异常: 取帧失败连续 10 次 → 切 mock
        """

    # ---- VLM 工作线程 (线程5) ----
    def _init_vlm(self):
        """懒初始化 VLMEngine (首次 parse 时)。"""
        # from ai.vlm import VLMEngine
        # self._vlm = VLMEngine(); self._vlm.load()

    def submit_parse(self, text: str) -> None:
        """HTTP handler 调用: 把指令解析请求入队 (非阻塞)。"""
        self._vlm_queue.put(text)

    def _vlm_worker(self):
        """VLM 单工作线程 (线程5)。
        - 消费 _vlm_queue
        - 首次请求: load() (10-30s), 期间发 type=vlm loading=true
        - 推理: self._vlm.chat([sys_prompt, user_text])
        - 解析 JSON → tasks, 发 type=vlm 正常结果
        - 空闲 60s: unload() + empty_cache()
        """

    # ---- broadcast_loop 读取接口 ----
    def get_frame_jpeg(self) -> "tuple[str, int] | None":
        """broadcast_loop 调用: 返回 (base64 jpeg, detections_count) 或 None。
        - 读 _latest_frame, cv2.imencode('.jpg', frame, [JPEG_QUALITY, 50])
        - base64.b64encode(jpeg.tobytes()).decode()
        - 读 _latest_dets 长度作为 detections_count
        """

    def get_detections_world(self, robot_x: float, robot_y: float,
                              robot_yaw: float) -> list:
        """broadcast_loop 调用: 返回 slam.data.detections 格式 [{x,y,class}]。
        - 读 _latest_dets, bbox 中心 x 归一化 → 方位角 (假设 FOV=70°)
        - 距离假设 3m, 世界坐标 = robot + 3m×(cos(yaw+ang), sin(yaw+ang))
        """

    # ---- 显存监控 (线程6) ----
    def _mem_monitor(self):
        """每 30s 日志显存: torch.cuda.memory_allocated/reserved。"""
        # 决策 2: 周期日志确认 VLM unload 后显存释放


class MockFrameGenerator:
    """不依赖狗硬件的 mock 视频源 (验证用)。
    生成带'假目标'的 720p 帧 (画几个矩形当人/物), 让 YOLO 真检测能跑通。
    """
    def __init__(self, width=1280, height=720):
        self._w, self._h = width, height
        self._t0 = time.time()

    def next_frame(self) -> np.ndarray:
        """生成一帧: 灰底 + 几个移动的彩色矩形 (模拟人/物) + 时间戳。
        YOLO 在 mock 帧上可能检不出 (无真目标), 但能验证:
          1. 取帧→检测→画框→编码→WS 推送 全链路通
          2. VLM 按需 load/unload
        若需 YOLO 真检出, 可在 mock 帧里贴一张 COCO 类目标图 (如 person 裁图)。
        """
```

**实现要点**：
- **懒加载是核心**：`__init__` 和 `start()` 都不 import torch/ultralytics/transformers。首次 `_init_video` / `_init_detector` / `_init_vlm` 时才 import（保证 nx_web 启动仍是秒级）。
- **mock 模式自动切换**：`_init_video` 失败 → `_mock_mode=True`，`_get_frame` 走 MockFrameGenerator。日志明确打印"[AI] 视频源: mock (狗未连接)"。
- **JPEG 质量参数**：`cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])`（决策 3）。
- **720p resize**：`cv2.resize(frame, (1280, 720))` 后再编码（决策 3）。
- **VLM sys_prompt**：照抄 panel.py:472-495 的 `_vlm_parse_command` sys_prompt（指令解析 JSON 任务格式），保持解析行为一致。
- **VLM 卸载时机**：`_vlm_worker` 每次循环检查 `now - self._vlm_last_use > self._vlm_idle_timeout`，unload 后 `self._vlm_last_use = 0`（标记已卸载）。

#### `web/verify_nx_ai.sh`（验证脚本，约 60 行）
**职责**：NX 上启动 nx_web + mock 视频，验证 AI 链路。**不依赖狗硬件**。

**验证项**（每项独立 PASS/FAIL）：
1. `curl http://localhost:8000/` → 200（阶段A 契约不破坏）
2. python websocket 客户端连 ws://localhost:8001，10 秒内收到 `type=frame` 消息，`data` 是非空 base64 字符串，base64 解码后是合法 JPEG（`cv2.imdecode` 不返回 None）
3. 10 秒内 `type=slam` 的 `data.detections` 字段存在（即使为空数组 `[]`，字段必须在，证明接线通）
4. `curl -X POST 'http://localhost:8000/api/command?text=前进两米'` → `{"ok":true}`，且 30 秒内收到 `type=vlm` 消息（`response` 字段非空）
5. VLM 空闲 70s 后（`GO2W_VLM_IDLE=60` + 10s 余量），`nvidia-smi` 显示 VLM 显存释放（与加载时对比，reserved 下降 ~6GB）。**若 NX 无 GPU 或 VLM 未装，此项 SKIP 不算 FAIL**。
6. `curl -X POST http://localhost:8000/api/e_stop` → `{"ok":true}`，且后续仍收到新 `type=frame`（e_stop 不应中断视频流，验证线程存活）

**通过标准**：1-4 必须 PASS，5 条件 PASS（有 GPU 才验），6 PASS。

#### `docker/deploy_nx_ai.sh`（部署脚本，约 40 行）
**职责**：把 nx_ai_node.py + verify_nx_ai.sh 部署到 NX，scp 模型文件（可选，大文件走 rsync），打印"在 NX 跑 verify_nx_ai.sh 验证"。

### 7.2 修改文件（仅 nx_web_server.py，改动最小化）

#### `web/nx_web_server.py`（阶段A 文件，阶段B 注入 AI）
**改动点**（保持阶段A 4 线程 + 契约不变，只加 AI 注入）：

1. **import NxAiEngine**（顶部，懒加载不拖慢启动）：
   ```python
   # 阶段B: AI 适配层 (懒加载, start() 时不 import torch)
   try:
       from nx_ai_node import NxAiEngine
       AI_OK = True
   except Exception as _e:
       AI_OK = False
       _AI_ERR = str(_e)
   ```

2. **TaskManager 改造**（detector/vlm 从 None → NxAiEngine 提供的代理）：
   ```python
   # TaskManager 需要的接口:
   #   - detector.detect(frame) → [{class, confidence, bbox}]  (panel.py:584)
   #   - vlm.loaded (bool) + vlm.chat(messages) → str          (panel.py:463, 496)
   #   - robot.get_frame() → numpy frame                        (panel.py:581, 628)
   # 阶段B: 把 NxAiEngine 包装成 detector/vlm 代理传入 TaskManager
   ```

   **关键**：TaskManager 的 `_execute_search`（panel.py:607-646）调 `self.robot.get_frame()` 取帧检测。阶段A 的 NxRobotBridge **没有 get_frame 方法**。阶段B 需给 NxRobotBridge 加一个 `get_frame()` 方法，委托给 NxAiEngine（NxAiEngine 持有 VideoClient/mock）。

3. **NxRobotBridge 加 get_frame**（阶段B 新增方法）：
   ```python
   def get_frame(self):
       """阶段B: 委托给 NxAiEngine 取最新帧 (供 TaskManager._execute_search 检测)。"""
       if self._ai_engine:
           with self._ai_engine._lock:
               return self._ai_engine._latest_frame.copy() if self._ai_engine._latest_frame is not None else None
       return None
   ```

4. **broadcast_loop 改造**（阶段A 不发 frame → 阶段B 发 frame + 填 detections）：
   ```python
   # 在 slam_counter 推送前, 加 frame 推送 (每帧都推, ~8fps)
   if ai_engine:
       result = ai_engine.get_frame_jpeg()
       if result:
           b64, det_count = result
           ws_broadcast({"type": "frame", "data": b64, "detections": det_count})

   # slam 的 detections 字段从 [] → ai_engine.get_detections_world(x, y, yaw)
   ws_broadcast({"type": "slam", "data": {
       ...
       "detections": ai_engine.get_detections_world(x, y, yaw) if ai_engine else [],
       ...
   }})
   ```

5. **main() 改造**（创建 NxAiEngine + 启动 + 注入）：
   ```python
   # 阶段A: task_mgr = TaskManager(robot, vlm_engine=None, detector=None)
   # 阶段B:
   ai_engine = NxAiEngine() if AI_OK else None
   if ai_engine:
       ai_engine.start()
       robot._ai_engine = ai_engine  # 让 NxRobotBridge.get_frame 能委托
       # VLM 代理: 让 TaskManager 的 vlm.loaded 触发 _vlm_parse_command, 实际推理走 ai_engine
       vlm_proxy = NxAiVlmProxy(ai_engine)  # 永远 loaded=True, chat 转发到 ai_engine.submit_parse + 同步等结果
       detector_proxy = NxAiDetectorProxy(ai_engine)  # detect 转发到 ai_engine._detector
       task_mgr = TaskManager(robot, vlm_engine=vlm_proxy, detector=detector_proxy)
   else:
       task_mgr = TaskManager(robot, vlm_engine=None, detector=None)  # AI 不可用, 退化阶段A 行为
   ```

   > ⚠️ **VLM 代理的同步问题**：panel.py 的 `_process_command_bg`（panel.py:461）在**独立线程**里同步调 `self.vlm.chat()`。阶段B 的 NxAiVlmProxy.chat() 需要同步返回结果（阻塞调用线程直到 VLM 推理完）。实现方式：`submit_parse` 入队后，用 `threading.Event` 等结果（_vlm_worker 推理完 set event，proxy 返回结果）。**绝不能在 HTTP handler 线程同步调 chat**（会阻塞 HTTP）——必须在 `_process_command_bg` 线程（panel.py 已是这个模式）。

### 7.3 修改文件（配置）

#### `.gitignore`（加模型文件排除）
```
# 阶段B: TensorRT engine / ONNX 不入库 (大文件 + 设备绑定)
models/*.engine
models/*.onnx
# Qwen2.5-VL 模型 (GB 级, 不入库)
models/Qwen/
```

#### `ai/config.py`（加 NX 路径默认值，不改 PC 路径）
```python
# 阶段B: NX 上的模型路径 (env 覆盖, 部署时传)
YOLO_ENGINE_PATH = os.environ.get('GO2W_YOLO_ENGINE', 'models/yolov8n.engine')
YOLO_ONNX_PATH = os.environ.get('GO2W_YOLO_ONNX', 'models/yolov8n.onnx')
# VLM 模型路径在 NX 上的位置 (Qwen2.5-VL-3B)
VLM_MODEL_NAME_NX = os.environ.get('GO2W_VLM_MODEL_NX', '/home/nx/models/Qwen/Qwen2___5-VL-3B-Instruct')
# VLM 空闲卸载超时 (秒)
VLM_IDLE_TIMEOUT = float(os.environ.get('GO2W_VLM_IDLE', '60'))
# 视频流参数
VIDEO_JPEG_QUALITY = int(os.environ.get('GO2W_VIDEO_JPEG_QUALITY', '50'))
VIDEO_TARGET_WIDTH = int(os.environ.get('GO2W_VIDEO_WIDTH', '1280'))   # 720p
VIDEO_TARGET_HEIGHT = int(os.environ.get('GO2W_VIDEO_HEIGHT', '720'))
```

### 7.4 不动文件清单（Generator 勿碰，Critic 会核对）

- `web/static/panel.html`（前端无感，禁止改 JS）
- `web/static/map.js`（同上）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py`（控狗逻辑红线）
- `src/go2w_bridge/go2w_bridge/nx_sensor_node.py`（NX 传感器，本阶段不动）
- `ai/detector.py`（保持平台无关，NX 适配在 nx_ai_node.py）
- `ai/vlm.py`（同上）
- `ai/tracker.py`（阶段B 不集成 tracker，保留供后续）
- `web/panel.py`（PC fallback，退役不删）
- **TensorRT engine 文件**（不入库）

---

## 8. 不依赖狗硬件的验证方法（Generator 必须实现并跑通）

### 8.1 mock 视频源策略

**MockFrameGenerator**（spec §7.1）生成 720p 帧，内含：
- 灰色背景 + 时间戳文字（证明帧在更新）
- 2-3 个移动的彩色矩形（模拟运动目标，YOLO 可能检不出但全链路通）
- **可选**：贴一张真实的 COCO 类目标裁图（如从 COCO 数据集裁一张 person 图贴到帧里），让 YOLO 真检出 person —— 这能完整验证"YOLO 真检测 → detections 进 slam → 地图标记"全链路。Generator 实现时优先做这个（准备 `web/static/mock_person.png` 一张人像裁图）。

### 8.2 一键验证 `web/verify_nx_ai.sh`

**前置**：阶段A 的 `verify_nx_web.sh` 已 PASS（阶段A 契约不破坏）。

**流程**：
```bash
# 1. 启动 nx_web (阶段B 模式, 自动初始化 NxAiEngine)
GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py &
sleep 5

# 2. 等 VideoClient 初始化失败 → 自动切 mock (日志打印 "[AI] 视频源: mock")
# 3. 跑验证
bash web/verify_nx_ai.sh

# 4. 验证完 kill
kill %1
```

**验证项**（见 spec §7.1 verify_nx_ai.sh）：
- WS 收到 `type=frame`，base64 解码是合法 JPEG
- `type=slam` 的 `data.detections` 字段存在
- `/api/command` 触发 VLM 加载 + 返回 `type=vlm`（mock 帧贴了 person 裁图时，VLM 能"看到"内容）
- 空闲后 VLM unload（`nvidia-smi` 证据，有 GPU 才验）

**通过标准**：1-4 PASS，5 条件 PASS（无 GPU 跳过）。

### 8.3 实车就绪验证（硬件装完后，spec 不强制本阶段跑）

1. NX 直连狗（USB 网卡 `enxc8a362616c4c`），`python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient; v=VideoClient(); v.Init(); print(v.GetImageSample()[0])"` → 0（取帧成功）
2. 启动 nx_web，浏览器看到真狗摄像头画面 + YOLO 检测框
3. 输入"跟着前面的人"，VLM 加载后真解析，follow 任务入队

---

## 9. 分 Sprint 实现顺序（每步可独立验证，前 3 步不依赖狗）

### Sprint 1：NxAiEngine 骨架 + mock 视频 + WS frame 推送（不接 YOLO/VLM）
**目标**：浏览器"第一视角"看到 mock 视频帧（灰底+时间戳），证明视频线程 + frame 推送链路通。
**Features**：`nx_ai_node.py` 的 NxAiEngine 骨架（__init__/start/stop/_video_yolo_loop/_get_frame）；MockFrameGenerator；nx_web_server.broadcast_loop 加 frame 推送；NxRobotBridge 不动。
**Definition of Done**：
- `GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py` 启动，日志打印"[AI] 视频源: mock"
- 浏览器"第一视角"区域显示灰底+时间戳画面（~8fps 更新）
- WS 抓包：收到 `type=frame`，`data` base64 解码为合法 JPEG，`detections=0`
- **不依赖狗硬件**：mock 模式
- **不依赖 GPU**：无 YOLO，纯 mock 帧

### Sprint 2：YOLO 接入（mock 帧 → 真检测 → detections）
**目标**：mock 帧贴 person 裁图，YOLO 检出 person，detections 进 slam 地图标记。
**Features**：NxAiEngine._init_detector（降级链 engine>onnx>pt）；_video_yolo_loop 调 detector.detect + annotate；get_detections_world（bbox→世界坐标）；broadcast_loop 填 slam.data.detections。
**Definition of Done**：
- mock 帧贴 person 裁图，YOLO 检出 person（日志打印"detect: person 0.85"）
- 浏览器"第一视角"画面有绿色检测框
- 浏览器地图出现 detections 标记点（红点 + class 标签）
- WS 抓包：`type=frame` 的 `detections=1`（整数）；`type=slam` 的 `data.detections=[{x,y,class}]`
- **不依赖狗硬件**：mock 帧
- **依赖 GPU**：YOLO 在 NX GPU 跑（无 GPU 则降级 PT，慢但能验）

### Sprint 3：VLM 按需加载 + 指令解析（mock 帧不依赖狗）
**目标**：`/api/command` 触发 VLM 加载，解析"前进两米"，空闲 unload。
**Features**：NxAiEngine._init_vlm/_vlm_worker；submit_parse；NxAiVlmProxy；TaskManager 用真 vlm_proxy（_vlm_parse_command 走真 VLM）；type=vlm 消息（含 loading/fallback 状态）。
**Definition of Done**：
- 输入"前进两米"，日志打印"[VLM] 加载中... (10-30s)"，前端 console 看到 `type=vlm` 的 `loading:true`
- VLM 加载完，解析出 `{"tasks":[{"type":"move","params":{"vx":0.5,"duration":4.0}}]}`，前端任务队列出现 move 任务
- `type=vlm` 的 `response` 是 VLM 生成的中文（"前进"/"向前移动"等），不是 fallback 的关键词匹配
- 空闲 70s 后，`nvidia-smi` 显示显存释放（VLM unload）
- 再输入"左转"，VLM 重新 load（日志"[VLM] 重新加载"），解析成功
- **不依赖狗硬件**：VLM 不需要狗帧，纯文本解析（locate 才需要帧，阶段B 的 process_command 只用 chat）
- **依赖 GPU**：VLM 在 NX GPU 跑

### Sprint 4：VideoClient 真取帧（狗硬件就绪后，本阶段可选）
**目标**：NX 直连狗，VideoClient.GetImageSample 取真帧，YOLO 检测真场景。
**Features**：NxAiEngine._init_video（ChannelFactory + VideoClient）；mock 模式自动切换；_get_frame 走 VideoClient。
**Definition of Done**：
- NX 连狗，启动 nx_web，日志"[AI] 视频源: unitree VideoClient"
- 浏览器看到真狗摄像头画面（1080p→720p）+ YOLO 检测框
- 取帧失败（狗断电）连续 10 次 → 自动切 mock（graceful 退化）
- **依赖狗硬件**：是

### Sprint 5（可选）：部署 + systemd 集成
**目标**：nx_web + AI 开机自启，deploy_nx_ai.sh 一键部署。
**Features**：deploy_nx_ai.sh；go2w-web.service 的环境变量加 `GO2W_AI_MOCK_VIDEO` / `DOG_INTERFACE` / `GO2W_VLM_IDLE`；verify_nx_ai.sh 在 systemd 启动的服务上跑通。
**Definition of Done**：`systemctl start go2w-web` 后浏览器有视频流 + AI；reboot NX 后自动起。

---

## 10. 边界情况与状态处理（Critic 必查）

| 场景 | 期望行为 | 实现位置 |
|---|---|---|
| VideoClient 初始化失败（狗没连/SDK 没装） | 切 mock 模式，日志告警，浏览器看 mock 帧（不黑屏） | NxAiEngine._init_video |
| YOLO 所有模型加载失败（无 engine/onnx/pt） | `_detector=None`，`_video_yolo_loop` 跳过检测，帧原样推送（无框），slam.detections=[] | NxAiEngine._init_detector |
| VLM 加载失败（模型路径错/显存不足） | 日志 error，发 `type=vlm` 的 `fallback:true`，走 _fallback_parse 关键词匹配 | NxAiEngine._vlm_worker |
| VLM 推理超时（>30s） | 超时 fallback，不阻塞 worker 线程 | NxAiEngine._vlm_worker |
| VLM unload 期间来了新请求 | 新请求触发重新 load，前端看到 loading 状态 | NxAiEngine._vlm_worker |
| VideoClient 取帧返回 code≠0 | 跳过该帧，不更新缓存，连续 10 次切 mock | NxAiEngine._video_yolo_loop |
| YOLO 检测耗时 > 取帧间隔（~125ms） | 视频线程自然降速（不丢帧，处理完才取下一帧），日志告警 | NxAiEngine._video_yolo_loop |
| 显存不足（VLM load 时 OOM） | VLM load 失败，fallback，日志 error + memory_summary | NxAiEngine._init_vlm |
| WS 无客户端 | frame 推送照常（ws_broadcast 内部判 WS_CLIENTS 空） | 照抄 panel.py:96 |
| mock 帧 person 裁图丢失 | MockFrameGenerator 用纯色矩形兜底，YOLO 检不出但不崩 | MockFrameGenerator |
| 浏览器跨热点带宽不足 | JPEG 质量降（env GO2W_VIDEO_JPEG_QUALITY 可调），帧率自然降（取帧慢） | NxAiEngine |

---

## 11. Anti-AI-slop / 反模式清单（Generator 自查）

- ❌ 不要在 NxAiEngine.__init__ 里 import torch/ultralytics/transformers（懒加载，启动秒级）
- ❌ 不要在 broadcast_loop 里同步取帧/检测（阻塞 status/slam 推送）
- ❌ 不要在 HTTP handler 线程同步调 VLM.chat（HTTP 超时）
- ❌ 不要把 type=frame 的 detections 写成数组（panel.html:389 要整数计数）
- ❌ 不要把 type=slam 的 data.detections 写成整数（map.js:52 要 `[{x,y,class}]` 数组）
- ❌ 不要常驻 VLM（必须空闲 unload，腾显存给 FAST_LIO/Nav2）
- ❌ 不要把 .engine/.onnx 模型入库（gitignore）
- ❌ 不要改 ai/detector.py / ai/vlm.py（平台无关，NX 适配在 nx_ai_node.py）
- ❌ 不要用 MultiThreadedExecutor（阶段A 决策 2，继续生效）
- ❌ 不要在 NX web 进程初始化 ChannelFactory 多次（单例，只 Init 一次）
- ❌ 不要删阶段A 的 mock_dog_state_publisher（阶段B 仍用它模拟狗状态）
- ❌ 不要给 detections 做精确深度定位（YOLO 无深度，固定假设 3m 即可，后续接 LiDAR）
- ❌ 不要把 VLM 加载状态做成"加载中浏览器转圈"动画（不改 panel.html，loading 走 type=vlm 的 console.log）
- ❌ 不要在 mock 帧用复杂物理模拟（简单灰底+矩形+person 裁图即可）

---

## 12. Evaluation Criteria（见 gan-harness/eval-rubric-stage-b.md，权重已定）

详见独立的 `gan-harness/eval-rubric-stage-b.md`，Critic 直接消费。核心五维：
- 阶段A 契约不破坏（0.25）：HTTP API + WS 消息字段与阶段A/panel.py 逐字对齐；前端零改动
- AI 链路正确性（0.25）：视频/YOLO/VLM 线程模型、TensorRT 降级、ChannelFactory 单例、坐标转换
- 显存管理（0.20）：VLM 按需 load/unload、空闲超时、显存预算不超红线
- 可验证性（0.20）：verify_nx_ai.sh PASS、不依赖狗硬件、mock 视频真检测
- 工程质量（0.10）：懒加载、错误降级、日志可观测
