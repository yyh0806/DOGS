#!/usr/bin/env python3
"""Go2W 阶段B — NX AI 适配层 (spec-stage-b §7.1)。

== 职责 ==
作为"组件"注入 nx_web_server.py (同进程, 不是独立 rclpy 节点, spec 决策 1 方案 b):
  1. 视频: unitree VideoClient.GetImageSample() 取狗 1080p 帧 (狗硬件到位后),
           或 mock 帧 (不依赖狗, spec §8)
  2. YOLO: TensorRT engine 优先, 降级 onnx > pt (spec §4.2); 实时检测画框, ~25ms/帧
  3. VLM : Qwen2.5-VL-3B 按需 load/unload (懒加载, 空闲 60s unload, spec 决策 2)
  4. 推理结果缓存 + 暴露 get_frame_jpeg / get_detections_world / submit_parse 给
     nx_web_server 的 broadcast_loop 和 TaskManager

== 线程模型 (spec §5, 阶段A 4 线程 + 阶段B 新增 3 daemon 线程) ==
  线程4 (daemon) = _video_yolo_loop   视频+YOLO (取帧→检测→画框→缓存)
  线程5 (daemon) = _vlm_worker        VLM 单工作线程 (队列消费 + 按需 load/unload)
  线程6 (daemon) = _mem_monitor       每 30s 日志显存

== 红线 (spec §0 + §11) ==
  - 懒加载: __init__/start() 不 import torch/ultralytics/transformers (启动秒级)
  - 不改 ai/detector.py / ai/vlm.py / ai/tracker.py (本文件 import 它们)
  - VideoClient 取帧不阻塞 broadcast_loop (线程4 内部消化 GetImageSample 的 200-500ms)
  - VLM 不在 HTTP handler 线程同步推理 (submit_parse 入队异步处理)
  - ChannelFactory 本进程只 Init 一次 (spec 决策 4, 与 nx_sensor_node 进程隔离)
  - 不入库 engine/onnx (gitignore)

== round-3 解耦: 视频流与 YOLO 解耦 (NX 可能无 ultralytics/torch) ==
  - 无 ultralytics (或 GO2W_AI_NO_DETECT=1) → detector=None, 视频流照常推 type=frame
    (detections=0, slam.data.detections=[]), 不崩、不退化为"AI 完全关闭"。
  - 检测调用统一走 _run_detector(frame) 抽象方法, 为后续 locateanything 开放词汇
    定位预留可替换接口 (同签名替换本方法, 视频流路径不动)。
  - VLM 同样 graceful: 无 transformers → vlm=None → TaskManager 走 _fallback_parse。
"""

import base64
import json
import logging
import math
import os
import queue
import threading
import time

import numpy as np

logger = logging.getLogger("go2w.nx_ai")

# web/ 目录 (mock_person.png 等资源与本文件同目录的 static 子目录)
_AI_DIR = os.path.dirname(os.path.abspath(__file__))

# ============================================================================
# ws_broadcast 注入点 (避免循环 import: nx_web_server → nx_ai_node → nx_web_server)
# ----------------------------------------------------------------------------
# nx_ai_node 不能顶层 import nx_web_server (后者顶层 import rclpy, 无 rclpy 环境
# 会留下部分导入的 sys.modules 条目, 反复重导入可能挂起 worker 线程)。
# 解决: nx_web_server.py 在 main() 启动 NxAiEngine 之前调本模块的 set_ws_broadcast()
# 注入 ws_broadcast 函数引用, _safe_broadcast 直接用, 零 import。
# ============================================================================
_WS_BROADCAST_FN = None


def set_ws_broadcast(fn):
    """nx_web_server.main() 调用: 把全局 ws_broadcast 注入本模块 (一次)。
    nx_web_server import 本文件时 ws_broadcast 尚未定义 (在本文件顶部),
    所以 nx_web_server 必须在定义 ws_broadcast 之后 (模块顶层第 64 行之后) 调本函数。
    """
    global _WS_BROADCAST_FN
    if _WS_BROADCAST_FN is None and fn is not None:
        _WS_BROADCAST_FN = fn
        logger.debug("[AI] ws_broadcast 已注入")

# 相机水平 FOV (度), 用于 bbox 中心 x 归一化 → 方位角 (spec §6.2 简化策略)
_CAMERA_HFOV_DEG = float(os.environ.get("GO2W_CAMERA_HFOV", "70"))
# YOLO 无深度, 检测目标固定假设距离 (米) (spec §11: 不做精确深度定位)
_DETECT_ASSUME_DIST_M = float(os.environ.get("GO2W_DETECT_DIST", "3.0"))

# ============================================================================
# 检测器解耦 (round-3): 视频流不再依赖 ultralytics/torch
# ----------------------------------------------------------------------------
# NX 上可能没装 ultralytics/torch (用户不走 YOLO, 后续换 locateanything 开放
# 词汇定位)。要求: 即使无 ultralytics, nx_ai_node 仍取帧 + 推 type=frame 视频流
# (detections 空), 让前端显示第一视角。
#
# 两个开关 (均只影响检测, 不影响视频流):
#   1. GO2W_AI_NO_DETECT=1 → 显式禁检测 (即使装了 ultralytics 也不 YOLO, 纯视频流)
#   2. ultralytics/torch 缺失 → 自动禁检测 (detector=None, 记 warning, 不崩)
# 任一触发 → self._detector=None → _run_detector 返回 [] → 视频流照常。
# ============================================================================
# 显式禁检测开关 (round-3 需求 3)
_DETECT_DISABLED_BY_ENV = str(os.environ.get("GO2W_AI_NO_DETECT", "")).strip() in ("1", "true", "True", "yes")

# ultralytics 是否可导入 (NX 可能没装; 探测一次, 记 warning, 不崩)
# 注意: 这里只探测 ultralytics 本身, 不在顶层 import ai.detector (保持 ai.detector
# 懒加载, 避免本模块 import 时触发 ai.config 等链路)。ai.detector 内部在
# Detector.__init__ 才 import ultralytics, 所以探测 ultralytics 等价于探测检测可行性。
def _probe_ultralytics():
    """探测 ultralytics 是否可导入 (NX 可能没装)。返回 bool。
    只调用一次 (模块加载时), 结果缓存到 _ULTRALYTICS_AVAILABLE。
    缺失不是错误: 后续走纯视频流路径 (detector=None, detections 空)。
    """
    try:
        import importlib
        importlib.import_module("ultralytics")  # 不保留引用, 仅探测
        return True
    except Exception as e:
        # 不崩: 记 warning (不是 error), 视频流仍工作, 仅 detections 空
        logger.warning(f"[AI] ultralytics 不可导入 ({type(e).__name__}), YOLO 检测将禁用 "
                       f"(视频流不受影响, detections 为空; GO2W_AI_NO_DETECT 或换 locateanything)")
        return False

_ULTRALYTICS_AVAILABLE = _probe_ultralytics()

# 最终是否允许 YOLO 检测: 环境没禁 + ultralytics 在
_DETECT_ALLOWED = (not _DETECT_DISABLED_BY_ENV) and _ULTRALYTICS_AVAILABLE
if _DETECT_DISABLED_BY_ENV:
    logger.info("[AI] GO2W_AI_NO_DETECT=1 → 显式禁检测 (纯视频流模式, detections 恒空)")
elif not _ULTRALYTICS_AVAILABLE:
    logger.warning("[AI] 检测禁用 (无 ultralytics) → 纯视频流模式 (type=frame 照推, detections=0)")


# ============================================================================
# MockFrameGenerator — 不依赖狗硬件的 mock 视频源 (spec §8.1)
# ----------------------------------------------------------------------------
# 生成 720p 灰底帧 + 时间戳 + 几个移动彩色矩形 (链路通);
# 优先把 web/static/mock_person.png 贴到帧里 (COCO 人物裁图),
# 让 YOLO 在 mock 模式真检出 person (C4.4 mock 视频真检测)。
# mock_person.png 缺失时退化纯色矩形兜底 (不崩, spec 边界情况表)。
# ============================================================================
class MockFrameGenerator:
    def __init__(self, width=1280, height=720):
        self._w = int(width)
        self._h = int(height)
        self._t0 = time.time()
        self._person_img = None
        self._person_hw = (0, 0)
        self._load_person_png()

    def _load_person_png(self):
        """加载 web/static/mock_person.png (COCO 人物裁图, spec §8.1)。
        缺失/解码失败 → self._person_img=None, next_frame 用纯色矩形兜底。
        """
        path = os.path.join(_AI_DIR, "static", "mock_person.png")
        if not os.path.exists(path):
            logger.warning(f"[AI] mock_person.png 不存在: {path} (mock 帧将用纯色矩形, YOLO 可能检不出)")
            return
        try:
            import cv2
            img = cv2.imread(path, cv2.IMREAD_COLOR)
            if img is None:
                logger.warning(f"[AI] mock_person.png 解码失败: {path}")
                return
            # 缩放到画面合适大小 (高度占画面 60%, 保持纵横比)
            target_h = int(self._h * 0.6)
            scale = target_h / img.shape[0]
            target_w = max(1, int(img.shape[1] * scale))
            self._person_img = cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_AREA)
            self._person_hw = (target_h, target_w)
            logger.info(f"[AI] mock_person.png 加载: {target_w}x{target_h} (YOLO 应能检出 person)")
        except Exception as e:
            logger.warning(f"[AI] 加载 mock_person.png 失败: {e} (用纯色矩形兜底)")
            self._person_img = None

    def next_frame(self):
        """生成一帧 720p BGR ndarray。
        灰底 + 时间戳 + 移动彩色矩形 + (若有) person 裁图贴在画面中部偏右。
        """
        try:
            import cv2
        except Exception:
            # cv2 不在 (NX 上应装, 但兜底): 返回纯 numpy 灰帧
            frame = np.full((self._h, self._w, 3), 80, dtype=np.uint8)
            return frame

        frame = np.full((self._h, self._w, 3), 80, dtype=np.uint8)
        t = time.time() - self._t0

        # 几个移动彩色矩形 (模拟运动目标; YOLO 通常检不出, 但链路通)
        colors = [(0, 0, 200), (200, 100, 0), (0, 180, 180)]
        for i, c in enumerate(colors):
            cx = int(self._w * (0.15 + 0.10 * i + 0.03 * math.sin(t * 0.7 + i)))
            cy = int(self._h * (0.25 + 0.05 * math.cos(t * 0.9 + i)))
            x1, y1 = max(0, cx - 40), max(0, cy - 60)
            x2, y2 = min(self._w, cx + 40), min(self._h, cy + 60)
            cv2.rectangle(frame, (x1, y1), (x2, y2), c, -1)

        # 贴 person 裁图 (让 YOLO 真检出, C4.4)
        if self._person_img is not None:
            ph, pw = self._person_hw
            # 贴在画面中部偏左, 缓慢左右移动
            cx = int(self._w * (0.30 + 0.04 * math.sin(t * 0.5)))
            x1 = max(0, min(self._w - pw, cx - pw // 2))
            y1 = max(0, self._h - ph - 20)  # 贴底
            roi = frame[y1:y1 + ph, x1:x1 + pw]
            if roi.shape[0] == ph and roi.shape[1] == pw:
                frame[y1:y1 + ph, x1:x1 + pw] = self._person_img

        # 时间戳 (证明帧在更新)
        ts = time.strftime("%H:%M:%S") + f".{int((t * 100) % 100):02d}"
        cv2.putText(frame, f"[MOCK] {ts}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        return frame


# ============================================================================
# NxAiEngine — AI 推理引擎统一管理器 (spec §7.1)
# ============================================================================
class NxAiEngine:
    """NX AI 推理引擎: 视频+YOLO+VLM 的统一管理器。

    生命周期: nx_web_server.main() 创建一个实例, 注入 TaskManager (作 detector+vlm)
    和 broadcast_loop (读 _latest_frame/_latest_dets)。
    """

    def __init__(self):
        self._lock = threading.Lock()
        # 视频源 (spec 决策 4: ChannelFactory 单例 + VideoClient)
        self._video = None              # unitree VideoClient (懒初始化)
        self._factory = None            # ChannelFactory 单例 (本进程)
        self._video_inited = False
        self._mock_mode = False         # True=用 mock 帧 (狗没连/SDK 没装)
        self._mock_frame_gen = None     # MockFrameGenerator 实例
        self._video_fail_streak = 0     # 取帧连续失败计数 (>=10 切 mock, spec 边界表)
        # YOLO
        self._detector = None           # ai.detector.Detector (懒初始化)
        self._detector_inited = False
        # 缓存 (视频/YOLO 线程写, broadcast_loop 读)
        self._latest_frame = None       # numpy BGR (带检测框, 720p)
        self._latest_dets = []          # [{class, confidence, bbox}]
        self._frame_count = 0
        # MEDIUM-5: 检测发生时刻的输入帧宽 (bbox 坐标系)。
        # YOLO 在原始帧 (如 1080p=1920宽) detect, bbox 是 1920 系; 但帧随后 resize 到
        # 720p(1280宽) 存 _latest_frame。get_detections_world 必须用 _detect_frame_w 归一化,
        # 否则 cx_norm 系统偏小 → slam 检测标记系统偏左。
        self._detect_frame_w = 1280
        # VLM (spec 决策 2: 懒加载 + 单工作线程 + 空闲超时 unload)
        self._vlm = None                # ai.vlm.VLMEngine (懒初始化)
        self._vlm_inited = False
        # VLM 构造失败后的节流重试 (HIGH-1): 记录上次构造尝试时间, _vlm_worker 据此
        # 在 _vlm is None 且距上次尝试 >60s 时复位 _vlm_inited=False 允许自愈重试。
        self._vlm_last_init_attempt = 0.0
        self._vlm_init_retry_interval = float(os.environ.get("GO2W_VLM_RETRY", "60"))
        self._vlm_queue = queue.Queue()  # parse 请求队列 [(text, result_event, result_box), ...]
        self._vlm_last_use = 0.0
        self._vlm_idle_timeout = float(os.environ.get("GO2W_VLM_IDLE", "60"))
        self._vlm_loading = False       # load 进行中 (H3.2 loading 状态)
        # 控制
        self._running = False
        self._threads = []
        # mock 强制开关 (GO2W_AI_MOCK_VIDEO=1 → 跳过 VideoClient, 直接 mock)
        self._force_mock = bool(os.environ.get("GO2W_AI_MOCK_VIDEO"))

    # ------------------------------------------------------------------
    # 启动/停止
    # ------------------------------------------------------------------
    def start(self):
        """启动视频/YOLO 线程 + VLM 工作线程 + 显存监控 (spec §5)。
        关键: 启动时不 import torch/ultralytics (懒加载, 启动秒级, §11 反模式)。

        round-3: 即使无 ultralytics/torch, start() 仍正常创建并启动 3 线程
        (video/vlm/mem)。视频线程 _init_detector 见无 ultralytics → detector=None,
        仍取帧推 type=frame (detections 空); VLM 线程见无 transformers → 走 fallback。
        即: 缺重依赖时 start() 不抛、视频流不断, 满足 NX 纯视频流部署。
        """
        self._running = True
        t1 = threading.Thread(target=self._video_yolo_loop, name="nx_ai_video", daemon=True)
        t2 = threading.Thread(target=self._vlm_worker, name="nx_ai_vlm", daemon=True)
        t3 = threading.Thread(target=self._mem_monitor, name="nx_ai_mem", daemon=True)
        t1.start(); t2.start(); t3.start()
        self._threads = [t1, t2, t3]
        logger.info("[AI] NxAiEngine 启动 (3 daemon 线程: video/vlm/mem)")

    def stop(self):
        self._running = False
        try:
            if self._vlm is not None and getattr(self._vlm, "loaded", False):
                self._vlm.unload()
        except Exception as e:
            logger.warning(f"[AI] stop 卸载 VLM 失败: {e}")

    # ------------------------------------------------------------------
    # 视频 + YOLO 线程 (线程4, spec §5)
    # ------------------------------------------------------------------
    def _init_video(self):
        """懒初始化 VideoClient (首次取帧时)。失败 → 切 mock 模式 (spec 决策 4)。
        ChannelFactory 本进程只 Init 一次 (C2.4, 与 nx_sensor_node 进程隔离)。
        """
        if self._video_inited:
            return
        self._video_inited = True  # 标记已尝试, 失败也不再重试 (避免每帧重试拖慢)

        if self._force_mock:
            logger.info("[AI] 视频源: mock (GO2W_AI_MOCK_VIDEO=1 强制)")
            self._mock_mode = True
            self._mock_frame_gen = MockFrameGenerator()
            return

        try:
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.go2.video.video_client import VideoClient
        except Exception as e:
            logger.warning(f"[AI] unitree_sdk2py 未装/不可导入 ({e}), 视频源切 mock")
            self._mock_mode = True
            self._mock_frame_gen = MockFrameGenerator()
            return

        # 决策 4: 网卡 enxc8a362616c4c (与 nx_sensor_node:57 / nx_motion_node:56 一致)
        iface = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")
        try:
            self._factory = ChannelFactory()
            try:
                self._factory.Init(0, iface)
            except Exception:
                # 自动检测兜底 (网卡名不对时)
                logger.warning(f"[AI] ChannelFactory.Init(0, {iface}) 失败, 尝试 Init(0, None)")
                self._factory.Init(0, None)
            self._video = VideoClient()
            self._video.SetTimeout(10.0)
            self._video.Init()
            logger.info(f"[AI] 视频源: unitree VideoClient (iface={iface})")
        except Exception as e:
            logger.warning(f"[AI] VideoClient 初始化失败 ({e}), 视频源切 mock")
            self._mock_mode = True
            self._mock_frame_gen = MockFrameGenerator()
            self._video = None
            self._factory = None

    def _init_detector(self):
        """懒初始化 YOLO (降级链 engine>onnx>pt, spec §4.2)。
        Detector.__init__ 已支持 .engine/.onnx/.pt 任意格式 + warm-up (detector.py:28-30)。

        round-3 解耦: 若 _DETECT_ALLOWED=False (GO2W_AI_NO_DETECT=1 或无 ultralytics),
        直接 detector=None 返回, 不尝试加载任何模型 → 视频流不受影响。
        """
        if self._detector_inited:
            return
        self._detector_inited = True

        # round-3: 检测被禁 (环境禁 / 无 ultralytics) → 纯视频流模式
        # 不崩、不报错 (warning 已在模块加载时记), 视频线程照常取帧推流。
        if not _DETECT_ALLOWED:
            self._detector = None
            if _DETECT_DISABLED_BY_ENV:
                logger.info("[AI] _init_detector: GO2W_AI_NO_DETECT=1, 跳过 YOLO (纯视频流)")
            else:
                logger.info("[AI] _init_detector: 无 ultralytics, 跳过 YOLO (纯视频流)")
            return

        try:
            from ai.detector import Detector
        except Exception as e:
            # ai.detector 顶层 import ai.config, 一般不会失败; 若失败也不崩
            logger.error(f"[AI] 导入 ai.detector 失败 ({e}), YOLO 检测禁用 (视频流照常)")
            self._detector = None
            return

        # 降级链 (spec §4.2): engine > onnx > pt
        candidates = [
            os.environ.get("GO2W_YOLO_ENGINE", "models/yolov8n.engine"),
            os.environ.get("GO2W_YOLO_ONNX", "models/yolov8n.onnx"),
            os.environ.get("GO2W_YOLO_MODEL", "yolov8n.pt"),
        ]
        for path in candidates:
            if not path or not os.path.exists(path):
                continue
            try:
                det = Detector(model_path=path)
                if det.available:
                    logger.info(f"[AI] YOLO 加载成功: {path}")
                    self._detector = det
                    return
                else:
                    logger.warning(f"[AI] YOLO 加载失败 (available=False): {path}")
            except Exception as e:
                logger.warning(f"[AI] YOLO 加载异常 {path}: {e}, 尝试下一个候选")
        logger.error("[AI] 所有 YOLO 模型加载失败, 检测降级为禁用 (帧原样推送, slam.detections=[])")
        self._detector = None

    # ------------------------------------------------------------------
    # 检测器抽象 (round-3: 为 locateanything 预留可替换接口)
    # ------------------------------------------------------------------
    def _run_detector(self, frame):
        """统一检测入口 (round-3 检测器抽象)。

        把 YOLO 调用集中到这单一方法, _video_yolo_loop 只调它, 不直接碰 self._detector。
        当前实现: 调 self._detector.detect(frame) (YOLO/ultralytics)。

        契约: 输入 BGR ndarray 帧, 返回 list[dict{class,confidence,bbox}] (可能空)。
        - detector=None (无 ultralytics / GO2W_AI_NO_DETECT=1 / 模型加载失败) → 返回 []
        - 异常 → 记 warning + 返回 [] (绝不抛, 保证视频流不断)

        === 后续 locateanything 接入说明 (重要) ===
        locateanything (开放词汇定位) 上线时, 只需替换本方法体:
            def _run_detector(self, frame):
                # 例: return self._locate_anything.detect(frame, vocab=self._detect_vocab)
                ...
        签名不变 (frame → list[dict{class,confidence,bbox}]), 视频流路径 (_video_yolo_loop /
        get_frame_jpeg / get_detections_world) 完全不动。bbox 坐标系约定: 像素 xyxy 在
        _detect_frame_w (输入帧宽) 系下, get_detections_world 据此归一化方位角。
        """
        if frame is None:
            return []
        det = self._detector
        if det is None:
            # 纯视频流模式 (round-3): 不调任何检测器, detections 空
            return []
        try:
            dets = det.detect(frame)
            return dets if dets else []
        except Exception as e:
            logger.warning(f"[AI] 检测异常 ({type(det).__name__}): {e} (本帧 detections 空, 视频流继续)")
            return []

    def _get_frame(self):
        """取一帧 BGR ndarray (VideoClient.GetImageSample 或 mock, spec §7.1)。
        返回 None 表示该帧取失败 (调用方跳过)。
        """
        if self._mock_mode:
            if self._mock_frame_gen is None:
                self._mock_frame_gen = MockFrameGenerator()
            return self._mock_frame_gen.next_frame()

        if self._video is None:
            return None
        try:
            # SDK_CAPABILITIES §1.1: code, data = video.GetImageSample()
            code, data = self._video.GetImageSample()
            if code != 0 or not data:
                return None
            import cv2
            frame = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return None
            return frame
        except Exception as e:
            logger.debug(f"[AI] GetImageSample 失败: {e}")
            return None

    def _video_yolo_loop(self):
        """视频+检测 主循环 (线程4, spec §5)。
        - 懒初始化 (首次循环 _init_video + _init_detector)
        - 取帧 (~8fps, GetImageSample 200-500ms, SDK_CAPABILITIES §1.1)
        - 检测经 _run_detector 抽象 (detector=None 时返回 [], 帧原样, 视频流不断)
        - 缓存 _latest_frame (720p resize 后) + _latest_dets (Lock 保护)
        - 取帧连续 10 次失败 → 切 mock (spec 边界表)

        round-3: detector=None (无 ultralytics / GO2W_AI_NO_DETECT=1) 时本循环
        仍正常取帧 + resize + 缓存 + 推 type=frame, 只是 detections 空。
        """
        self._init_video()
        self._init_detector()

        # 目标 8fps 节流 (对齐取帧频率, 决策 3)
        target_fps = float(os.environ.get("GO2W_VIDEO_FPS", "8"))
        min_interval = 1.0 / target_fps if target_fps > 0 else 0.125

        while self._running:
            t0 = time.time()
            try:
                frame = self._get_frame()
                if frame is None:
                    self._video_fail_streak += 1
                    # 连续 10 次失败 → 切 mock (spec 边界表, H2.5)
                    if not self._mock_mode and self._video_fail_streak >= 10:
                        logger.warning(f"[AI] VideoClient 取帧连续失败 {self._video_fail_streak} 次, 切 mock")
                        self._mock_mode = True
                        self._mock_frame_gen = MockFrameGenerator()
                    time.sleep(min_interval)
                    continue
                self._video_fail_streak = 0

                # 检测 + 画框 (经 _run_detector 抽象, round-3 解耦)
                # detector=None (无 ultralytics / GO2W_AI_NO_DETECT=1) 时 _run_detector 返回 [],
                # 帧原样推送, 视频流不断 (spec 边界表 + round-3 纯视频流模式)。
                # MEDIUM-5: 记录检测发生时刻的输入帧宽 (bbox 坐标系)。
                # 检测在原始帧 (可能 1080p=1920宽) 上做, 随后 resize 到 720p;
                # 必须用此刻的宽度归一化 bbox, 不能用 resize 后的 _latest_frame.shape[1]。
                detect_frame_w = frame.shape[1]
                dets = self._run_detector(frame)
                if dets:
                    try:
                        # 画框 (detector=None 时 dets 必空, 不会进这里; 保守起见仍 try)
                        if self._detector is not None:
                            frame = self._detector.annotate(frame, dets)
                        # 抽样日志 (M4.2): 每 30 帧打印
                        if self._frame_count % 30 == 0:
                            det_str = ", ".join(f"{d['class']} {d['confidence']:.2f}" for d in dets[:5])
                            logger.info(f"[AI] detect: {det_str}")
                    except Exception as e:
                        logger.warning(f"[AI] annotate 异常: {e}")

                # resize 到 720p (决策 3: 1080p→720p 省带宽)
                try:
                    from ai.config import VIDEO_TARGET_WIDTH, VIDEO_TARGET_HEIGHT
                    tw, th = int(VIDEO_TARGET_WIDTH), int(VIDEO_TARGET_HEIGHT)
                except Exception:
                    tw, th = 1280, 720
                if frame.shape[1] != tw or frame.shape[0] != th:
                    import cv2
                    frame = cv2.resize(frame, (tw, th), interpolation=cv2.INTER_AREA)

                # 缓存 (Lock 保护, 广播线程读, §5 线程间通信)
                with self._lock:
                    self._latest_frame = frame
                    self._latest_dets = dets
                    # MEDIUM-5: 与 _latest_dets 同帧写入检测时的帧宽 (bbox 坐标系),
                    # get_detections_world 读它归一化, 不被 resize 后的 720p 宽度污染。
                    self._detect_frame_w = detect_frame_w
                    self._frame_count += 1
            except Exception as e:
                logger.warning(f"[AI] video_yolo_loop 异常: {e}")

            # 节流 (YOlO 慢则自然降速, 不丢帧, spec 边界表)
            elapsed = time.time() - t0
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)

    # ------------------------------------------------------------------
    # VLM 工作线程 (线程5, spec §5 + 决策 2)
    # ------------------------------------------------------------------
    def _init_vlm(self):
        """懒初始化 VLMEngine (首次 parse 时, 决策 2 约束 4)。
        失败 (OOM/路径错) → self._vlm=None 且 **不**置 _vlm_inited=True,
        允许 _vlm_worker 节流重试 (HIGH-1: 模型就位后能自愈, 不永久 fallback)。

        round-3 graceful: NX 上没装 transformers 时, VLMEngine 构造会抛
        (ai.vlm 内部懒 import transformers), 落入下面 except → _vlm=None。
        TaskManager 走 _fallback_parse (阶段A 行为, 已支持), 不崩、不退出。
        即: VLM 不可用时指令解析降级为关键词 fallback, 不影响视频流与检测。
        """
        if self._vlm_inited:
            return
        # 记录构造尝试时刻 (HIGH-1: 供 _vlm_worker 节流重试判断)
        self._vlm_last_init_attempt = time.time()
        try:
            from ai.vlm import VLMEngine
            # NX 上 VLM 模型路径 (config.py: VLM_MODEL_NAME_NX, 部署时 env 覆盖)
            try:
                from ai.config import VLM_MODEL_NAME_NX
                model_name = VLM_MODEL_NAME_NX
            except Exception:
                model_name = None
            self._vlm = VLMEngine(model_name=model_name) if model_name else VLMEngine()
            # 构造成功才标记完成 (HIGH-1: 失败留重试机会)
            self._vlm_inited = True
        except Exception as e:
            logger.error(f"[AI] 导入/构造 VLMEngine 失败 ({e}), VLM 不可用 (走 fallback, "
                         f"{self._vlm_init_retry_interval:.0f}s 后可重试)")
            self._vlm = None
            # 注意: 不置 _vlm_inited=True, 允许节流重试

    def submit_parse(self, text):
        """HTTP handler 调用: 把指令解析请求入队 (非阻塞, C2.2)。
        返回一个 result_box dict (含 threading.Event), 调用方 (TaskManager 后台线程) 等 Event。
        绝不在 HTTP handler 线程同步调 chat (会 HTTP 超时, §11 反模式)。
        """
        result_box = {"text": text, "response": None, "done": threading.Event()}
        self._vlm_queue.put((text, result_box))
        return result_box

    def _vlm_worker(self):
        """VLM 单工作线程 (线程5, C2.3: 串行消费, 不并发推理)。
        - 消费 _vlm_queue
        - 首次请求: load() (10-30s), 期间 ws_broadcast type=vlm loading=true
        - 推理: VLMEngine.chat([sys_prompt, user_text])
        - JSON 解析 → tasks (panel.py:472-518 同款)
        - 空闲 60s: unload() (C3.2)
        - unload 后新请求 → 重新 load (H3.4)
        """
        while self._running:
            # 本轮待处理的 result_box (MEDIUM-1: 异常时据此回写 fallback 并 set done)
            pending_box = None
            try:
                # HIGH-1 自愈: 若上次构造失败 (_vlm is None 且 _vlm_inited 仍 False),
                # 距上次尝试超过节流间隔则允许 _init_vlm 重入重试 (模型可能已就位/OOM 已释放)。
                if (self._vlm is None and not self._vlm_inited
                        and self._vlm_last_init_attempt > 0
                        and time.time() - self._vlm_last_init_attempt > self._vlm_init_retry_interval):
                    logger.info(f"[VLM] 距上次构造失败 {int(time.time()-self._vlm_last_init_attempt)}s, "
                                f"重试构造 (HIGH-1 自愈)")

                # 空闲 unload 检查 (C3.2): 没有待处理请求 + VLM 已加载 + 超过空闲超时
                if (self._vlm is not None and getattr(self._vlm, "loaded", False)
                        and self._vlm_last_use > 0
                        and time.time() - self._vlm_last_use > self._vlm_idle_timeout):
                    idle = int(time.time() - self._vlm_last_use)
                    logger.info(f"[VLM] 卸载 (空闲 {idle}s ≥ {self._vlm_idle_timeout}s)")
                    try:
                        self._vlm.unload()  # vlm.py:163 已含 torch.cuda.empty_cache
                    except Exception as e:
                        logger.warning(f"[VLM] unload 异常: {e}")
                    self._vlm_last_use = 0.0

                # 阻塞等待请求 (短超时让空闲检查每 5s 跑一次)
                try:
                    text, result_box = self._vlm_queue.get(timeout=5.0)
                except queue.Empty:
                    continue
                pending_box = result_box  # 记录本轮请求 (MEDIUM-1)

                # 重新 load (首次 或 unload 后又来请求, H3.4)
                self._init_vlm()
                need_load = (self._vlm is not None and not getattr(self._vlm, "loaded", False))
                if need_load:
                    reloading = self._vlm_last_use > 0  # 之前用过 → 这次是重载
                    self._vlm_loading = True
                    self._safe_broadcast({"type": "vlm", "data": {
                        "text": text, "response": "(VLM 加载中...)" if not reloading else "(VLM 重新加载中...)",
                        "tasks": [], "loading": True}})
                    tag = "重新加载" if reloading else "加载中"
                    logger.info(f"[VLM] {tag}... (首次 load 10-30s)")
                    t0 = time.time()
                    ok = False
                    try:
                        ok = self._vlm.load()
                    except Exception as e:
                        logger.error(f"[VLM] load 异常: {e}")
                    self._vlm_loading = False
                    if not ok:
                        try:
                            from ai.config import memory_summary
                            logger.error(f"[VLM] load 失败, {memory_summary()}")
                        except Exception:
                            logger.error("[VLM] load 失败 (无显存摘要)")
                        # fallback (H3.3)
                        fallback = self._fallback_parse(text)
                        self._safe_broadcast({"type": "vlm", "data": {
                            "text": text, "response": fallback["response"] + "(fallback)",
                            "tasks": fallback["tasks"], "fallback": True}})
                        result_box["response"] = fallback
                        result_box["done"].set()
                        continue
                    logger.info(f"[VLM] 就绪 (load 用时 {time.time()-t0:.1f}s)")

                if self._vlm is None or not getattr(self._vlm, "loaded", False):
                    # VLM 不可用 (导入失败/load 失败), fallback
                    fallback = self._fallback_parse(text)
                    self._safe_broadcast({"type": "vlm", "data": {
                        "text": text, "response": fallback["response"] + "(fallback)",
                        "tasks": fallback["tasks"], "fallback": True}})
                    result_box["response"] = fallback
                    result_box["done"].set()
                    continue

                # 推理 (panel.py:472-495 sys_prompt + 496-499 chat + 500-518 JSON 解析)
                result = self._vlm_parse_command(text)
                self._vlm_last_use = time.time()
                self._safe_broadcast({"type": "vlm", "data": {
                    "text": text, "response": result.get("response", ""),
                    "tasks": result.get("tasks", [])}})
                result_box["response"] = result
                result_box["done"].set()
            except Exception as e:
                logger.error(f"[AI] vlm_worker 异常: {e}")
                # MEDIUM-1: 若已取到本轮请求的 result_box, 回写 fallback 并 set done,
                # 否则调用线程 (NxAiVlmProxy.chat) 会 wait(120) 卡满超时。
                if pending_box is not None and not pending_box["done"].is_set():
                    try:
                        fb = self._fallback_parse(pending_box.get("text", ""))
                        self._safe_broadcast({"type": "vlm", "data": {
                            "text": pending_box.get("text", ""),
                            "response": fb["response"] + "(worker异常)",
                            "tasks": fb["tasks"], "fallback": True}})
                        pending_box["response"] = fb
                        pending_box["done"].set()
                    except Exception as e2:
                        logger.error(f"[AI] vlm_worker 异常回写也失败: {e2}")
                        pending_box["response"] = {"tasks": [], "response": "(worker异常)"}
                        pending_box["done"].set()

    def _safe_broadcast(self, data):
        """转发到 nx_web_server.ws_broadcast, 但容错: rclpy/nx_web_server 不在时仅记日志。
        正常部署: nx_web_server 进程内本文件 import, ws_broadcast 已注入 _WS_BROADCAST_FN。
        单测/无 rclpy 环境: 跳过广播 (不影响 vlm worker 主逻辑, spec §10 边界)。

        实现注意: 不在每次调用都 `from nx_web_server import ws_broadcast`——
        nx_web_server.py 顶部 `import rclpy`, 无 rclpy 环境会留下部分导入的模块对象
        在 sys.modules, 反复重导入可能挂起线程。改用 nx_web_server 在 main() 里
        调 set_ws_broadcast() 一次性注入本模块全局, worker 直接读全局, 零 import。
        """
        fn = _WS_BROADCAST_FN
        if fn is not None:
            try:
                fn(data)
            except Exception as e:
                logger.debug(f"[AI] ws_broadcast 调用失败: {e}")
        else:
            logger.debug(f"[AI] broadcast 跳过 (ws_broadcast 未注入; 无 nx_web_server/rclpy?)")

    # panel.py:472-518 同款 sys_prompt + JSON 解析 (M2.1/M2.2)
    _SYS_PROMPT = """你是机器狗指令解析器。把用户中文指令转成JSON任务列表。

任务类型和参数:
- move: {"vx":前进速度m/s, "vy":侧移, "vyaw":旋转(正=左转), "duration":秒}
- follow: {"target":"目标"}
- search_area: {"pattern":"lawnmower", "width":米, "height":米}
- stop: {}
- return_home: {}

示例:
输入"前进两米然后左转"
输出: {"tasks":[{"type":"move","priority":8,"params":{"vx":0.5,"duration":4.0}},{"type":"move","priority":7,"params":{"vyaw":0.5,"duration":3.0}}]}

输入"搜索这个房间"
输出: {"tasks":[{"type":"search_area","priority":5,"params":{"pattern":"lawnmower","width":8,"height":8}}]}

输入"跟着前面的人"
输出: {"tasks":[{"type":"follow","priority":8,"params":{"target":"前面的人"}}]}

输入"后退"
输出: {"tasks":[{"type":"move","priority":6,"params":{"vx":-0.5,"duration":2.0}}]}

只输出JSON, 不要解释, 不要markdown代码块。注意: 后退要用vx负数, 不是vyaw!"""

    def _vlm_parse_command(self, text):
        """VLM 真解析 (panel.py:472-518 等价实现)。
        返回 {response, tasks}; VLM 失败 → _fallback_parse。
        """
        try:
            response = self._vlm.chat([
                {"role": "system", "content": self._SYS_PROMPT},
                {"role": "user", "content": text}
            ], max_new_tokens=512)
            import re
            logger.info(f"VLM 原始响应: {response[:200]}")
            try:
                clean = re.sub(r'```(?:json)?\s*', '', response)
                clean = re.sub(r'```\s*$', '', clean)
                clean = re.sub(r'//[^\n]*', '', clean)
                m = re.search(r'\{', clean)
                if m:
                    start = m.start(); depth = 0; end = start
                    for i, ch in enumerate(clean[start:], start):
                        if ch == '{': depth += 1
                        elif ch == '}':
                            depth -= 1
                            if depth == 0: end = i + 1; break
                    data = json.loads(clean[start:end])
                    if "tasks" in data:
                        data.setdefault("response", "已解析")
                        return data
            except Exception as e:
                logger.warning(f"VLM JSON 解析失败: {e}")
        except Exception as e:
            logger.error(f"VLM chat 失败: {e}")
        return self._fallback_parse(text)

    @staticmethod
    def _fallback_parse(text):
        """关键词 fallback (panel.py:520-545 同款, M2.2)。"""
        r = {"understanding": text, "tasks": [], "response": ""}
        if "跟着" in text or "跟随" in text:
            target = ""
            for kw in ["跟着", "跟随"]:
                if kw in text:
                    target = text[text.index(kw) + len(kw):].strip().rstrip("。，！？")
            r["tasks"] = [{"type": "follow", "priority": 8, "params": {"target": target}}]
            r["response"] = f"跟踪{target}"
        elif "搜索" in text or "找" in text:
            r["tasks"] = [{"type": "search_area", "priority": 5,
                           "params": {"pattern": "lawnmower", "width": 10, "height": 10}}]
            r["response"] = "开始搜索"
        elif "停" in text:
            r["tasks"] = [{"type": "stop", "priority": 10, "params": {}}]
            r["response"] = "已停止"
        elif "回来" in text or "返回" in text:
            r["tasks"] = [{"type": "return_home", "priority": 7, "params": {}}]
            r["response"] = "返回"
        elif "前进" in text or "向前" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": 0.5, "duration": 2.0}}]
            r["response"] = "前进"
        elif "后退" in text or "向后" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": -0.5, "duration": 2.0}}]
            r["response"] = "后退"
        elif "左转" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": 0.5, "duration": 2.0}}]
            r["response"] = "左转"
        elif "右转" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": -0.5, "duration": 2.0}}]
            r["response"] = "右转"
        else:
            r["response"] = f"收到: {text}"
        return r

    # ------------------------------------------------------------------
    # broadcast_loop 读取接口 (spec §7.1)
    # ------------------------------------------------------------------
    def get_frame_jpeg(self):
        """broadcast_loop 调用: 返回 (base64 jpeg, detections_count) 或 None (C1.4 整数!)。
        - 读 _latest_frame (Lock 保护)
        - cv2.imencode('.jpg', frame, [JPEG_QUALITY, 50])  (决策 3: 质量 50)
        - base64 编码
        - detections_count = len(_latest_dets) (整数计数, 不是数组!)
        """
        with self._lock:
            frame = self._latest_frame
            dets = self._latest_dets
        if frame is None:
            return None
        try:
            import cv2
            try:
                from ai.config import VIDEO_JPEG_QUALITY
                q = int(VIDEO_JPEG_QUALITY)
            except Exception:
                q = 50
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, q])
            if not ok:
                return None
            b64 = base64.b64encode(jpeg.tobytes()).decode()
            # C1.4: detections 是整数计数, 不是数组!
            return (b64, len(dets))
        except Exception as e:
            logger.debug(f"[AI] get_frame_jpeg 异常: {e}")
            return None

    def get_detections_world(self, robot_x, robot_y, robot_yaw):
        """broadcast_loop 调用: 返回 slam.data.detections 格式 [{x,y,class}] (C1.5 数组!)。
        bbox 中心 x 归一化 → 方位角 (假设 FOV=70°), 距离假设 3m (spec §6.2 简化)。
        世界坐标 = robot + 3m×(cos(yaw+ang), sin(yaw+ang))。
        与 type=frame 的整数 detections 相反, slam 这里必须是数组。

        MEDIUM-5: 归一化用检测时刻的帧宽 _detect_frame_w (bbox 坐标系),
        不用 resize 后的 _latest_frame.shape[1] (720p=1280), 否则方位系统偏左。
        """
        with self._lock:
            dets = list(self._latest_dets)
            fw = self._detect_frame_w if self._detect_frame_w > 0 else 1280
        if not dets:
            return []
        half_fov = math.radians(_CAMERA_HFOV_DEG / 2.0)
        out = []
        for d in dets:
            try:
                bbox = d.get("bbox", [0, 0, fw, 0])
                if not bbox or len(bbox) < 4:
                    continue
                x1, y1, x2, y2 = bbox[:4]
                cx_px = (float(x1) + float(x2)) / 2.0
                # 归一化中心 x (0=画面左, 0.5=中, 1=右) → 方位角 (相对狗朝向)
                cx_norm = cx_px / float(fw) if fw > 0 else 0.5
                angle = (cx_norm - 0.5) * 2.0 * half_fov  # rad, 正=右
                # 世界坐标 (spec §6.2): robot + dist×(cos(yaw+ang), sin(yaw+ang))
                # 注: 图像右 = 机器人右; 机器人朝向 yaw (世界系); 右转 = yaw - angle
                # 这里用 yaw + angle (angle 正=右) 对齐 map.js 习惯, 后续 LiDAR 融合再校准
                wx = robot_x + _DETECT_ASSUME_DIST_M * math.cos(robot_yaw + angle)
                wy = robot_y + _DETECT_ASSUME_DIST_M * math.sin(robot_yaw + angle)
                out.append({
                    "x": round(wx, 2),
                    "y": round(wy, 2),
                    "class": d.get("class", "?"),
                })
            except Exception:
                continue
        return out

    # ------------------------------------------------------------------
    # 显存监控 (线程6, spec §5)
    # ------------------------------------------------------------------
    def _mem_monitor(self):
        """每 30s 日志显存: torch.cuda.memory_allocated/reserved (决策 2, H3.1)。
        torch 不在 (无 GPU/CPU 环境) → 仅打印 mock/video 状态, 不报错。
        """
        interval = float(os.environ.get("GO2W_MEM_MONITOR_INTERVAL", "30"))
        while self._running:
            try:
                time.sleep(interval)
                if not self._running:
                    break
                try:
                    import torch
                    if torch.cuda.is_available():
                        alloc = torch.cuda.memory_allocated(0) // (1024 * 1024)
                        reserved = torch.cuda.memory_reserved(0) // (1024 * 1024)
                        vlm_loaded = (self._vlm is not None and getattr(self._vlm, "loaded", False))
                        logger.info(f"[AI] 显存: allocated={alloc}MB reserved={reserved}MB "
                                    f"(YOLO={'on' if self._detector else 'off'}, "
                                    f"VLM={'loaded' if vlm_loaded else 'unloaded'})")
                    else:
                        logger.info(f"[AI] CUDA 不可用 (mock 模式={self._mock_mode}, "
                                    f"frame_count={self._frame_count})")
                except Exception as e:
                    logger.debug(f"[AI] mem_monitor torch 不在: {e}")
            except Exception as e:
                logger.warning(f"[AI] mem_monitor 异常: {e}")


# ============================================================================
# NxAiVlmProxy / NxAiDetectorProxy — TaskManager 用的代理 (spec §7.2)
# ----------------------------------------------------------------------------
# TaskManager 的 _process_command_bg (panel.py:461) 在独立线程同步调 vlm.chat,
# 这里 chat() 通过 submit_parse 入队 + threading.Event 等结果 (M2.3, 不 busy-wait)。
# vlm.loaded 恒 True (让 TaskManager 走 _vlm_parse_command 分支),
# 实际推理走 NxAiEngine._vlm_worker 单线程 (C2.3)。
# ============================================================================
class NxAiVlmProxy:
    """让 TaskManager 以为有 vlm (loaded=True), 实际转发到 NxAiEngine 异步队列。"""

    def __init__(self, ai_engine: NxAiEngine):
        self._ai = ai_engine

    @property
    def loaded(self):
        # 恒 True: TaskManager._process_command_bg (nx_web_server.py:421) 据此走 _vlm_parse_command
        # 真正的 VLM 状态由 ai_engine 内部按需 load/unload 管理
        return True

    def chat(self, messages, max_new_tokens=200):
        """同步阻塞调用线程 (TaskManager._process_command_bg), 等队列结果 (M2.3 Event)。
        绝不在 HTTP handler 线程调 (会 HTTP 超时, §11 反模式)。
        messages 格式: [{"role":..,"content":..}, ...]
        本代理把 messages 转成 text (取 user content), 由 _vlm_worker 走 sys_prompt 推理。
        """
        text = ""
        try:
            for m in messages:
                if m.get("role") == "user":
                    c = m.get("content", "")
                    if isinstance(c, str):
                        text = c
                    elif isinstance(c, list):
                        # Qwen 格式 content 可能是 list[{type, text/image}]
                        text = " ".join(it.get("text", "") for it in c if it.get("type") == "text")
                    if text:
                        break
        except Exception:
            text = str(messages)
        if not text:
            text = "(空指令)"

        result_box = self._ai.submit_parse(text)
        # 同步等结果 (调用方已是后台线程, 阻塞 OK); 设上限防 worker 卡死
        if not result_box["done"].wait(timeout=120.0):
            # HIGH-2: 超时分支必须返回合法 JSON (含 tasks/response),
            # 用 _fallback_parse 保证 TaskManager._vlm_parse_command 能正常解析,
            # 不能返回裸 "(超时)" 字符串 (会让 JSON 解析抛异常落 fallback, 路径混乱)。
            # 同时 set done: worker 若迟到完成也不应再写已过期的 result_box。
            logger.warning(f"[VLM] proxy 等结果超时 (120s), 走 fallback")
            fb = self._ai._fallback_parse(text)
            result_box["done"].set()
            return json.dumps(fb, ensure_ascii=False)
        result = result_box.get("response")
        if result is None:
            # 无结果 (worker 未写 response): 同样用 fallback 保证契约一致
            logger.warning(f"[VLM] proxy 收到空结果, 走 fallback")
            fb = self._ai._fallback_parse(text)
            return json.dumps(fb, ensure_ascii=False)
        # TaskManager._vlm_parse_command 期望 chat 返回字符串 (vlm.chat 的契约)
        # 把解析后的 dict 重新序列化成 JSON 字符串, 让 TaskManager 再 parse 一次 (保持契约)
        try:
            return json.dumps(result, ensure_ascii=False)
        except Exception:
            return '{"tasks":[],"response":"' + str(result.get("response", "")) + '"}'


class NxAiDetectorProxy:
    """让 TaskManager._execute_search (panel.py:627-639) 用真检测。
    detect 转发到 NxAiEngine._detector; annotate 同。
    """

    def __init__(self, ai_engine: NxAiEngine):
        self._ai = ai_engine

    @property
    def available(self):
        return self._ai._detector is not None

    def detect(self, frame, target_classes=None):
        det = self._ai._detector
        if det is None or frame is None:
            return []
        try:
            return det.detect(frame, target_classes=target_classes)
        except Exception as e:
            logger.warning(f"[AI] detector proxy detect 异常: {e}")
            return []

    def annotate(self, frame, detections):
        det = self._ai._detector
        if det is None or frame is None:
            return frame
        try:
            return det.annotate(frame, detections)
        except Exception:
            return frame
