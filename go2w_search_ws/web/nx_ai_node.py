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
  - VLM 同样 graceful: 无 transformers → 返回可见解析错误和空任务，不生成运动指令。
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

from nx_camera_calibration import resolve_camera_calibration

from nx_mission_schema import (
    MissionValidationError,
    canonicalize_search_tasks,
)

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

# C13 可见光标注基线。任务编排器与前端世界坐标投影共用同一解析逻辑，
# 防止同一 bbox 在两条路径上得到不同方位角。
_CAMERA_CALIBRATION = resolve_camera_calibration("c13_vis")
_CAMERA_HFOV_DEG = _CAMERA_CALIBRATION["hfov_deg"]
_CAMERA_YAW_OFFSET_DEG = _CAMERA_CALIBRATION["effective_yaw_offset_deg"]
_DETECTION_SNAPSHOT_MAX = max(64, int(os.environ.get("GO2W_DETECTION_SNAPSHOT_MAX", "256")))
try:
    _DETECTION_MIN_CONFIDENCE = float(os.environ.get(
        "GO2W_DETECTION_MIN_CONFIDENCE", "0.8"))
except (TypeError, ValueError):
    _DETECTION_MIN_CONFIDENCE = 0.8
if not math.isfinite(_DETECTION_MIN_CONFIDENCE):
    _DETECTION_MIN_CONFIDENCE = 0.8
_DETECTION_MIN_CONFIDENCE = min(1.0, max(0.0, _DETECTION_MIN_CONFIDENCE))


def _filter_confident_detections(detections):
    """Keep only detector results meeting the product confidence floor."""

    accepted = []
    for detection in detections or []:
        if not isinstance(detection, dict):
            continue
        raw = detection.get(
            "confidence", detection.get("score", detection.get("probability", 0.0)))
        try:
            confidence = float(raw)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(confidence) and confidence >= _DETECTION_MIN_CONFIDENCE:
            accepted.append(detection)
    return accepted

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
# GO2W_VLM_DISABLED=1 → 完全跳过 VLM (不启 _vlm_worker 线程, 不构造 VLMEngine,
# NxAiVlmProxy.loaded=False 让 TaskManager 走 _parse_product_command 确定性路径).
# 用途: NX 资源紧张时砍 Qwen2.5-VL-3B (省 3-4GB GPU+内存), 仅保留 YOLO 检测.
# 新硬件恢复 AI 时 unset 此 env 即可, 无需 redeploy.
_VLM_DISABLED = str(os.environ.get("GO2W_VLM_DISABLED", "")).strip() in ("1", "true", "True", "yes")
_AI_VIDEO_ENABLED = str(os.environ.get("GO2W_AI_VIDEO_ENABLE", "0")).strip() in ("1", "true", "True", "yes", "on")
_AI_EXTERNAL_VIDEO_ENABLED = str(os.environ.get("GO2W_AI_EXTERNAL_VIDEO_ENABLE", "1")).strip() in ("1", "true", "True", "yes", "on")


def _dog_interface_ready(iface, sys_class_net="/sys/class/net"):
    """Return True when the configured dog-camera DDS interface is usable."""
    if not iface:
        return True
    iface_dir = os.path.join(sys_class_net, str(iface))
    if not os.path.isdir(iface_dir):
        return False
    try:
        with open(os.path.join(iface_dir, "operstate"), "r", encoding="utf-8") as f:
            operstate = f.read().strip().lower()
    except Exception:
        operstate = ""
    if operstate and operstate not in ("up", "unknown"):
        return False
    carrier_path = os.path.join(iface_dir, "carrier")
    if os.path.exists(carrier_path):
        try:
            with open(carrier_path, "r", encoding="utf-8") as f:
                if f.read().strip() == "0":
                    return False
        except Exception:
            return False
    return True

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
        self._locate_lock = threading.Lock()  # M5 fix: 串行化 locate_target (subprocess fork 3B 模型 ~2GB, 并发 OOM)
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
        self._detection_targets = None  # mission-scoped detector vocabulary
        self._last_detector_target_classes = None
        # 缓存 (视频/YOLO 线程写, broadcast_loop 读)
        self._latest_frame = None       # numpy BGR (带检测框, 720p)
        self._latest_dets = []          # [{class, confidence, bbox}]
        self._frame_count = 0
        # MEDIUM-5: 检测发生时刻的输入帧宽 (bbox 坐标系)。
        # YOLO 在原始帧 (如 1080p=1920宽) detect, bbox 是 1920 系; 但帧随后 resize 到
        # 720p(1280宽) 存 _latest_frame。get_detections_world 必须用 _detect_frame_w 归一化,
        # 否则 cx_norm 系统偏小 → slam 检测标记系统偏左。
        self._detect_frame_w = 1280
        self._detect_frame_h = 720
        self._latest_detection_source = ""
        self._latest_detection_overlay = []
        self._latest_detection_overlays = {}
        self._detection_source_order = []
        self._latest_source_frame_jpegs = {}
        self._latest_source_frame_meta = {}
        self._detection_seq = 0
        self._detection_snapshots = {}
        self._detection_snapshot_order = []
        self._external_frames = {}
        self._external_seq_by_source = {}
        self._processed_external_seq_by_source = {}
        self._external_source_order = []
        self._external_rr_index = 0
        self._detection_input_rr_index = 0
        # VLM (spec 决策 2: 懒加载 + 单工作线程 + 空闲超时 unload)
        self._vlm = None                # ai.vlm.VLMEngine (懒初始化)
        self._vlm_disabled = _VLM_DISABLED  # GO2W_VLM_DISABLED=1 → 跳过 vlm 线程+构造, proxy.loaded=False
        self._vlm_inited = False
        # VLM 构造失败后的节流重试 (HIGH-1): 记录上次构造尝试时间, _vlm_worker 据此
        # 在 _vlm is None 且距上次尝试 >60s 时复位 _vlm_inited=False 允许自愈重试。
        self._vlm_last_init_attempt = 0.0
        self._vlm_init_retry_interval = float(os.environ.get("GO2W_VLM_RETRY", "60"))
        self._vlm_queue = queue.Queue()  # parse 请求队列 [(text, result_event, result_box), ...]
        self._vlm_last_use = 0.0
        self._vlm_idle_timeout = float(os.environ.get("GO2W_VLM_IDLE", "60"))
        self._vlm_loading = False       # load 进行中 (H3.2 loading 状态)
        # LocateAnything (开放词汇定位; 低频按需, 不进 8fps YOLO 主循环)
        self._locate = None
        self._locate_inited = False
        self._latest_locate_dets = []
        self._latest_locate_frame_w = 1280
        self._latest_locate_target = ""
        self._latest_locate_status = "unloaded"
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
        仍取帧推 type=frame (detections 空); VLM 线程见无 transformers → 返回空任务。
        即: 缺重依赖时 start() 不抛、视频流不断, 满足 NX 纯视频流部署。
        """
        self._running = True
        t3 = threading.Thread(target=self._mem_monitor, name="nx_ai_mem", daemon=True)
        threads = []
        if _AI_VIDEO_ENABLED:
            t1 = threading.Thread(target=self._video_yolo_loop, name="nx_ai_video", daemon=True)
            t1.start()
            threads.append(t1)
        elif _AI_EXTERNAL_VIDEO_ENABLED:
            t1 = threading.Thread(target=self._video_yolo_loop, name="nx_ai_c13_video", daemon=True)
            t1.start()
            threads.append(t1)
            logger.info("[AI] C13 external video detection loop enabled (dog camera loop remains off)")
        else:
            logger.warning("[AI] dog camera video loop disabled (GO2W_AI_VIDEO_ENABLE=0) — "
                           "无狗原生视频帧; locate/follow 仅在 C13 云台启用时有帧, 否则 /api/locate 返回'无可用帧'")
        if not self._vlm_disabled:
            t2 = threading.Thread(target=self._vlm_worker, name="nx_ai_vlm", daemon=True)
            t2.start()
            threads.append(t2)
        t3.start()
        threads.append(t3)
        self._threads = threads
        vlm_stat = "vlm-off(GO2W_VLM_DISABLED)" if self._vlm_disabled else "vlm"
        logger.info(f"[AI] NxAiEngine 启动 ({len(threads)} daemon 线程: "
                    f"{'video/' if (_AI_VIDEO_ENABLED or _AI_EXTERNAL_VIDEO_ENABLED) else ''}{vlm_stat}/mem)")

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

        # 决策 4: 网卡 enxc8a362616c4c (与 nx_sensor_node:57 / nx_motion_node:56 一致)
        iface = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")
        if not _dog_interface_ready(iface):
            logger.warning(f"[AI] dog interface not ready (DOG_INTERFACE={iface}), 视频源切 mock")
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
    def set_detection_targets(self, target_classes=None):
        """Atomically replace the mission detector vocabulary.

        Returns the previous value so callers can restore it in ``finally``.
        ``None`` restores normal detector behavior (YOLO all classes;
        YOLO-World's configured defaults).
        """
        normalized = None
        if target_classes is not None:
            normalized = []
            for target in target_classes:
                value = str(target or "").strip()
                if value and value not in normalized:
                    normalized.append(value)
            if not normalized:
                normalized = None
        with self._lock:
            previous = (
                None if self._detection_targets is None
                else list(self._detection_targets)
            )
            self._detection_targets = normalized
        return previous

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
        with self._lock:
            target_classes = (
                None if self._detection_targets is None
                else list(self._detection_targets)
            )
            # The video loop copies this metadata immediately after inference
            # and stores it with the frame.  This prevents a just-switched
            # mission from consuming a frame inferred for the old vocabulary.
            self._last_detector_target_classes = (
                None if target_classes is None else list(target_classes)
            )
        det = self._detector
        if det is None:
            # 纯视频流模式 (round-3): 不调任何检测器, detections 空
            return []
        try:
            dets = det.detect(frame, target_classes=target_classes)
            return _filter_confident_detections(dets)
        except Exception as e:
            logger.warning(f"[AI] 检测异常 ({type(det).__name__}): {e} (本帧 detections 空, 视频流继续)")
        return []

    # ------------------------------------------------------------------
    # LocateAnything 按需定位 (低频 grounding, 给 /api/locate 和 follow 用)
    # ------------------------------------------------------------------
    def _init_locate_anything(self):
        """Lazy-load the locate-anything.cpp CLI adapter.

        This only checks the binary/model paths. The 3B model itself is loaded
        by the external CLI process when a locate request arrives.
        """
        if self._locate_inited:
            return
        self._locate_inited = True
        backend = os.environ.get("GO2W_LOCATE_BACKEND", "cpp").strip().lower()
        if backend in ("0", "off", "false", "none", "no"):
            self._latest_locate_status = "disabled"
            logger.info("[LocateAnything] disabled by GO2W_LOCATE_BACKEND")
            return
        try:
            from ai.locate_anything import LocateAnythingCli
            locate = LocateAnythingCli()
            self._locate = locate
            self._latest_locate_status = "ready" if locate.available else "missing_model_or_binary"
            logger.info(f"[LocateAnything] backend=cpp status={self._latest_locate_status} "
                        f"bin={locate.binary} model={locate.model}")
        except Exception as e:
            self._locate = None
            self._latest_locate_status = f"error: {e}"
            logger.warning(f"[LocateAnything] init failed: {e}")

    def locate_target(self, frame, target):
        """Locate a natural-language target in one frame.

        Returns the same result shape as ai.vlm.VLMEngine.locate, with an extra
        `detections` list for page/status rendering.
        """
        if frame is None:
            return {"found": False, "bbox": None, "cx": 0, "cy": 0,
                    "label": "", "confidence": 0.0, "detections": [],
                    "description": "empty frame"}
        self._init_locate_anything()
        # M5 fix: 串行化 locate (subprocess fork 3B 模型 ~2GB, 并发 OOM); /api/locate + tracker 两路调用
        with self._locate_lock:
            result = None
            if self._locate is not None and getattr(self._locate, "available", False):
                result = self._locate.locate(frame, target)
            elif self._vlm is not None and getattr(self._vlm, "loaded", False):
                result = self._vlm.locate(frame, target)
                if "detections" not in result and result.get("found") and result.get("bbox"):
                    result["detections"] = [{
                        "class": result.get("description", target),
                        "confidence": 1.0,
                        "bbox": result.get("bbox"),
                        "source": "vlm",
                    }]
            else:
                result = {"found": False, "bbox": None, "cx": 0, "cy": 0,
                          "label": "", "confidence": 0.0, "detections": [],
                          "description": self._latest_locate_status or "locate unavailable"}

        try:
            frame_w = int(frame.shape[1])
            frame_h = int(frame.shape[0])
        except Exception:
            frame_w, frame_h = 1280, 720
        dets = result.get("detections") or []
        if result.get("found") and result.get("bbox") and not dets:
            dets = [{
                "class": result.get("label") or result.get("description") or target,
                "label_zh": result.get("label_zh") or result.get("label") or result.get("description") or target,
                "confidence": result.get("confidence", 1.0),
                "bbox": result.get("bbox"),
                "source": "locate_anything",
            }]
        detected_at = time.time()
        dets = [
            {
                **d,
                "frame_width": int(d.get("frame_width") or frame_w),
                "frame_height": int(d.get("frame_height") or frame_h),
                "ts": detected_at,
            }
            for d in dets
            if isinstance(d, dict)
        ]
        result["frame_width"] = frame_w
        result["frame_height"] = frame_h
        result["detections"] = dets

        with self._lock:
            self._latest_locate_target = target
            self._latest_locate_dets = dets
            self._latest_locate_frame_w = frame_w
            if result.get("found"):
                self._latest_locate_status = "found"
            elif self._latest_locate_status == "ready":
                self._latest_locate_status = "not_found"

        if result.get("found") and result.get("bbox"):
            x1, y1, x2, y2 = [float(v) for v in result["bbox"][:4]]
            result["cx"] = (x1 + x2) / 2.0 / max(1.0, float(frame_w))
            result["cy"] = (y1 + y2) / 2.0 / max(1.0, float(frame_h))
        else:
            result.setdefault("cx", 0)
            result.setdefault("cy", 0)

        self._safe_broadcast({"type": "locate", "data": {
            "target": target,
            "status": result.get("description") or self._latest_locate_status,
            "found": bool(result.get("found")),
            "bbox": result.get("bbox"),
            "label": result.get("label", ""),
            "label_zh": result.get("label_zh", ""),
            "confidence": result.get("confidence", 0.0),
            "cx": result.get("cx", 0),
            "cy": result.get("cy", 0),
            "frame_width": frame_w,
            "frame_height": frame_h,
            "detections": dets,
            "description": result.get("description", ""),
        }})
        return result

    def track_target(self, frame, target, img_w=640, img_h=480):
        """Locate target and convert its bbox into follow vx/vyaw.

        C1 fix (2026-07-01): vx 永远 >= 0 (禁后退 — 轮足狗后退看不到目标且易撞身后);
        用 bbox 宽度比(不受物体高度影响)替代面积比(贴地小物体面积小会被误判 ratio<0.3 持续前冲撞墙)。
        执行层 guard(connected/前方障碍) 在 NxRobotBridge.move, 覆盖所有调用者。"""
        loc = self.locate_target(frame, target)
        if not loc.get("found"):
            return {**loc, "vx": 0.0, "vyaw": 0.0}
        bbox = loc.get("bbox")
        cx = float(loc.get("cx", 0.5))
        offset_x = cx - 0.5
        vyaw = max(-1.0, min(1.0, -offset_x * 2.0))
        vx = 0.0
        if bbox and len(bbox) >= 4:
            x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
            # C1: 宽度比(高度无关), 贴地物体(背包/箱子)也能正确判距
            width_ratio = max(0.0, x2 - x1) / max(1.0, float(img_w))
            if width_ratio < 0.15:        # 目标窄(远) → 前进靠近
                vx = 0.2
            elif width_ratio < 0.3:       # 中等距离 → 慢速
                vx = 0.1
            elif width_ratio > 0.5:       # 目标宽(近) → 停下只转向, 不后退
                vx = 0.0
        return {**loc, "vx": vx, "vyaw": vyaw}

    def get_detection_list(self):
        """Return detections for the page-side detection list."""
        with self._lock:
            dets = []
            for source in self._detection_source_order:
                dets.extend(dict(d) for d in self._latest_detection_overlays.get(source, []))
            if not dets:
                dets = [dict(d) for d in (self._latest_detection_overlay or self._latest_dets)]
            dets.extend(dict(d) for d in self._latest_locate_dets)
        return dets[:8]

    def submit_external_frame(self, frame, source="c13_vis"):
        """Queue the latest non-dog camera frame for the YOLO loop.

        The broadcast loop calls this with C13 and other camera frames. The YOLO
        thread consumes the newest frame per source, so slow inference drops
        stale frames instead of blocking video streaming.
        """
        if frame is None:
            return 0
        try:
            copied = frame.copy()
        except Exception:
            copied = frame
        source = source or "external"
        with self._lock:
            seq = self._external_seq_by_source.get(source, 0) + 1
            self._external_seq_by_source[source] = seq
            self._external_frames[source] = copied
            if source not in self._external_source_order:
                self._external_source_order.append(source)
            return seq

    def _take_external_frame(self):
        with self._lock:
            sources = list(self._external_source_order)
            if not sources:
                return None
            start = self._external_rr_index % len(sources)
            selected = None
            for offset in range(len(sources)):
                idx = (start + offset) % len(sources)
                source = sources[idx]
                seq = self._external_seq_by_source.get(source, 0)
                processed = self._processed_external_seq_by_source.get(source, 0)
                frame = self._external_frames.get(source)
                if frame is None or seq == processed:
                    continue
                self._processed_external_seq_by_source[source] = seq
                self._external_rr_index = (idx + 1) % len(sources)
                selected = (frame, source)
                break
            if selected is None:
                return None
            frame, source = selected
        try:
            frame = frame.copy()
        except Exception:
            pass
        return frame, source

    @staticmethod
    def _clean_bbox(bbox, frame_w, frame_h):
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
            return None
        try:
            x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        except (TypeError, ValueError):
            return None
        frame_w = max(1.0, float(frame_w))
        frame_h = max(1.0, float(frame_h))
        x1 = max(0.0, min(frame_w, x1))
        y1 = max(0.0, min(frame_h, y1))
        x2 = max(0.0, min(frame_w, x2))
        y2 = max(0.0, min(frame_h, y2))
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        return [x1, y1, x2, y2]

    @staticmethod
    def _encode_jpeg(frame, quality=68):
        if frame is None:
            return None
        try:
            import cv2
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
            return jpeg.tobytes() if ok else None
        except Exception as e:
            logger.debug(f"[AI] jpeg encode failed: {e}")
            return None

    def _cache_detection_result(self, raw_frame, display_frame, dets, source,
                                detect_frame_w=None, detect_frame_h=None,
                                target_classes=None):
        """Store the latest detection frame, overlay payload, and JPEG snapshots."""
        if raw_frame is None:
            return
        if display_frame is None:
            display_frame = raw_frame
        try:
            raw_h, raw_w = raw_frame.shape[:2]
        except Exception:
            raw_h, raw_w = 720, 1280
        detect_frame_w = int(detect_frame_w or raw_w or 1280)
        detect_frame_h = int(detect_frame_h or raw_h or 720)
        source = source or "video"
        now = time.time()

        frame_bytes = self._encode_jpeg(display_frame, quality=68)
        normalized = []
        snapshots = {}

        with self._lock:
            self._detection_seq += 1
            seq = self._detection_seq

        for idx, det in enumerate(_filter_confident_detections(dets)[:16]):
            if not isinstance(det, dict):
                continue
            bbox = self._clean_bbox(det.get("bbox"), detect_frame_w, detect_frame_h)
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            ix1 = max(0, min(raw_w - 1, int(math.floor(x1))))
            iy1 = max(0, min(raw_h - 1, int(math.floor(y1))))
            ix2 = max(ix1 + 1, min(raw_w, int(math.ceil(x2))))
            iy2 = max(iy1 + 1, min(raw_h, int(math.ceil(y2))))
            crop = raw_frame[iy1:iy2, ix1:ix2]
            crop_bytes = self._encode_jpeg(crop, quality=72) or frame_bytes
            snapshot_id = f"{source}-{seq}-{idx}"
            if crop_bytes or frame_bytes:
                snapshots[snapshot_id] = {
                    "crop": crop_bytes or frame_bytes,
                    "frame": frame_bytes or crop_bytes,
                    "ts": now,
                }
            confidence = det.get("confidence", det.get("score", det.get("probability", 0.0)))
            try:
                confidence = float(confidence)
            except (TypeError, ValueError):
                confidence = 0.0
            obs = {
                "id": snapshot_id,
                "snapshot_id": snapshot_id,
                "source": source,
                "class": det.get("class", det.get("label", det.get("name", "?"))),
                "label": det.get("label", det.get("class", det.get("name", "?"))),
                "confidence": confidence,
                "bbox": bbox,
                "frame_width": detect_frame_w,
                "frame_height": detect_frame_h,
                "ts": now,
            }
            if det.get("label_zh"):
                obs["label_zh"] = det.get("label_zh")
            if snapshot_id in snapshots:
                obs["crop_url"] = f"/api/detection_snapshot?id={snapshot_id}&kind=crop"
                obs["frame_url"] = f"/api/detection_snapshot?id={snapshot_id}&kind=frame"
            normalized.append(obs)

        with self._lock:
            self._latest_frame = display_frame
            self._latest_dets = normalized
            self._latest_detection_overlay = normalized
            self._latest_detection_source = source
            self._latest_detection_overlays[source] = normalized
            if source not in self._detection_source_order:
                self._detection_source_order.append(source)
            if frame_bytes:
                self._latest_source_frame_jpegs[source] = frame_bytes
            self._latest_source_frame_meta[source] = {
                "frame_width": detect_frame_w,
                "frame_height": detect_frame_h,
                "ts": now,
                "target_classes": (
                    None if target_classes is None else list(target_classes)
                ),
            }
            self._detect_frame_w = detect_frame_w
            self._detect_frame_h = detect_frame_h
            self._frame_count += 1
            for snapshot_id, payload in snapshots.items():
                self._detection_snapshots[snapshot_id] = payload
                self._detection_snapshot_order.append(snapshot_id)
            while len(self._detection_snapshot_order) > _DETECTION_SNAPSHOT_MAX:
                old_id = self._detection_snapshot_order.pop(0)
                self._detection_snapshots.pop(old_id, None)

    @staticmethod
    def _detection_stream_payload(source, dets, frame_meta=None):
        dets = [dict(d) for d in dets]
        frame_w = 0
        frame_h = 0
        if dets:
            try:
                frame_w = int(dets[0].get("frame_width") or 0)
                frame_h = int(dets[0].get("frame_height") or 0)
            except Exception:
                frame_w, frame_h = 0, 0
        elif frame_meta:
            try:
                frame_w = int(frame_meta.get("frame_width") or 0)
                frame_h = int(frame_meta.get("frame_height") or 0)
            except Exception:
                frame_w, frame_h = 0, 0
        return {
            "source": source,
            "frame_width": frame_w,
            "frame_height": frame_h,
            "count": len(dets),
            "detections": dets,
        }

    def get_detection_overlay(self):
        with self._lock:
            dets = [dict(d) for d in self._latest_detection_overlay]
            return {
                "source": self._latest_detection_source,
                "frame_width": int(self._detect_frame_w or 0),
                "frame_height": int(self._detect_frame_h or 0),
                "count": len(dets),
                "detections": dets,
            }

    def get_detection_overlays(self):
        with self._lock:
            streams = []
            flat = []
            for source in self._detection_source_order:
                dets = [dict(d) for d in self._latest_detection_overlays.get(source, [])]
                streams.append(self._detection_stream_payload(
                    source,
                    dets,
                    self._latest_source_frame_meta.get(source),
                ))
                flat.extend(dets)
            if not streams and self._latest_detection_overlay:
                dets = [dict(d) for d in self._latest_detection_overlay]
                streams.append(self._detection_stream_payload(
                    self._latest_detection_source,
                    dets,
                    self._latest_source_frame_meta.get(self._latest_detection_source),
                ))
                flat.extend(dets)
            return {
                "source": self._latest_detection_source,
                "count": len(flat),
                "detections": flat,
                "streams": streams,
            }

    def get_video_frame_jpeg(self, source):
        if not source:
            return None
        with self._lock:
            return self._latest_source_frame_jpegs.get(str(source))

    def get_detection_snapshot_jpeg(self, snapshot_id, kind="crop"):
        if not snapshot_id:
            return None
        key = "frame" if kind == "frame" else "crop"
        with self._lock:
            payload = self._detection_snapshots.get(str(snapshot_id))
            if not payload:
                return None
            return payload.get(key) or payload.get("crop") or payload.get("frame")

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

    def _get_next_detection_frame(self, min_interval):
        source_kinds = ["external"]
        if _AI_VIDEO_ENABLED:
            source_kinds.append("dog")
        if not source_kinds:
            time.sleep(min_interval)
            return None

        start = self._detection_input_rr_index % len(source_kinds)
        for offset in range(len(source_kinds)):
            idx = (start + offset) % len(source_kinds)
            kind = source_kinds[idx]
            if kind == "external":
                external = self._take_external_frame()
                if external is not None:
                    self._detection_input_rr_index = (idx + 1) % len(source_kinds)
                    return external
            elif kind == "dog":
                frame = self._get_frame()
                if frame is not None:
                    self._detection_input_rr_index = (idx + 1) % len(source_kinds)
                    source = "mock" if self._mock_mode else "dog"
                    return frame, source

        time.sleep(min_interval)
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
        if _AI_VIDEO_ENABLED:
            self._init_video()
        self._init_detector()

        # 目标 8fps 节流 (对齐取帧频率, 决策 3)
        target_fps = float(os.environ.get("GO2W_VIDEO_FPS", "8"))
        min_interval = 1.0 / target_fps if target_fps > 0 else 0.125

        while self._running:
            t0 = time.time()
            try:
                frame_source = self._get_next_detection_frame(min_interval)
                if frame_source is None:
                    continue
                frame, source = frame_source
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
                raw_frame = frame.copy()
                detect_frame_w = frame.shape[1]
                detect_frame_h = frame.shape[0]
                dets = self._run_detector(frame)
                with self._lock:
                    inference_target_classes = (
                        None if self._last_detector_target_classes is None
                        else list(self._last_detector_target_classes)
                    )
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
                    self._detect_frame_h = detect_frame_h
                    self._frame_count += 1
                self._cache_detection_result(
                    raw_frame,
                    frame,
                    dets,
                    source,
                    detect_frame_w,
                    detect_frame_h,
                    target_classes=inference_target_classes,
                )
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
        允许 _vlm_worker 节流重试 (HIGH-1: 模型就位后能自愈)。

        round-3 graceful: NX 上没装 transformers 时, VLMEngine 构造会抛
        (ai.vlm 内部懒 import transformers), 落入下面 except → _vlm=None。
        VLM 不可用时返回带 parse_error 的空任务，不崩、不退出，也不影响视频流与检测。
        """
        if self._vlm_disabled:
            # GO2W_VLM_DISABLED=1: 标记已初始化阻止节流重试, _vlm 永远 None,
            # NxAiVlmProxy.loaded=False 让 TaskManager 不走 vlm 路径.
            self._vlm_inited = True
            return
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
            logger.error(f"[AI] 导入/构造 VLMEngine 失败 ({e}), VLM 不可用 ("
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
            # 本轮待处理的 result_box (异常时据此回写空任务并 set done)
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
                        failure = self._parse_failure("vlm_load_failed", text=text)
                        self._safe_broadcast({"type": "vlm", "data": {
                            "text": text, "response": failure["response"],
                            "tasks": [], "parse_error": failure["parse_error"]}})
                        result_box["response"] = failure
                        result_box["done"].set()
                        continue
                    logger.info(f"[VLM] 就绪 (load 用时 {time.time()-t0:.1f}s)")

                if self._vlm is None or not getattr(self._vlm, "loaded", False):
                    # VLM 不可用时必须失败关闭，不能合成任何运动任务。
                    failure = self._parse_failure("vlm_unavailable", text=text)
                    self._safe_broadcast({"type": "vlm", "data": {
                        "text": text, "response": failure["response"],
                        "tasks": [], "parse_error": failure["parse_error"]}})
                    result_box["response"] = failure
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
                # 若已取到本轮请求的 result_box，回写空任务并 set done，
                # 否则调用线程 (NxAiVlmProxy.chat) 会 wait(120) 卡满超时。
                if pending_box is not None and not pending_box["done"].is_set():
                    try:
                        failure = self._parse_failure(
                            "vlm_worker_error",
                            text=pending_box.get("text", ""),
                        )
                        self._safe_broadcast({"type": "vlm", "data": {
                            "text": pending_box.get("text", ""),
                            "response": failure["response"],
                            "tasks": [], "parse_error": failure["parse_error"]}})
                        pending_box["response"] = failure
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

    _SYS_PROMPT = """你是机器狗搜索任务解析器。只把用户的搜索指令转换成一个 JSON 任务。

唯一允许的任务类型和参数:
- search_room: {"room":"房间名或__current__(当前房间)", "target_classes":["英文视觉类别"], "require_photos":true, "mark_on_map":true}

target_classes 必须是非空的英文视觉类别数组，例如 person、dining table、chair。
无法确定房间、目标类别或搜索意图时输出 {"tasks":[]}。

示例:
输入"搜索这个房间，把椅子标注出来"
输出: {"tasks":[{"type":"search_room","priority":8,"params":{"room":"__current__","target_classes":["chair"],"require_photos":true,"mark_on_map":true}}]}

输入"去客厅搜索所有人并标在地图上"
输出: {"tasks":[{"type":"search_room","priority":8,"params":{"room":"客厅","target_classes":["person"],"require_photos":true,"mark_on_map":true}}]}

只输出 JSON，不要解释，不要 markdown 代码块。"""

    @staticmethod
    def _parse_failure(reason, *, text=""):
        """Return a user-visible, fail-closed command result."""
        return {
            "understanding": str(text or ""),
            "response": "搜索任务解析失败",
            "tasks": [],
            "parse_error": str(reason or "invalid_vlm_mission"),
        }

    @staticmethod
    def _validate_vlm_search_result(data):
        """Validate one VLM result into the canonical search mission schema."""
        try:
            if not isinstance(data, dict):
                raise MissionValidationError("VLM result must be an object")
            return {
                "response": str(data.get("response") or "已解析搜索任务"),
                "tasks": canonicalize_search_tasks(data.get("tasks")),
            }
        except (MissionValidationError, TypeError, ValueError, OverflowError) as exc:
            logger.warning("VLM 搜索任务校验失败: %s", exc)
            return NxAiEngine._parse_failure("invalid_vlm_mission")

    def _vlm_parse_command(self, text):
        """Parse and validate exactly one canonical search-room mission."""
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
                    return self._validate_vlm_search_result(data)
            except Exception as e:
                logger.warning(f"VLM JSON 解析失败: {e}")
        except Exception as e:
            logger.error(f"VLM chat 失败: {e}")
            return self._parse_failure("vlm_unavailable", text=text)
        return self._parse_failure("invalid_vlm_mission", text=text)

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

    def get_frame_detection_count(self):
        """Return the latest detection count without JPEG encoding."""
        with self._lock:
            frame = self._latest_frame
            if self._latest_detection_overlays:
                count = sum(len(self._latest_detection_overlays.get(source, []))
                            for source in self._detection_source_order)
            else:
                count = len(self._latest_dets)
        if frame is None:
            return None
        return count

    def get_person_detection_snapshot(self):
        """Return the latest YOLO person detections with a copied frame."""
        return self.get_detection_snapshot(["person"])

    def get_detection_snapshot(self, target_classes=None):
        """Return a copied frame with detections filtered to requested classes."""
        normalized_targets = []
        for target in target_classes or []:
            value = str(target or "").strip()
            if value and value not in normalized_targets:
                normalized_targets.append(value)
        allowed = set(normalized_targets)
        with self._lock:
            source = str(self._latest_detection_source or "")
            frame_meta = dict(self._latest_source_frame_meta.get(source) or {})
            vocabulary_tracked = "target_classes" in frame_meta
            inference_target_classes = frame_meta.get("target_classes")
            if inference_target_classes is not None:
                inference_target_classes = [
                    str(value) for value in inference_target_classes]
            target_vocabulary_ready = (
                not normalized_targets
                or not vocabulary_tracked
                or inference_target_classes == normalized_targets
            )
            try:
                captured_at = float(frame_meta.get("ts") or 0.0)
            except (TypeError, ValueError):
                captured_at = 0.0
            if not target_vocabulary_ready:
                captured_at = 0.0
            if self._latest_frame is None:
                return {
                    "frame": None,
                    "frame_width": 0,
                    "frame_height": 0,
                    "source": source,
                    "timestamp": captured_at,
                    "detections": [],
                    "target_classes": normalized_targets,
                    "inference_target_classes": inference_target_classes,
                    "target_vocabulary_ready": target_vocabulary_ready,
                }
            frame = self._latest_frame.copy()
            height, width = frame.shape[:2]
            detect_frame_w = getattr(self, "_detect_frame_w", width)
            detect_frame_h = getattr(self, "_detect_frame_h", height)
            detections = []
            for det in self._latest_dets if target_vocabulary_ready else []:
                if allowed and det.get("class") not in allowed:
                    continue
                copied = dict(det)
                bbox = copied.get("bbox")
                if isinstance(bbox, (list, tuple)):
                    scaled_bbox = list(bbox)
                    if len(scaled_bbox) >= 4:
                        src_w = copied.get("frame_width") or detect_frame_w
                        src_h = copied.get("frame_height") or detect_frame_h
                        try:
                            src_w = float(src_w)
                            if src_w > 0 and src_w != width:
                                x_scale = float(width) / src_w
                                scaled_bbox[0] = float(scaled_bbox[0]) * x_scale
                                scaled_bbox[2] = float(scaled_bbox[2]) * x_scale
                        except (TypeError, ValueError):
                            pass
                        try:
                            src_h = float(src_h)
                            if src_h > 0 and src_h != height:
                                y_scale = float(height) / src_h
                                scaled_bbox[1] = float(scaled_bbox[1]) * y_scale
                                scaled_bbox[3] = float(scaled_bbox[3]) * y_scale
                        except (TypeError, ValueError):
                            pass
                    copied["bbox"] = scaled_bbox
                copied["frame_width"] = int(width)
                copied["frame_height"] = int(height)
                copied["source"] = "yolo"
                detections.append(copied)
        return {
            "frame": frame,
            "frame_width": int(width),
            "frame_height": int(height),
            "source": source,
            "timestamp": captured_at,
            "detections": detections,
            "target_classes": normalized_targets,
            "inference_target_classes": inference_target_classes,
            "target_vocabulary_ready": target_vocabulary_ready,
        }

    def get_person_detection_health(self, max_age_sec=None):
        """Return lightweight detector/frame health without copying image data."""
        try:
            max_age_sec = float(
                max_age_sec if max_age_sec is not None
                else os.environ.get("GO2W_DETECTION_MAX_AGE_SEC", "2.0"))
        except (TypeError, ValueError):
            max_age_sec = 2.0
        if not math.isfinite(max_age_sec) or max_age_sec <= 0.0:
            max_age_sec = 2.0

        with self._lock:
            running = bool(self._running)
            detector_initialized = bool(self._detector_inited)
            detector = self._detector
            detector_ready = detector is not None
            detector_open_vocabulary = bool(
                getattr(detector, "is_world", False))
            detector_model = str(
                getattr(detector, "_model_path", "") or "")
            source = str(self._latest_detection_source or "")
            frame_meta = dict(self._latest_source_frame_meta.get(source) or {})
            frame = self._latest_frame
            detections = list(self._latest_dets)
            if frame is not None:
                try:
                    frame_height, frame_width = frame.shape[:2]
                except Exception:
                    frame_width = frame_height = 0
            else:
                frame_width = frame_height = 0
            if frame_width <= 0:
                try:
                    frame_width = int(frame_meta.get("frame_width") or 0)
                except (TypeError, ValueError):
                    frame_width = 0
            if frame_height <= 0:
                try:
                    frame_height = int(frame_meta.get("frame_height") or 0)
                except (TypeError, ValueError):
                    frame_height = 0
            try:
                captured_at = float(frame_meta.get("ts") or 0.0)
            except (TypeError, ValueError):
                captured_at = 0.0

        now = time.time()
        timestamp_valid = math.isfinite(captured_at) and captured_at > 0.0
        age_sec = now - captured_at if timestamp_valid else None
        frame_available = frame is not None and frame_width > 0 and frame_height > 0
        person_count = sum(
            1 for detection in detections
            if isinstance(detection, dict) and detection.get("class") == "person"
        )

        if not running:
            reason = "ai_engine_stopped"
        elif not detector_ready:
            reason = "detector_not_ready"
        elif not frame_available:
            reason = "no_detection_frame"
        elif not source:
            reason = "no_detection_source"
        elif source.strip().lower() == "mock":
            reason = "mock_detection_source"
        elif age_sec is None:
            reason = "invalid_detection_timestamp"
        elif not math.isfinite(age_sec) or age_sec < -0.5 or age_sec > max_age_sec:
            reason = "stale_detection_frame"
        else:
            reason = "ok"

        return {
            "healthy": reason == "ok",
            "reason": reason,
            "running": running,
            "detector_initialized": detector_initialized,
            "detector_ready": detector_ready,
            "detector_open_vocabulary": detector_open_vocabulary,
            "detector_model": detector_model,
            "frame_available": frame_available,
            "source": source,
            "timestamp": captured_at,
            "age_sec": age_sec,
            "max_age_sec": max_age_sec,
            "frame_width": int(frame_width),
            "frame_height": int(frame_height),
            "detection_count": len(detections),
            "person_count": person_count,
        }

    @staticmethod
    def _range_for_bearing(ranges, bearing):
        if not ranges:
            return None
        try:
            vals = list(ranges)
        except Exception:
            return None
        n = len(vals)
        if n <= 0:
            return None
        bearing = (float(bearing) + math.pi) % (2.0 * math.pi) - math.pi
        step = (2.0 * math.pi) / float(n)
        center = int(round((bearing + math.pi) / step))
        center = max(0, min(n - 1, center))
        radius = max(1, int(round(math.radians(4.0) / step)))
        candidates = []
        for idx in range(max(0, center - radius), min(n, center + radius + 1)):
            try:
                r = float(vals[idx])
            except (TypeError, ValueError):
                continue
            if 0.08 < r < 20.0 and math.isfinite(r):
                candidates.append(r)
        return min(candidates) if candidates else None

    @staticmethod
    def _range_for_livox_points(points, bearing):
        if not points:
            return None
        try:
            bearing = float(bearing)
        except (TypeError, ValueError):
            return None
        window = math.radians(5.0)
        candidates = []
        for pt in points:
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                continue
            try:
                x = float(pt[0])
                y = float(pt[1])
            except (TypeError, ValueError):
                continue
            dist = math.hypot(x, y)
            if not (0.08 < dist < 20.0 and math.isfinite(dist)):
                continue
            point_bearing = math.atan2(-y, x)  # Livox y is left; bbox positive is image-right.
            delta = (point_bearing - bearing + math.pi) % (2.0 * math.pi) - math.pi
            if abs(delta) <= window:
                candidates.append(dist)
        return min(candidates) if candidates else None

    def get_detections_world(self, robot_x, robot_y, robot_yaw, ranges=None, lidar_points=None):
        """broadcast_loop 调用: 返回 slam.data.detections 格式 [{x,y,class}] (C1.5 数组!)。
        bbox 中心 x 归一化 → 方位角 (假设 FOV=70°)。只有 LaserScan 或
        Livox 提供可靠距离时才发布地图坐标；纯单目方位不会伪造固定距离。
        与 type=frame 的整数 detections 相反, slam 这里必须是数组。

        MEDIUM-5: 归一化用检测时刻的帧宽 _detect_frame_w (bbox 坐标系),
        不用 resize 后的 _latest_frame.shape[1] (720p=1280), 否则方位系统偏左。
        """
        with self._lock:
            base_dets = []
            for source in self._detection_source_order:
                if source == "mock":
                    continue
                base_dets.extend(dict(d) for d in self._latest_detection_overlays.get(source, []))
            if not base_dets:
                base_dets = list(self._latest_detection_overlay) if self._latest_detection_overlay else list(self._latest_dets)
            locate_dets = list(self._latest_locate_dets)
            default_w = self._detect_frame_w if self._detect_frame_w > 0 else 1280
            locate_w = self._latest_locate_frame_w if self._latest_locate_frame_w > 0 else default_w
        dets = []
        for d in base_dets:
            if isinstance(d, dict):
                if d.get("source") == "mock":
                    continue
                dets.append((dict(d), int(d.get("frame_width") or default_w)))
        for d in locate_dets:
            if isinstance(d, dict):
                dets.append((dict(d), int(d.get("frame_width") or locate_w)))
        if not dets:
            return []
        try:
            max_detection_age = float(
                os.environ.get("GO2W_DETECTION_MAX_AGE_SEC", "2.0"))
        except (TypeError, ValueError):
            max_detection_age = 2.0
        if (not math.isfinite(max_detection_age)
                or max_detection_age <= 0.0):
            max_detection_age = 2.0
        now = time.time()
        half_fov = math.radians(_CAMERA_HFOV_DEG / 2.0)
        yaw_offset = math.radians(_CAMERA_YAW_OFFSET_DEG)
        out = []
        for d, fw in dets:
            try:
                captured_at = float(d.get("ts"))
                frame_age = now - captured_at
                if (not math.isfinite(captured_at) or captured_at <= 0.0
                        or not math.isfinite(frame_age) or frame_age < -0.5
                        or frame_age > max_detection_age):
                    continue
                bbox = d.get("bbox", [0, 0, fw, 0])
                if not bbox or len(bbox) < 4:
                    continue
                x1, _y1, x2, _y2 = bbox[:4]
                cx_px = (float(x1) + float(x2)) / 2.0
                cx_norm = cx_px / float(fw) if fw > 0 else 0.5
                angle = (cx_norm - 0.5) * 2.0 * half_fov + yaw_offset
                lidar_range = self._range_for_bearing(ranges, angle)
                range_source = "lidar" if lidar_range is not None else None
                if lidar_range is None:
                    lidar_range = self._range_for_livox_points(lidar_points, angle)
                    if lidar_range is not None:
                        range_source = "livox"
                # A monocular bbox has bearing but no trustworthy map range.
                # Never publish a fabricated fixed-distance marker: mission
                # localization will keep it unresolved and retry another view.
                if lidar_range is None:
                    continue
                dist = lidar_range
                wx = robot_x + dist * math.cos(robot_yaw + angle)
                wy = robot_y + dist * math.sin(robot_yaw + angle)
                out.append({
                    "x": round(wx, 2),
                    "y": round(wy, 2),
                    "class": d.get("class", "?"),
                    "confidence": d.get("confidence", 0.0),
                    "source": d.get("source", "yolo"),
                    "range": round(float(dist), 2),
                    "range_source": range_source,
                    "snapshot_id": d.get("snapshot_id") or d.get("id"),
                    "crop_url": d.get("crop_url", ""),
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
        # GO2W_VLM_DISABLED=1 时报 False, 让 TaskManager 走 _parse_product_command
        # 确定性路径 (nx_web_server.py:1763), 不入队 vlm_worker (省线程 + 省 3-4GB 模型内存).
        if getattr(self._ai, "_vlm_disabled", False):
            return False
        # 否则恒 True: TaskManager._process_command_bg 据此走 _vlm_parse_command
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
            # 超时必须返回合法的失败关闭 JSON；迟到结果不得再被消费。
            logger.warning("[VLM] proxy 等结果超时 (120s)")
            failure = self._ai._parse_failure("vlm_timeout", text=text)
            result_box["done"].set()
            return json.dumps(failure, ensure_ascii=False)
        result = result_box.get("response")
        if result is None:
            logger.warning("[VLM] proxy 收到空结果")
            failure = self._ai._parse_failure("vlm_no_result", text=text)
            return json.dumps(failure, ensure_ascii=False)
        # TaskManager._vlm_parse_command 期望 chat 返回字符串 (vlm.chat 的契约)
        # 把解析后的 dict 重新序列化成 JSON 字符串, 让 TaskManager 再 parse 一次 (保持契约)
        try:
            return json.dumps(result, ensure_ascii=False)
        except Exception:
            return '{"tasks":[],"response":"' + str(result.get("response", "")) + '"}'

    def locate(self, image, target_description):
        """TargetTracker-compatible locate() using LocateAnything first."""
        if hasattr(self._ai, "locate_target"):
            return self._ai.locate_target(image, target_description)
        return {"found": False, "bbox": None, "cx": 0, "cy": 0,
                "description": "locate unavailable"}

    def track_target(self, image, target_description, img_w=640, img_h=480):
        """TargetTracker-compatible track_target() using LocateAnything first."""
        if hasattr(self._ai, "track_target"):
            return self._ai.track_target(image, target_description, img_w=img_w, img_h=img_h)
        return {"found": False, "bbox": None, "cx": 0, "cy": 0,
                "vx": 0.0, "vyaw": 0.0, "description": "track unavailable"}


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
