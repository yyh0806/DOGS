#!/usr/bin/env python3
"""C13 云台 RTSP 双流桥接 (组件, 注入 nx_web_server 同进程)。

拉 Skydroid C13 的可见光 + 红外两路 RTSP (cv2.VideoCapture), 缓存最新 jpeg base64,
广播线程合并成一条 type=gimbal WS 消息推前端 (vis+ir 一起, 省一半 WS 往返)。

线程: daemon ×2 = _capture_loop (vis/ir) + daemon ×1 = _broadcast_loop。
配置 (环境变量, 默认对应 C13 出厂 192.168.144.108):
  C13_VIS_URL / C13_IR_URL / C13_FPS(30) / C13_JPEG_Q(38) / C13_ENABLE(1) / C13_BACKEND(gst=NVDEC硬解)
红线: 懒加载 cv2 (缺失则禁用, 不崩主服务); 异常只 warning 不抛; 断流自动重连;
      不动 nx_ai_node 的 VideoClient/YOLO 路径 (type=frame 与 type=gimbal 并存)。
"""
import base64
import logging
import os
import threading
import time

import numpy as np

logger = logging.getLogger("go2w.gimbal")

_VIS_URL = os.environ.get("C13_VIS_URL", "rtsp://192.168.144.108:554/stream=1")
_IR_URL = os.environ.get("C13_IR_URL", "rtsp://192.168.144.108:555/stream=2")
# 默认 gst = Jetson NVDEC 硬解 (nvv4l2decoder), 30fps 对齐 C13 源帧率。
# 不再依赖 service Environment 注入 — 手动启 / 旧 service / env 漏了也走硬解,
# 治 fps 回归 (service 未重部署时默认 12 + ffmpeg 软解 → 前端 ~13fps 卡顿)。
# gi.Gst 不可用时, 顶部 _BACKEND in ("gst",..) 分支会 warning + _open 自动降级 ffmpeg。
_BACKEND = os.environ.get("C13_BACKEND", "gst").strip().lower()
_FPS = max(1.0, float(os.environ.get("C13_FPS", "30")))
_JPEG_Q = int(os.environ.get("C13_JPEG_Q", "38"))
_DROP_GRABS = max(0, int(os.environ.get("C13_DROP_GRABS", "0")))  # M2 fix: 默认 0 (FFmpeg 已 nobuffer+max_delay, grab-drop 反增延迟降帧率; 需要时 export C13_DROP_GRABS=2)
_VIS_WIDTH = max(1, int(os.environ.get("C13_VIS_WIDTH", "640")))  # M3: 480→640 提 locate bbox 精度 (grounding 模型低分辨率 bbox 粗略; 带宽代价可接受)
_IR_WIDTH = max(1, int(os.environ.get("C13_IR_WIDTH", "256")))
_VIS_HEIGHT = max(1, int(os.environ.get("C13_VIS_HEIGHT", str(round(_VIS_WIDTH * 9 / 16)))))
_IR_HEIGHT = max(1, int(os.environ.get("C13_IR_HEIGHT", str(round(_IR_WIDTH * 0.8)))))
_ENABLED = os.environ.get("C13_ENABLE", "1").strip() not in ("0", "false", "False", "no")

# FFmpeg 低延迟选项 (治 RTSP 缓冲堆积根因, 方案 A):
# cv2.VideoCapture(CAP_FFMPEG) 内部解码线程以源帧率(~30fps)持续收包入 FIFO, read() 取
# 最老帧; 消费速率 < 源帧率时队列单向堆积 → 延迟随运行时间线性增长。
# 本变量在 VideoCapture 构造时被 FFmpeg 后端读进 AVDictionary (avformat_open_input),
# 虽然读取发生在 _open() 而非 import cv2, 但本文件顶层就 import cv2, 故在此 setdefault
# 最保险且意图清晰。setdefault: 启动前 export 可覆盖做 A/B 对比 (例换 udp)。
#   rtsp_transport=tcp : 避免 UDP 乱序触发的 reorder_queue 等待 (HOL blocking 换稳定);
#   fflags=nobuffer    : demuxer 层不缓冲包;
#   flags=low_delay    : 解码器低延迟模式;
#   max_delay=500000   : 500ms 延迟上限 (微秒)。
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;500000",
)

try:
    import cv2  # noqa: F401
    _CV2_OK = True
except Exception as _e:
    _CV2_OK = False
    logger.warning(f"[gimbal] cv2 不可导入 ({_e}), C13 双流桥接禁用")

_GST_OK = False
Gst = None
if _BACKEND in ("auto", "gst", "gstreamer"):
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst as _Gst
        _Gst.init(None)
        Gst = _Gst
        _GST_OK = True
    except Exception as _e:
        if _BACKEND in ("gst", "gstreamer"):
            logger.warning(f"[gimbal] GStreamer 不可用 ({_e}), C13 gst 后端禁用")


def _resize_for_ws(frame, max_width):
    try:
        h, w = frame.shape[:2]
        if w <= max_width:
            return frame
        scale = float(max_width) / float(w)
        return cv2.resize(frame, (int(max_width), max(1, int(h * scale))))
    except Exception:
        return frame


class _GstRtspCapture:
    """VideoCapture-like wrapper around Jetson GStreamer hardware decode."""

    def __init__(self, url, name, target_width=None, target_height=None):
        self._url = url
        self._name = name
        self._target_width = target_width
        self._target_height = target_height
        self._pipeline = None
        self._sink = None
        self._opened = False
        self._open()

    def _open(self):
        if not _GST_OK or Gst is None:
            return
        scale_caps = "video/x-raw,format=BGRx"
        if self._target_width and self._target_height:
            scale_caps = (
                "video/x-raw,format=BGRx,"
                f"width={int(self._target_width)},height={int(self._target_height)}"
            )
        pipeline_desc = (
            f"rtspsrc location={self._url} latency=0 protocols=tcp drop-on-latency=true ! "
            "rtph265depay ! h265parse ! "
            "nvv4l2decoder enable-max-performance=1 ! "
            f"nvvidconv ! {scale_caps} ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
        )
        try:
            self._pipeline = Gst.parse_launch(pipeline_desc)
            self._sink = self._pipeline.get_by_name("sink")
            self._pipeline.set_state(Gst.State.PLAYING)
            ret, _, _ = self._pipeline.get_state(5 * Gst.SECOND)
            self._opened = ret != Gst.StateChangeReturn.FAILURE and self._sink is not None
        except Exception as e:
            logger.warning(f"[gimbal] {self._name} gst 打开失败: {e}")
            self.release()

    def isOpened(self):
        return self._opened

    def grab(self):
        return True

    def read(self):
        if not self._opened or self._sink is None:
            return False, None
        try:
            sample = self._sink.emit("try-pull-sample", 250 * Gst.MSECOND)
            if sample is None:
                return False, None
            caps = sample.get_caps()
            struct = caps.get_structure(0)
            width = int(struct.get_value("width"))
            height = int(struct.get_value("height"))
            buf = sample.get_buffer()
            ok, map_info = buf.map(Gst.MapFlags.READ)
            if not ok:
                return False, None
            try:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
                return True, frame
            finally:
                buf.unmap(map_info)
        except Exception as e:
            logger.debug(f"[gimbal] {self._name} gst read 异常: {e}")
            return False, None

    def release(self):
        if self._pipeline is not None:
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
        self._opened = False
        self._pipeline = None
        self._sink = None


class GimbalRtspBridge:
    """C13 RTSP 双流 → WS type=gimbal。nx_web_server.main() 实例化并 start()。"""

    def __init__(self, ws_broadcast_fn):
        self._ws = ws_broadcast_fn
        self._lock = threading.Lock()
        self._vis_b64 = None
        self._ir_b64 = None
        self._vis_frame = None
        self._ir_frame = None
        self._running = False
        self._captures = []
        self._threads = []

    def _open(self, url, name, target_width=None, target_height=None):
        if _BACKEND in ("auto", "gst", "gstreamer") and _GST_OK:
            cap = _GstRtspCapture(url, name, target_width, target_height)
            if cap.isOpened():
                logger.info(f"[gimbal] {name} 已连接 {url} (backend=gst/nvv4l2decoder)")
                return cap
            if _BACKEND in ("gst", "gstreamer"):
                logger.warning(f"[gimbal] {name} gst 打不开 {url}")
                return None
        import cv2
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            logger.info(f"[gimbal] {name} 已连接 {url}")
            return cap
        logger.warning(f"[gimbal] {name} 打不开 {url}")
        return None

    def _capture_loop(self, url, name, attr, max_width=None, max_height=None):
        import cv2
        cap = None
        fail = 0
        interval = 1.0 / _FPS
        next_capture_t = 0.0
        while self._running:
            if cap is None or not cap.isOpened():
                cap = self._open(url, name, max_width, max_height)
                if cap is None:
                    time.sleep(2.0)
                    continue
                with self._lock:
                    self._captures.append(cap)
            now = time.monotonic()
            if now < next_capture_t:
                time.sleep(min(0.02, next_capture_t - now))
                continue
            next_capture_t = now + interval
            for _ in range(_DROP_GRABS):
                try:
                    if not cap.grab():
                        break
                except Exception:
                    break
            ret, frame = cap.read()
            if not ret or frame is None:
                fail += 1
                if fail > int(_FPS * 5):
                    logger.warning(f"[gimbal] {name} 连续 {fail} 帧失败, 重连")
                    try:
                        cap.release()
                    except Exception:
                        pass
                    with self._lock:
                        if cap in self._captures:
                            self._captures.remove(cap)
                    cap = None
                    fail = 0
                time.sleep(interval)
                continue
            fail = 0
            try:
                # M4: vis 存原始高清帧 (给 locate/跟随用; 实测 640x360 退化成全屏 person, 1280x720 才能分类);
                #     ws 广播单独 resize 到 _VIS_WIDTH 省带宽 (前端显示 640 够)。detection.frame_width=1280
                #     由前端 renderLocateOverlay (x1/frameW)*imgRect.width 自动映射到显示尺寸, 框对齐。
                if name == "vis":
                    with self._lock:
                        self._vis_frame = frame.copy()
                elif name == "ir":
                    with self._lock:
                        self._ir_frame = frame.copy()
                ws_frame = _resize_for_ws(frame, max_width) if max_width else frame
                ok, jpg = cv2.imencode('.jpg', ws_frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_Q])
                if ok:
                    with self._lock:
                        setattr(self, attr, base64.b64encode(jpg.tobytes()).decode())
            except Exception as e:
                logger.debug(f"[gimbal] {name} encode 异常: {e}")
        try:
            if cap is not None:
                cap.release()
                with self._lock:
                    if cap in self._captures:
                        self._captures.remove(cap)
        except Exception:
            pass

    def _broadcast_loop(self):
        interval = 1.0 / _FPS
        warmup = time.time() + 5.0
        warned = False
        while self._running:
            with self._lock:
                v, i = self._vis_b64, self._ir_b64
            if v or i:
                try:
                    self._ws({"type": "gimbal", "vis": v, "ir": i}, force=True)
                except Exception as e:
                    logger.debug(f"[gimbal] broadcast 异常: {e}")
            elif time.time() > warmup and not warned:
                logger.warning("[gimbal] 启动 5s 仍无帧, 检查 C13 是否上电/网线/网络")
                warned = True
            time.sleep(interval)

    def start(self):
        if not _CV2_OK:
            logger.warning("[gimbal] cv2 缺失, 不启动 C13 桥接")
            return
        self._running = True
        for args in [(_VIS_URL, "vis", "_vis_b64", _VIS_WIDTH, _VIS_HEIGHT),
                     (_IR_URL, "ir", "_ir_b64", _IR_WIDTH, _IR_HEIGHT)]:
            th = threading.Thread(target=self._capture_loop, args=args,
                                  name=f"gimbal_{args[1]}", daemon=True)
            th.start()
            self._threads.append(th)
        th = threading.Thread(target=self._broadcast_loop, name="gimbal_bc", daemon=True)
        th.start()
        self._threads.append(th)
        logger.info(f"[gimbal] C13 双流桥接启动 (vis={_VIS_URL}, ir={_IR_URL}, fps={_FPS})")

    def get_vis_frame(self):
        """Return the latest visible-light frame for grounding/follow tasks."""
        with self._lock:
            return self._vis_frame.copy() if self._vis_frame is not None else None

    def get_ir_frame(self):
        """Return the latest infrared frame for detection overlays."""
        with self._lock:
            return self._ir_frame.copy() if self._ir_frame is not None else None

    def stop(self):
        self._running = False
        with self._lock:
            captures = list(self._captures)
        for cap in captures:
            try:
                cap.release()
            except Exception:
                pass
        for th in list(self._threads):
            try:
                th.join(timeout=1.0)
            except Exception:
                pass


def is_enabled():
    """nx_web_server 顶层判断: 环境没关 + cv2 在。"""
    return _ENABLED and _CV2_OK
