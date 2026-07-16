#!/usr/bin/env python3
"""C13 可见光 RTSP → ROS sensor_msgs/Image 桥 (Fast-LIVO2 视觉输入源)。

独立进程 (bringup_livo.sh 用 systemd-run + python3 起, 跟 map_odom_fuser.py 同范式),
订阅无, 发布:
  /c13/image_raw    sensor_msgs/Image     (encoding=bgr8, NVDEC 硬解零拷贝)
  /c13/camera_info  sensor_msgs/CameraInfo (从 c13_intrinsic.yaml 加载, 标定后覆盖)

为何独立于 nx_gimbal_node:
  - gimbal 节点把帧 base64 推 web, **不发 ROS topic**, 是 LIVO 缺的那一环。
  - 复用其 NVDEC pipeline (rtspsrc latency=0 + nvv4l2decoder + appsink drop=true max-buffers=1),
    但包成标准 ROS publisher, 让 Fast-LIVO2 直接订阅。
  - LIVO bringup 时 web 未必起, 独立进程解耦。

时间戳红线 (LIVO 时延敏感):
  header.stamp = 拉到 Gst sample 瞬间的 clock.now(), **不是发布时刻**。
  RTSP 传输+解码延迟 (50-150ms) 由 Fast-LIVO2 的时间偏移估计器吸收;
  若用发布时刻, 延迟会被解释成位姿漂移, 视觉约束拉坏状态。

线程: daemon ×1 = _capture_thread (NVDEC 拉 frame → 发 Image/CameraInfo)。
配置 (环境变量, 默认对应 C13 出厂 192.168.144.108 + LIVO 期望 topic):
  C13_VIS_URL / C13_IMAGE_TOPIC(/c13/image_raw) / C13_CAMERA_INFO_TOPIC(/c13/camera_info)
  C13_FRAME_ID(c13_optical) / C13_INTRINSIC_YAML(可选, 缺则发占位 CameraInfo)
  C13_FPS(30) / C13_BACKEND(gst=NVDEC硬解) / C13_IMAGE_WIDTH(1280) / C13_IMAGE_HEIGHT(720)
红线: 懒加载 cv2/gi (缺失禁用不崩); 异常只 warning 不抛; 断流自动重连。
"""
import logging
import os
import threading
import time

import numpy as np

logger = logging.getLogger("go2w.c13img")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s %(message)s")

_VIS_URL = os.environ.get("C13_VIS_URL", "rtsp://192.168.144.108:554/stream=1")
_IMAGE_TOPIC = os.environ.get("C13_IMAGE_TOPIC", "/c13/image_raw")
_CAMINFO_TOPIC = os.environ.get("C13_CAMERA_INFO_TOPIC", "/c13/camera_info")
_FRAME_ID = os.environ.get("C13_FRAME_ID", "c13_optical")
_INTRINSIC_YAML = os.environ.get("C13_INTRINSIC_YAML", "")  # 空=发占位 CameraInfo (LIVO 待标定)
_FPS = max(1.0, float(os.environ.get("C13_FPS", "30")))
_BACKEND = os.environ.get("C13_BACKEND", "gst").strip().lower()
_IMG_WIDTH = max(1, int(os.environ.get("C13_IMAGE_WIDTH", "1280")))
_IMG_HEIGHT = max(1, int(os.environ.get("C13_IMAGE_HEIGHT", "720")))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    _ROS_OK = True
except Exception as _e:
    _ROS_OK = False
    logger.error(f"[c13img] rclpy/sensor_msgs 不可用 ({_e}); 节点无法运行")

try:
    import cv2  # noqa: F401  (FFmpeg fallback 路径需要)
    _CV2_OK = True
except Exception as _e:
    _CV2_OK = False
    logger.warning(f"[c13img] cv2 不可用 ({_e}); FFmpeg 后端禁用, 仅 gst 可用")

# ---- GStreamer NVDEC 硬解 (复用 nx_gimbal_node.py 验证过的 pipeline) ----
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
            logger.warning(f"[c13img] GStreamer 不可用 ({_e}); gst 后端禁用")


class _GstRtspCapture:
    """NVDEC 硬解 RTSP 拉 BGR frame (复用 nx_gimbal_node.py 同款 pipeline)。

    drop-on-latency=true + appsink drop=true max-buffers=1 + latency=0:
    治 RTSP FIFO 单向堆积 (消费<源帧率时延迟线性增长), 保 LIVO 视觉低延迟。
    """

    def __init__(self, url, target_width, target_height):
        self._url = url
        self._tw = target_width
        self._th = target_height
        self._pipeline = None
        self._sink = None
        self._opened = False
        self._open()

    def _open(self):
        if not _GST_OK or Gst is None:
            return
        scale_caps = (
            f"video/x-raw,format=BGRx,width={int(self._tw)},height={int(self._th)}"
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
            logger.warning(f"[c13img] gst 打开失败: {e}")
            self.release()

    def isOpened(self):
        return self._opened

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
            logger.debug(f"[c13img] gst read 异常: {e}")
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


def _load_camera_info(yaml_path, width, height):
    """加载标准 ROS camera_info YAML → sensor_msgs/CameraInfo。

    缺 YAML 返回占位 (内参=单位阵, LIVO 建图粗但能跑; 标定后用 C13_INTRINSIC_YAML 覆盖)。
    格式兼容 camera_calibration / Kalibr 输出 (camera_matrix/distortion_coefficients/...)。
    """
    if not yaml_path:
        logger.warning("[c13img] 未提供 C13_INTRINSIC_YAML, 用占位内参 (LIVO 建图粗, 连通后必标定)")
        return None
    try:
        import yaml
        with open(yaml_path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        msg = CameraInfo()
        msg.height = int(d.get("image_height", height))
        msg.width = int(d.get("image_width", width))
        msg.distortion_model = d.get("distortion_model", "plumb_bob")
        D = d.get("distortion_coefficients", {})
        msg.d = list(D.get("data", [])) if isinstance(D, dict) else list(D or [])
        K = d.get("camera_matrix", {}).get("data", [1, 0, 0, 0, 1, 0, 0, 0, 1])
        msg.k = list(K)
        R = d.get("rectification_matrix", {}).get("data", [1, 0, 0, 0, 1, 0, 0, 0, 1])
        msg.r = list(R)
        # projection_matrix 缺省: [fx 0 cx 0; 0 fy cy 0; 0 0 1 0] (无立体基线)
        P = d.get("projection_matrix", {}).get("data")
        if P is None:
            K9 = list(K) + [0, 0, 0]
            fx, _, cx, _, fy, cy = K9[0], K9[1], K9[2], K9[3], K9[4], K9[5]
            P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        msg.p = list(P)
        logger.info(f"[c13img] CameraInfo 加载自 {yaml_path} ({msg.width}x{msg.height}, {msg.distortion_model})")
        return msg
    except Exception as e:
        logger.warning(f"[c13img] CameraInfo 加载失败 ({e}); 用占位 (建图粗, 标定后覆盖)")
        return None


class C13ImageNode(Node):
    """C13 RTSP → /c13/image_raw + /c13/camera_info。"""

    def __init__(self):
        super().__init__("nx_c13_image")
        self._pub_img = self.create_publisher(Image, _IMAGE_TOPIC, 10)
        self._pub_info = self.create_publisher(CameraInfo, _CAMINFO_TOPIC, 10)
        self._caminfo = _load_camera_info(_INTRINSIC_YAML, _IMG_WIDTH, _IMG_HEIGHT)
        self._running = False
        self._cap = None
        self._fail = 0
        self._frame_cnt = 0

    def _open_capture(self):
        if _BACKEND in ("auto", "gst", "gstreamer") and _GST_OK:
            cap = _GstRtspCapture(_VIS_URL, _IMG_WIDTH, _IMG_HEIGHT)
            if cap.isOpened():
                logger.info(f"[c13img] NVDEC 已连接 {_VIS_URL} ({_IMG_WIDTH}x{_IMG_HEIGHT})")
                return cap
            cap.release()
            if _BACKEND in ("gst", "gstreamer"):
                logger.warning(f"[c13img] gst 打不开 {_VIS_URL}")
                return None
        if not _CV2_OK:
            return None
        # FFmpeg 低延迟 fallback (gst 不可用时)
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;500000",
        )
        import cv2
        cap = cv2.VideoCapture(_VIS_URL, cv2.CAP_FFMPEG)
        if cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            logger.info(f"[c13img] FFmpeg fallback 已连接 {_VIS_URL}")
            return cap
        logger.warning(f"[c13img] 打不开 {_VIS_URL}")
        return None

    def _capture_loop(self):
        interval = 1.0 / _FPS
        next_t = 0.0
        while self._running:
            if self._cap is None or (
                hasattr(self._cap, "isOpened") and not self._cap.isOpened()
            ):
                self._cap = self._open_capture()
                if self._cap is None:
                    time.sleep(2.0)
                    continue
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.02, next_t - now))
                continue
            next_t = now + interval
            ret, frame = self._cap.read()
            if not ret or frame is None:
                self._fail += 1
                if self._fail > int(_FPS * 5):
                    logger.warning(f"[c13img] 连续 {self._fail} 帧失败, 重连")
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    self._cap = None
                    self._fail = 0
                time.sleep(interval)
                continue
            self._fail = 0
            try:
                self._publish_frame(frame)
            except Exception as e:
                logger.debug(f"[c13img] publish 异常: {e}")
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass

    def _publish_frame(self, frame):
        h, w = frame.shape[:2]
        # ★ 时间戳红线: 拉到帧的瞬间打戳 (不是发布时刻)。RTSP 延迟由 LIVO 时间偏移估计器吸收。
        stamp = self.get_clock().now()
        msg = Image()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = _FRAME_ID
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"  # Gst videoconvert 输出 BGR, 零拷贝; LIVO 配置视觉前端认 bgr8
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = frame.tobytes()
        self._pub_img.publish(msg)
        # CameraInfo 同时间戳发 (接收方免缓存)
        if self._caminfo is not None:
            ci = CameraInfo()
            ci.header.stamp = stamp.to_msg()
            ci.header.frame_id = _FRAME_ID
            ci.height = self._caminfo.height
            ci.width = self._caminfo.width
            ci.distortion_model = self._caminfo.distortion_model
            ci.d = list(self._caminfo.d)
            ci.k = list(self._caminfo.k)
            ci.r = list(self._caminfo.r)
            ci.p = list(self._caminfo.p)
            self._pub_info.publish(ci)
        self._frame_cnt += 1
        if self._frame_cnt % 150 == 0:  # 每 ~5s (30fps) 一次进度日志
            logger.info(f"[c13img] 已发 {self._frame_cnt} 帧 → {_IMAGE_TOPIC}")

    def start(self):
        if not _ROS_OK:
            logger.error("[c13img] ROS 不可用, 不启动")
            return
        self._running = True
        threading.Thread(target=self._capture_loop, name="c13img_cap", daemon=True).start()
        logger.info(
            f"[c13img] C13 Image 桥启动 (url={_VIS_URL}, topic={_IMAGE_TOPIC}, "
            f"{_FPS}fps, backend={_BACKEND}, intrinsic={'已加载' if self._caminfo else '占位(待标定)'})"
        )

    def stop(self):
        self._running = False


def main():
    if not _ROS_OK:
        return
    rclpy.init()
    node = C13ImageNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
