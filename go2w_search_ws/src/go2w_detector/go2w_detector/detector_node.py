"""Go2W 目标检测节点 - YOLO + TensorRT 实时检测。

职责:
1. 从 Go2W 前置摄像头获取视频流 (RTSP 或 DDS)
2. 使用 YOLO 模型进行实时目标检测
3. 支持 TensorRT FP16 加速 (Jetson NX)
4. 将检测结果发布到 ROS2 话题
5. 保存检测图片到磁盘

节点名: go2w_detector
话题:
  发布: /go2w/detections  (std_msgs/String JSON, 实际用 go2w_interfaces/TargetDetection)
  订阅: /go2w/robot_state (获取机器人位置, 用于标记检测位置)
  发布: /go2w/detection_image (sensor_msgs/Image, 检测结果可视化)
"""

import os
import json
import time
import logging
import threading
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String as StringMsg

logger = logging.getLogger(__name__)

# 尝试导入依赖
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logger.warning("opencv-python 未安装")

try:
    import numpy as np
    NP_AVAILABLE = True
except ImportError:
    NP_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logger.warning("ultralytics 未安装, 检测功能不可用")


class Detection:
    """单个检测结果。"""

    def __init__(self, class_name: str, confidence: float, bbox: List[float]):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # [x1, y1, x2, y2] 归一化坐标

    def to_dict(self) -> dict:
        return {
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": self.bbox,
        }


class TargetDetector:
    """YOLO 目标检测器。

    支持:
    - YOLOv8 模型 (PyTorch / ONNX / TensorRT)
    - TensorRT FP16 加速
    - 指定目标类别过滤
    - 置信度阈值
    """

    def __init__(self, model_path: str = "yolov8n.pt",
                 confidence: float = 0.45,
                 target_classes: Optional[List[str]] = None,
                 use_tensorrt: bool = True,
                 image_size: int = 640):
        """初始化检测器。

        Args:
            model_path: YOLO 模型路径或名称
                - "yolov8n.pt" - 自动下载 PyTorch 模型
                - "yolov8n.engine" - TensorRT 引擎文件
                - "yolov8n.onnx" - ONNX 模型
            confidence: 检测置信度阈值
            target_classes: 要检测的目标类别 (None=所有)
            use_tensorrt: 是否尝试使用 TensorRT 加速
            image_size: 推理图像尺寸
        """
        self._confidence = confidence
        self._target_classes = set(target_classes) if target_classes else None
        self._image_size = image_size
        self._model = None
        self._class_names = {}

        if not YOLO_AVAILABLE:
            logger.warning("YOLO 不可用，检测器将以模拟模式运行")
            return

        # 尝试加载模型
        self._load_model(model_path, use_tensorrt)

    def _load_model(self, model_path: str, use_tensorrt: bool):
        """加载 YOLO 模型。"""
        try:
            # 如果需要 TensorRT 且是 .pt 模型，先尝试导出 .engine
            if use_tensorrt and model_path.endswith('.pt'):
                engine_path = model_path.replace('.pt', '.engine')
                if os.path.exists(engine_path):
                    logger.info("加载 TensorRT 引擎: %s", engine_path)
                    model_path = engine_path
                else:
                    logger.info("TensorRT 引擎不存在，尝试从 PyTorch 模型导出...")
                    try:
                        model = YOLO(model_path)
                        engine_path = model.export(
                            format='engine',
                            half=True,  # FP16
                            imgsz=self._image_size,
                        )
                        model_path = str(engine_path)
                        logger.info("TensorRT 导出成功: %s", model_path)
                    except Exception as e:
                        logger.warning("TensorRT 导出失败，使用 PyTorch: %s", e)

            # 加载模型
            self._model = YOLO(model_path)
            self._class_names = self._model.names
            logger.info("YOLO 模型加载成功: %s (类别数: %d)",
                        model_path, len(self._class_names))

        except Exception as e:
            logger.error("模型加载失败: %s", e)
            self._model = None

    def detect(self, frame) -> List[Detection]:
        """对单帧图像进行目标检测。

        Args:
            frame: OpenCV BGR 图像 (numpy array)

        Returns:
            检测结果列表
        """
        if self._model is None:
            return self._simulate_detection(frame)

        try:
            results = self._model(
                frame,
                conf=self._confidence,
                iou=0.45,
                imgsz=self._image_size,
                verbose=False,
            )

            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue

                for i in range(len(boxes)):
                    cls_id = int(boxes.cls[i])
                    conf = float(boxes.conf[i])
                    xyxy = boxes.xyxy[i].cpu().numpy()

                    class_name = self._class_names.get(cls_id, f"class_{cls_id}")

                    # 目标类别过滤
                    if self._target_classes and class_name not in self._target_classes:
                        continue

                    # 归一化坐标
                    h, w = frame.shape[:2]
                    bbox = [
                        float(xyxy[0]) / w,
                        float(xyxy[1]) / h,
                        float(xyxy[2]) / w,
                        float(xyxy[3]) / h,
                    ]

                    detections.append(Detection(class_name, conf, bbox))

            return detections

        except Exception as e:
            logger.error("检测推理失败: %s", e)
            return []

    def _simulate_detection(self, frame) -> List[Detection]:
        """模拟检测（无模型时使用）。"""
        return []

    def annotate_frame(self, frame, detections: List[Detection]) -> 'np.ndarray':
        """在图像上绘制检测结果。

        Args:
            frame: 原始图像
            detections: 检测结果

        Returns:
            标注后的图像
        """
        if not NP_AVAILABLE or not CV2_AVAILABLE:
            return frame

        annotated = frame.copy()
        h, w = annotated.shape[:2]

        for det in detections:
            x1 = int(det.bbox[0] * w)
            y1 = int(det.bbox[1] * h)
            x2 = int(det.bbox[2] * w)
            y2 = int(det.bbox[3] * h)

            # 绘制边界框
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 绘制标签
            label = f"{det.class_name} {det.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(annotated, (x1, y1 - th - 10), (x1 + tw, y1), (0, 255, 0), -1)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        return annotated


class CameraCapture:
    """Go2W 摄像头捕获。

    支持:
    - RTSP 视频流 (Go2W 内置摄像头)
    - USB 摄像头
    - 图像文件 (测试用)
    """

    def __init__(self, source: str = "rtsp://192.168.123.161:8554/camera",
                 fps: int = 15, width: int = 640, height: int = 480):
        self._source = source
        self._fps = fps
        self._width = width
        self._height = height
        self._cap = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_frame = None

    def start(self) -> bool:
        """启动摄像头捕获。"""
        if not CV2_AVAILABLE:
            logger.warning("OpenCV 不可用，摄像头无法启动")
            return False

        try:
            self._cap = cv2.VideoCapture(self._source)
            if not self._cap.isOpened():
                logger.error("无法打开视频源: %s", self._source)
                return False

            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)

            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()

            logger.info("摄像头已启动: %s (%dx%d @ %dfps)",
                        self._source, self._width, self._height, self._fps)
            return True

        except Exception as e:
            logger.error("摄像头启动失败: %s", e)
            return False

    def stop(self):
        """停止摄像头捕获。"""
        self._running = False
        if self._cap:
            self._cap.release()
            self._cap = None

    def get_frame(self):
        """获取最新帧。"""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _capture_loop(self):
        """捕获线程主循环。"""
        frame_interval = 1.0 / self._fps

        while self._running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._latest_frame = frame
            else:
                logger.warning("读取帧失败，尝试重连...")
                self._cap.release()
                time.sleep(1.0)
                self._cap = cv2.VideoCapture(self._source)

            time.sleep(frame_interval * 0.8)  # 略快于目标帧率


class DetectorNode(Node):
    """ROS2 目标检测节点。"""

    def __init__(self):
        super().__init__('go2w_detector')

        # 声明参数
        self.declare_parameter('camera_source', 'rtsp://192.168.123.161:8554/camera')
        self.declare_parameter('camera_fps', 15)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('target_classes', [])
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('save_images', True)
        self.declare_parameter('image_dir', 'output/detections')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('detect_interval', 0.1)  # 检测间隔 (秒)

        # 获取参数
        camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().integer_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        confidence = self.get_parameter('confidence').get_parameter_value().double_value
        target_classes = list(
            self.get_parameter('target_classes').get_parameter_value().string_array_value
        )
        use_tensorrt = self.get_parameter('use_tensorrt').get_parameter_value().bool_value
        self._save_images = self.get_parameter('save_images').get_parameter_value().bool_value
        self._image_dir = Path(
            self.get_parameter('image_dir').get_parameter_value().string_value
        )
        detect_interval = self.get_parameter('detect_interval').get_parameter_value().double_value

        # 创建输出目录
        if self._save_images:
            self._image_dir.mkdir(parents=True, exist_ok=True)

        # 初始化检测器
        self._detector = TargetDetector(
            model_path=model_path,
            confidence=confidence,
            target_classes=target_classes if target_classes else None,
            use_tensorrt=use_tensorrt,
        )

        # 初始化摄像头
        self._camera = CameraCapture(
            source=camera_source,
            fps=camera_fps,
        )

        # 机器人状态 (从 /go2w/robot_state 获取)
        self._robot_state = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        # ROS2 接口
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._detection_pub = self.create_publisher(StringMsg, '/go2w/detections', qos)
        self._image_pub = self.create_publisher(StringMsg, '/go2w/detection_image', qos)

        self._state_sub = self.create_subscription(
            StringMsg, '/go2w/robot_state', self._state_callback, qos
        )

        # 检测定时器
        self._detect_timer = self.create_timer(detect_interval, self._detect_callback)

        # 统计
        self._frame_count = 0
        self._detection_count = 0
        self._last_fps_time = time.time()
        self._last_fps_frames = 0

        # 启动摄像头
        if self._camera.start():
            self.get_logger().info("摄像头已启动")
        else:
            self.get_logger().warn("摄像头启动失败，将等待视频源")

        self.get_logger().info(
            f"检测器就绪 (模型: {model_path}, 置信度: {confidence}, "
            f"目标类别: {target_classes or '全部'})"
        )

    def _state_callback(self, msg: StringMsg):
        """更新机器人状态。"""
        try:
            self._robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _detect_callback(self):
        """定时检测回调。"""
        frame = self._camera.get_frame()
        if frame is None:
            return

        self._frame_count += 1

        # 执行检测
        detections = self._detector.detect(frame)

        # 发布每个检测结果
        for det in detections:
            self._detection_count += 1
            self._publish_detection(det, frame)

        # 发布标注图像 (降频)
        if detections and self._save_images:
            self._save_detection_image(frame, detections)

        # FPS 统计
        now = time.time()
        if now - self._last_fps_time >= 5.0:
            fps = (self._frame_count - self._last_fps_frames) / (now - self._last_fps_time)
            self.get_logger().info(
                f"检测 FPS: {fps:.1f}, 总帧: {self._frame_count}, "
                f"检测数: {self._detection_count}"
            )
            self._last_fps_time = now
            self._last_fps_frames = self._frame_count

    def _publish_detection(self, det: Detection, frame):
        """发布单个检测结果到 ROS2 话题。"""
        msg = StringMsg()
        payload = {
            "class_name": det.class_name,
            "confidence": det.confidence,
            "bbox": det.bbox,
            "robot_x": self._robot_state.get("x", 0.0),
            "robot_y": self._robot_state.get("y", 0.0),
            "robot_yaw": self._robot_state.get("yaw", 0.0),
            "timestamp": time.time(),
            "image_path": "",
        }
        msg.data = json.dumps(payload)
        self._detection_pub.publish(msg)

        self.get_logger().info(
            f"发现目标: {det.class_name} (置信度: {det.confidence:.2f})"
        )

    def _save_detection_image(self, frame, detections: List[Detection]):
        """保存标注后的检测图片。"""
        if not CV2_AVAILABLE:
            return

        annotated = self._detector.annotate_frame(frame, detections)
        filename = f"det_{self._detection_count:06d}.jpg"
        filepath = str(self._image_dir / filename)

        cv2.imwrite(filepath, annotated, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.get_logger().debug(f"检测图片已保存: {filepath}")

    def destroy_node(self):
        """清理。"""
        self._camera.stop()
        self.get_logger().info(
            f"检测器关闭 (总帧: {self._frame_count}, 检测数: {self._detection_count})"
        )
        super().destroy_node()


def main(args=None):
    """节点入口点。"""
    rclpy.init(args=args)
    node = DetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("检测器收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
