"""
YOLO 目标检测器 (YOLOv8 闭集 + YOLO-World 开放词汇)
=====================================================
model_path 含 "world" → YOLO-World 开放词汇 (运行时 set_classes 指定任意类);
否则 → YOLOv8 闭集 (COCO 80 类)。
同一个 ultralytics 库, 改模型路径即切换。
"""

import logging
import os
from pathlib import Path
from typing import List, Optional

import numpy as np

from ai.config import YOLO_MODEL_PATH, YOLO_CONFIDENCE

logger = logging.getLogger("go2w.detector")


def _configure_ultralytics_weights_dir(
        ultralytics_utils, text_model_module=None) -> Path:
    """Keep YOLO-World's CLIP encoder outside atomic release directories."""

    configured = os.environ.get("GO2W_ULTRALYTICS_WEIGHTS_DIR")
    weights_dir = Path(configured or Path(YOLO_MODEL_PATH).expanduser().parent)
    weights_dir = weights_dir.expanduser()
    weights_dir.mkdir(parents=True, exist_ok=True)
    ultralytics_utils.WEIGHTS_DIR = weights_dir
    if text_model_module is not None:
        # ultralytics.nn.text_model imports WEIGHTS_DIR by value, so update
        # that module too when it has already been loaded.
        text_model_module.WEIGHTS_DIR = weights_dir
    return weights_dir


class Detector:
    """YOLOv8 / YOLO-World 目标检测器。

    model_path 含 "world" → YOLO-World (开放词汇, 需 set_classes);
    否则 → YOLOv8 (闭集 COCO 80 类)。

    开放词汇用法: detect(frame, target_classes=["person", "backpack"])
    → 运行时检测任意类, 不需重训练。
    """

    def __init__(self, model_path: str = YOLO_MODEL_PATH,
                 confidence: float = YOLO_CONFIDENCE,
                 default_classes: Optional[List[str]] = None):
        self._model = None
        self._confidence = confidence
        self._model_path = model_path
        self._is_world = "world" in model_path.lower()
        # YOLO-World 默认 classes (warm-up + detect 无 target_classes 时用)
        self._default_classes = list(default_classes or ["person"])
        self._current_classes: Optional[List[str]] = None  # 已 set 的 (避免重复 set)
        try:
            if self._is_world:
                from ultralytics import YOLOWorld
                from ultralytics import utils as ultralytics_utils
                from ultralytics.nn import text_model
                _configure_ultralytics_weights_dir(
                    ultralytics_utils, text_model)
                self._model = YOLOWorld(model_path)
                self._model.set_classes(self._default_classes)
                self._current_classes = list(self._default_classes)
                logger.info(f"YOLO-World 加载成功: {model_path}, "
                            f"default_classes={self._default_classes}")
            else:
                from ultralytics import YOLO
                self._model = YOLO(model_path)
                logger.info(f"YOLO 加载成功: {model_path}")
            # Warm-up: 触发延迟初始化, 避免实时推理时 ctypes 崩溃
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self._model(dummy, verbose=False, imgsz=640)
            logger.info("YOLO warm-up 完成")
        except Exception as e:
            logger.error(f"YOLO 加载失败: {e}")
            self._model = None

    @property
    def available(self) -> bool:
        return self._model is not None

    @property
    def is_world(self) -> bool:
        """是否 YOLO-World 开放词汇模式。"""
        return self._is_world

    def detect(self, frame: np.ndarray,
               target_classes: Optional[List[str]] = None) -> List[dict]:
        if self._model is None or frame is None:
            return []
        # YOLO-World 开放词汇: 动态 set_classes (target 优先, 否则默认)
        # 这样 detect(frame, ["backpack"]) 即时检测背包, 不需重训练
        if self._is_world:
            classes = list(target_classes) if target_classes else self._default_classes
            if classes and classes != self._current_classes:
                self._model.set_classes(classes)
                self._current_classes = list(classes)
        results = self._model(frame, conf=self._confidence, iou=0.45,
                              verbose=False, imgsz=640)
        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for i in range(len(result.boxes)):
                cls_id = int(result.boxes.cls[i])
                conf = float(result.boxes.conf[i])
                xyxy = result.boxes.xyxy[i].cpu().numpy()
                name = self._model.names.get(cls_id, f"cls_{cls_id}")
                if target_classes and name not in target_classes:
                    continue
                detections.append({
                    "class": name,
                    "confidence": round(conf, 3),
                    "bbox": [round(float(v), 1) for v in xyxy],
                })
        return detections

    def annotate(self, frame: np.ndarray, detections: List[dict]) -> np.ndarray:
        import cv2
        out = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det['class']} {det['confidence']:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(out, (x1, y1 - th - 8), (x1 + tw, y1), (0, 255, 0), -1)
            cv2.putText(out, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        return out
