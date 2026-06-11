"""
YOLO 目标检测器
===============
从 server.py 抽出的独立模块，平台无关。
"""

import logging
from typing import List, Optional

import numpy as np

from ai.config import YOLO_MODEL_PATH, YOLO_CONFIDENCE

logger = logging.getLogger("go2w.detector")


class Detector:
    """YOLOv8 目标检测器。"""

    def __init__(self, model_path: str = YOLO_MODEL_PATH,
                 confidence: float = YOLO_CONFIDENCE):
        self._model = None
        self._confidence = confidence
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_path)
            logger.info(f"YOLO 加载成功: {model_path}")
            # Warm-up：触发延迟初始化，避免实时推理时 ctypes 崩溃
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self._model(dummy, verbose=False, imgsz=640)
            logger.info("YOLO warm-up 完成")
        except Exception as e:
            logger.error(f"YOLO 加载失败: {e}")
            self._model = None

    @property
    def available(self) -> bool:
        return self._model is not None

    def detect(self, frame: np.ndarray,
               target_classes: Optional[List[str]] = None) -> List[dict]:
        if self._model is None or frame is None:
            return []
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
