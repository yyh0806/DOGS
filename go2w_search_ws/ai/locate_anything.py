"""LocateAnything CLI adapter.

This module wraps the locate-anything.cpp CLI behind the same light-weight
dictionary contract used by the existing Go2W AI stack:

    locate(frame_or_path, target) -> {found, bbox, label, confidence, ...}

The actual model weights stay outside the repository.
"""

import json
import logging
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Callable, List, Sequence

logger = logging.getLogger("go2w.locate_anything")


def label_to_zh(label: str) -> str:
    """Translate common LocateAnything labels for the operator UI."""
    key = (label or "").strip().lower()
    mapping = {
        "object": "物体",
        "objects": "物体",
        "person": "人",
        "people": "人",
        "person wearing white clothes": "白衣人",
        "person wearing black clothes": "黑衣人",
        "person wearing blue clothes": "蓝衣人",
        "person wearing red clothes": "红衣人",
        "chair": "椅子",
        "table": "桌子",
        "desk": "桌子",
        "door": "门",
        "window": "窗户",
        "car": "车",
        "vehicle": "车",
        "backpack": "背包",
        "bag": "包",
        "box": "箱子",
        "bottle": "瓶子",
        "cup": "杯子",
        "screen": "屏幕",
    }
    return mapping.get(key, label or "物体")


def _normalize_target_text(target: str) -> str:
    """Make common Chinese follow targets easier for LocateAnything prompts."""
    text = (target or "").strip()
    all_object_terms = {
        "所有可见物体",
        "所有可见对象",
        "所有识别结果",
        "全部识别结果",
        "全部物体",
        "全部对象",
        "所有东西",
        "识别出来的东西",
        "全部",
        "all objects",
        "all visible objects",
        "objects",
    }
    if text.lower() in all_object_terms:
        return "object"
    # L1 注意: 长串(含"人"的复合词 + "前面的人")必须在单字"人"之前 — for 按列表顺序
    # 命中, 先替换长串后剩余的孤立"人"才走兜底 person。否定句(如"不要找人")超出关键词法能力。
    replacements = [
        ("穿白色衣服的人", "person wearing white clothes"),
        ("白色衣服的人", "person wearing white clothes"),
        ("穿白衣服的人", "person wearing white clothes"),
        ("白衣服的人", "person wearing white clothes"),
        ("穿黑色衣服的人", "person wearing black clothes"),
        ("黑色衣服的人", "person wearing black clothes"),
        ("穿蓝色衣服的人", "person wearing blue clothes"),
        ("蓝色衣服的人", "person wearing blue clothes"),
        ("穿红色衣服的人", "person wearing red clothes"),
        ("红色衣服的人", "person wearing red clothes"),
        ("前面的人", "person in front"),
        ("人", "person"),
    ]
    for src, dst in replacements:
        if src in text:
            text = text.replace(src, dst)
    return text or "target object"


def build_locate_prompt(target: str) -> str:
    """Build the open-vocabulary prompt expected by locate-anything.cpp."""
    desc = _normalize_target_text(target)
    return f"Locate all the instances that matches the following description: {desc}."


def parse_cli_output(output: str) -> List[dict]:
    """Parse locate-anything.cpp JSON output into Go2W detection dicts."""
    if not output:
        return []
    data = json.loads(output)
    if isinstance(data, list):
        raw_dets = data
    else:
        raw_dets = data.get("detections", [])
    dets = []
    for det in raw_dets:
        if not isinstance(det, dict):
            continue
        box = det.get("bbox", det.get("box"))
        if not isinstance(box, Sequence) or len(box) < 4:
            continue
        try:
            bbox = [round(float(v), 3) for v in box[:4]]
        except (TypeError, ValueError):
            continue
        label = det.get("class", det.get("label", "object"))
        conf = det.get("confidence", det.get("score", 1.0))
        try:
            confidence = round(float(conf), 3)
        except (TypeError, ValueError):
            confidence = 1.0
        dets.append({
            "class": str(label),
            "label_zh": label_to_zh(str(label)),
            "confidence": confidence,
            "bbox": bbox,
            "source": "locate_anything",
        })
    return dets


class LocateAnythingCli:
    """Thin wrapper around locate-anything.cpp's `locate-anything-cli`."""

    def __init__(
        self,
        binary: str | None = None,
        model: str | None = None,
        mode: str | None = None,
        threads: int | None = None,
        timeout: float | None = None,
        runner: Callable | None = None,
    ):
        self.binary = binary or os.environ.get(
            "GO2W_LOCATE_BIN",
            "/home/nx/locate-anything.cpp/build/locate-anything-cli",
        )
        self.model = model or os.environ.get(
            "GO2W_LOCATE_MODEL",
            "/home/nx/models/locate-anything-q8_0.gguf",
        )
        self.mode = mode or os.environ.get("GO2W_LOCATE_MODE", "hybrid")
        self.threads = int(threads or os.environ.get("GO2W_LOCATE_THREADS", "4"))
        self.timeout = float(timeout or os.environ.get("GO2W_LOCATE_TIMEOUT", "45"))
        self._runner = runner or subprocess.run

    @property
    def available(self) -> bool:
        return os.path.exists(self.binary) and os.path.exists(self.model)

    def locate_path(self, image_path, target: str) -> dict:
        """Run LocateAnything against an image file path."""
        image_path = Path(image_path)
        if not self.available:
            return {
                "found": False,
                "bbox": None,
                "label": "",
                "confidence": 0.0,
                "detections": [],
                "description": "locate_anything unavailable: missing binary or model",
            }
        if not image_path.exists():
            return {
                "found": False,
                "bbox": None,
                "label": "",
                "confidence": 0.0,
                "detections": [],
                "description": f"input image missing: {image_path}",
            }

        with tempfile.TemporaryDirectory(prefix="go2w_locate_") as td:
            out_json = Path(td) / "boxes.json"
            cmd = [
                self.binary,
                "detect",
                "--model",
                self.model,
                "--input",
                str(image_path),
                "--prompt",
                build_locate_prompt(target),
                "--mode",
                self.mode,
                "--output",
                str(out_json),
                "--threads",
                str(self.threads),
            ]
            try:
                completed = self._runner(
                    cmd,
                    timeout=self.timeout,
                    capture_output=True,
                    text=True,
                    check=False,
                )
            except TypeError:
                # Tests can inject a minimal runner that only accepts cmd/timeout.
                completed = self._runner(cmd, self.timeout)
            except Exception as e:
                logger.warning(f"LocateAnything CLI failed to start: {e}")
                return self._not_found(f"locate_anything error: {e}")

            if getattr(completed, "returncode", 1) != 0:
                stderr = getattr(completed, "stderr", "") or ""
                logger.warning(f"LocateAnything CLI returned {completed.returncode}: {stderr[:200]}")
                return self._not_found(f"locate_anything exit={completed.returncode}")

            raw = ""
            if out_json.exists():
                raw = out_json.read_text(encoding="utf-8")
            else:
                raw = getattr(completed, "stdout", "") or ""
            try:
                detections = parse_cli_output(raw)
            except Exception as e:
                logger.warning(f"LocateAnything output parse failed: {e}")
                return self._not_found(f"locate_anything parse error: {e}")
            return self._result_from_detections(detections)

    def locate(self, frame, target: str) -> dict:
        """Run LocateAnything against a BGR/RGB numpy frame."""
        if frame is None:
            return self._not_found("empty frame")
        try:
            import cv2
        except Exception as e:
            return self._not_found(f"cv2 unavailable: {e}")
        with tempfile.NamedTemporaryFile(prefix="go2w_locate_", suffix=".jpg", delete=False) as f:
            tmp_path = f.name
        try:
            ok = cv2.imwrite(tmp_path, frame)
            if not ok:
                return self._not_found("failed to encode frame")
            return self.locate_path(tmp_path, target)
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

    @staticmethod
    def _not_found(description: str) -> dict:
        return {
            "found": False,
            "bbox": None,
            "label": "",
            "confidence": 0.0,
            "detections": [],
            "description": description,
        }

    @staticmethod
    def _result_from_detections(detections: List[dict]) -> dict:
        if not detections:
            return LocateAnythingCli._not_found("no detections")
        best = max(detections, key=lambda d: d.get("confidence", 0.0))
        return {
            "found": True,
            "bbox": best.get("bbox"),
            "label": best.get("class", "object"),
            "label_zh": best.get("label_zh") or label_to_zh(best.get("class", "object")),
            "confidence": best.get("confidence", 1.0),
            "detections": detections,
            "description": best.get("label_zh") or label_to_zh(best.get("class", "object")),
        }
