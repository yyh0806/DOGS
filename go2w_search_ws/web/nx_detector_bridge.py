"""nx_detector_bridge — nx_ai_node 检测/VLM 到 DrowningDetector 的注入适配 (M7.2)。

目标机代码 (NX 上由 nx_web_server/brain 装配); 本模块零 nx_ai_node 导入,
对上游对象形状做防御式归一化, 单元测试用仿形引擎验证契约。

契约 (DrowningDetector 的插拔口):
  person_detector(PIL.Image) -> [{"bbox": (x0,y0,x1,y1), "score": float}]
  vlm_verify(PIL.Image, prompt) -> {"in_water": bool, "struggling": bool,
                                    "confidence": float}
"""
from __future__ import annotations

import json
import re
from typing import Any, Callable

PERSON_CLASS_NAMES = ("person", "人", "person_in_water", "swimmer", "0")
# "0" = COCO 类别 id (person) 的字符串化形态


def make_yolo_person_detector(ai_engine) -> Callable:
    """把 nx_ai 的 YOLO 引擎包装成 person_detector。

    ai_engine 需提供 _run_detector(frame) (返回检测列表) 或
    detect(frame, target_classes) (NxAiDetectorProxy 形态)。检测条目
    字段容忍: bbox/xyxy/box + conf/confidence/score + cls/class/name。
    """
    detector_fn = getattr(ai_engine, "_run_detector", None) or \
        getattr(ai_engine, "detect", None)
    if detector_fn is None:
        raise ValueError("ai_engine 无 _run_detector/detect")

    def person_detector(frame) -> list[dict[str, Any]]:
        try:
            raw = detector_fn(frame)
        except Exception:  # noqa: BLE001 — 检测异常按空结果处理 (fail-soft)
            return []
        out = []
        for det in raw or []:
            if not isinstance(det, (dict, list, tuple)):
                continue
            name = _class_name(det)
            if name is not None and name not in PERSON_CLASS_NAMES:
                continue  # 只收 person 类 (水域上下文在 B 级判断)
            bbox = _bbox(det)
            score = _score(det)
            if bbox is None:
                continue
            out.append({"bbox": bbox, "score": score})
        return out

    return person_detector


def make_vlm_verifier(vlm_proxy) -> Callable:
    """把 nx_ai 的 VLM worker 包装成 vlm_verify。

    vlm_proxy 需提供 chat(messages, max_new_tokens=...) (NxAiVlmProxy 形态);
    返回文本中的 JSON 宽容解析; 解析失败 → 保守否决 (in_water=False)。
    """
    chat_fn = getattr(vlm_proxy, "chat", None)
    if chat_fn is None:
        raise ValueError("vlm_proxy 无 chat")

    def vlm_verify(crop, prompt: str) -> dict[str, Any]:
        try:
            raw = chat_fn([{"role": "user", "content": prompt}],
                          max_new_tokens=120)
            text = raw if isinstance(raw, str) else str(raw)
            return _parse_verdict(text)
        except Exception:  # noqa: BLE001
            return {"in_water": False, "struggling": False,
                    "confidence": 0.0, "engine": "vlm_error"}

    return vlm_verify


# ---------- 归一化 ------------------------------------------------------------

def _class_name(det) -> str | None:
    if isinstance(det, dict):
        for key in ("cls", "class", "name", "label", "class_name"):
            value = det.get(key)
            if isinstance(value, str):
                return value
            if isinstance(value, (int, float)):
                return str(int(value))
    if isinstance(det, (list, tuple)) and len(det) >= 6:
        return str(int(det[5])) if isinstance(det[5], (int, float)) else None
    return None


def _bbox(det) -> tuple[float, float, float, float] | None:
    raw = None
    if isinstance(det, dict):
        for key in ("bbox", "xyxy", "box", "boxes"):
            value = det.get(key)
            if value is not None:
                raw = value
                break
    else:
        raw = det
    if isinstance(raw, (list, tuple)) and len(raw) >= 4:
        try:
            x0, y0, x1, y1 = (float(v) for v in raw[:4])
        except (TypeError, ValueError):
            return None
        if x1 > x0 and y1 > y0:
            return (x0, y0, x1, y1)
        # 容错: x1<x0 (非标准顺序)
        if x1 < x0:
            x0, x1 = x1, x0
        if y1 < y0:
            y0, y1 = y1, y0
        if x1 > x0 and y1 > y0:
            return (x0, y0, x1, y1)
    return None


def _score(det) -> float:
    if isinstance(det, dict):
        for key in ("conf", "confidence", "score"):
            value = det.get(key)
            if isinstance(value, (int, float)):
                return float(value)
    if isinstance(det, (list, tuple)) and len(det) >= 5:
        try:
            return float(det[4])
        except (TypeError, ValueError):
            pass
    return 0.5


def _parse_verdict(text: str) -> dict[str, Any]:
    """宽容解析 VLM 输出里的 JSON; 失败保守否决。"""
    match = re.search(r"\{[^{}]*\"in_water\"[^{}]*\}", text, re.S)
    if not match:
        # 次级: 关键词判断
        lowered = text.lower()
        if "不是" in text or "没有" in text or "无人" in text:
            return {"in_water": False, "struggling": False,
                    "confidence": 0.3, "engine": "keyword"}
        return {"in_water": False, "struggling": False,
                "confidence": 0.0, "engine": "unparsed"}
    try:
        payload = json.loads(match.group(0))
        return {"in_water": bool(payload.get("in_water")),
                "struggling": bool(payload.get("struggling")),
                "confidence": float(payload.get("confidence", 0.5)),
                "engine": "vlm"}
    except (json.JSONDecodeError, TypeError, ValueError):
        return {"in_water": False, "struggling": False,
                "confidence": 0.0, "engine": "bad_json"}
