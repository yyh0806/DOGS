"""vlm_mask — VLM 直接在卫星影像上做语义 mask (2026-09-05, 用户要求)。

不是"画半径圆 + 换街道图": 多模态大模型直接看卫星图, 输出目标区域的
归一化闭合多边形 (0~1 影像坐标), 转成经纬度多边形即 mask。

可插拔: 真机上把 vlm 换成 locate-anything/SAM 类分割引擎, 输出换成
像素级 mask, poly 接口不变 (segment 结果 → 边界多边形)。
"""
from __future__ import annotations

import math
from typing import Any, Optional

from PIL import Image

_CAMPUS_MASK_PROMPT = (
    "这是卫星影像, 绿圈标注的是机器人本体当前位置。请勾画出机器人所在的"
    "【工业园区】的边界: 沿园区围墙/道路, 把园区建筑与地块连片区域圈成一个"
    "闭合多边形。用影像归一化坐标输出 (x 向右, y 向下, 0~1), 顶点 8~20 个, "
    "按顺序首尾相接。只输出 JSON:\n"
    '{"polygon": [[x,y],[x,y],...], "why": "一句话"}'
)

_LAKE_MASK_PROMPT = (
    "这是园区内的高分辨率卫星影像。请勾画出影像中【湖/水体的岸线】边界: "
    "沿水岸走一圈, 输出闭合多边形, 用影像归一化坐标 (x 向右, y 向下, 0~1), "
    "顶点 10~24 个, 按顺序首尾相接。只输出 JSON:\n"
    '{"polygon": [[x,y],[x,y],...], "why": "一句话"}'
)


def vlm_polygon(vlm, image: Image.Image, prompt: str,
                max_tokens: int = 4096,
                attempts: int = 2,
                min_area_frac: float = 0.005) -> Optional[dict[str, Any]]:
    """VLM 直接分割 → 归一化多边形 mask。失败/退化 → None (诚实降级)。

    返回 {"polygon": [[x01,y01],...], "why": str, "raw": str}。
    """
    if vlm is None or not getattr(vlm, "available", lambda: False)():
        return None
    from go2w_brain.vlm import parse_json_loose
    for _ in range(attempts):
        try:
            raw = vlm.vision(image, prompt, max_tokens=max_tokens)
        except Exception:  # noqa: BLE001
            continue
        payload = parse_json_loose(raw) or {}
        poly = payload.get("polygon")
        if (isinstance(poly, list) and len(poly) >= 4
                and all(isinstance(p, (list, tuple)) and len(p) == 2
                        and isinstance(p[0], (int, float))
                        and isinstance(p[1], (int, float))
                        for p in poly)):
            pts = [[float(p[0]), float(p[1])] for p in poly]
            if all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for x, y in pts):
                # 首尾闭合 + 面积校验 (鞋带, 归一化)
                if (pts[0][0] != pts[-1][0] or pts[0][1] != pts[-1][1]):
                    pts = pts + [pts[0][:]]
                area2 = 0.0
                for i in range(len(pts) - 1):
                    area2 += (pts[i][0] * pts[i + 1][1]
                              - pts[i + 1][0] * pts[i][1])
                if abs(area2) / 2.0 >= min_area_frac:
                    return {"polygon": pts, "why": str(payload.get("why") or ""),
                            "raw": raw[:300]}
    return None


def poly_to_latlon(poly: list[list[float]], georef) -> list[tuple[float, float]]:
    """归一化多边形 → 经纬度 [(lat, lng), ...] (闭合)。"""
    w, h = georef.w, georef.h
    out = [georef.pixel_to_latlon(x * w, y * h) for x, y in poly]
    if len(out) > 1 and out[0] != out[-1]:
        out.append(out[0])
    return out


def circle_poly(lat: float, lng: float, radius_m: float,
                n: int = 24) -> list[tuple[float, float]]:
    """规则后备 mask: 半径圆多边形 (VLM 不可用时的园区近似)。"""
    kx = 111320.0 * math.cos(math.radians(lat))
    ky = 110540.0
    ring = [(lat + radius_m * math.sin(2 * math.pi * i / n) / ky,
             lng + radius_m * math.cos(2 * math.pi * i / n) / kx)
            for i in range(n)]
    ring.append(ring[0])
    return ring


CAMPUS_MASK_PROMPT = _CAMPUS_MASK_PROMPT
LAKE_MASK_PROMPT = _LAKE_MASK_PROMPT
