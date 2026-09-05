"""semantic_anchor — 语义锚定 (M7.3): 本体在哪 / 湖是哪个, 零歧义。

用户核心要求: 规划前先确定 (1) 本体位置, (2) 任务所指的水体是哪个 ——
语义上没有分歧。实现:
- build_anchor_image: 卫星影像上绘制本体标记 (绿圈) 与候选水体编号
  (红圈), 作为多模态大模型的输入;
- anchor_semantics: VLM (可插拔, NX 上可换 locate-anything 类分割引擎)
  确认本体位置与目标水体, 返回带歧义清单的结构化锚定; VLM 不可用时
  规则退化 (最近水体 + GPS 即本体), 并明确 source="rule";
- 锚定结果附在规划结果上, 全程留痕 —— "湖指的是哪个"从此有据可查。

可插拔分割引擎接口 (map_segmenter, NX 接 locate-anything):
  segment_water(image) -> [{"bbox": (x0,y0,x1,y1), "score": float,
                             "label": str}]  (开放词汇"水体/湖泊")
"""
from __future__ import annotations

import math
from typing import Any, Callable, Optional

from PIL import Image, ImageDraw

from . import tiles

_ANCHOR_PROMPT = (
    "这是卫星影像。绿色圆圈标注的是机器人本体当前位置 (约 {lat:.5f}, "
    "{lng:.5f}); 红色圆圈标注的是候选水体 (编号从 0 开始, 圈中心即该水体"
    "质心, 圈的大小不代表水体大小)。\n"
    "任务: {task}\n"
    "请回答三个问题: 1) 本体是否就在水边 (self_near_water, true/false); "
    "2) 任务所指的目标水体最可能是哪个编号 (target_idx, 整数); "
    "3) 还有哪些水体可能造成歧义 (ambiguity, 编号数组, 没有则空数组); "
    "补充: target_name (若影像上看得出名称) 与 why (一句话理由)。\n"
    "只输出 JSON: {{\"self_near_water\": bool, \"target_idx\": int, "
    "\"target_name\": \"\", \"ambiguity\": [int], \"why\": \"\"}}"
)


def build_anchor_image(provider: str, center_lat: float, center_lng: float,
                       z: int, nx: int, ny: int,
                       robot: dict[str, float],
                       candidates: list[dict[str, Any]]) -> tuple[Image.Image, dict]:
    """卫星拼接图 + 本体绿圈 + 候选红圈编号 (大字号, VLM 可辨)。"""
    img, detail = tiles.stitch_centered(provider, center_lat, center_lng,
                                        z, nx, ny)
    georef = tiles.georef_from_detail(detail)
    draw = ImageDraw.Draw(img)
    from PIL import ImageFont
    font_idx = ImageFont.load_default(size=42)
    font_label = ImageFont.load_default(size=30)

    def px(lat, lng):
        return georef.latlon_to_pixel(lat, lng)

    # 本体 (绿圈 + 十字 + 文字标)
    rx, ry = px(robot["lat"], robot["lng"])
    draw.ellipse([rx - 18, ry - 18, rx + 18, ry + 18],
                 outline=(40, 220, 90), width=5)
    draw.line([(rx - 26, ry), (rx + 26, ry)], fill=(40, 220, 90), width=4)
    draw.line([(rx, ry - 26), (rx, ry + 26)], fill=(40, 220, 90), width=4)
    draw.text((rx + 26, ry - 40), "本体", font=font_label,
              fill=(40, 220, 90), stroke_width=3, stroke_fill=(255, 255, 255),
              anchor="lm")
    # 候选水体 (红圈 + 大号编号)
    for idx, cand in enumerate(candidates):
        centroid = cand.get("centroid")
        if not centroid:
            continue
        cx, cy = px(centroid[0], centroid[1])
        draw.ellipse([cx - 20, cy - 20, cx + 20, cy + 20],
                     outline=(255, 80, 60), width=5)
        draw.text((cx, cy), str(idx), font=font_idx,
                  fill=(255, 255, 255), stroke_width=4,
                  stroke_fill=(255, 80, 60), anchor="mm")
    return img, detail


def anchor_semantics(vlm, task: str, robot: dict[str, float],
                     candidates: list[dict[str, Any]],
                     center: tuple[float, float],
                     provider: str = "esri", z: int = 16,
                     nx: int = 10, ny: int = 8) -> dict[str, Any]:
    """语义锚定主入口。vlm 为 VLMClient 或 None (规则退化)。

    返回: {"source": "vlm"|"rule", "self": {...}, "target": {...},
           "ambiguity": [...], "why": ...}
    """
    if not candidates:
        return {"source": "rule", "self": {"confirmed": False},
                "target": None, "ambiguity": [],
                "why": "no_candidates"}
    try:
        img, detail = build_anchor_image(provider, center[0], center[1], z, nx,
                                         ny, robot, candidates)
    except tiles.TileError:
        # 卫星图不可得 → 规则锚定 (诚实降级)
        return _rule_anchor(robot, candidates, why="satellite_unavailable")
    # 占位灰底不可信: 覆盖不足一半瓦片时禁止喂给 VLM (防误锚定)
    if (detail.get("tiles_ok") or 0) * 2 < nx * ny:
        return _rule_anchor(robot, candidates, why="satellite_unavailable")

    if vlm is not None and getattr(vlm, "available", lambda: False)():
        # 瞬时失败 (限流/网络抖动) 重试一次; 两次都失败 → 规则锚定
        for _attempt in range(2):
            try:
                from go2w_brain.vlm import parse_json_loose
                # 供应商自适应: DeepSeek 4096 才稳定出 JSON, GLM 4096 会 400
                raw = vlm.vision(img, _ANCHOR_PROMPT.format(
                    lat=robot["lat"], lng=robot["lng"], task=task),
                    max_tokens=getattr(vlm, "max_output_tokens",
                                       lambda: 4096)())
                payload = parse_json_loose(raw) or {}
                target_idx = payload.get("target_idx")
                if not isinstance(target_idx, int) or not (
                        0 <= target_idx < len(candidates)):
                    raise ValueError("bad target_idx")
                target = dict(candidates[target_idx])
                ambiguity = [i for i in payload.get("ambiguity", [])
                             if isinstance(i, int)
                             and 0 <= i < len(candidates)]
                return {
                    "source": "vlm",
                    "self": {"lat": robot["lat"], "lng": robot["lng"],
                             "confirmed": bool(payload.get(
                                 "self_near_water", True))},
                    "target": {"idx": target_idx,
                               "centroid": target.get("centroid"),
                               "name": str(payload.get("target_name") or ""),
                               "why": str(payload.get("why") or "")},
                    "ambiguity": ambiguity,
                    "why": str(payload.get("why") or ""),
                    "raw": raw[:300],
                }
            except Exception:  # noqa: BLE001 — VLM 任何失败 → 规则锚定
                continue
    return _rule_anchor(robot, candidates, why="vlm_unavailable")


def _rule_anchor(robot, candidates, why: str) -> dict[str, Any]:
    nearest = min(candidates, key=lambda c: c.get("dist_km", 1e9))
    return {
        "source": "rule",
        "self": {"lat": robot["lat"], "lng": robot["lng"],
                 "confirmed": True},
        "target": {"idx": 0, "centroid": nearest.get("centroid"),
                   "name": "nearest_water", "why": why},
        "ambiguity": [],
        "why": why,
    }
