"""M4 落水检测管线回归: 合成图像数据集 + 检出率/误报率指标。

数据集 (种子固定, PIL 合成, 零网络):
- in-water 落水者 12 例 (20/30/40/55m 档, 3 帧/例) → recall 阈值;
- 空水面 10 例 → 不得 confirmed;
- 岸上行人 5 例 → 不得告警 (B 级水域上下文拦截);
- 反光干扰 6 例 → 不得 confirmed;
- 漂浮物 (白箱) 6 例 → 不得 confirmed (色彩窗 + C 级拦截)。
"""
from __future__ import annotations

import math
import random

import numpy as np
from PIL import Image, ImageDraw

from nx_drowning_detect import (DrowningDetector, _bearing_from_bbox,
                                _range_from_bbox, _world_offset,
                                water_mask_camera)

W, H = 1280, 720
ROBOT = {"lat": 31.488192, "lng": 120.369486, "yaw_deg": 0.0}


# ---------- 合成帧生成 --------------------------------------------------------

def _draw_scene(persons=(), glare=False, shore_person=False, floater=False,
                jitter=(0, 0), sky_shift=0):
    """persons: [(cx_px, head_h_px)] 落水者; 其余为干扰项。"""
    rng = random.Random(hash((tuple(map(tuple, persons)), glare,
                              shore_person, floater,
                              jitter, sky_shift)) & 0xFFFFFFFF)
    img = Image.new("RGB", (W, H))
    d = ImageDraw.Draw(img)
    horizon = int(H * 0.45) + sky_shift
    # 天空 (亮蓝)
    d.rectangle([0, 0, W, horizon], fill=(150, 200, 235))
    # 水面 (暗蓝, 行间渐变)
    for i in range(horizon, H, 4):
        t = (i - horizon) / max(1, H - horizon)
        base = int(40 + 30 * t)
        d.rectangle([0, i, W, i + 4], fill=(base // 2, base, base + 45))
    # 岸带 (左下角绿地)
    if shore_person or rng.random() < 0.4:
        d.polygon([(0, H), (0, int(H * 0.72)), (int(W * 0.25), H)],
                  fill=(90, 120, 60))
    # 波纹/反光
    if glare:
        for _ in range(9):
            gx = rng.randint(100, W - 100)
            gy = rng.randint(horizon + 20, H - 30)
            gw = rng.randint(30, 120)
            d.ellipse([gx, gy, gx + gw, gy + 10],
                      fill=(200, 220, 240))
    # 漂浮物 (白/灰箱 — 非人色)
    if floater:
        fx, fy = rng.randint(200, W - 200), rng.randint(horizon + 60,
                                                        H - 60)
        d.rectangle([fx, fy, fx + 46, fy + 22], fill=(225, 225, 228))
    # 岸上行人 (人色但明确站在绿地内部, 远离岸线)
    if shore_person:
        sx = rng.randint(25, 70)
        sy = H - 28
        d.ellipse([sx, sy - 62, sx + 26, sy - 2], fill=(210, 120, 60))
        d.rectangle([sx - 8, sy - 2, sx + 34, sy + 26], fill=(60, 70, 90))
    # 落水者 (橙, 头+手臂, 半沉)
    for cx, head_h in persons:
        jx, jy = jitter
        cx, head_h = cx + jx, max(8, head_h + jy)
        cy = horizon + int((H - horizon) * 0.45)
        d.ellipse([cx - head_h // 2, cy - head_h,
                   cx + head_h // 2, cy], fill=(235, 120, 30))
        d.line([cx - head_h, cy - head_h // 2,
                cx - head_h // 2, cy - head_h // 3],
               fill=(220, 140, 80), width=max(3, head_h // 6))
        d.line([cx + head_h, cy - head_h // 2,
                cx + head_h // 2, cy - head_h // 3],
               fill=(220, 140, 80), width=max(3, head_h // 6))
    return img


def _range_to_head_h(range_m: float) -> int:
    """距离 → 头部像素高 (与 _range_from_bbox 同一模型反解)。"""
    from nx_drowning_detect import PERSON_H_REF_PX, RANGE_REF_M
    return max(8, int(PERSON_H_REF_PX * 0.5 / range_m))


# ---------- 数据集 -------------------------------------------------------------

def _inwater_cases():
    """12 例: (距离档, cx)。20/30/40m 应被检出 (≤40m 验收带), 55m 低置信。"""
    dists = [20, 25, 30, 35, 40, 20, 30, 40, 25, 35, 55, 55]
    return [(d, 200 + i * 90) for i, d in enumerate(dists)]


def _sequence(persons_kwargs, n=3):
    """同一场景 3 帧 (微抖动) — 时序确认需要。"""
    return [_draw_scene(jitter=(random.Random(k).randint(-4, 4),
                                random.Random(k + 50).randint(-2, 2)),
                         **persons_kwargs) for k in range(n)]


def _run_sequence(detector, frames):
    events = []
    for frame in frames:
        events += detector.process_frame(frame, ROBOT)
    return events


# ---------- 指标测试 -----------------------------------------------------------

def test_recall_within_40m():
    """验收: ≤40m 落水者 recall ≥ 2/3 (confirmed)。"""
    cases = [(d, cx) for d, cx in _inwater_cases() if d <= 40]
    hits = 0
    for d, cx in cases:
        detector = DrowningDetector()
        frames = _sequence({"persons": [(cx, _range_to_head_h(d))]})
        events = _run_sequence(detector, frames)
        if any(e["tier"] == "confirmed" for e in events):
            hits += 1
    recall = hits / len(cases)
    assert recall >= 2 / 3, f"recall {recall:.2f} ({hits}/{len(cases)})"


def test_alert_contains_world_position():
    """验收: 告警含 WGS-84 坐标, 且距真值 < 25m (测距模型允许误差)。"""
    d, cx = 30, 640
    detector = DrowningDetector()
    frames = _sequence({"persons": [(cx, _range_to_head_h(d))]})
    events = _run_sequence(detector, frames)
    confirmed = [e for e in events if e["tier"] == "confirmed"]
    assert confirmed, "30m 中央目标应 confirmed"
    # 真值: cx=640 → bearing 0 (机头正前), 距离 30m
    lat_t, lng_t = _world_offset(0.0, d, ROBOT)
    event = confirmed[0]
    dist_err = math.hypot((event["lat"] - lat_t) * 110540,
                          (event["lng"] - lng_t) * 94800)
    assert dist_err < 25, f"定位误差 {dist_err:.1f}m"
    assert event["est_range_m"] > 5


def test_empty_water_never_confirms():
    """空水面 10 例: 零 confirmed (suspect 也应为 0)。"""
    for i in range(10):
        detector = DrowningDetector()
        frames = _sequence({"glare": i % 2 == 0, "sky_shift": i * 3})
        events = _run_sequence(detector, frames)
        assert not events, f"空水面例 {i} 误报: {events}"


def test_shore_person_not_alerted():
    """岸上行人 5 例: 水域上下文 (B 级) 拦截, 零事件。"""
    for i in range(5):
        detector = DrowningDetector()
        frames = _sequence({"shore_person": True, "sky_shift": i * 7})
        events = _run_sequence(detector, frames)
        assert not events, f"岸上行人例 {i} 误报: {events}"


def test_glare_no_confirmation():
    """反光干扰 6 例 (无落水者): 零 confirmed。"""
    for i in range(6):
        detector = DrowningDetector()
        frames = _sequence({"glare": True, "sky_shift": i * 5})
        events = _run_sequence(detector, frames)
        assert not any(e["tier"] == "confirmed" for e in events), (
            f"反光例 {i} 误报: {events}")


def test_floating_object_no_confirmation():
    """白箱漂浮物 6 例: 色彩窗 (A 级) 拦截, 零事件。"""
    for i in range(6):
        detector = DrowningDetector()
        frames = _sequence({"floater": True, "sky_shift": i * 9})
        events = _run_sequence(detector, frames)
        assert not events, f"漂浮物例 {i} 误报: {events}"


def test_single_frame_only_suspect():
    """单帧出现 → 只 suspect 不 confirmed (D 级时序)。"""
    detector = DrowningDetector()
    frame = _draw_scene(persons=[(640, _range_to_head_h(25))])
    events = detector.process_frame(frame, ROBOT)
    assert all(e["tier"] == "suspect" for e in events)
    assert detector.stats()["confirmed"] == 0


def test_vlm_veto_blocks_confirmation():
    """C 级否决: VLM 判非落水 → 永不 confirmed (只有 suspect)。"""
    def vetoing_vlm(crop, prompt):
        return {"in_water": False, "struggling": False,
                "confidence": 0.9, "engine": "test-veto"}

    detector = DrowningDetector(vlm_verify=vetoing_vlm)
    frames = _sequence({"persons": [(640, _range_to_head_h(25))]})
    events = _run_sequence(detector, frames)
    assert not any(e["tier"] == "confirmed" for e in events)


def test_geometry_helpers():
    frame = _draw_scene(persons=[(640, 60)])
    mask = water_mask_camera(frame)
    assert mask[horizon_of(frame):, :].mean() > 0.5  # 水区判水
    lat, lng = _world_offset(90.0, 10.0, ROBOT)      # 正东 10m
    assert lat == ROBOT["lat"]
    assert lng > ROBOT["lng"]
    assert _range_from_bbox((0, 0, 10, 60), H) < _range_from_bbox(
        (0, 0, 10, 30), H)  # 近大远小


def horizon_of(frame):
    arr = np.asarray(frame.convert("RGB"))
    # 找亮蓝(天空)到暗蓝(水)的行跳变
    row_means = arr.mean(axis=(1, 2))
    for y in range(1, arr.shape[0]):
        if row_means[y] < 120 < row_means[y - 1]:
            return y
    return int(arr.shape[0] * 0.45)
