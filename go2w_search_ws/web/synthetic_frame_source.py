"""synthetic_frame_source — 演练/干跑用合成巡逻帧源 (M4)。

产生"沿湖巡逻"视角的连续合成帧: 空水面为主, 偶发落水者。
真实帧源在 NX 生产由 nx_ai_node 提供 (M6 接入点)。
"""
from __future__ import annotations

import random
from typing import Any, Callable

from PIL import Image, ImageDraw

_W, _H = 1280, 720
_ROBOT = {"lat": 31.488192, "lng": 120.369486, "yaw_deg": 0.0}


def _robot_meta() -> dict[str, Any]:
    """帧元数据本体位置: 跟随 GO2W_MOCK_GPS (控制台演示切换园区时告警
    坐标随之移动), 未设置时回退 M2.1 基准点。"""
    import os
    meta = dict(_ROBOT)
    mock = os.environ.get("GO2W_MOCK_GPS", "")
    if mock:
        try:
            lat, lng = (float(v) for v in mock.split(",")[:2])
            meta["lat"], meta["lng"] = lat, lng
        except ValueError:
            pass
    return meta


class SyntheticPatrolSource:
    """callable 帧源 → (PIL.Image, robot_meta)。drowning_prob 控制出场率。"""

    def __init__(self, drowning_prob: float = 0.5,
                 seed: int = 20260831):
        self._prob = float(drowning_prob)
        self._rng = random.Random(seed)
        self._frame_i = 0
        self._last_had_person = False
        self._person_cx = 640

    def __call__(self) -> tuple[Image.Image, dict[str, Any]]:
        self._frame_i += 1
        # 同一落水者连续出现 (时序确认需要)
        if self._last_had_person and self._rng.random() < 0.8:
            has_person = True
        else:
            has_person = self._rng.random() < self._prob
            if has_person:
                self._person_cx = self._rng.randint(300, 980)
        self._last_had_person = has_person
        img = Image.new("RGB", (_W, _H))
        d = ImageDraw.Draw(img)
        horizon = int(_H * 0.45)
        d.rectangle([0, 0, _W, horizon], fill=(150, 200, 235))
        for i in range(horizon, _H, 4):
            t = (i - horizon) / max(1, _H - horizon)
            base = int(40 + 30 * t)
            d.rectangle([0, i, _W, i + 4],
                        fill=(base // 2, base, base + 45))
        if has_person:
            cx = self._person_cx + self._rng.randint(-4, 4)
            head_h = self._rng.randint(14, 26)
            cy = horizon + int((_H - horizon) * 0.45)
            d.ellipse([cx - head_h // 2, cy - head_h,
                       cx + head_h // 2, cy], fill=(235, 120, 30))
            d.line([cx - head_h, cy - head_h // 2,
                    cx - head_h // 2, cy - head_h // 3],
                   fill=(220, 140, 80), width=max(3, head_h // 6))
            d.line([cx + head_h, cy - head_h // 2,
                    cx + head_h // 2, cy - head_h // 3],
                   fill=(220, 140, 80), width=max(3, head_h // 6))
        return img, _robot_meta()
