"""water 多边形兜底测试 (2026-09-04): 感知期凸包兜底 vs 细化期严格拒绝。"""
from __future__ import annotations

import numpy as np

from lake_plan import water


def _notched_blob():
    """带凹口的小水斑 (桥/缺口形态)。"""
    h, w = 60, 80
    mask = np.zeros((h, w), dtype=bool)
    mask[10:50, 20:60] = True
    mask[10:30, 32:48] = False  # 缺口
    return mask


def _comp(mask, down=1):
    ys, xs = np.where(mask)
    return {"mask": mask, "area_small": int(mask.sum()), "down": down,
            "bbox_small": (int(xs.min()), int(ys.min()),
                           int(xs.max()), int(ys.max())),
            "centroid_small": (float(ys.mean()), float(xs.mean()))}


def test_convex_hull_basic():
    pts = [(0, 0), (10, 0), (10, 10), (5, 5), (0, 10)]
    hull = water._convex_hull(pts)
    assert len(hull) == 4  # 内部点被剔除
    assert water._poly_area_px(hull) == 100.0


def test_hull_fallback_only_in_perception():
    """契约: 感知路径 (allow_hull=True) 在退化时必须兜底出有效多边形;
    细化路径 (allow_hull=False) 保持原样由调用方弃级 —— 凸包不得
    把粘连水渠等退化形状"救活"冒充细化结果。"""
    mask = _notched_blob()
    strict = water.polygon_from_component(_comp(mask), mask.shape,
                                          allow_hull=False)
    relaxed = water.polygon_from_component(_comp(mask), mask.shape,
                                           allow_hull=True)
    if water._poly_area_px(strict) < 4.0:
        assert len(relaxed) >= 4
        assert water._poly_area_px(relaxed) >= 4.0
    else:
        assert relaxed == strict  # 非退化时两路径必须一致
