"""nx_drowning_detect — 落水人员检测管线 (纯逻辑核心, M4)。

五级流水线 (每级拦一类错误):
  A 检出    person_detector(frame) → 候选框 (可插拔: 合成域用内置色彩
            候选检测器; NX 生产注入 nx_ai_node 的 YOLO);
  B 水域    相机视角水面掩膜 (HSV 蓝窗) + 候选框邻域水占比 → "在水里";
  C 语义    vlm_verify(crop, ctx) → 可插拔 (生产=VLM, 默认=姿态启发式);
  D 时序    连续 N 帧/T 秒内世界位置一致 → confirmed (单帧只 suspect);
  E 定位    bbox 方位角 + 高度测距 → ENU → WGS-84 (需机器人 gps+航向)。

告警两级: suspect (A+B 过) / confirmed (C+D 过)。
零 ROS/零第三方依赖 (numpy/PIL 除外), 全部可单测。
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Optional

import numpy as np
from PIL import Image

# ---- 参数 (M4 校准值; 真机标定在 M6) ----
FOV_H_DEG = 90.0        # 水平视场角 (C13 广角近似)
PERSON_H_REF_PX = 300.0  # 1m 高的人 @ 假设 2m 距离的像素高 (标定常数)
RANGE_REF_M = 2.0
TEMPORAL_FRAMES = 2      # D: 连续出现帧数 (含首帧)
TEMPORAL_WINDOW_S = 8.0
CLUSTER_RADIUS_M = 10.0
MAX_RANGE_M = 120.0      # 超出即低置信

# C 级 VLM 提示词 (生产: nx_ai_node VLM worker; 兜底: 启发式)
VLM_VERIFY_PROMPT = (
    "图中是水面巡查相机裁剪出的一块区域。请判断: 区域内是否有一个"
    "在水中的人 (落水者)? 特征: 头/手臂露出水面、人体肤色或衣物、"
    "可能在挣扎挥手。注意区分: 正常游泳者(姿态放松)、漂浮物(桶/箱)、"
    "水面反光。只回答 JSON: {\"in_water\": bool, \"struggling\": bool, "
    "\"confidence\": 0.0-1.0}"
)

FrameSource = Callable[[], tuple[Image.Image, dict[str, Any]]]


class DrowningDetector:
    """有状态检测引擎: 逐帧喂数, 内部维护时序簇, 产出两级告警。"""

    def __init__(self, person_detector: Optional[Callable] = None,
                 vlm_verify: Optional[Callable] = None,
                 monotonic=time.monotonic):
        self._person_detector = person_detector or _color_candidate_detector
        self._vlm_verify = vlm_verify or _heuristic_verify
        self._monotonic = monotonic
        self._lock = threading.RLock()
        self._clusters: list[dict[str, Any]] = []
        self._events: list[dict[str, Any]] = []
        self._frames_seen = 0

    # ---------- 主入口 -------------------------------------------------------

    def process_frame(self, frame: Image.Image,
                      robot: dict[str, Any]) -> list[dict[str, Any]]:
        """处理一帧。robot: {lat, lng, yaw_deg (机头方位, 真北起)}。

        返回本帧新产生的告警事件 (suspect/confirmed 升级时各发一次)。
        """
        width, height = frame.size
        water_mask = water_mask_camera(frame)
        detections = self._person_detector(frame)
        new_events: list[dict[str, Any]] = []
        self._frames_seen += 1
        now = self._monotonic()
        for det in detections:
            bbox = det["bbox"]  # (x0, y0, x1, y1)
            score = float(det.get("score", 0.5))
            in_water, water_frac = _bbox_in_water(bbox, water_mask)
            if not in_water:
                continue  # B: 岸上的人/天空误检 → 拦
            bearing = _bearing_from_bbox(bbox, width)
            est_range = _range_from_bbox(bbox, height)
            world = _world_offset(bearing, est_range, robot)
            # D: 时序簇
            cluster = self._match_cluster(world, now)
            if cluster is None:
                cluster = {"hits": [], "first_seen": now, "world": world,
                           "vlm": None, "tier": None, "bearing": bearing,
                           "est_range_m": est_range}
                with self._lock:
                    self._clusters.append(cluster)
            cluster["hits"].append({"ts": now, "world": world, "bbox": bbox,
                                    "score": score})
            cluster["world"] = world
            # 告警判定
            tier = None
            if len(cluster["hits"]) >= TEMPORAL_FRAMES:
                vlm = cluster.get("vlm")
                if vlm is None:
                    crop = frame.crop(bbox)
                    vlm = self._vlm_verify(crop, VLM_VERIFY_PROMPT)
                    cluster["vlm"] = vlm
                if vlm and vlm.get("in_water"):
                    tier = "confirmed"
            if tier is None and score >= 0.3:
                tier = "suspect"
            if tier and _tier_rank(tier) > _tier_rank(cluster["tier"]):
                event = {
                    "tier": tier,
                    "lat": world[0], "lng": world[1],
                    "bearing_deg": round(bearing, 1),
                    "est_range_m": round(est_range, 1),
                    "confidence": round(
                        min(0.99, score * (1.2 if tier == "confirmed"
                                           else 0.7)), 2),
                    "frames": len(cluster["hits"]),
                    "vlm": cluster.get("vlm"),
                    "ts": now,
                }
                cluster["tier"] = tier
                with self._lock:
                    self._events.append(event)
                new_events.append(event)
        return new_events

    # ---------- 查询 ---------------------------------------------------------

    def events(self, tier: Optional[str] = None) -> list[dict[str, Any]]:
        with self._lock:
            events = list(self._events)
        if tier:
            events = [e for e in events if e["tier"] == tier]
        return events

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {"frames_seen": self._frames_seen,
                    "clusters": len(self._clusters),
                    "events": len(self._events),
                    "confirmed": sum(1 for e in self._events
                                     if e["tier"] == "confirmed")}

    # ---------- 内部 ---------------------------------------------------------

    def _match_cluster(self, world, now):
        with self._lock:
            best = None
            best_d = CLUSTER_RADIUS_M
            for cluster in self._clusters:
                if now - cluster["first_seen"] > TEMPORAL_WINDOW_S * 3:
                    continue  # 老簇不复活
                d = math.hypot(world[0] - cluster["world"][0],
                               world[1] - cluster["world"][1])
                if d < best_d:
                    best, best_d = cluster, d
        return best


def _tier_rank(tier):
    return {"suspect": 1, "confirmed": 2}.get(tier, 0)


# ---------- B: 相机视角水面掩膜 ----------------------------------------------

def water_mask_camera(frame: Image.Image) -> np.ndarray:
    """蓝青色窗 + 比天空暗 → bool 掩码 (相机视角, 与卫星图分割独立)。"""
    arr = np.asarray(frame.convert("RGB"), dtype=np.float32) / 255.0
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    mx = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    diff = mx - mn + 1e-9
    # 蓝青主导: b 最大且差值明显
    blue_dom = (b >= mx - 1e-6) & (diff > 0.08)
    dark = mx < 0.75            # 排除天空亮蓝
    return blue_dom & dark


def _bbox_in_water(bbox, water_mask):
    """候选框是否"在水中": 框下缘邻域水占比 ≥ 0.55 且框内含部分水。"""
    x0, y0, x1, y1 = [int(round(v)) for v in bbox]
    h, w = water_mask.shape
    x0, x1 = max(0, x0), min(w, x1)
    y0, y1 = max(0, y0), min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return False, 0.0
    inside = water_mask[y0:y1, x0:x1]
    if inside.mean() < 0.15:
        return False, 0.0
    # 邻域: 框下方+左右扩展带
    pad_y = max(4, (y1 - y0) // 2)
    pad_x = max(4, (x1 - x0) // 2)
    ry0 = min(h, y1)
    ry1 = min(h, y1 + pad_y)
    rx0 = max(0, x0 - pad_x)
    rx1 = min(w, x1 + pad_x)
    ring = water_mask[ry0:ry1, rx0:rx1]
    frac = float(ring.mean()) if ring.size else 0.0
    return frac >= 0.55, round(frac, 3)


# ---------- A (合成域内置): 色彩候选检测器 ------------------------------------

def _color_candidate_detector(frame: Image.Image) -> list[dict]:
    """人色/救生橙亮斑检测 (合成域 stand-in; 生产注入 YOLO person)。"""
    arr = np.asarray(frame.convert("RGB"), dtype=np.float32)
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    # 橙/肤色/亮红: R 高、显著高于 B
    person_color = ((r > 130) & (r > b + 40) & (r > g)).astype(np.uint8)
    detections = []
    for (x0, y0, x1, y1, npx) in label_blobs(person_color, min_px=12):
        # 太宽的亮斑是岸/墙, 不是人头
        if (x1 - x0) > 120 or (y1 - y0) > 160:
            continue
        detections.append({
            "bbox": (x0, y0, x1, y1),
            "score": min(0.95, 0.4 + npx / 400.0),
        })
    return detections


def label_blobs(mask: np.ndarray, min_px: int = 12):
    """连通域 (纯 numpy BFS, 同 water.py 风格但阈值更小)。"""
    from collections import deque
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    blobs = []
    for y in range(h):
        for x in range(w):
            if mask[y, x] and not seen[y, x]:
                q = deque([(y, x)])
                seen[y, x] = True
                pix = []
                while q:
                    cy, cx = q.popleft()
                    pix.append((cy, cx))
                    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        ny, nx = cy + dy, cx + dx
                        if (0 <= ny < h and 0 <= nx < w
                                and mask[ny, nx] and not seen[ny, nx]):
                            seen[ny, nx] = True
                            q.append((ny, nx))
                if len(pix) >= min_px:
                    ys = [p[0] for p in pix]
                    xs = [p[1] for p in pix]
                    blobs.append((min(xs), min(ys), max(xs) + 1,
                                  max(ys) + 1, len(pix)))
    return blobs


# ---------- C 兜底: 启发式"VLM" ----------------------------------------------

def _heuristic_verify(crop: Image.Image, prompt: str) -> dict[str, Any]:
    """无 VLM 时的保守兜底: 人色像素占比高的裁剪 → in_water。"""
    arr = np.asarray(crop.convert("RGB"), dtype=np.float32)
    if arr.size == 0:
        return {"in_water": False, "struggling": False, "confidence": 0.0}
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    person_frac = float(((r > 130) & (r > b + 40)).mean())
    return {"in_water": person_frac > 0.25, "struggling": person_frac > 0.4,
            "confidence": round(min(0.9, person_frac), 2),
            "engine": "heuristic"}


# ---------- E: 几何定位 -------------------------------------------------------

def _bearing_from_bbox(bbox, frame_w) -> float:
    """机体系方位角 (度, 机头为 0, 右正)。"""
    center_x = (bbox[0] + bbox[2]) / 2.0
    return (center_x / frame_w - 0.5) * FOV_H_DEG


def _range_from_bbox(bbox, frame_h) -> float:
    """像素高 → 距离估计 (小孔成像; 露出部分按 0.5m 计)。"""
    h_px = max(1.0, bbox[3] - bbox[1])
    exposed_m = 0.5
    est = RANGE_REF_M * (PERSON_H_REF_PX / h_px) * exposed_m
    return min(est, MAX_RANGE_M)


def _world_offset(bearing_deg, range_m, robot):
    """机体系 (方位, 距离) + 机器人航向 → WGS-84。"""
    yaw = float(robot.get("yaw_deg", 0.0))
    world_bearing = math.radians(yaw + bearing_deg)
    d_north = range_m * math.cos(world_bearing)
    d_east = range_m * math.sin(world_bearing)
    lat = float(robot["lat"]) + d_north / 110540.0
    lng = float(robot["lng"]) + d_east / (
        111320.0 * math.cos(math.radians(float(robot["lat"]))))
    return lat, lng
