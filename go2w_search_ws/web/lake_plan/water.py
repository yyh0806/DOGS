"""从瓦片拼接图中提取水域：颜色分割 + 连通域 + 边界追踪。

设计要点：
- 不同底图水色不同，用 HSV 色相窗口(蓝青) 与参考色距离 双条件判定；
- 下采样后 BFS 连通域标记（纯 numpy + deque，无 scipy 依赖）；
- Moore 邻域追踪外边界 + RDP 多边形简化。
"""
import math
from collections import deque

import numpy as np
from PIL import Image


# ---------------- 掩码 ----------------

def water_mask(img: Image.Image, ref_rgb=(170, 211, 223), hue_lo=170, hue_hi=240,
               min_sat=0.06, min_val=0.35, color_tol=70) -> np.ndarray:
    """返回 bool 掩码：True=水。img: PIL RGB 图。"""
    arr = np.asarray(img.convert("RGB"), dtype=np.uint8)
    r = arr[..., 0].astype(np.float32) / 255.0
    g = arr[..., 1].astype(np.float32) / 255.0
    b = arr[..., 2].astype(np.float32) / 255.0
    mx = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    diff = mx - mn + 1e-9
    v = mx
    s = diff / (mx + 1e-9)
    h = np.zeros_like(mx)
    m = (mx == r) & (diff > 0)
    h[m] = (60 * ((g[m] - b[m]) / diff[m])) % 360
    m = (mx == g) & (diff > 0)
    h[m] = 60 * ((b[m] - r[m]) / diff[m] + 2)
    m = (mx == b) & (diff > 0)
    h[m] = 60 * ((r[m] - g[m]) / diff[m] + 4)
    hue_ok = (h >= hue_lo) & (h <= hue_hi)
    cond_hsv = hue_ok & (s >= min_sat) & (v >= min_val)
    rr, gg, bb = ref_rgb
    dist = np.sqrt(((arr[..., 0].astype(np.float32) - rr) ** 2 +
                    (arr[..., 1].astype(np.float32) - gg) ** 2 +
                    (arr[..., 2].astype(np.float32) - bb) ** 2))
    return cond_hsv & (dist <= color_tol)


# ---------------- 连通域 ----------------

def label_components(mask: np.ndarray, down=4, min_area_px=24):
    """下采样 down 倍后 BFS 标记连通域。返回按面积降序的组件列表。"""
    h, w = mask.shape
    H, W = max(1, h // down), max(1, w // down)
    small = mask[: H * down, : W * down].reshape(H, down, W, down).max(axis=(1, 3))
    seen = np.zeros_like(small, dtype=bool)
    comps = []
    for sy in range(H):
        for sx in range(W):
            if small[sy, sx] and not seen[sy, sx]:
                q = deque([(sy, sx)])
                seen[sy, sx] = True
                pix = []
                while q:
                    y, x = q.popleft()
                    pix.append((y, x))
                    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < H and 0 <= nx < W and small[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            q.append((ny, nx))
                if len(pix) >= max(1, min_area_px // (down * down)):
                    ys = [p[0] for p in pix]
                    xs = [p[1] for p in pix]
                    cm = np.zeros((H, W), dtype=bool)
                    cm[np.array(ys), np.array(xs)] = True
                    comps.append({
                        "mask": cm,
                        "area_small": len(pix),
                        "bbox_small": (min(xs), min(ys), max(xs), max(ys)),
                        "centroid_small": (float(np.mean(ys)), float(np.mean(xs))),
                        "down": down,
                    })
    comps.sort(key=lambda c: -c["area_small"])
    return comps


def comp_to_full(c, shape):
    """把下采样连通域掩码放大回原图尺度。"""
    down = c["down"]
    h, w = shape
    up = np.kron(c["mask"], np.ones((down, down), dtype=bool))
    return up[:h, :w]


# ---------------- 边界 ----------------

def moore_boundary(mask: np.ndarray):
    """Moore 邻域追踪外边界（返回 [(row,col)...]）。"""
    coords = np.argwhere(mask)
    if len(coords) == 0:
        return []
    start = tuple(min(coords, key=lambda rc: (rc[0], rc[1])))
    nbrs = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
    H, W = mask.shape
    boundary = [start]
    cur, prev = start, (start[0], start[1] - 1)
    for _ in range(int(mask.sum()) * 4 + 16):
        d = (prev[0] - cur[0], prev[1] - cur[1])
        idx = nbrs.index(d) if d in nbrs else 0
        found = False
        for k in range(1, 9):
            n = nbrs[(idx + k) % 8]
            nxt = (cur[0] + n[0], cur[1] + n[1])
            if 0 <= nxt[0] < H and 0 <= nxt[1] < W and mask[nxt]:
                pk = nbrs[(idx + k - 1) % 8]
                prev = (cur[0] + pk[0], cur[1] + pk[1])
                cur = nxt
                boundary.append(cur)
                found = True
                break
        if not found or (cur == start and len(boundary) > 3):
            break
    return boundary


def rdp_simplify(pts, eps=2.0):
    """Ramer-Douglas-Peucker，pts: [(x,y)...]"""
    if len(pts) < 3:
        return pts
    (x1, y1), (x2, y2) = pts[0], pts[-1]
    dx, dy = x2 - x1, y2 - y1
    norm = math.hypot(dx, dy) or 1e-9
    best, bi = -1.0, -1
    for i, (px, py) in enumerate(pts[1:-1], start=1):
        d = abs(dy * px - dx * py + x2 * y1 - y2 * x1) / norm
        if d > best:
            best, bi = d, i
    if best <= eps:
        return [pts[0], pts[-1]]
    return rdp_simplify(pts[: bi + 1], eps)[:-1] + rdp_simplify(pts[bi:], eps)


def polygon_from_component(c, shape, eps=2.5):
    """连通域 -> 原图像素坐标多边形 [(x,y)...]（已简化）。"""
    full = comp_to_full(c, shape)
    bnd = moore_boundary(full)
    if len(bnd) >= 8:
        pts = [(cc, rr) for (rr, cc) in bnd]  # -> (x, y)
        simp = rdp_simplify(pts, eps)
        if len(simp) >= 4:
            return simp
        simp = rdp_simplify(pts, eps * 0.5)
        if len(simp) >= 4:
            return simp
    # 兜底：径向采样轮廓
    ys, xs = np.where(full)
    if len(ys) == 0:
        return []
    cy, cx = ys.mean(), xs.mean()
    poly = []
    for ang in range(0, 360, 5):
        a = np.deg2rad(ang)
        last = (int(cx), int(cy))
        for r in range(0, max(full.shape)):
            y = int(cy + r * np.sin(a))
            x = int(cx + r * np.cos(a))
            if not (0 <= y < full.shape[0] and 0 <= x < full.shape[1]) or not full[y, x]:
                break
            last = (x, y)
        poly.append(last)
    return poly


# ---------------- 统计 ----------------

def poly_stats_latlon(poly_latlon, ref_lat):
    """多边形 [(lat,lng)...] 的周长(米)与面积(平方米)，等距圆柱近似。"""
    kx = 111320.0 * math.cos(math.radians(ref_lat))
    ky = 110540.0
    xs = [lng * kx for _, lng in poly_latlon]
    ys = [lat * ky for lat, _ in poly_latlon]
    n = len(xs)
    perim = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(n))
    area2 = 0.0
    for i in range(n):
        area2 += xs[i] * ys[i - 1] - xs[i - 1] * ys[i]
    return perim, abs(area2) / 2.0
