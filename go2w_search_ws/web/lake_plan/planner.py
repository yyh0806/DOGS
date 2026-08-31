"""绕湖路线几何规划：水域多边形 -> 离岸外扩 -> 重采样 -> 校验 -> 导出。

核心思想：
- 水域连通域给出湖岸多边形（像素坐标，附 Georef）；
- 对多边形每个顶点沿「角平分线法线」向外偏移 d 米（miter 补偿，封顶防尖刺）；
- 外扩方向不确定（顺/逆时针）时两种符号都试，取「落在水外的点最多」者；
- 按步长重采样成闭合折线（WGS-84 经纬度）；
- 校验：闭合性、长度合理性、跨水率（采样点落在水域多边形内的比例）。
"""
import math

from .geo import haversine_m
from .water import rdp_simplify


# ---------------- 多边形基础 ----------------

def signed_area(poly):
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def centroid(poly):
    n = float(len(poly))
    return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)


def point_in_poly(pt, poly):
    """射线法。pt=(x,y)。"""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xcross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < xcross:
                inside = not inside
        j = i
    return inside


def dist_point_poly(pt, poly):
    """点到多边形边的最小距离（像素）。"""
    x, y = pt
    best = float("inf")
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy or 1e-9
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        px, py = x1 + t * dx, y1 + t * dy
        best = min(best, math.hypot(x - px, y - py))
    return best


# ---------------- 外扩 ----------------

def offset_ring(poly, d_px, miter_cap=3.0):
    """把闭合环向外偏移 d_px。"""
    n = len(poly)
    out = []
    for i in range(n):
        px, py = poly[i]
        qx, qy = poly[(i - 1) % n]
        rx, ry = poly[(i + 1) % n]
        v1 = (px - qx, py - qy)
        v2 = (rx - px, ry - py)
        l1 = math.hypot(*v1) or 1.0
        l2 = math.hypot(*v2) or 1.0
        m1 = (v1[1] / l1, -v1[0] / l1)
        m2 = (v2[1] / l2, -v2[0] / l2)
        sx, sy = m1[0] + m2[0], m1[1] + m2[1]
        sl = math.hypot(sx, sy) or 1e-9
        miter = min(2.0 / sl, miter_cap)
        out.append((px + d_px * miter * sx / sl, py + d_px * miter * sy / sl))
    return out


def outward_offset(poly, d_px):
    """自适应方向的向外偏移：两种符号都算，选落入多边形内部点数少的那组。"""
    best, best_in = None, None
    for sign in (1.0, -1.0):
        cand = offset_ring(poly, sign * d_px)
        inside = sum(1 for p in cand if point_in_poly(p, poly))
        if inside == 0:
            return cand
        if best_in is None or inside < best_in:
            best, best_in = cand, inside
    return best


def offset_via_dilation(poly_px, shape, d_px, down=8):
    """形态学膨胀等距外扩：水域栅格 -> 半径 d 膨胀 -> 外边界。

    相比顶点法线外扩(miter)，天然无尖刺、无自交，且保证路线在水域之外；
    宽度小于 2d 的湾岔会被自动「跨过」（符合该离岸距离下的可行绕行）。
    返回外边界像素环 [(x,y)...]。
    """
    from PIL import Image, ImageDraw
    import numpy as np
    h, w = shape
    down = max(1, int(down))
    H, W = max(16, h // down), max(16, w // down)
    img = Image.new("1", (W, H), 0)
    ImageDraw.Draw(img).polygon(
        [(x / down, y / down) for (x, y) in poly_px], fill=1)
    m = np.array(img, dtype=bool)
    R = max(2, int(round(d_px / down)))
    pad = R + 2
    padded = np.zeros((H + 2 * pad, W + 2 * pad), dtype=bool)
    padded[pad:pad + H, pad:pad + W] = m
    dil = m.copy()
    offs = [(dx, dy) for dx in range(-R, R + 1) for dy in range(-R, R + 1)
            if dx * dx + dy * dy <= R * R]
    for dx, dy in offs:
        np.logical_or(dil, padded[pad + dy:pad + dy + H,
                                  pad + dx:pad + dx + W], out=dil)
    from .water import moore_boundary
    bnd = moore_boundary(dil)
    if len(bnd) < 16:
        return []
    ring = [(float(x) * down, float(y) * down) for (y, x) in bnd]
    return ring


def snap_outside(route, poly, cx, cy):
    """把仍落在水域内的点从形心向外推到岸外。"""
    out = []
    for (x, y) in route:
        if not point_in_poly((x, y), poly):
            out.append((x, y))
            continue
        dx, dy = x - cx, y - cy
        L = math.hypot(dx, dy) or 1.0
        pushed = (x, y)
        for f in (0.25, 0.5, 1.0, 2.0, 4.0, 6.0):
            cand = (x + dx / L * f * 20.0, y + dy / L * f * 20.0)
            if not point_in_poly(cand, poly):
                pushed = cand
                break
        out.append(pushed)
    return out


# ---------------- 重采样 ----------------

def chaikin(points, iters=1):
    """闭合折线 Chaikin 圆角平滑。"""
    pts = list(points)
    if pts[0] == pts[-1]:
        pts = pts[:-1]
    for _ in range(iters):
        out = []
        n = len(pts)
        for i in range(n):
            p, q = pts[i], pts[(i + 1) % n]
            out.append((0.75 * p[0] + 0.25 * q[0], 0.75 * p[1] + 0.25 * q[1]))
            out.append((0.25 * p[0] + 0.75 * q[0], 0.25 * p[1] + 0.75 * q[1]))
        pts = out
    pts.append(pts[0])
    return pts


def resample_ring(points, step_px):
    """沿折线每隔 step_px 取点，保持闭合（首尾相同）。"""
    if len(points) < 3:
        return points
    ring = list(points)
    if ring[0] != ring[-1]:
        ring.append(ring[0])
    out = [ring[0]]
    acc = 0.0
    prev = ring[0]
    for cur in ring[1:]:
        seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        while acc + seg >= step_px:
            t = (step_px - acc) / (seg or 1e-9)
            nx = prev[0] + t * (cur[0] - prev[0])
            ny = prev[1] + t * (cur[1] - prev[1])
            out.append((nx, ny))
            prev = (nx, ny)
            seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            acc = 0.0
        acc += seg
        prev = cur
    out.append(ring[-1])
    if len(out) > 2 and math.hypot(out[-1][0] - out[-2][0], out[-1][1] - out[-2][1]) < step_px * 0.3:
        out.pop()
    return out


def polyline_length_m(latlon_pts):
    return sum(haversine_m(latlon_pts[i - 1][0], latlon_pts[i - 1][1],
                           latlon_pts[i][0], latlon_pts[i][1])
               for i in range(1, len(latlon_pts)))


# ---------------- 主入口 ----------------

def plan_loop_around_polygon(water_poly_px, georef, offset_m=140.0, step_m=50.0,
                             simplify_eps_px=None, use_miter=False):
    """返回 dict: route_px, route_latlon, poly_px, stats。"""
    mppx = georef.mppx()
    if simplify_eps_px is None:
        # 自适应：高分辨率时用更大 epsilon 平滑小湾岔（约15米），粗分辨率保持细节
        simplify_eps_px = max(2.5, min(8.0, 15.0 / mppx))
    poly = []
    for eps in (simplify_eps_px, simplify_eps_px * 0.4, 0.0):
        poly = rdp_simplify(list(water_poly_px), eps) if eps > 0 else list(water_poly_px)
        if len(poly) >= 4:
            break
    if len(poly) < 4:
        raise ValueError("水域多边形顶点过少，无法规划")
    cx, cy = centroid(poly)
    d_px = offset_m / mppx
    step_px = max(4.0, step_m / mppx)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    shape = (int(max(ys)) + 16, int(max(xs)) + 16)
    ring = [] if use_miter else offset_via_dilation(poly, shape, d_px)
    if len(ring) >= 16:
        if ring[0] == ring[-1]:
            ring = ring[:-1]  # RDP 不能处理退化弦(首尾相同)
        if len(ring) > 24:
            ring = rdp_simplify(ring, 12.0)  # 平滑栅格台阶
    else:
        # 膨胀法失败兜底：顶点法线外扩
        ring = outward_offset(poly, d_px)
        ring = snap_outside(ring, poly, cx, cy)
    route_px = resample_ring(ring, step_px)
    # 圆角平滑 + 离水校正（平滑可能切角进水，须再推一次）
    route_px = chaikin(route_px, iters=1)
    route_px = snap_outside(route_px, poly, cx, cy)
    route_px = resample_ring(route_px, step_px)
    if len(route_px) < 8:
        raise ValueError("环线退化: 路线点过少")
    if route_px[0] != route_px[-1]:
        route_px.append(route_px[0])
    route_latlon = [georef.pixel_to_latlon(x, y) for (x, y) in route_px]
    length_m = polyline_length_m(route_latlon)
    inside = sum(1 for p in route_px[:-1:3] if point_in_poly(p, poly))
    sampled = len(route_px[:-1:3]) or 1
    shore_px = [dist_point_poly(p, poly) for p in route_px[:-1:8]]
    stats = {
        "vertices_poly": len(poly),
        "route_points": len(route_latlon),
        "length_m": round(length_m, 1),
        "closed": haversine_m(route_latlon[0][0], route_latlon[0][1],
                              route_latlon[-1][0], route_latlon[-1][1]) < 30.0,
        "water_cross_ratio": round(inside / sampled, 4),
        "min_shore_m": round(min(shore_px) * mppx, 1) if shore_px else None,
        "offset_m": round(offset_m, 1),
        "mppx": round(mppx, 3),
    }
    return {"route_px": route_px, "route_latlon": route_latlon,
            "poly_px": poly, "stats": stats}


# ---------------- 导出 ----------------

def to_geojson(route_latlon, lake_poly_latlon, lake_name="", length_m=None,
               closed=True):
    def pt(p):
        return [round(p[1], 6), round(p[0], 6)]  # [lng, lat]

    coords = [pt(p) for p in route_latlon]
    if closed and coords and coords[0] != coords[-1]:
        coords.append(coords[0])
    feats = [{
        "type": "Feature",
        "geometry": {"type": "LineString", "coordinates": coords},
        "properties": {"type": "lake-loop", "name": lake_name or "loop",
                       "length_km": round((length_m or 0) / 1000.0, 2)},
    }]
    if lake_poly_latlon and len(lake_poly_latlon) >= 4:
        ring = [pt(p) for p in lake_poly_latlon]
        if ring[0] != ring[-1]:
            ring.append(ring[0])
        feats.append({
            "type": "Feature",
            "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"type": "water-body", "name": lake_name},
        })
    return {"type": "FeatureCollection", "features": feats}


def to_gpx(route_latlon, name="lake-loop"):
    head = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<gpx version="1.1" creator="lake-loop-agent" '
            'xmlns="http://www.topografix.com/GPX/1/1">\n'
            '<trk><name>' + name + '</name><trkseg>\n')
    body = []
    for lat, lng in route_latlon:
        body.append('<trkpt lat="' + format(lat, ".6f") + '" lon="'
                    + format(lng, ".6f") + '"/>')
    return head + "\n".join(body) + "\n</trkseg></trk>\n</gpx>\n"


# ---------------- 预览渲染 ----------------

def render_preview(stitched_img, route_px, poly_px, georef, title, sub,
                   out_path, every_km_label=True, length_m=None):
    """在拼接图上画：水域轮廓(蓝) + 路线(红) + 起点(绿) + 方向箭头 + 比例尺。"""
    from PIL import Image, ImageDraw, ImageFont
    img = stitched_img.copy()
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("arial.ttf", 16)
        small = ImageFont.truetype("arial.ttf", 13)
    except OSError:
        font = small = ImageFont.load_default()

    # 水域：半透明填充（比锯齿轮廓线更易读）
    if poly_px and len(poly_px) >= 3:
        from PIL import Image as _Im
        lake_layer = _Im.new("RGBA", img.size, (0, 0, 0, 0))
        ld = ImageDraw.Draw(lake_layer)
        ld.polygon([(p[0], p[1]) for p in poly_px], fill=(30, 110, 200, 70),
                   outline=(20, 80, 160, 160))
        img = Image.alpha_composite(img.convert("RGBA"), lake_layer).convert("RGB")
        d = ImageDraw.Draw(img)
    d.line([(p[0], p[1]) for p in route_px], fill=(235, 40, 40), width=4)

    # 方向箭头（路线 1/4 处）
    if len(route_px) > 8:
        i = len(route_px) // 4
        x1, y1 = route_px[i]
        x2, y2 = route_px[min(i + 2, len(route_px) - 1)]
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        tip = (x1 + ux * 14, y1 + uy * 14)
        d.line([(x1 - ux * 8, y1 - uy * 8), tip], fill=(235, 40, 40), width=5)
        for s in (-0.6, 0.6):
            wx = tip[0] - ux * 10 + s * -uy * 7
            wy = tip[1] - uy * 10 + s * ux * 7
            d.line([tip, (wx, wy)], fill=(235, 40, 40), width=4)

    if route_px:
        sx, sy = route_px[0]
        r = 8
        d.ellipse([sx - r, sy - r, sx + r, sy + r], outline=(0, 150, 60), width=4)
        d.text((sx + 12, sy - 10), "START", fill=(0, 120, 50), font=small)

    # 每公里标记
    if every_km_label and length_m and len(route_px) > 10:
        acc = 0.0
        nxt = 1.0
        latlon = [georef.pixel_to_latlon(x, y) for (x, y) in route_px]
        for i in range(1, len(route_px)):
            acc += haversine_m(latlon[i - 1][0], latlon[i - 1][1],
                               latlon[i][0], latlon[i][1])
            if acc >= nxt * 1000:
                x, y = route_px[i]
                d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(235, 40, 40))
                d.text((x + 6, y - 8), str(int(nxt)) + "km",
                       fill=(120, 20, 20), font=small)
                nxt += 1.0

    # 标题栏
    d.rectangle([0, 0, img.width, 46], fill=(255, 255, 255))
    d.line([(0, 46), (img.width, 46)], fill=(200, 60, 60), width=3)
    d.text((10, 6), title, fill=(30, 30, 30), font=font)
    d.text((10, 24), sub, fill=(90, 90, 90), font=small)

    # 比例尺（右上角）
    mppx = georef.mppx()
    for cand_m in (500, 1000, 2000, 5000, 10000):
        px = cand_m / mppx
        if 60 <= px <= 220:
            x0, y0 = img.width - px - 20, 70
            d.rectangle([x0, y0, x0 + px, y0 + 6], outline=(30, 30, 30), width=2)
            lbl = str(cand_m // 1000) + " km" if cand_m >= 1000 else str(cand_m) + " m"
            d.text((x0, y0 + 8), "0", fill=(30, 30, 30), font=small)
            d.text((x0 + px - 30, y0 + 8), lbl, fill=(30, 30, 30), font=small)
            break
    # 指北针
    d.line([(img.width - 30, 110), (img.width - 30, 90)], fill=(30, 30, 30), width=3)
    d.polygon([(img.width - 34, 94), (img.width - 26, 94), (img.width - 30, 84)],
              fill=(30, 30, 30))
    d.text((img.width - 38, 112), "N", fill=(30, 30, 30), font=small)

    img.save(out_path)
    return out_path


def render_html(geojson_dict, out_path, title, server_base, provider="osm"):
    """Leaflet 预览页：底图用本地瓦片服务器，路线叠加。"""
    import json as _json
    gj = _json.dumps(geojson_dict, ensure_ascii=False).replace("</", "<\\/")
    url = server_base + "/tile/" + provider + "/{z}/{x}/{y}.png"
    html = ('<!doctype html>\n<html><head><meta charset="utf-8"/>\n'
            '<title>' + title + '</title>\n'
            '<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>\n'
            '<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>\n'
            '<style>body{margin:0}#map{width:100vw;height:100vh}</style></head>\n'
            '<body><div id="map"></div>\n'
            '<script type="application/json" id="gjdata">' + gj + '</script>\n'
            '<script>\n'
            'var map = L.map("map");\n'
            'L.tileLayer("' + url + '", {maxZoom: 19, attribution: "local tile server"}).addTo(map);\n'
            'var gj = L.geoJSON(JSON.parse(document.getElementById("gjdata").textContent),\n'
            '  { style: function(f) {\n'
            '      if (f.geometry.type === "LineString") return {color:"#e62828",weight:4};\n'
            '      return {color:"#155ab0",weight:2,fillOpacity:0.1};\n'
            '    }}).addTo(map);\n'
            'map.fitBounds(gj.getBounds().pad(0.15));\n'
            '</script>\n</body></html>\n')
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path
