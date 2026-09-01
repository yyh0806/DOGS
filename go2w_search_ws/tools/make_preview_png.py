#!/usr/bin/env python3
"""make_preview_png.py — 巡逻预览图生成 (M5 演示可视化)。

在真实 OSM 瓦片底图上画: 水域多边形 (半透明蓝) / 环线 (琥珀色) /
扫描点与朝向箭头 / 起点 / 告警标记。瓦片从 GO2W_LAKE_CACHE_DIR 缓存
读取 (夹具即可), 离线可跑。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "web"))
from lake_plan import tiles  # noqa: E402


def _font(size):
    for name in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/arial.ttf",
                 "segoeui.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", required=True)
    parser.add_argument("--trace", default=None)
    parser.add_argument("--out", required=True)
    parser.add_argument("--zoom", type=int, default=17)
    args = parser.parse_args(argv)

    plan = json.loads(Path(args.plan).read_text(encoding="utf-8"))
    plan = plan.get("result", plan)
    lat = plan["target"]["centroid"][0]
    lng = plan["target"]["centroid"][1]
    z = args.zoom
    img, detail = tiles.stitch_centered("osm", lat, lng, z, 10, 8)
    georef = tiles.georef_from_detail(detail)
    d = ImageDraw.Draw(img)

    def px(pt):
        if isinstance(pt, dict):
            pt = (pt["lat"], pt["lon"])
        return georef.latlon_to_pixel(pt[0], pt[1])

    # 水域多边形
    water = [px(p) for p in plan["water_polygon"]]
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ld = ImageDraw.Draw(layer)
    ld.polygon(water, fill=(42, 111, 151, 90), outline=(42, 111, 151, 220))
    img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
    d = ImageDraw.Draw(img)

    # 环线 + 起点
    route = [px(p) for p in plan["waypoints"]]
    d.line(route + [route[0]], fill=(240, 145, 59), width=5)
    sx, sy = route[0]
    r = 10
    d.ellipse([sx - r, sy - r, sx + r, sy + r], outline=(62, 220, 151),
              width=4)

    # 扫描点 + 朝向箭头
    font = _font(14)
    for sp in plan["scan_points"]:
        x, y = px([sp["lat"], sp["lon"]])
        d.ellipse([x - 5, y - 5, x + 5, y + 5], fill=(240, 145, 59))
        rad = math.radians(sp["look_bearing_deg"])
        tip = (x + 34 * math.sin(rad), y - 34 * math.cos(rad))
        d.line([(x, y), tip], fill=(240, 145, 59), width=3)
        d.text((x + 8, y - 22), f"{sp['look_bearing_deg']}°",
               fill=(240, 145, 59), font=font)

    # 告警
    alerts = []
    if args.trace:
        entries = [json.loads(line) for line in
                   Path(args.trace).read_text(encoding="utf-8").splitlines()
                   if line.strip()]
        for entry in entries:
            if (entry.get("kind") == "tool_result"
                    and entry.get("name") == "scan_water"):
                for ev in (entry.get("result") or {}).get("new_events", []):
                    if ev.get("lat") is not None:
                        alerts.append(ev)
    for i, alert in enumerate(alerts):
        x, y = px([alert["lat"], alert["lng"]])
        color = (255, 92, 92) if alert["tier"] == "confirmed" else (240, 145, 59)
        d.ellipse([x - 12, y - 12, x + 12, y + 12], outline=color, width=4)
        d.line([(x - 5, y), (x + 5, y)], fill=color, width=3)
        d.line([(x, y - 5), (x, y + 5)], fill=color, width=3)
        d.text((x + 14, y - 24),
               f"{alert['tier']} {alert.get('est_range_m', '?')}m",
               fill=color, font=font)

    # 标题栏 + 图例
    d.rectangle([0, 0, img.width, 56], fill=(10, 17, 30))
    d.text((12, 8), "绕湖巡查 · 干跑预演 (园区基准点)", fill=(216, 226, 240),
           font=_font(17))
    d.text((12, 30),
           f"{len(route)} 航点 / {round(plan['stats']['length_m'] / 1000, 2)}km "
           f"/ 扫描点 {len(plan['scan_points'])} / 告警 {len(alerts)}",
           fill=(130, 148, 172), font=font)
    img.save(args.out)
    print(f"written: {args.out}")


if __name__ == "__main__":
    main()
