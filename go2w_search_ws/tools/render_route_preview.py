"""preview_route.py — 临时: 把回放页里的路线静态渲染成一张图 (自包含, 必可见)。

把 plan(water_polygon/waypoints/scan_points) 画在 esri z17 卫星拼接图上,
无浏览器依赖, 直接看图。
"""
import json
import re
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from PIL import Image, ImageDraw, ImageFont

from lake_plan import tiles

url = "http://127.0.0.1:8088/runs/brain/20260905_095626_466553_live.html"
html = urllib.request.urlopen(url, timeout=15).read().decode("utf-8")
m = re.search(r"var DATA = (\{.*?\});\s*\nvar", html, re.S)
data = json.loads(m.group(1))
plan = data["plan"]

# 以环线质心为中心取 esri z17 卫星图 (已预热)
wps = plan["waypoints"]
clat = sum(p["lat"] for p in wps) / len(wps)
clng = sum(p["lon"] for p in wps) / len(wps)
img, detail = tiles.stitch_centered("esri", clat, clng, 17, 8, 8)
georef = tiles.georef_from_detail(detail)
draw = ImageDraw.Draw(img)
font = ImageFont.load_default(size=30)
ROBOT = (31.488192, 120.369486)  # 任务触发时 GNSS
print("tiles_ok:", detail["tiles_ok"], "img:", img.size)

# 水域多边形 (蓝)
if plan.get("water_polygon"):
    draw.polygon([georef.latlon_to_pixel(p[0], p[1])
                  for p in plan["water_polygon"]],
                 outline=(40, 120, 230), width=5)
# 环线 (橙)
poly_pts = [georef.latlon_to_pixel(p["lat"], p["lon"]) for p in wps]
for i in range(len(poly_pts) - 1):
    draw.line([poly_pts[i], poly_pts[i + 1]], fill=(255, 150, 40), width=5)
# 起点 (绿) + 扫描点朝向 (橙点/线)
sx, sy = poly_pts[0]
draw.ellipse([sx - 16, sy - 16, sx + 16, sy + 16],
             outline=(80, 230, 140), width=6)
for s in plan.get("scan_points") or []:
    px, py = georef.latlon_to_pixel(s["lat"], s["lon"])
    rad = s["look_bearing_deg"] * 3.14159265 / 180
    draw.ellipse([px - 9, py - 9, px + 9, py + 9],
                 outline=(255, 150, 40), width=5)
    draw.line([(px, py),
               (px + 46 * (rad and __import__("math").sin(rad)),
                py - 46 * __import__("math").cos(rad))],
              fill=(255, 150, 40), width=4)
# 告警 (红)
for a in data.get("alerts") or []:
    ax, ay = georef.latlon_to_pixel(a["lat"], a["lng"])
    draw.ellipse([ax - 22, ay - 22, ax + 22, ay + 22],
                 outline=(255, 80, 80), width=6)
    draw.text((ax + 26, ay - 12), f"{a.get('tier')} {a.get('confidence')}",
              font=font, fill=(255, 120, 120))
# 本体
bx, by = georef.latlon_to_pixel(*ROBOT)
draw.ellipse([bx - 14, by - 14, bx + 14, by + 14],
             outline=(60, 240, 110), width=5)
draw.text((bx + 20, by - 44), "本体", font=font, fill=(60, 240, 110))

out = Path("runs/brain/route_preview_20260905.jpg")
img.convert("RGB").save(out, quality=88)
print("saved:", out)
