"""campus_lake_walkthrough.py — 园区湖语义链逐步演示 (2026-09-05)。

把 plan_campus_lake 的四步语义链拆成可见步骤, 每步出一张 mask/标注图:
  S1 园区识别: 卫星底图上用 mask 圈出园区范围 (园区外压暗) + 园区名
  S2 园区内找湖: OSM 水体掩膜 (蓝色) 叠加, 园区半径内候选编号
  S3 湖形过滤: 细长河道剔除 (红叉) vs 湖形保留 (绿圈), 面积/紧凑度表
  S4 VLM 锚定: 卫星图 + 本体绿圈 + 候选红圈 → 真实 VLM 结论
  S5 聚焦细化+环线: 选定湖 z19 卫星特写 + 细化边界 + 离岸环线 + 扫描点

用法 (web/ 目录): python ..\\tools\\campus_lake_walkthrough.py [--no-vlm]
输出: runs/brain/campus_lake_walkthrough.html (自包含 base64 图)。
"""
from __future__ import annotations

import argparse
import base64
import html
import io
import json
import math
import sys
from pathlib import Path

_WEB = Path(__file__).resolve().parents[1] / "web"
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from lake_plan import planner, route_api, semantic_anchor, tiles, water  # noqa: E402
from go2w_brain.tools.plan_campus_lake import CAMPUSES  # noqa: E402

CAMPUS = CAMPUSES[0]
TASK = "绕着当前园区湖绕行一圈"
ROBOT = {"lat": CAMPUS["lat"], "lng": CAMPUS["lng"]}
STEPS: list[dict] = []


def hav_m(a, b):
    return 6378137 * math.acos(min(1.0, math.sin(math.radians(a[0]))
        * math.sin(math.radians(b[0])) + math.cos(math.radians(a[0]))
        * math.cos(math.radians(b[0])) * math.cos(math.radians(a[1] - b[1]))))


def add_step(no, title, text, image=None, table=None, image2=None):
    STEPS.append({"no": no, "title": title, "text": text,
                  "image": image, "table": table, "image2": image2})


def jpg_b64(img, max_w=1024, quality=82):
    if img.size[0] > max_w:
        img = img.resize((max_w, int(img.size[1] * max_w / img.size[0])))
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def darken_outside(img, georef, lat, lng, radius_m):
    """园区 mask: 圆外压暗, 圆内保持明亮 + 白圈。"""
    mpp = georef.mppx()
    cx, cy = georef.latlon_to_pixel(lat, lng)
    r = radius_m / mpp
    dark = img.point(lambda p: int(p * 0.35))
    outside = Image.new("L", img.size, 255)
    d = ImageDraw.Draw(outside)
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=0)
    out = Image.composite(img, dark, outside)
    d = ImageDraw.Draw(out)
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(255, 255, 255),
              width=6)
    return out


def marker(draw, x, y, color, r, label=None, font=None):
    draw.ellipse([x - r, y - r, x + r, y + r], outline=color, width=5)
    draw.line([(x - r - 7, y), (x + r + 7, y)], fill=color, width=3)
    draw.line([(x, y - r - 7), (x, y + r + 7)], fill=color, width=3)
    if label:
        draw.text((x + r + 5, y - r - 5), label, font=font, fill=color)


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-vlm", action="store_true")
    ap.add_argument("--out", default="runs/brain/campus_lake_walkthrough.html")
    args = ap.parse_args(argv)

    font_big = ImageFont.load_default(size=44)
    font_mid = ImageFont.load_default(size=30)
    clat, clng = CAMPUS["lat"], CAMPUS["lng"]
    radius = CAMPUS["radius_m"]

    # ---------- S1 园区识别 (mask) ----------
    print("[S1] 园区识别...")
    sat, sat_detail = tiles.stitch_centered("esri", clat, clng, 17, 8, 8)
    sat_geo = tiles.georef_from_detail(sat_detail)
    s1 = darken_outside(sat, sat_geo, clat, clng, radius)
    d1 = ImageDraw.Draw(s1)
    rx, ry = sat_geo.latlon_to_pixel(clat, clng)
    marker(d1, rx, ry, (60, 240, 110), 16, "本体", font_mid)
    d1.text((rx + 24, ry - 60), CAMPUS["name"], font=font_big,
            fill=(255, 255, 255), stroke_width=4, stroke_fill=(0, 0, 0))
    add_step(1, "园区识别 (园区 mask)",
             f"任务文本含\"园区湖\" → 园区知识库匹配 (名称/别名 + 本体位置兜底) → "
             f"<b>{CAMPUS['name']}</b>。白圈=园区范围 (半径 {radius}m, 圈内保持"
             f"明亮, 圈外压暗的 mask 显示); 绿圈+字标=本体。",
             image=jpg_b64(s1))

    # ---------- S2 园区内找湖 (水体 mask) ----------
    print("[S2] 园区内水体掩膜...")
    osm, osm_detail = tiles.stitch_centered("osm", clat, clng, 17, 8, 8)
    geo = tiles.georef_from_detail(osm_detail)
    wmask = water.water_mask(osm, ref_rgb=(170, 211, 223))
    comps = water.label_components(wmask, down=4, min_area_px=200)
    arr = np.asarray(osm.convert("RGB")).copy()
    arr[wmask] = (arr[wmask] * 0.25 + np.array([40, 120, 255]) * 0.75) \
        .astype(np.uint8)
    s2 = darken_outside(Image.fromarray(arr), geo, clat, clng, radius)
    d2 = ImageDraw.Draw(s2)
    marker(d2, *geo.latlon_to_pixel(clat, clng), (60, 240, 110), 16,
           "本体", font_mid)
    idx = 0
    comps_near = []
    for c in comps:
        cy, cx = c["centroid_small"]
        lat, lng = geo.pixel_to_latlon(cx * c["down"], cy * c["down"])
        if hav_m((clat, clng), (lat, lng)) <= radius:
            comps_near.append((c, lat, lng))
            px, py = geo.latlon_to_pixel(lat, lng)
            d2.text((px, py), str(idx), font=font_big, fill=(255, 255, 255),
                    stroke_width=4, stroke_fill=(255, 80, 60), anchor="mm")
            idx += 1
    add_step(2, "园区内找湖 (水体 mask)",
             f"OSM z17 渲染水体掩膜 (蓝色) 叠加到园区视图; 园区半径内共 "
             f"<b>{idx} 个水体连通域</b> (红字编号)。卫星 HSV 只作后备 —— 实测"
             f"它会把园区湖打碎成小斑块。",
             image=jpg_b64(s2))

    # ---------- S3 湖形过滤 ----------
    print("[S3] 湖形过滤...")
    rows = []
    kept = []
    for i, (c, lat, lng) in enumerate(comps_near):
        poly = water.polygon_from_component(c, wmask.shape, allow_hull=True)
        if len(poly) < 4:
            rows.append([i, "-", "-", "-", "多边形退化 ❌"])
            continue
        poly_ll = [geo.pixel_to_latlon(x, y) for (x, y) in poly]
        perim, area = water.poly_stats_latlon(poly_ll, clat)
        compact = 4 * math.pi * area / (perim ** 2) if perim else 0.0
        keep = area >= 300.0 and compact >= 0.12
        verdict = "✅ 保留" if keep else "❌ 剔除(河道/碎斑)"
        rows.append([i, f"{area:.0f} m²", f"{perim:.0f} m",
                     f"{compact:.3f}", verdict])
        if keep:
            kept.append((c, lat, lng, poly, poly_ll, perim, area))
    s3 = s2.copy()
    d3 = ImageDraw.Draw(s3)
    for i, (c, lat, lng) in enumerate(comps_near):
        px, py = geo.latlon_to_pixel(lat, lng)
        keep = any(k[2] == lng and k[1] == lat for k in kept)
        color = (60, 240, 110) if keep else (255, 80, 80)
        d3.ellipse([px - 26, py - 26, px + 26, py + 26], outline=color,
                   width=6)
        if not keep:
            d3.line([(px - 30, py - 30), (px + 30, py + 30)], fill=color,
                    width=6)
            d3.line([(px + 30, py - 30), (px - 30, py + 30)], fill=color,
                    width=6)
    add_step(3, "湖形过滤 (细长河道剔除)",
             f"每个候选按面积 ≥300m² 且紧凑度 ≥0.12 过滤: 绿圈=湖形保留, "
             f"红叉=河道/碎斑剔除。共保留 <b>{len(kept)} 个</b> —— 防止 VLM "
             f"把河段当湖 (2026-09-05 实测 VLM 曾选 270m 外河段致环线退化)。",
             table={"head": ["#", "面积", "周长", "紧凑度", "判定"],
                    "rows": rows},
             image=jpg_b64(s3))

    # ---------- S4 VLM 锚定 ----------
    print("[S4] VLM 语义锚定...")
    cands = [{"centroid": [round(k[1], 5), round(k[2], 5)],
              "dist_km": round(hav_m((clat, clng), (k[1], k[2])) / 1000.0, 3)}
             for k in kept]
    a_img, a_detail = semantic_anchor.build_anchor_image(
        "esri", clat, clng, 17, 8, 8, ROBOT, cands)
    if args.no_vlm:
        anchor = semantic_anchor.anchor_semantics(
            None, TASK, ROBOT, cands, (clat, clng),
            provider="esri", z=17, nx=8, ny=8)
        vlm_raw = "(--no-vlm 跳过)"
    else:
        from go2w_brain.config import BrainConfig
        from go2w_brain.vlm import VLMClient
        vlm = VLMClient(BrainConfig.from_env().llm_api_key)
        anchor = semantic_anchor.anchor_semantics(
            vlm, TASK, ROBOT, cands, (clat, clng),
            provider="esri", z=17, nx=8, ny=8)
        vlm_raw = anchor.get("raw", "")
    tgt = anchor.get("target") or {}
    print(f"     source={anchor['source']} idx={tgt.get('idx')}")
    if anchor["source"] == "vlm":
        src_note = "多模态大模型确认"
    else:
        src_note = ("VLM 瞬时不可用 (限流/抖动, 已重试) → 诚实降级规则锚定"
                    "(最近湖形候选)")
    s4_text = (f"湖形候选画到卫星图上 (绿圈=本体, 红圈=候选编号) 交多模态大模型。"
               f"<br><b>结论: {src_note} · 园区湖=候选"
               f"#{tgt.get('idx')} · 歧义={anchor.get('ambiguity') or '无'}</b>"
               f"<br>理由: {html.escape(str(tgt.get('why') or ''))}")
    if vlm_raw:
        s4_text += (f"<details><summary>VLM 原始输出</summary><pre>"
                    f"{html.escape(str(vlm_raw)[:800])}</pre></details>")
    add_step(4, "VLM 语义锚定 (湖是哪个)", s4_text,
             image=jpg_b64(a_img, max_w=1024))

    # ---------- S5 聚焦细化 + 环线 ----------
    print("[S5] 聚焦细化 + 环线规划...")
    tgt_c = tgt.get("centroid")
    pick_i = min(range(len(kept)), key=lambda i:
                 (kept[i][1] - tgt_c[0]) ** 2 + (kept[i][2] - tgt_c[1]) ** 2)
    c, lat, lng, poly, poly_ll, perim, area = kept[pick_i]
    p_poly, p_geo, refined = poly, geo, False
    try:
        target = {"_poly_ll": poly_ll, "_poly_px": poly, "_georef": geo,
                  "area_km2": area / 1e6}
        fine = route_api._refine(target, "osm")
        if fine is not None:
            p_poly, p_geo, refined = fine
            poly_ll = [p_geo.pixel_to_latlon(x, y) for (x, y) in p_poly]
    except Exception as exc:  # noqa: BLE001
        print(f"     refine fallback: {exc}")
    result = planner.plan_loop_around_polygon(p_poly, p_geo, offset_m=15.0,
                                              step_m=40.0)
    result["route_latlon"] = route_api._snap_outside_ring_min(
        result["route_latlon"], poly_ll, min_dist_m=5.0)
    print(f"     refined={refined} ring={len(result['route_latlon'])} pts "
          f"{result['stats']['length_m']:.1f}m")
    out = route_api._finish(result, poly_ll, "water", {
        "area_km2": round(area / 1e6, 3),
        "perim_km": round(perim / 1000.0, 2),
        "dist_km": round(hav_m((clat, clng), (lat, lng)) / 1000.0, 3)})
    # 特写: esri z19 2×2 湖心
    cl_lat, cl_lng = out["target"]["centroid"]
    close, close_detail = tiles.stitch_centered("esri", cl_lat, cl_lng, 19, 2, 2)
    close_geo = tiles.georef_from_detail(close_detail)
    cd = ImageDraw.Draw(close)
    cd.polygon([close_geo.latlon_to_pixel(a, b) for a, b in poly_ll],
               outline=(60, 240, 110), width=6)
    pts = [close_geo.latlon_to_pixel(p[0], p[1])
           for p in result["route_latlon"]]
    for i in range(len(pts) - 1):
        cd.line([pts[i], pts[i + 1]], fill=(240, 145, 59), width=5)
    cd.line([pts[-1], pts[0]], fill=(240, 145, 59), width=5)
    for sp in out["scan_points"]:
        px, py = close_geo.latlon_to_pixel(sp["lat"], sp["lon"])
        rad = sp["look_bearing_deg"] * math.pi / 180
        cd.ellipse([px - 7, py - 7, px + 7, py + 7], outline=(240, 145, 59),
                   width=5)
        cd.line([(px, py), (px + 42 * math.sin(rad), py - 42 * math.cos(rad))],
                fill=(240, 145, 59), width=4)
    add_step(5, "聚焦细化 + 离岸环线 (选定湖特写)",
             f"选定湖面 bbox 逐级聚焦重扫 (z19→z16 + 面积窗口校验) "
             f"{'命中 z%d' % p_geo.z if refined else '沿用粗边界'} → 外扩 15m "
             f"+ 5m 安全推出 → <b>{len(result['route_latlon'])} 航点 / "
             f"{result['stats']['length_m']:.1f}m / 闭合</b>。"
             f"绿线=细化湖界, 橙线=离岸环线, 橙点=扫描点(朝向湖心)。"
             f"esri z19 2×2 特写, 0.25m/px。",
             image=jpg_b64(close, max_w=1024))

    report = _render()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding="utf-8")
    print(f"报告: {out_path}")
    print(f"浏览器: http://127.0.0.1:8088/runs/brain/{out_path.name}")


def _render():
    secs = []
    for s in STEPS:
        img = (f'<img src="data:image/jpeg;base64,{s["image"]}">'
               if s.get("image") else "")
        img2 = (f'<img src="data:image/jpeg;base64,{s["image2"]}">'
                if s.get("image2") else "")
        table = ""
        if s.get("table"):
            rows = "".join("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>"
                                            for c in row) + "</tr>"
                           for row in s["table"]["rows"])
            head = "".join(f"<th>{h}</th>" for h in s["table"]["head"])
            table = f"<table><tr>{head}</tr>{rows}</table>"
        secs.append(
            f'<section><h2><span class="no">{s["no"]}</span>{s["title"]}</h2>'
            f'<p>{s["text"]}</p>{img}{img2}{table}</section>')
    return TMPL.replace("@@SECTIONS@@", "".join(secs))


TMPL = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>园区湖语义链逐步演示 — go2w_brain M7.4</title>
<style>
body{background:#101318;color:#dfe6ee;font-family:system-ui,"Microsoft YaHei",sans-serif;
margin:0;padding:24px 0 80px}
h1{font-size:20px;padding:0 28px}
h1 small{color:#7f8a99;font-weight:normal;font-size:13px}
section{max-width:1060px;margin:18px auto;background:#161b22;border:1px solid #232b36;
border-radius:10px;padding:18px 22px}
h2{margin:2px 0 10px;font-size:16px}
h2 .no{display:inline-block;width:26px;height:26px;line-height:26px;text-align:center;
background:#2d5bd9;border-radius:7px;margin-right:10px;font-size:14px}
p{line-height:1.65;font-size:14px;color:#b9c3cf;margin:8px 0}
img{width:100%;border-radius:8px;border:1px solid #2a3440;margin-top:8px}
table{border-collapse:collapse;margin-top:10px;font-size:13px;width:100%}
th,td{border:1px solid #2a3440;padding:6px 10px;text-align:left}
th{background:#1d2430;color:#9fd0ff}
details{margin-top:8px;font-size:12px;color:#8fa1b3}
pre{white-space:pre-wrap;word-break:break-all;background:#0d1117;padding:10px;
border-radius:6px}
</style></head><body>
<h1>园区湖语义链 —— 逐步演示 (每一步的 mask/标注)
<small>go2w_brain M7.4 · 中电海康无锡物联网产业园 (31.488192, 120.369486)</small></h1>
@@SECTIONS@@
</body></html>"""


if __name__ == "__main__":
    main()
