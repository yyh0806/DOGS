"""campus_lake_walkthrough.py — 园区湖语义链逐步演示 (VLM 直接 mask 版)。

每步一张 mask/标注图 (2026-09-05 用户要求: 卫星图上直接用 VLM 做 mask
圈出园区, 再圈园区里的湖 —— 不是画半径圆/换街道图):
  S1 园区识别: 知识库/本体位置 → 园区名 + 中心
  S2 VLM 圈园区: 卫星 z17 → VLM 直接勾画园区边界 → 填充 mask 叠加
  S3 VLM 圈湖: 园区 mask bbox 放大 (z18) → VLM 勾画湖岸线 → 填充 mask 叠加
     (VLM 失败 → 规则后备并如实标注)
  S4 沿湖环线: 湖 z19 特写 + 离岸环线 + 扫描点

用法 (web/ 目录): python ..\\tools\\campus_lake_walkthrough.py [--no-vlm]
输出: runs/brain/campus_lake_walkthrough.html (自包含 base64 图)。
"""
from __future__ import annotations

import argparse
import base64
import html
import io
import math
import sys
from pathlib import Path

_WEB = Path(__file__).resolve().parents[1] / "web"
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from lake_plan import (planner, route_api, tiles, vlm_mask,  # noqa: E402
                       water)
from lake_plan.osm_client import _point_in_ring  # noqa: E402
from go2w_brain.tools.plan_campus_lake import (  # noqa: E402
    CAMPUSES, _hav_m, _mc_iou, _osm_lake_candidates, _osm_lake_final,
    _zoom_to_bbox)

CAMPUS = CAMPUSES[0]
CAMPUS_Z = 18   # 园区 mask 层级 (±520m, 园区充满画面)
LAKE_Z, LAKE_NX, LAKE_NY = 19, 4, 4  # 湖 mask 特写层级
TASK = "绕着当前园区湖绕行一圈"
ROBOT = {"lat": CAMPUS["lat"], "lng": CAMPUS["lng"]}
STEPS: list[dict] = []


def add_step(no, title, text, image=None, table=None, image2=None):
    STEPS.append({"no": no, "title": title, "text": text,
                  "image": image, "table": table, "image2": image2})


def jpg_b64(img, max_w=1024, quality=82):
    if img.size[0] > max_w:
        img = img.resize((max_w, int(img.size[1] * max_w / img.size[0])))
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def fill_mask(img, poly_px, color, alpha=90, outline_w=4):
    """多边形 mask: 半透明填充 + 描边。"""
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    d.polygon([(x, y) for x, y in poly_px], fill=color + (alpha,))
    out = Image.alpha_composite(img.convert("RGBA"), overlay)
    d = ImageDraw.Draw(out)
    d.polygon([(x, y) for x, y in poly_px], outline=color, width=outline_w)
    return out.convert("RGB")


def marker(draw, x, y, color, r, label=None, font=None):
    draw.ellipse([x - r, y - r, x + r, y + r], outline=color, width=5)
    draw.line([(x - r - 7, y), (x + r + 7, y)], fill=color, width=3)
    draw.line([(x, y - r - 7), (x, y + r + 7)], fill=color, width=3)
    if label:
        draw.text((x + r + 5, y - r - 5), label, font=font, fill=color)


def _centroid(ll):
    return (sum(p[0] for p in ll) / len(ll), sum(p[1] for p in ll) / len(ll))


def _build_vlm(config_key: str = "llm_api_key"):
    from go2w_brain.vlm import build_vlm
    return build_vlm()


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-vlm", action="store_true")
    ap.add_argument("--out", default="runs/brain/campus_lake_walkthrough.html")
    args = ap.parse_args(argv)

    font_big = ImageFont.load_default(size=40)
    font_mid = ImageFont.load_default(size=28)
    clat, clng = CAMPUS["lat"], CAMPUS["lng"]
    vlm = None if args.no_vlm else _build_vlm()

    # ---------- S1 园区识别 ----------
    print("[S1] 园区识别...")
    add_step(1, "园区识别 (任务 → 园区实体)",
             f"任务含\"园区湖\" → 园区知识库 (名称/别名 + 本体位置兜底) → "
             f"<b>{CAMPUS['name']}</b>, 中心 ({clat}, {clng})。"
             f"这一步只确定\"是哪个园区\", 不画几何。")

    # ---------- S2 VLM 圈园区 (mask) ----------
    print("[S2] VLM 圈园区 (z18 特写)...")
    sat, sat_detail = tiles.stitch_centered("esri", clat, clng, CAMPUS_Z,
                                            8, 8)
    sat_geo = tiles.georef_from_detail(sat_detail)
    campus_poly = vlm_mask.vlm_polygon(vlm, sat, vlm_mask.CAMPUS_MASK_PROMPT)
    s2 = sat.convert("RGB")
    d2 = ImageDraw.Draw(s2)
    rx, ry = sat_geo.latlon_to_pixel(clat, clng)
    marker(d2, rx, ry, (60, 240, 110), 18, "本体", font_mid)
    if campus_poly is not None:
        campus_ll = vlm_mask.poly_to_latlon(campus_poly["polygon"], sat_geo)
        px = [sat_geo.latlon_to_pixel(a, b) for a, b in campus_ll]
        s2 = fill_mask(s2, px, (255, 120, 60))
        src_note = ("VLM 直接看卫星图勾画园区边界 (红橙填充 = 园区 mask, "
                    f"{len(campus_ll) - 1} 顶点): {campus_poly.get('why') or ''}")
    else:
        campus_ll = vlm_mask.circle_poly(clat, clng, CAMPUS["radius_m"])
        px = [sat_geo.latlon_to_pixel(a, b) for a, b in campus_ll]
        s2 = fill_mask(s2, px, (255, 120, 60))
        src_note = "VLM 不可用/退化 → 规则半径圆后备 (红橙填充 = 园区近似 mask)"
    print(f"     campus_mask: {'vlm' if campus_poly else 'rule'} "
          f"{len(campus_ll) - 1} 顶点")
    add_step(2, "VLM 圈园区 (卫星图直接 mask)",
             src_note + "。绿圈=本体。", image=jpg_b64(s2))

    # ---------- S3 湖定位 (OSM) → VLM 在 z19 湖心特写窗上圈湖 ----------
    print("[S3] VLM 圈湖 (z19 湖心特写)...")
    osm_cands = _osm_lake_candidates(CAMPUS, clat, clng, campus_ll,
                                     tiles, water)
    nudge = _centroid(osm_cands[0][0]) if osm_cands else (clat, clng)
    if osm_cands:
        lake_img, ld = tiles.stitch_centered("esri", nudge[0], nudge[1],
                                             LAKE_Z, LAKE_NX, LAKE_NY)
        lake_geo = tiles.georef_from_detail(ld)
    else:
        lake_img, lake_geo = _zoom_to_bbox("esri", campus_ll, z=18)
    lake_poly = vlm_mask.vlm_polygon(vlm, lake_img, vlm_mask.LAKE_MASK_PROMPT)
    lake_ll, lake_src, lake_iou = None, "rule", None
    if lake_poly is not None:
        cand = vlm_mask.poly_to_latlon(lake_poly["polygon"], lake_geo)
        c = _centroid(cand)
        frac = vlm_mask.poly_area_frac(lake_poly["polygon"])
        if _point_in_ring(c, campus_ll) and 0.002 <= frac <= 0.30:
            lake_ll = cand
            if osm_cands:
                lake_iou = _mc_iou(lake_ll, osm_cands[0][0])
    s3 = lake_img.convert("RGB")
    if lake_ll is None:
        # 规则后备: OSM 湖形候选 (含精修在 S4)
        if osm_cands:
            lake_ll = osm_cands[0][0]
            lake_geo = osm_cands[0][1]
        src_note = ("VLM 圈湖失败/被拒收 → 规则后备: OSM 渲染水体 + 湖形"
                    "过滤 (面积/紧凑度) + 园区 mask 内最近。"
                    if lake_ll else "无湖形水体 (诚实失败)")
    else:
        src_note = (f"VLM 在 z19 湖心特写上勾画湖岸线 (蓝填充 = 湖 mask, "
                    f"{len(lake_ll) - 1} 顶点)"
                    + (f"; 与 OSM 真值 IoU = <b>{lake_iou:.2f}</b>"
                       f"{' ✅≥0.3 确认' if (lake_iou or 0) >= 0.3 else ' ❌<0.3 不可信'}"
                       if lake_iou is not None else "")
                    + f": {lake_poly.get('why') or ''}")
    if lake_ll:
        px = [lake_geo.latlon_to_pixel(a, b) for a, b in lake_ll]
        s3 = fill_mask(s3, px, (60, 130, 255), alpha=100)
    d3 = ImageDraw.Draw(s3)
    marker(d3, *lake_geo.latlon_to_pixel(clat, clng), (60, 240, 110), 16,
           "本体", font_mid)
    add_step(3, "VLM 圈湖 (z19 湖心特写 + IoU 真值校验)",
             src_note + f"。底图 z{LAKE_Z} {LAKE_NX}×{LAKE_NY} 湖心特写窗。",
             image=jpg_b64(s3))

    # ---------- S4 湖岸线精修 + 沿湖环线 ----------
    print("[S4] 湖岸线精修 + 沿湖环线...")
    if lake_ll is None:
        add_step(4, "沿湖环线规划", "园区内未找到可规划的湖形水体 —— 如实终止。")
    else:
        # 湖岸线精修 (与工具同路径): OSM 候选 → z19..z16 聚焦重扫
        refined = False
        if osm_cands:
            _null_log = type("L", (), {"append":
                                       staticmethod(lambda *a, **k: None)})
            final = _osm_lake_final({"log": _null_log()},
                                    osm_cands, clat, clng, campus_ll,
                                    nudge, route_api)
            if final is not None:
                lake_ll, lake_geo = final
                refined = True
        poly_px = [lake_geo.latlon_to_pixel(a, b) for a, b in lake_ll]
        result = planner.plan_loop_around_polygon(poly_px, lake_geo,
                                                  offset_m=15.0, step_m=40.0)
        result["route_latlon"] = route_api._snap_outside_ring_min(
            result["route_latlon"], lake_ll, min_dist_m=5.0)
        out = route_api._finish(result, lake_ll, "water", {
            "area_km2": 0.001, "perim_km": 0.3, "dist_km": 0.1})
        c_lat, c_lng = _centroid(lake_ll)
        close, cd_detail = tiles.stitch_centered("esri", c_lat, c_lng,
                                                 19, 2, 2)
        cgeo = tiles.georef_from_detail(cd_detail)
        px = [cgeo.latlon_to_pixel(a, b) for a, b in lake_ll]
        close = fill_mask(close, px, (60, 130, 255), alpha=60)
        cd = ImageDraw.Draw(close)
        pts = [cgeo.latlon_to_pixel(p[0], p[1])
               for p in result["route_latlon"]]
        for i in range(len(pts) - 1):
            cd.line([pts[i], pts[i + 1]], fill=(240, 145, 59), width=5)
        cd.line([pts[-1], pts[0]], fill=(240, 145, 59), width=5)
        for sp in out["scan_points"]:
            x, y = cgeo.latlon_to_pixel(sp["lat"], sp["lon"])
            rad = sp["look_bearing_deg"] * math.pi / 180
            cd.ellipse([x - 7, y - 7, x + 7, y + 7], outline=(240, 145, 59),
                       width=5)
            cd.line([(x, y), (x + 42 * math.sin(rad),
                              y - 42 * math.cos(rad))],
                    fill=(240, 145, 59), width=4)
        add_step(4, "湖岸线精修 + 沿湖环线 (z19 特写)",
                 f"蓝=湖 mask (VLM 或规则), 绿=精修岸线"
                 f"({'已聚焦重扫' if refined else '沿用手动 mask'}), "
                 f"橙=离岸环线 (15m+5m安全推出): "
                 f"<b>{len(result['route_latlon'])} 航点 / "
                 f"{result['stats']['length_m']:.1f}m / "
                 f"闭合={result['stats']['closed']}</b>, 扫描点 "
                 f"{len(out['scan_points'])} 个。",
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
        secs.append(
            f'<section><h2><span class="no">{s["no"]}</span>{s["title"]}</h2>'
            f'<p>{s["text"]}</p>{img}</section>')
    return TMPL.replace("@@SECTIONS@@", "".join(secs))


TMPL = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>园区湖语义链 — VLM 直接 mask 演示</title>
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
</style></head><body>
<h1>园区湖语义链 —— VLM 直接在卫星图上做 mask
<small>中电海康无锡物联网产业园 (31.488192, 120.369486)</small></h1>
@@SECTIONS@@
</body></html>"""


if __name__ == "__main__":
    main()
