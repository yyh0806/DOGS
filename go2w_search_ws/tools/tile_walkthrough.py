"""tile_walkthrough.py — 瓦片分析逐步演示 (M7.3)。

把 plan_lake_loop 的确定性管线拆成 9 个可见步骤, 每步:
- 调用真实管线函数 (与大脑执行的是同一份代码);
- 保存中间产物图像 (拼接图/掩膜/连通域/环线/锚定图);
- 打印数值证据;
- 汇总进一个自包含 HTML 报告 (图片 base64 内嵌)。

用法 (在 web/ 目录下):
  python ..\\tools\\tile_walkthrough.py [--no-vlm] [--out runs/brain/tile_walkthrough.html]
"""
from __future__ import annotations

import argparse
import base64
import html
import io
import json
import sys
from pathlib import Path

_WEB = Path(__file__).resolve().parents[1] / "web"
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from lake_plan import config, planner, plan_route, route_api, tiles, water  # noqa: E402
from lake_plan.geo import haversine_m  # noqa: E402

CENTER = (31.488192, 120.369486)  # 太科园园区 (基准点 = 机器人 GNSS)
ROBOT = {"lat": CENTER[0], "lng": CENTER[1]}
TASK = "绕湖一周，并巡查有没有落水人员"

STEPS: list[dict] = []  # {no, title, text, image, table}


def add_step(no, title, text, image=None, table=None, image2=None):
    STEPS.append({"no": no, "title": title, "text": text,
                  "image": image, "table": table, "image2": image2})


def jpg_b64(img: Image.Image, max_w=1024, quality=82) -> str:
    if img.size[0] > max_w:
        img = img.resize((max_w, int(img.size[1] * max_w / img.size[0])))
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def overlay_polyline(img, pts_ll, georef, color, width=3, dash=None):
    draw = ImageDraw.Draw(img)
    pts = [georef.latlon_to_pixel(a, b) for a, b in pts_ll]
    for i in range(len(pts) - 1):
        draw.line([pts[i], pts[i + 1]], fill=color, width=width)
    draw.line([pts[-1], pts[0]], fill=color, width=width)
    return draw


def marker(draw, x, y, color, r, label=None):
    draw.ellipse([x - r, y - r, x + r, y + r], outline=color, width=4)
    draw.line([(x - r - 6, y), (x + r + 6, y)], fill=color, width=2)
    draw.line([(x, y - r - 6), (x, y + r + 6)], fill=color, width=2)
    if label:
        draw.text((x + r + 4, y - r - 4), label, fill=color)


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-vlm", action="store_true")
    ap.add_argument("--out", default="runs/brain/20260901_tile_walkthrough.html")
    args = ap.parse_args(argv)

    provider = "osm"  # 规划底图 (矢量, 水色可分割); 卫星只用于语义锚定
    print("=" * 70)
    print(f"基准点: {CENTER} (机器人 GNSS)  底图: {provider}")
    print(f"缓存: {config.cache_dir()}  offline={config.offline()}")
    print("=" * 70)

    # ---------- S0 感知窗口 ----------
    add_step(0, "感知窗口 (本地两级, 2026-09-04)",
             "命令通常针对周边 → 感知只扫本地窗口: 最细可用的 z17 8×8"
             "(约 ±1.0km, z18/z19 上 OSM 会把小湖与邻近水渠渲染成一体而被"
             "淘汰) 找不到再放宽 z16 8×8 (约 ±2.4km)。不再逐层远扫城区。"
             "找到合格水体即停, 选距离最近者; 精确岸线由 S5 聚焦重扫给出。")
    print("[S0] 感知窗口 z17(8×8, ±1.0km) → z16(8×8, ±2.4km)")

    # ---------- S1 瓦片拼接 ----------
    print("[S1] 瓦片获取与拼接 (z17, 8×8)...")
    img, detail = tiles.stitch_centered(provider, *CENTER, 17, 8, 8)
    georef = tiles.georef_from_detail(detail)
    span_m = img.size[0] * georef.mppx()
    print(f"     尺寸 {img.size}  命中 {detail['tiles_ok']}/64 "
          f"未命中 {len(detail['tiles_miss'])}  (约 ±{span_m / 2:.0f}m)")
    s1 = img.copy()
    d1 = ImageDraw.Draw(s1)
    rx, ry = georef.latlon_to_pixel(*CENTER)
    marker(d1, rx, ry, (60, 240, 110), 14, "本体")
    add_step(1, "瓦片获取与拼接 (OSM z17, 8×8=64 张)",
             f"64 张 256px 瓦片拼成 {img.size[0]}×{img.size[1]} 影像, 覆盖本体"
             f"周围约 ±{span_m / 2:.0f}m (周边尺度)。缓存命中 "
             f"{detail['tiles_ok']}/64, 未命中 {len(detail['tiles_miss'])} 张。"
             f"绿圈=机器人本体。",
             image=jpg_b64(s1))

    # ---------- S2 水域分割 ----------
    print("[S2] 水域颜色分割...")
    mask = water.water_mask(img, ref_rgb=config.PROVIDERS[provider]["water_rgb"])
    ratio = float(mask.mean())
    print(f"     水像素占比 {ratio*100:.2f}%  (HSV 蓝青相 + 参考色距离双条件)")
    s2 = img.copy()
    arr = np.asarray(s2.convert("RGB")).copy()
    arr[mask] = (arr[mask] * 0.25 + np.array([40, 120, 255]) * 0.75).astype(np.uint8)
    s2 = Image.fromarray(arr)
    d2 = ImageDraw.Draw(s2)
    marker(d2, rx, ry, (60, 240, 110), 14, "本体")
    add_step(2, "水域颜色分割 (HSV + 参考色)",
             f"对每个像素判两个条件: ①HSV 色相在蓝青窗 (170°~240°) 且饱和度/明度"
             f"达标; ②RGB 与 OSM 水色参考 (170,211,223) 距离 ≤70。同时满足才认"
             f"为水。本图水像素占 {ratio*100:.2f}% (蓝色高亮)。",
             image=jpg_b64(s2))

    # ---------- S3 连通域 ----------
    print("[S3] 连通域标记...")
    comps = water.label_components(mask, down=4, min_area_px=200)
    print(f"     连通域 {len(comps)} 个 (按面积降序)")
    palette = [tuple(int(255 * c) for c in hsv) for hsv in _palette(12)]
    s3 = Image.new("RGB", img.size, (24, 24, 28))
    draw3 = ImageDraw.Draw(s3)
    font3 = ImageFont.load_default(size=26)
    top = comps[:12]
    for i, c in enumerate(top):
        full = water.comp_to_full(c, mask.shape)
        color_img = np.zeros((*mask.shape, 3), dtype=np.uint8)
        color_img[full] = palette[i]
        s3.paste(Image.fromarray(color_img), (0, 0), Image.fromarray(
            (full * 255).astype(np.uint8)))
        y0, x0, y1, x1 = c["bbox_small"]
        draw3.rectangle([x0 * 4, y0 * 4, x1 * 4, y1 * 4],
                        outline=palette[i], width=3)
        cy, cx = c["centroid_small"]
        draw3.text((cx * 4, cy * 4), str(i), font=font3,
                   fill=(255, 255, 255), stroke_width=3,
                   stroke_fill=(0, 0, 0), anchor="mm")
    marker(draw3, rx, ry, (60, 240, 110), 14, "本体")
    add_step(3, "连通域标记 (BFS, 按面积取前 12)",
             f"4×4 下采样后 BFS 洪水填充, 最小 200px²。共 {len(comps)} 个水斑,"
             f" 这里标出面积前 12 (编号即后续候选编号)。",
             image=jpg_b64(s3))

    # ---------- S4 候选筛选 ----------
    print("[S4] 候选摘要与筛选...")
    cands = route_api._candidates(comps, georef, mask.shape, *CENTER)
    rows = []
    for i, c in enumerate(cands):
        usable = (not c["clipped"]
                  and config.DEFAULT_MIN_PERIM_KM <= c["perim_km"] <= config.DEFAULT_MAX_PERIM_KM
                  and c["compact"] >= config.COMPACT_MIN)
        rows.append([i, c["area_km2"], c["perim_km"], c["compact"],
                     "贴边" if c["clipped"] else "",
                     round(c["dist_km"], 2),
                     "✅ 合格" if usable else "❌ 淘汰"])
        print(f"     #{i} area={c['area_km2']}km² perim={c['perim_km']}km "
              f"compact={c['compact']} dist={c['dist_km']}km clipped={c['clipped']} "
              f"→ {'合格' if usable else '淘汰'}")
    add_step(4, "候选筛选 (周长/紧凑度/贴边)",
             f"共 {len(cands)} 个候选。淘汰规则: 贴边(不完整)、周长<"
             f"{config.DEFAULT_MIN_PERIM_KM}km 或 >{config.DEFAULT_MAX_PERIM_KM}km"
             f"(荒谬过滤)、紧凑度<{config.COMPACT_MIN}(线状河渠不是湖)。",
             table={"head": ["#", "面积km²", "周长km", "紧凑度", "贴边",
                             "距本体km", "判定"],
                    "rows": rows})

    usable = [c for c in cands
              if not c["clipped"]
              and config.DEFAULT_MIN_PERIM_KM <= c["perim_km"] <= config.DEFAULT_MAX_PERIM_KM
              and c["compact"] >= config.COMPACT_MIN]

    # ---------- S5 选湖 + 细化 ----------
    print("[S5] 选湖 (最近) + 聚焦重扫...")
    target = min(usable, key=lambda c: c["dist_km"])
    print(f"     选中 #{cands.index(target)} 质心={target['centroid']} "
          f"距本体 {target['dist_km']}km")
    refined = False
    poly_px, georef_plan = target["_poly_px"], target["_georef"]
    try:
        fine = route_api._refine(target, provider)
        if fine is not None:
            poly_px, georef_plan, refined = fine
            print(f"     聚焦重扫命中: z{georef_plan.z} "
                  f"({georef_plan.w // 256}x{georef_plan.h // 256} 瓦片)")
        else:
            print("     聚焦重扫未命中 (各层级均粘连/退化, 沿用粗扫多边形)")
    except tiles.TileError as exc:
        print(f"     聚焦重扫降级: {exc}")
    # M7.3 聚焦视图: 用规划器实际使用的层级出近景 (而不是 z16 大视野)
    poly_ll = [georef_plan.pixel_to_latlon(x, y) for (x, y) in poly_px]
    if refined:
        fine_img, _ = tiles.stitch_area(
            provider, georef_plan.z, georef_plan.x0, georef_plan.y0,
            georef_plan.w // 256, georef_plan.h // 256)
    else:
        clat, clng = target["centroid"]
        fine_img, fd = tiles.stitch_centered(provider, clat, clng, 17, 4, 3)
        georef_plan = tiles.georef_from_detail(fd)
        # 近景窗口换了地理参照 → 粗扫多边形换算到新窗口像素
        poly_px = [georef_plan.latlon_to_pixel(a, b)
                   for a, b in target["_poly_ll"]]
    fine_mask = water.water_mask(
        fine_img, ref_rgb=config.PROVIDERS[provider]["water_rgb"])
    s5 = fine_img.copy()
    arr5 = np.asarray(s5.convert("RGB")).copy()
    arr5[fine_mask] = (arr5[fine_mask] * 0.35
                       + np.array([40, 120, 255]) * 0.65).astype(np.uint8)
    s5 = Image.fromarray(arr5)
    d5 = ImageDraw.Draw(s5)
    d5.polygon([georef_plan.latlon_to_pixel(a, b)
                for a, b in target["_poly_ll"]],
               outline=(90, 140, 255), width=2)  # 粗扫多边形
    d5.polygon([georef_plan.latlon_to_pixel(a, b) for a, b in poly_ll],
               outline=(60, 240, 110), width=4)  # 细化多边形
    marker(d5, *georef_plan.latlon_to_pixel(*CENTER),
           (60, 240, 110), 12, "本体")
    add_step(5, "选湖 + 聚焦重扫 (近景)",
             f"合格候选里取距本体最近的: #{cands.index(target)}。随后对湖面 bbox "
             f"逐级聚焦重扫 (z19→z16, 四周各留 1 瓦片边距), 并按面积窗口 "
             f"(粗扫的 0.4×~2.5×) 校验: 园区湖在高倍级上会与邻近水渠渲染成一体"
             f" (z19~z17 实测粘连 → 弃用), 命中 "
             f"{'z%d 聚焦重扫' % georef_plan.z if refined else '失败, 沿用粗扫边界'}。"
             f"蓝线=粗扫多边形, 绿线=细化多边形 (面积 ×1.03)。",
             image=jpg_b64(s5))

    # ---------- S5.5 卫星特写 ----------
    print("[S5.5] 最大倍率卫星特写 (esri z19, 2×2)...")
    clat, clng = target["centroid"]
    sat_img, sat_detail = tiles.stitch_centered("esri", clat, clng, 19, 2, 2)
    sat_geo = tiles.georef_from_detail(sat_detail)
    sat_span = sat_img.width * sat_geo.mppx()
    print(f"     {sat_img.size}  命中 {sat_detail['tiles_ok']}/4  "
          f"(约 {sat_span:.0f}m 见方, {sat_geo.mppx():.2f} m/px)")
    sd = ImageDraw.Draw(sat_img)
    sd.polygon([sat_geo.latlon_to_pixel(a, b)
                for a, b in target["_poly_ll"]],
               outline=(90, 140, 255), width=2)  # 粗扫
    sd.polygon([sat_geo.latlon_to_pixel(a, b) for a, b in poly_ll],
               outline=(60, 240, 110), width=4)  # 细化
    rp = sat_geo.latlon_to_pixel(*CENTER)
    if 0 <= rp[0] < sat_img.width and 0 <= rp[1] < sat_img.height:
        marker(sd, *rp, (60, 240, 110), 12, "本体")
    add_step("5.5", "最大倍率卫星特写 (esri z19, 2×2=4 张瓦片)",
             f"公共 Esri 服务在此点位的最高真实层级是 z19 (z20+ 返回无影像占位,"
             f" OSM 同样止于 z19) —— {sat_geo.mppx():.2f} m/px, 4 张瓦片拼成约 "
             f"{sat_span:.0f}m 见方, 湖占满画面, 岸线肉眼可辨。"
             f"蓝线=粗扫边界, 绿线=细化边界。",
             image=jpg_b64(sat_img, max_w=1024))

    # ---------- S6 离岸环线 ----------
    print("[S6] 离岸环线规划 (外扩 15m)...")
    result = planner.plan_loop_around_polygon(poly_px, georef_plan,
                                              offset_m=15.0, step_m=40.0)
    result["route_latlon"] = route_api._snap_outside_ring_min(
        result["route_latlon"], poly_ll, min_dist_m=5.0)
    stats = result["stats"]
    print(f"     航点 {len(result['route_latlon'])}  长度 {stats['length_m']:.1f}m "
          f"闭合={stats['closed']}  压水比 {stats['water_cross_ratio']:.3f}")
    s6 = sat_img.copy()
    overlay_polyline(s6, result["route_latlon"], sat_geo,
                     (240, 145, 59), 4)
    add_step(6, "离岸环线规划 (膨胀外扩 + 5m 安全推出, 卫星特写)",
             f"细化多边形外扩 15m 得闭合环线, 再执行规划-安全一致性: 每个航点"
             f"必须距水域多边形 ≥5m (守卫 veto 2m + 布防 margin 2m + 1m 余量)。"
             f"结果: {len(result['route_latlon'])} 航点 / {stats['length_m']:.1f}m "
             f"/ 闭合={stats['closed']} / 压水比 {stats['water_cross_ratio']:.3f}。"
             f"绿线=湖面, 橙线=离岸环线 (画在 z19 卫星特写上)。",
             image=jpg_b64(s6, max_w=1024))

    # ---------- S7 扫描点 ----------
    print("[S7] 扫描点生成...")
    out = route_api._finish(result, poly_ll, "water", {
        "area_km2": target["area_km2"], "perim_km": target["perim_km"],
        "dist_km": target["dist_km"]})
    sp = out["scan_points"]
    print(f"     扫描点 {len(sp)} 个 (间距 {config.DEFAULT_SCAN_SPACING_M}m, "
          f"朝向湖心)")
    s7 = sat_img.copy()
    d7 = ImageDraw.Draw(s7)
    for p in sp:
        px, py = sat_geo.latlon_to_pixel(p["lat"], p["lon"])
        rad = p["look_bearing_deg"] * 3.14159 / 180
        d7.ellipse([px - 6, py - 6, px + 6, py + 6],
                   outline=(240, 145, 59), width=4)
        d7.line([(px, py),
                 (px + 40 * np.sin(rad), py - 40 * np.cos(rad))],
                fill=(240, 145, 59), width=3)
    add_step(7, "扫描点生成 (间距 150m, 朝向湖心, 卫星特写)",
             f"沿环线每 {config.DEFAULT_SCAN_SPACING_M}m 放一个扫描点, 附朝向"
             f"湖质心的真北方位角 —— 云台扫视的基准。共 {len(sp)} 个 (橙点+朝向线)。",
             image=jpg_b64(s7, max_w=1024))

    # ---------- S8 语义锚定 ----------
    print("[S8] 语义锚定 (esri 卫星 + VLM)...")
    from lake_plan import semantic_anchor
    candidates = [{"centroid": out["target"]["centroid"], "dist_km": 0.0}]
    candidates += route_api._slim([c for c in usable if c is not target])
    print(f"     候选水体 {len(candidates)} 个 (红圈编号)")
    a_img, a_detail = semantic_anchor.build_anchor_image(
        "esri", *CENTER, 16, 10, 8, ROBOT, candidates)
    print(f"     卫星图 {a_img.size}  命中 {a_detail['tiles_ok']}/80")
    if args.no_vlm:
        anchor = semantic_anchor.anchor_semantics(
            None, TASK, ROBOT, candidates, CENTER)
        vlm_raw = "(--no-vlm: 跳过)"
    else:
        from go2w_brain.config import BrainConfig
        from go2w_brain.vlm import VLMClient
        vlm = VLMClient(BrainConfig.from_env().llm_api_key)
        anchor = semantic_anchor.anchor_semantics(
            vlm, TASK, ROBOT, candidates, CENTER)
        vlm_raw = anchor.get("raw", "")
    print(f"     锚定 source={anchor['source']} "
          f"target={anchor.get('target')} ambiguity={anchor.get('ambiguity')}")
    tgt = anchor.get("target") or {}
    # 选定湖的 z19 卫星特写 (近景验证"湖是哪个")
    a2 = None
    chosen = tgt.get("centroid")
    if chosen:
        c_img, c_detail = tiles.stitch_centered(
            "esri", chosen[0], chosen[1], 19, 2, 2)
        c_geo = tiles.georef_from_detail(c_detail)
        cd = ImageDraw.Draw(c_img)
        cx, cy = c_geo.latlon_to_pixel(chosen[0], chosen[1])
        cd.ellipse([cx - 26, cy - 26, cx + 26, cy + 26],
                   outline=(255, 80, 60), width=6)
        cd.text((cx, cy), str(tgt.get("idx")), font=ImageFont.load_default(
            size=46), fill=(255, 255, 255), stroke_width=4,
            stroke_fill=(255, 80, 60), anchor="mm")
        rp = c_geo.latlon_to_pixel(ROBOT["lat"], ROBOT["lng"])
        if 0 <= rp[0] < c_img.width and 0 <= rp[1] < c_img.height:
            marker(cd, *rp, (40, 220, 90), 16, "本体")
        a2 = jpg_b64(c_img, max_w=1024)
    robot_dist = haversine_m(ROBOT["lat"], ROBOT["lng"],
                             chosen[0], chosen[1]) if chosen else 0.0
    a_text = (f"卫星影像 (esri z16, 10×8=80 张, 命中 {a_detail['tiles_ok']}/80)。"
              f"绿圈+字标=本体, 红圈=候选水体 (圈心即质心)。\n\n"
              f"<b>VLM 结论: source={anchor['source']} · 湖=候选#{tgt.get('idx')}"
              f" · 歧义={anchor.get('ambiguity') or '无'}</b>\n"
              f"理由: {html.escape(str(tgt.get('why') or anchor.get('why') or ''))}")
    if a2:
        a_text += (f"\n<br><b>选定湖特写 (esri z19, 2×2=4 张瓦片, 0.25m/px)</b>: "
                   f"红圈=候选#{tgt.get('idx')} 湖面质心"
                   f"(本体距湖心约 {robot_dist:.0f}m, 在特写窗口外)。")
    if vlm_raw:
        a_text += (f"\n<details><summary>VLM 原始输出</summary><pre>"
                   f"{html.escape(str(vlm_raw)[:800])}</pre></details>")
    add_step(8, "语义锚定 (卫星影像 × 多模态大模型)",
             a_text, image=jpg_b64(a_img, max_w=1024), image2=a2)

    # ---------- 交叉验证 ----------
    print("[S9] 全管线交叉验证 (plan_route 一次成型)...")
    full = plan_route(*CENTER, kind="water", offset_m=15.0)
    print(f"     plan_route: ok={full['ok']} 航点={len(full['waypoints'])} "
          f"长度={full['stats']['length_m']:.1f}m "
          f"扫描点={len(full['scan_points'])} "
          f"refined={full['stats'].get('refined')} "
          f"邻近候选={len(full.get('nearby_candidates') or [])}")
    match = (len(full["waypoints"]) == len(out["waypoints"])
             and abs(full["stats"]["length_m"] - stats["length_m"]) < 0.5)
    print(f"     分步结果与一次成型一致: {'✅ 是' if match else '❌ 否'}")

    report = _render_report(match)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding="utf-8")
    print(f"报告: {out_path}")
    print(f"浏览器打开: http://127.0.0.1:8088/runs/brain/{out_path.name}")


def _palette(n):
    import colorsys
    return [colorsys.hsv_to_rgb(i / n, 0.65, 1.0) for i in range(n)]


def _render_report(match):
    secs = []
    for s in STEPS:
        img = (f'<img src="data:image/jpeg;base64,{s["image"]}">'
               if s.get("image") else "")
        img2 = (f'<img src="data:image/jpeg;base64,{s["image2"]}">'
                if s.get("image2") else "")
        table = ""
        if s.get("table"):
            rows = "".join(
                "<tr>" + "".join(f"<td>{html.escape(str(c))}</td>"
                                 for c in row) + "</tr>"
                for row in s["table"]["rows"])
            head = "".join(f"<th>{h}</th>" for h in s["table"]["head"])
            table = f"<table><tr>{head}</tr>{rows}</table>"
        secs.append(
            f'<section><h2><span class="no">{s["no"]}</span>{s["title"]}</h2>'
            f'<p>{s["text"]}</p>{img}{img2}{table}</section>')
    return HTML_TMPL.replace("@@SECTIONS@@", "".join(secs)).replace(
        "@@MATCH@@", "✅ 一致" if match else "❌ 不一致")


HTML_TMPL = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>瓦片分析逐步演示 — go2w_brain M7.3</title>
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
.banner{max-width:1060px;margin:0 auto 12px;background:#13241a;border:1px solid #1f4d33;
border-radius:10px;padding:10px 22px;font-size:14px}
</style></head><body>
<h1>大脑如何分析瓦片地图 —— 逐步演示 <small>go2w_brain M7.3 · 无锡太科园 (31.488192, 120.369486)</small></h1>
<div class="banner">分步结果与一次成型 plan_route 交叉验证: <b>@@MATCH@@</b></div>
@@SECTIONS@@
</body></html>"""


if __name__ == "__main__":
    main()
