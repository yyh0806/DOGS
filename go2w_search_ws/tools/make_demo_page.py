#!/usr/bin/env python3
"""make_demo_page.py — 任务回放页生成器 (M5 演示可视化)。

把 go2w_brain 的 jsonl 轨迹 + lake_plan 规划数据渲染成一个自包含的
HTML 任务控制台: 左侧地图 (CARTO 深色底图 + 环线/水域/扫描点/告警),
右侧"黑匣子"时间线 (大脑每步思考与工具裁决, 拦截事件红色标记)。

用法:
  python tools/make_demo_page.py \
      --trace runs/brain/<ts>_brain.jsonl \
      --plan  runs/brain/m5_plan_view.json \
      --out   runs/brain/<ts>_mission_viewer.html \
      --title "M5 绕湖巡查干跑预演"
纯 stdlib; 地图与 Leaflet 走 CDN (需网络), 数据全部内嵌。
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

# ---- 工具名 → 中文 (操作员视角命名) ----
TOOL_ZH = {
    "get_gps": "读 GNSS 定位", "get_battery": "读电量",
    "get_pose": "读位姿", "speak": "语音播报",
    "plan_lake_loop": "规划绕湖环线", "plan_campus_loop": "规划绕园区环线",
    "follow_route": "受理航线", "cancel_route": "取消航线",
    "calibrate_heading": "北向标定", "get_route_state": "读航线进度",
    "arm_water_guard": "布防离水守卫", "disarm_water_guard": "解除守卫",
    "get_guard_state": "读守卫状态", "load_skill": "加载技能方法论",
    "scan_water": "扫描湖面检测落水者", "get_detection_events": "读检测告警",
    "approach_vantage": "选安全接近点", "patrol_report": "生成任务报告",
}

KIND_ZH = {
    "session_start": "会话开始", "task": "任务", "llm_call": "大脑推理",
    "tool_call": "工具调用", "tool_result": "工具结果",
    "event": "事件", "snapshot": "遥测快照", "reply": "最终答复",
    "session_end": "会话结束",
}

HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>@@TITLE@@ · GO2W 任务回放</title>
<link rel="stylesheet" href="/static/vendor/leaflet/leaflet.css"/>
<script src="/static/vendor/leaflet/leaflet.js"></script>
<style>
:root{--bg:#0A111E;--panel:#101A2B;--edge:#1E2C44;--ink:#D8E2F0;
--muted:#8294AC;--accent:#F0913B;--water:#2A6F97;--ok:#3EDC97;
--veto:#FF5C5C;--info:#5C8DFF}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%}
body{background:var(--bg);color:var(--ink);
font-family:"Segoe UI",system-ui,-apple-system,sans-serif;font-size:14px}
.header{display:flex;flex-wrap:wrap;gap:12px;align-items:baseline;
padding:14px 20px;border-bottom:1px solid var(--edge);background:var(--panel)}
.header h1{font-size:17px;font-weight:600;letter-spacing:.14em;color:var(--ink)}
.header .sub{font-size:12px;color:var(--muted);letter-spacing:.06em}
.stats{margin-left:auto;display:flex;gap:10px;flex-wrap:wrap}
.stat{font-family:ui-monospace,Consolas,monospace;font-size:12px;
border:1px solid var(--edge);padding:4px 10px;border-radius:2px;color:var(--muted)}
.stat b{color:var(--ink);font-weight:600}
.layout{display:grid;grid-template-columns:minmax(0,1.1fr) minmax(340px,1fr);
grid-template-rows:minmax(0,1fr);height:calc(100% - 61px)}
.map{position:relative;border-right:1px solid var(--edge);min-height:0}
#map{position:absolute;inset:0}
.timeline{overflow-y:auto;background:var(--bg);min-height:0}
.tl-head{position:sticky;top:0;background:var(--panel);border-bottom:1px solid
var(--edge);padding:8px 16px;display:flex;gap:8px;z-index:5}
.tl-head button{background:none;border:1px solid var(--edge);color:var(--muted);
font:inherit;font-size:12px;padding:3px 12px;border-radius:2px;cursor:pointer}
.tl-head button.active{color:var(--bg);background:var(--ink);border-color:var(--ink)}
.step{padding:10px 16px;border-bottom:1px solid var(--edge);
border-left:3px solid transparent}
.step.llm_call{border-left-color:var(--info)}
.step.tool_call{border-left-color:var(--muted)}
.step.event{border-left-color:var(--accent)}
.step.task{border-left-color:var(--ink)}
.step.reply{border-left-color:var(--ok);background:var(--panel)}
.step.denied{border-left-color:var(--veto);background:#1A1220}
.step .kind{font-size:10px;letter-spacing:.16em;color:var(--muted);
text-transform:uppercase;font-family:ui-monospace,Consolas,monospace}
.step .body{margin-top:5px;font-family:ui-monospace,Consolas,monospace;
font-size:12.5px;line-height:1.55;color:var(--ink);word-break:break-word}
.step .badge{display:inline-block;font-size:10px;padding:1px 7px;border-radius:2px;
margin-right:6px;font-family:ui-monospace,Consolas,monospace}
.badge.ok{color:var(--ok);border:1px solid var(--ok)}
.badge.denied{color:var(--veto);border:1px solid var(--veto)}
.badge.warn{color:var(--accent);border:1px solid var(--accent)}
.step .meta{color:var(--muted);font-size:11px;margin-top:4px;
font-family:ui-monospace,Consolas,monospace}
.step details summary{cursor:pointer;color:var(--muted)}
.step details pre{max-height:140px;overflow:auto;background:var(--panel);
border:1px solid var(--edge);padding:8px;margin-top:6px;font-size:11px}
.legend{position:absolute;left:10px;bottom:10px;z-index:1000;background:rgba(10,17,30,.85);
border:1px solid var(--edge);padding:8px 10px;font-size:11px;color:var(--muted);
font-family:ui-monospace,Consolas,monospace;border-radius:2px}
.legend i{display:inline-block;width:9px;height:9px;margin-right:5px;border-radius:50%}
.tile-warn{position:absolute;top:10px;left:10px;right:10px;z-index:1100;
background:rgba(90,30,25,.92);border:1px solid #a05040;color:#ffd9c9;
padding:8px 12px;font-size:12px;border-radius:6px}
@media(max-width:860px){.layout{grid-template-columns:1fr;grid-template-rows:42vh 1fr}
.map{border-right:none;border-bottom:1px solid var(--edge)}}
@media (prefers-reduced-motion: reduce){*{transition:none!important}}
</style></head><body>
<div class="header">
  <h1>GO2W 任务回放</h1>
  <span class="sub">@@TITLE@@</span>
  <div class="stats">@@STATS@@</div>
</div>
<div class="layout">
  <div class="map">
    <div id="map"></div>
    <div id="tilewarn" class="tile-warn" hidden>⚠️ 卫星底图瓦片加载失败 (网络/缓存受限)，已自动切换本地 OSM 底图 —— 航线与步骤不受影响</div>
    <div class="legend">
      <span style="color:var(--accent)"><i style="background:var(--accent)"></i>环线/扫描点</span>
      <span style="color:var(--water)"><i style="background:var(--water)"></i>水域禁区</span>
      <span style="color:var(--veto)"><i style="background:var(--veto)"></i>confirmed 告警</span>
      <span style="color:var(--ok)"><i style="background:var(--ok)"></i>起点</span>
    </div>
  </div>
  <div class="timeline">
    <div class="tl-head">
      <button data-f="all" class="active">全部</button>
      <button data-f="think">推理</button>
      <button data-f="tool">工具</button>
      <button data-f="alert">告警</button>
    </div>
    <div id="steps"></div>
  </div>
</div>
<script>
var DATA = @@DATA@@;
var map = L.map("map");
var tileErrs = 0;
var base = L.tileLayer("/tiles/esri/{z}/{x}/{y}.png",
  {maxZoom: 19, attribution: "Esri 卫星影像"}
);
base.on("tileerror", function(){
  if (++tileErrs === 3) {
    // 卫星瓦片不可用 → 本地 OSM 兜底 (回放页永不白图)
    L.tileLayer("/tiles/osm/{z}/{x}/{y}.png",
      {maxZoom: 19, attribution: "OSM (本地缓存)"}
    ).addTo(map);
    var w = document.getElementById("tilewarn");
    if (w) w.hidden = false;
  }
});
base.addTo(map);
// 容器尺寸保险: grid 行在布局稳定后强制重算 (防 0 高度/行高塌缩 → 灰图)
setTimeout(function(){ map.invalidateSize(); }, 150);
setTimeout(function(){ map.invalidateSize(); }, 600);
var bounds = [];
// 兼容两种坐标形态: 数组 [lat,lng] (多边形) 与对象 {lat,lon} (航点)
function latlng(p){
  if (Array.isArray(p)) return [p[0], p[1]];
  return [p.lat, p.lon !== undefined ? p.lon : p.lng];
}
if (DATA.plan && DATA.plan.water_polygon) {
  var wp = DATA.plan.water_polygon.map(latlng);
  L.polygon(wp, {color:"#2A6F97", weight:2, fillOpacity:0.16}).addTo(map);
  wp.forEach(function(p){ bounds.push(p); });
}
if (DATA.plan && DATA.plan.waypoints) {
  var wps = DATA.plan.waypoints.map(latlng);
  L.polyline(wps, {color:"#F0913B", weight:3, dashArray:"2 6"}).addTo(map);
  wps.forEach(function(p){ bounds.push(p); });
  L.circleMarker(wps[0], {radius:7, color:"#3EDC97", weight:2,
    fillColor:"#3EDC97", fillOpacity:0.5}).addTo(map).bindPopup("起点 wp000");
}
if (DATA.plan && DATA.plan.scan_points) {
  DATA.plan.scan_points.forEach(function(sp){
    var p = latlng([sp.lat, sp.lon]);
    bounds.push(p);
    var rad = sp.look_bearing_deg * Math.PI / 180;
    var tip = [sp.lat + 0.00028*Math.cos(rad), sp.lon + 0.00028*Math.sin(rad)];
    L.circleMarker(p, {radius:4, color:"#F0913B", fillOpacity:0.7}).addTo(map)
      .bindPopup("扫描点 · 朝向 " + sp.look_bearing_deg + "°");
    L.polyline([p, tip], {color:"#F0913B", weight:2}).addTo(map);
  });
}
DATA.alerts.forEach(function(a, i){
  var p = latlng([a.lat, a.lng]);
  bounds.push(p);
  var color = a.tier === "confirmed" ? "#FF5C5C" : "#F0913B";
  L.circleMarker(p, {radius:9, color:color, weight:2, fillColor:color,
    fillOpacity:0.35}).addTo(map).bindPopup(
    "<b>" + (a.tier==="confirmed"?"confirmed 落水告警":"suspect 疑似") + "</b><br/>" +
    a.lat.toFixed(6) + ", " + a.lng.toFixed(6) + "<br/>方位 " + a.bearing_deg +
    "° · 距 " + a.est_range_m + "m · 置信 " + a.confidence);
});
if (bounds.length) map.fitBounds(L.latLngBounds(bounds).pad(0.25));
var stepsEl = document.getElementById("steps");
function esc(s){ return String(s).replace(/&/g,"&amp;").replace(/</g,"&lt;")
  .replace(/>/g,"&gt;"); }
function brief(obj, max){
  var s = JSON.stringify(obj === undefined ? null : obj);
  return s.length > max ? s.slice(0, max) + " …" : s;
}
DATA.steps.forEach(function(st, i){
  var div = document.createElement("div");
  div.className = "step " + st.kind + (st.denied ? " denied" : "");
  div.dataset.f = st.kind === "llm_call" ? "think"
    : (st.kind === "tool_call" || st.kind === "tool_result") ? "tool"
    : (st.alert ? "alert" : "all");
  var html = "<div class='kind'>" + st.kindZh + " · #" + i + "</div>";
  html += "<div class='body'>" + st.html + "</div>";
  if (st.badges) {
    html += "<div class='meta'>" + st.badges.map(function(b){
      return "<span class='badge " + b.cls + "'>" + b.text + "</span>";
    }).join("") + "</div>";
  }
  if (st.detail) {
    html += "<details><summary>原始数据</summary><pre>" + esc(st.detail)
          + "</pre></details>";
  }
  div.innerHTML = html;
  stepsEl.appendChild(div);
});
document.querySelectorAll(".tl-head button").forEach(function(btn){
  btn.addEventListener("click", function(){
    document.querySelectorAll(".tl-head button").forEach(function(b){
      b.classList.remove("active"); });
    btn.classList.add("active");
    var f = btn.dataset.f;
    document.querySelectorAll("#steps .step").forEach(function(step){
      step.style.display = (f === "all" || step.dataset.f === f
        || step.dataset.f === "all" || (f === "alert" && step.dataset.f === "alert"))
        ? "" : "none";
    });
  });
});
</script></body></html>
"""


def _compact(value, max_len=120):
    text = json.dumps(value, ensure_ascii=False, default=str)
    if len(text) > max_len:
        text = text[:max_len] + " …"
    return text


def build_steps(entries):
    steps = []
    for index, entry in enumerate(entries):
        kind = entry.get("kind", "?")
        step = {"kind": kind, "kindZh": KIND_ZH.get(kind, kind),
                "html": "", "badges": [], "detail": "", "denied": False,
                "alert": False}
        if kind == "task":
            step["html"] = f"<b>{entry.get('content', '')}</b>"
        elif kind == "session_start":
            step["html"] = f"go2w_brain v{entry.get('version', '?')} · "
            step["html"] += "轨迹开始 (jsonl 全量留痕)"
        elif kind == "llm_call":
            if entry.get("kind_note") == "system_prompt":
                step["html"] = ("组装系统提示词: 身份 → 人设 → 安全铁律 → "
                                "工具指引 → 技能目录")
                step["badges"].append({"cls": "ok", "text": "提示词"})
            else:
                calls = entry.get("tool_calls") or []
                text = "模型决策"
                if calls:
                    zh = [TOOL_ZH.get(c, c) for c in calls]
                    text += " → 计划调用: " + "、".join(zh)
                if entry.get("reasoning"):
                    text += ("<br/><span style='color:var(--muted)'>"
                             "推理: " + esc(entry["reasoning"][:180]) + "</span>")
                step["html"] = text
                step["badges"].append({"cls": "ok", "text": "推理"})
        elif kind == "tool_call":
            name = entry.get("name", "?")
            zh = TOOL_ZH.get(name, name)
            ok = bool(entry.get("ok"))
            step["html"] = f"<b>{zh}</b> ({name})"
            step["badges"].append({"cls": "ok" if ok else "denied",
                                   "text": "放行" if ok else
                                   f"拦截: {entry.get('reason', '')}"})
            if entry.get("args"):
                step["detail"] = _compact(entry["args"], 300)
        elif kind == "tool_result":
            name = entry.get("name", "?")
            result = entry.get("result") or {}
            zh = TOOL_ZH.get(name, name)
            denied = result.get("dispatch_denied") or (
                result.get("ok") is False and result.get("reason"))
            step["denied"] = bool(denied)
            step["html"] = (f"{zh} → "
                            + (f"<b style='color:var(--veto)'>{denied}</b>"
                               if denied else _result_summary(name, result)))
            if denied:
                step["badges"].append({"cls": "denied", "text": "拒绝/拦截"})
            elif name == "scan_water":
                confirmed = result.get("confirmed_count", 0)
                if confirmed:
                    step["alert"] = True
                    step["badges"].append({"cls": "warn",
                                           "text": f"confirmed ×{confirmed}"})
            step["detail"] = _compact(result, 400)
        elif kind == "event":
            event = entry.get("event", "?")
            zh = {"plan_result": "规划完成", "water_guard_armed": "守卫布防",
                  "water_guard_disarmed": "守卫解除",
                  "mission_lock_acquired": "任务锁获取",
                  "mission_lock_released": "任务锁释放",
                  "follow_route_dry": "航线受理 (干跑, 未下发)",
                  "scan_water": "湖面扫描", "patrol_report": "报告生成",
                  "vantage_selected": "安全接近点选定",
                  "semantic_anchor": "语义锚定",
                  "anchor_replan": "锚定纠正重规划",
                  "geometry_reused": "几何记忆复用",
                  "geometry_persisted": "几何记忆写入",
                  "waypoint_in_keepout_rejected": "禁区航点拦截"}.get(event, event)
            step["html"] = zh
            if event == "plan_result":
                step["html"] += (f" · {entry.get('waypoint_count', '?')} 航点 / "
                                 f"{round((entry.get('length_m') or 0) / 1000, 2)}km")
            if event == "semantic_anchor":
                target = entry.get("target") or {}
                step["html"] += (f" · 来源:{'VLM' if entry.get('source') == 'vlm' else '规则'}"
                                 + (f" · 湖=候选#{target.get('idx')}"
                                    if target.get("idx") is not None else "")
                                 + (f" {target.get('name')}" if target.get("name") else ""))
            if event == "waypoint_in_keepout_rejected":
                step["denied"] = True
                step["alert"] = True
        elif kind == "snapshot":
            data = entry.get("data") or {}
            gps = data.get("gps") or {}
            step["html"] = (f"GPS {'fix' if gps.get('available') else '无'} · "
                            f"电量 {data.get('battery_soc', '?')}% · "
                            f"守卫 {'在岗' if (data.get('guard') or {}).get('armed') else '未布防'}")
        elif kind == "reply":
            step["html"] = entry.get("content", "")
        elif kind == "session_end":
            step["html"] = (f"轨迹闭合 · 步数 {entry.get('steps')} · "
                            f"LLM {'在线' if entry.get('llm_used') else '离线'}")
        else:
            step["html"] = _compact(entry, 160)
        steps.append(step)
    return steps


def _result_summary(name, result):
    if name in ("plan_lake_loop", "plan_campus_loop"):
        return (f"{result.get('waypoint_count', '?')} 航点 · "
                f"{result.get('length_km', '?')}km · 闭合"
                f"{'是' if result.get('closed') else '否'} · "
                f"扫描点 {result.get('scan_points', '?')} · "
                f"续航 {((result.get('endurance') or {}).get('verdict') or '?')}")
    if name == "follow_route":
        return (f"受理 {result.get('waypoint_count', '?')} 航点"
                + (" (干跑, 未下发)" if result.get("dry") else ""))
    if name == "arm_water_guard":
        return f"禁区 {result.get('vertices', '?')} 顶点 · veto {result.get('veto_m')}m / limit {result.get('limit_m')}m"
    if name == "scan_water":
        return (f"{result.get('frames_scanned')} 帧 · "
                f"confirmed {result.get('confirmed_count', 0)}")
    if name == "approach_vantage":
        return (f"观察点距告警 {result.get('dist_to_alert_m', '?')}m"
                + (f" · 守卫距离 {result['guard_dist_m']}m"
                   if result.get("guard_dist_m") is not None else ""))
    if name == "patrol_report":
        s = result.get("summary") or {}
        return (f"进度 {round((s.get('completion') or 0) * 100)}% · "
                f"confirmed {s.get('confirmed')} · 扫描点 {s.get('scan_points')}")
    if name == "get_battery":
        return f"{result.get('battery_soc', '?')}%"
    if name == "calibrate_heading":
        return f"{result.get('heading_deg', '?')}°"
    if name == "get_gps" and result.get("ok"):
        return (f"{result.get('lat')}, {result.get('lng')} · "
                f"HDOP {result.get('hdop')} · {result.get('sats')} 星")
    if name == "load_skill" and result.get("ok"):
        return f"加载方法论: {result.get('name')}"
    return _compact(result, 100)


def extract_alerts(entries):
    alerts = []
    for entry in entries:
        if entry.get("kind") != "tool_result":
            continue
        if entry.get("name") != "scan_water":
            continue
        for event in (entry.get("result") or {}).get("new_events", []):
            if event.get("lat") is not None:
                alerts.append({k: event.get(k) for k in
                               ("tier", "lat", "lng", "bearing_deg",
                                "est_range_m", "confidence")})
    return alerts


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", required=True)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--title", default="绕湖巡查任务")
    args = parser.parse_args(argv)
    entries = [json.loads(line) for line in
               Path(args.trace).read_text(encoding="utf-8").splitlines()
               if line.strip()]
    plan = json.loads(Path(args.plan).read_text(encoding="utf-8"))
    plan = plan.get("result", plan)
    alerts = extract_alerts(entries)
    steps = build_steps(entries)
    tool_calls = sum(1 for e in entries if e["kind"] == "tool_call")
    denied = sum(1 for s in steps if s.get("denied"))
    confirmed = sum(1 for a in alerts if a["tier"] == "confirmed")
    stats = (
        f'<div class="stat">工具调用 <b>{tool_calls}</b></div>'
        f'<div class="stat">拦截 <b>{denied}</b></div>'
        f'<div class="stat">confirmed 告警 <b>{confirmed}</b></div>'
        f'<div class="stat">轨迹 <b>{len(entries)}</b> 条</div>'
    )
    data = {"plan": plan, "alerts": alerts, "steps": steps}
    payload = json.dumps(data, ensure_ascii=False).replace("</", "<\\/")
    html = (HTML.replace("@@TITLE@@", args.title)
                .replace("@@STATS@@", stats)
                .replace("@@DATA@@", payload))
    Path(args.out).write_text(html, encoding="utf-8")
    print(f"written: {args.out} ({len(steps)} steps, "
          f"{len(alerts)} alerts, {denied} denied)")


if __name__ == "__main__":
    main()
