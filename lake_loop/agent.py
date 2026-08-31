"""LangGraph 绕湖规划智能体（思维链流水线）。

图结构：
  intent -> perceive -> select_lake --ok--> refine -> plan -> verify --ok--> output -> END
                ^          |retry(wider/relocate)                |fail(<=3)
                +----------+                                     v
                                                            回 plan 调整 offset
                select_lake --dead--> output(失败报告) -> END

每个节点都会把「思考/推理/动作/观察」写入 state["trace"]，
DeepSeek 的 reasoning_content 作为思维链被完整保留导出。
"""
import json
import math
import shutil
import time
from datetime import datetime
from typing import Any, Optional, TypedDict

import config
import planner
import tiles
import water
from config import (DEFAULT_CENTER, DEFAULT_LOOP_OFFSET_M, DEFAULT_MAX_LAKE_PERIM_KM,
                    DEFAULT_PROVIDER, PROVIDERS, RUNS_DIR, RENDER_DIR, SERVER_URL)
from geo import haversine_m
from llm import LLM

BT = "\x60"  # markdown 反引号

# 区域兜底湖泊库（仅当视野内没有合适小湖时，供智能体「搬家」参考）
FALLBACK_LAKES = [
    {"name": "蠡湖", "lat": 31.5163, "lng": 120.2673, "note": "无锡市区，环湖步道成熟"},
    {"name": "愉树湾", "lat": 31.5500, "lng": 120.2230, "note": "无锡西郊小湖"},
    {"name": "东氿", "lat": 31.3520, "lng": 120.0150, "note": "宜兴城区东侧"},
    {"name": "团氿", "lat": 31.3500, "lng": 119.9680, "note": "宜兴城区西侧"},
]


class LoopState(TypedDict, total=False):
    task: str
    center: tuple
    provider: str
    mode: str
    offset_m: float
    max_perim_km: float
    direction: str
    trace: list
    artifacts: list
    perceive: dict
    candidates: list
    target: dict
    route: dict
    verdict: dict
    plan_tries: int
    perceive_tries: int
    status: str
    summary: str
    error: str
    run_dir: str
    lake_name: str


class LakeLoopAgent:
    def __init__(self, llm: LLM | None = None, provider: str = DEFAULT_PROVIDER):
        self.llm = llm or LLM()
        self.provider = provider
        self._imgs = {}

    def _think(self, state, node, thought, reasoning="", data=None, action=""):
        entry = {"node": node, "ts": round(time.time(), 2), "thought": thought,
                 "action": action, "reasoning": reasoning, "data": data}
        state.setdefault("trace", []).append(entry)
        return {}

    # ---------- 节点 1: 意图理解 ----------
    def node_intent(self, state: LoopState) -> dict:
        task = state.get("task", "绕湖走一圈")
        upd: dict[str, Any] = {}
        if self.llm.available:
            sys = ("你是任务规划助手。把用户的出行任务解析成 JSON。字段："
                   "mode(walk/bike/drive)、direction(cw/ccw, 默认 ccw 即湖在左手)、"
                   "offset_m(离岸距离,步行默认140)、max_perim_km(步行35/骑行150/驾车400)、"
                   "goal(一句话目标)。只输出 JSON。")
            try:
                obj, content, reasoning = self.llm.chat_json(
                    sys, "用户任务：" + task, temperature=0.1, max_tokens=1500)
                if obj:
                    upd["mode"] = str(obj.get("mode", "walk")).lower()
                    upd["direction"] = str(obj.get("direction", "ccw")).lower()
                    upd["offset_m"] = float(obj.get("offset_m", DEFAULT_LOOP_OFFSET_M))
                    dmax = {"walk": 35, "bike": 150, "drive": 400}
                    upd["max_perim_km"] = float(obj.get("max_perim_km",
                                                        dmax.get(upd["mode"], 35)))
                    self._think(state, "intent",
                                "任务解析完成：" + str(obj.get("goal", task)),
                                reasoning, obj, "llm.chat_json")
                    return upd
            except Exception as e:  # noqa: BLE001
                self._think(state, "intent", "大模型解析失败(" + str(e) + ")，退回规则解析",
                            action="fallback")
        t = task
        mode = "walk"
        if any(k in t for k in ("骑", "bike", "骑行", "自行车")):
            mode = "bike"
        elif any(k in t for k in ("开", "drive", "驾", "车")):
            mode = "drive"
        upd["mode"] = mode
        upd["direction"] = "ccw"
        upd["offset_m"] = DEFAULT_LOOP_OFFSET_M
        upd["max_perim_km"] = {"walk": 35, "bike": 150, "drive": 400}[mode]
        self._think(state, "intent", "规则解析：mode=" + mode, action="regex")
        return upd

    # ---------- 节点 2: 地图感知 ----------
    def node_perceive(self, state: LoopState) -> dict:
        lat, lng = state.get("center", DEFAULT_CENTER)
        tries = state.get("perceive_tries", 0)
        plans = [(13, 8, 8), (12, 10, 10), (11, 10, 10)]
        z, nx, ny = plans[min(tries, len(plans) - 1)]
        prov = state.get("provider", self.provider)
        img, detail = tiles.stitch_centered(prov, lat, lng, z, nx, ny)
        georef = tiles.georef_from_detail(detail)
        ref_rgb = PROVIDERS[prov]["water_rgb"]
        mask = water.water_mask(img, ref_rgb=ref_rgb)
        comps = water.label_components(mask, down=4, min_area_px=200)
        cands = []
        for i, c in enumerate(comps[:8]):
            poly = water.polygon_from_component(c, mask.shape)
            if len(poly) < 4:
                continue
            poly_ll = [georef.pixel_to_latlon(x, y) for (x, y) in poly]
            perim_m, area_m2 = water.poly_stats_latlon(poly_ll, lat)
            bb = c["bbox_small"]
            clipped = (bb[0] <= 1 or bb[1] <= 1 or
                       bb[2] >= c["mask"].shape[1] - 2 or
                       bb[3] >= c["mask"].shape[0] - 2)
            cy, cx = c["centroid_small"]
            clat, clng = georef.pixel_to_latlon(cx * c["down"], cy * c["down"])
            compact = 0.0
            if perim_m > 0:
                compact = 4 * math.pi * area_m2 / (perim_m ** 2)  # 圆=1，河流<<0.1
            cands.append({
                "idx": i, "area_km2": round(area_m2 / 1e6, 3),
                "perim_km": round(perim_m / 1000.0, 2),
                "compact": round(compact, 3),
                "clipped": bool(clipped),
                "centroid": [round(clat, 5), round(clng, 5)],
                "dist_km": round(haversine_m(lat, lng, clat, clng) / 1000.0, 2),
                "_poly_px": poly, "_poly_ll": poly_ll,
            })
        vision_note, vision_reasoning = "", ""
        if self.llm.available:
            try:
                prompt = ("这是一张地图瓦片拼接图（图像中心是用户所在位置）。"
                          "请只关注距离图像中心最近的水体：1) 它的名称(读地图上的文字标注，"
                          "如「蠡湖」「太湖」)；2) 它在图像中心的什么方向、大致多大；"
                          "3) 它是湖泊还是河流/水库。忽略远处的大水体。"
                          "用不超过120字中文回答。")
                vision_note, vision_reasoning = self.llm.vision(img, prompt)
                if not vision_note.strip():
                    # 推理耗尽输出预算的情况：换短提示词重试一次
                    vision_note, r2 = self.llm.vision(
                        img, "直接给出答案(50字内)：图像中心最近的水体名称？"
                             "是湖泊还是河流？", max_tokens=2000)
                    vision_reasoning = (vision_reasoning + " | " + r2).strip()
            except Exception as e:  # noqa: BLE001
                vision_note = "(视觉模型调用失败: " + str(e) + ")"
        self._imgs["last"] = (img, detail, georef)
        upd = {
            "perceive": {
                "provider": prov, "zoom": z, "tiles": str(nx) + "x" + str(ny),
                "bbox_wgs84": [round(v, 5) for v in georef.bbox()],
                "water_fraction": round(float(mask.mean()), 4),
                "n_components": len(cands),
                "vision": vision_note,
            },
            "candidates": cands,
            "perceive_tries": tries + 1,
        }
        top = [{k: c[k] for k in ("idx", "area_km2", "perim_km", "compact", "dist_km")}
               for c in cands[:5]]
        self._think(state, "perceive",
                    "视野 z" + str(z) + " " + str(nx) + "x" + str(ny) + " 瓦片，识别到 "
                    + str(len(cands)) + " 片水域"
                    + ("，最大 " + str(cands[0]["area_km2"]) + " km²" if cands else "")
                    + "。视觉模型：" + vision_note[:160],
                    vision_reasoning[:600], {"n": len(cands), "top": top},
                    "tiles.stitch + water.segment + llm.vision")
        return upd

    # ---------- 节点 3: 选湖 ----------
    def node_select_lake(self, state: LoopState) -> dict:
        cands = state.get("candidates", [])
        mode = state.get("mode", "walk")
        max_p = state.get("max_perim_km", DEFAULT_MAX_LAKE_PERIM_KM)
        min_p = {"walk": 2.0, "bike": 5.0, "drive": 10.0}.get(mode, 2.0)
        usable = [c for c in cands if not c["clipped"] and min_p <= c["perim_km"] <= max_p
                  and c.get("compact", 1.0) >= 0.12]  # 紧凑度<0.12 视为河流/水渠
        vision = (state.get("perceive") or {}).get("vision", "")
        chosen, thought, reasoning, relocated = None, "", "", None
        if usable:
            if self.llm.available:
                sys = ("你是选湖助手。给定候选水体(面积km2/周长km/离中心距离km/是否被视野裁剪)"
                       "和视觉观察，选出最适合用户绕行的一个。只输出 JSON："
                       "{\"idx\":数字, \"why\":\"一句话\"}。都不合适则 {\"idx\":-1}。")
                listing = json.dumps([{k: c[k] for k in ("idx", "area_km2", "perim_km",
                                                        "compact", "dist_km")}
                                      for c in usable], ensure_ascii=False)
                try:
                    obj, _, reasoning = self.llm.chat_json(
                        sys, "模式=" + mode + " 最大周长=" + str(max_p) + "km\n候选："
                        + listing + "\n视觉观察：" + vision,
                        temperature=0.1, max_tokens=1200)
                    if obj and obj.get("idx", -1) >= 0:
                        pick = next((c for c in usable if c["idx"] == obj["idx"]), None)
                        if pick is not None:  # 只接受合格候选
                            chosen = pick
                        thought = "大模型选择水体#" + str(obj.get("idx")) + "：" + str(obj.get("why", ""))
                except Exception:  # noqa: BLE001
                    pass
            if chosen is None:
                chosen = min(usable, key=lambda c: c["dist_km"])
                thought = ("规则选择：距离最近 " + str(chosen["dist_km"]) + "km，周长 "
                           + str(chosen["perim_km"]) + "km ≤ " + str(max_p) + "km")
        else:
            big = [c for c in cands if c["clipped"]]
            if big and state.get("perceive_tries", 0) < 3:
                thought = ("视野内只有被裁剪的大水体(面积疑似 " + str(big[0]["area_km2"])
                           + " km²)，扩大视野重扫")
                self._think(state, "select_lake", thought, reasoning, action="widen")
                return {"status": "widen"}
            if state.get("perceive_tries", 0) >= 2:
                fb = min(FALLBACK_LAKES,
                         key=lambda l: haversine_m(state.get("center", DEFAULT_CENTER)[0],
                                                   state.get("center", DEFAULT_CENTER)[1],
                                                   l["lat"], l["lng"]))
                relocated = fb
                thought = ("附近没有适合步行环湖的小水体，迁移到 " + fb["name"]
                           + " (" + str(fb["lat"]) + "," + str(fb["lng"]) + ")，" + fb["note"])
        if chosen is None and relocated is None:
            self._think(state, "select_lake", "未找到可绕行的湖泊", action="fail")
            return {"status": "no_lake"}
        if relocated is not None:
            self._think(state, "select_lake", thought, reasoning,
                        {"relocate": relocated["name"]}, "relocate")
            return {"status": "relocate", "center": (relocated["lat"], relocated["lng"]),
                    "lake_name": relocated["name"], "perceive_tries": 0}
        lake_name = ""
        if self.llm.available and vision:
            try:
                obj, _, _ = self.llm.chat_json(
                    "从地图观察中抽取选中水体的名称。输出 JSON {\"name\":\"...\"}，没有则 {\"name\":\"\"}。",
                    "观察：" + vision, temperature=0.0, max_tokens=400)
                lake_name = ((obj or {}).get("name") or "")
            except Exception:  # noqa: BLE001
                pass
        self._think(state, "select_lake", thought or "已选择目标水体", reasoning,
                    {k: chosen[k] for k in ("idx", "area_km2", "perim_km", "dist_km")},
                    "select")
        return {"status": "ok", "target": chosen, "lake_name": lake_name}

    # ---------- 节点 4: 细化 ----------
    def node_refine(self, state: LoopState) -> dict:
        target = state.get("target")
        if not target:
            return {}
        prov = state.get("provider", self.provider)
        poly_ll = target["_poly_ll"]
        lats = [p[0] for p in poly_ll]
        lngs = [p[1] for p in poly_ll]
        w, s, e, n = min(lngs), min(lats), max(lngs), max(lats)
        from geo import lng_to_global_px, lat_to_global_px
        best = None
        for z in range(16, 9, -1):
            gx0 = int(lng_to_global_px(w, z) // 256)
            gx1 = int(lng_to_global_px(e, z) // 256)
            gy0 = int(lat_to_global_px(n, z) // 256)
            gy1 = int(lat_to_global_px(s, z) // 256)
            nx, ny = gx1 - gx0 + 1, gy1 - gy0 + 1
            if nx * ny <= config.MAX_TILES_PER_STITCH and nx >= 2:
                best = (z, gx0, gy0, nx, ny)
                break
        if best is None:
            self._think(state, "refine", "湖太大或瓦片超限，沿用粗扫多边形", action="skip")
            return {}
        z, x0, y0, nx, ny = best
        if (nx + 2) * (ny + 2) <= config.MAX_TILES_PER_STITCH:
            x0, y0, nx, ny = x0 - 1, y0 - 1, nx + 2, ny + 2  # 留边给外扩
        img, detail = tiles.stitch_area(prov, z, x0, y0, nx, ny)
        georef = tiles.georef_from_detail(detail)
        ref_rgb = PROVIDERS[prov]["water_rgb"]
        mask = water.water_mask(img, ref_rgb=ref_rgb)
        comps = water.label_components(mask, down=2, min_area_px=400)
        if not comps:
            self._think(state, "refine", "高分辨率下未检出水域，沿用粗扫多边形", action="skip")
            return {}
        # 按质心就近匹配（而不是简单取最大），避免拿到同一视野里的其他水塘
        cc_lat = sum(p[0] for p in poly_ll) / len(poly_ll)
        cc_lng = sum(p[1] for p in poly_ll) / len(poly_ll)
        ref_px = georef.latlon_to_pixel(cc_lat, cc_lng)
        def _ccomp(c):
            cy, cx = c["centroid_small"]
            return math.hypot(cx * c["down"] - ref_px[0], cy * c["down"] - ref_px[1])
        comp = min(comps[:8], key=_ccomp)
        poly_px = water.polygon_from_component(comp, mask.shape)
        poly_ll2 = [georef.pixel_to_latlon(x, y) for (x, y) in poly_px]
        perim_m, area_m2 = water.poly_stats_latlon(poly_ll2, poly_ll2[0][0])
        # 守卫：细化后面积骤降(<40%)说明匹配到了别的小水体 -> 沿用粗扫多边形
        if area_m2 < 0.4 * (target["area_km2"] * 1e6):
            self._think(state, "refine",
                        "细化匹配面积骤降(" + format(area_m2 / 1e6, ".2") + "km2 < 40%粗扫"
                        + str(target["area_km2"]) + "km2)，沿用粗扫多边形",
                        action="guard")
            return {"target": dict(target, _poly_px_coarse=target["_poly_px"],
                                   _georef="coarse")}
        self._imgs["fine"] = (img, detail, georef)
        new_target = dict(target)
        new_target.update({"_poly_px": poly_px, "_georef": "fine",
                           "_poly_px_coarse": target["_poly_px"],
                           "area_km2": round(area_m2 / 1e6, 3),
                           "perim_km": round(perim_m / 1000.0, 2)})
        self._think(state, "refine",
                    "以 z" + str(z) + " 重扫湖面：" + str(nx) + "x" + str(ny)
                    + " 瓦片，精化周长 " + str(new_target["perim_km"]) + "km，面积 "
                    + str(new_target["area_km2"]) + "km2",
                    action="tiles.stitch_area(fine)")
        return {"target": new_target}

    # ---------- 节点 5: 规划 ----------
    def node_plan(self, state: LoopState) -> dict:
        target = state.get("target")
        if target.get("_georef") == "fine" and "fine" in self._imgs:
            img, detail, georef = self._imgs["fine"]
            ctx = "fine"
        else:
            img, detail, georef = self._imgs["last"]
            ctx = "last"
        offset = state.get("offset_m", DEFAULT_LOOP_OFFSET_M)
        if self.llm.available:
            try:
                sys = ("你是徒步规划师。给定湖的面积/周长/出行方式，建议离岸距离 offset_m"
                       "(步行60-200)与方向 direction。只输出 JSON {\"offset_m\":数,\"direction\":\"cw|ccw\"}。")
                obj, _, reasoning = self.llm.chat_json(
                    sys, "面积=" + str(target["area_km2"]) + "km2 周长="
                    + str(target["perim_km"]) + "km 模式=" + str(state.get("mode"))
                    + " 用户偏移=" + str(offset) + "m",
                    temperature=0.1, max_tokens=600)
                if obj and obj.get("offset_m"):
                    offset = float(min(max(float(obj["offset_m"]), 30), 400))
                    self._think(state, "plan",
                                "模型建议 offset=" + format(offset, ".0f") + "m, direction="
                                + str(obj.get("direction")), reasoning, obj, "llm.chat_json")
            except Exception:  # noqa: BLE001
                pass
        tries = state.get("plan_tries", 0)
        if tries:
            offset *= 1.5 + 0.5 * tries
        try:
            result = planner.plan_loop_around_polygon(
                target["_poly_px"], georef, offset_m=offset,
                step_m={"walk": 40, "bike": 60, "drive": 100}.get(state.get("mode"), 50))
        except ValueError:
            # 本级多边形退化 -> 先试 miter 外扩，再回退粗扫多边形
            try:
                result = planner.plan_loop_around_polygon(
                    target["_poly_px"], georef, offset_m=offset,
                    step_m={"walk": 40, "bike": 60, "drive": 100}.get(state.get("mode"), 50),
                    use_miter=True)
            except ValueError:
                result = None
            if result is None and target.get("_poly_px_coarse") and "last" in self._imgs:
                img2, detail2, georef2 = self._imgs["last"]
                target = dict(target, _poly_px=target["_poly_px_coarse"], _georef="coarse")
                img, detail, georef = img2, detail2, georef2
                ctx = "last"
                self._think(state, "plan", "细化多边形退化，回退粗扫多边形", action="fallback")
                result = planner.plan_loop_around_polygon(
                    target["_poly_px"], georef, offset_m=offset,
                    step_m={"walk": 40, "bike": 60, "drive": 100}.get(state.get("mode"), 50))
            if result is None:
                raise
        st = result["stats"]
        prev = state.get("route") or {}
        prev_st = (prev.get("result") or {}).get("stats")
        keep_prev = (prev_st is not None
                     and prev_st["water_cross_ratio"] < st["water_cross_ratio"])
        if keep_prev:
            result = prev["result"]  # 历次最优：跨水率更低者胜
            ctx = prev.get("img_ctx", ctx)
            st = result["stats"]
        self._think(state, "plan",
                    "几何引擎生成环线：" + str(st["route_points"]) + " 点，长 "
                    + format(st["length_m"] / 1000, ".2f") + "km，离岸 "
                    + format(st["offset_m"], ".0f") + "m，跨水率 "
                    + str(st["water_cross_ratio"])
                    + ("（保留历次最优）" if keep_prev else ""),
                    action="planner.plan_loop_around_polygon")
        return {"route": {"result": result, "img_ctx": ctx}, "plan_tries": tries + 1}

    # ---------- 节点 6: 校验 ----------
    def node_verify(self, state: LoopState) -> dict:
        st = state["route"]["result"]["stats"]
        target = state.get("target", {})
        issues = []
        if st["route_points"] < 8 or st["length_m"] < 500:
            issues.append("路线退化: 点数或长度异常")
        if not st["closed"]:
            issues.append("环线未闭合")
        if st["water_cross_ratio"] > 0.05:
            issues.append("跨水率过高 " + str(st["water_cross_ratio"]))
        if st["length_m"] / 1000.0 > state.get("max_perim_km", 35) * 1.6:
            issues.append("路线远超模式上限")
        if st.get("min_shore_m") is not None and st["min_shore_m"] < 12:
            issues.append("部分路段贴水过近 " + str(st["min_shore_m"]) + "m")
        llm_verdict, reasoning = "", ""
        if self.llm.available:
            try:
                sys = ("你是路线评审员。给定统计，判断该闭合环线是否合格。"
                       "只输出 JSON {\"ok\":bool,\"issues\":[\"...\"],\"score\":0-100}。")
                obj, _, reasoning = self.llm.chat_json(
                    sys, json.dumps(st, ensure_ascii=False)
                    + " 湖面积km2=" + str(target.get("area_km2"))
                    + " 周长km=" + str(target.get("perim_km")),
                    temperature=0.1, max_tokens=800)
                if obj is not None:
                    llm_verdict = ("LLM 评审: ok=" + str(obj.get("ok"))
                                   + " score=" + str(obj.get("score")))
                    for i in (obj.get("issues") or [])[:3]:
                        if i and str(i) not in issues:
                            issues.append(str(i))
            except Exception:  # noqa: BLE001
                pass
        ok = (not issues) or all(("贴水" in i) for i in issues)
        self._think(state, "verify",
                    ("校验通过" if ok else "校验未通过: " + "; ".join(issues))
                    + (("（" + llm_verdict + "）") if llm_verdict else ""),
                    reasoning, st, "rules + llm.chat_json")
        return {"verdict": {"ok": ok, "issues": issues, "stats": st, "llm": llm_verdict}}

    # ---------- 节点 7: 输出 ----------
    def node_output(self, state: LoopState) -> dict:
        run_dir = RUNS_DIR / datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        arts = []
        route = state.get("route")
        lake_name = state.get("lake_name") or "湖泊"
        status = state.get("status", "ok")
        if route and route.get("result"):
            res = route["result"]
            img, detail, georef = self._imgs.get(route.get("img_ctx", "last"),
                                                 self._imgs["last"])
            poly_ll_out = [georef.pixel_to_latlon(x, y) for (x, y) in res["poly_px"]]
            gj = planner.to_geojson(res["route_latlon"], poly_ll_out,
                                    lake_name=lake_name, length_m=res["stats"]["length_m"])
            (run_dir / "route.geojson").write_text(
                json.dumps(gj, ensure_ascii=False, indent=1), encoding="utf-8")
            (run_dir / "route.gpx").write_text(
                planner.to_gpx(res["route_latlon"], name="绕" + lake_name + "一圈"),
                encoding="utf-8")
            st = res["stats"]
            planner.render_preview(
                img, res["route_px"], res["poly_px"], georef,
                "绕" + lake_name + "一圈 · " + format(st["length_m"] / 1000, ".2f") + " km",
                "模式 " + str(state.get("mode")) + " | 离岸 " + format(st["offset_m"], ".0f")
                + " m | 跨水率 " + format(st["water_cross_ratio"] * 100, ".1f") + "% | "
                + str(state.get("task", "")),
                run_dir / "preview.png", length_m=st["length_m"])
            planner.render_html(gj, run_dir / "preview.html",
                                "绕" + lake_name + "一圈", SERVER_URL,
                                state.get("provider", self.provider))
            arts = ["route.geojson", "route.gpx", "preview.png", "preview.html"]
            for f in arts:
                src = run_dir / f
                if src.exists():
                    shutil.copy(src, RENDER_DIR / (run_dir.name + "_" + f))
            if st["length_m"] < 500:
                summary = "路线生成异常(长度接近0)，请检查日志"
                status = "degenerate"
            else:
                summary = ("已规划绕" + lake_name + "一圈：" + format(st["length_m"] / 1000, ".2f")
                           + " km，闭合=" + ("是" if st["closed"] else "否") + "，离岸 "
                           + format(st["offset_m"], ".0f") + "m，跨水率 "
                           + format(st["water_cross_ratio"] * 100, ".1f") + "%。产物："
                           + ", ".join(arts) + "（目录 " + run_dir.name + "）")
                status = "ok"
        else:
            summary = "未能规划路线：" + str(state.get("error") or "视野内没有合适的湖泊")
            status = state.get("status") or "no_lake"
        (run_dir / "trace.json").write_text(
            json.dumps(state.get("trace", []), ensure_ascii=False, indent=1),
            encoding="utf-8")
        lines = ["# 绕湖规划报告", "", "- 任务: " + str(state.get("task")),
                 "- 中心: " + str(tuple(round(v, 4) for v in state.get("center", DEFAULT_CENTER))),
                 "- 状态: " + str(status), "", summary, "",
                 "## 思维链（LangGraph 节点轨迹）", ""]
        for t in state.get("trace", []):
            lines.append("### [" + t["node"] + "] " + t["thought"])
            if t.get("action"):
                lines.append("- action: " + BT + str(t["action"]) + BT)
            if t.get("reasoning"):
                r = t["reasoning"]
                lines.append("> 思维链: " + r[:500] + ("…" if len(r) > 500 else ""))
            if t.get("data") is not None:
                lines.append("- data: " + BT + json.dumps(t["data"], ensure_ascii=False)[:400] + BT)
            lines.append("")
        (run_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")
        self._think(state, "output", summary, action="export artifacts")
        return {"status": status, "summary": summary, "artifacts": arts,
                "run_dir": str(run_dir)}

    # ---------- 路由 ----------
    def route_after_select(self, state: LoopState) -> str:
        s = state.get("status")
        if s in ("widen", "relocate"):
            return "perceive"
        if s == "no_lake":
            return "output"
        return "refine"

    def route_after_verify(self, state: LoopState) -> str:
        v = state.get("verdict") or {}
        if not v.get("ok") and state.get("plan_tries", 0) < 3:
            return "plan"
        return "output"

    def build(self):
        from langgraph.graph import END, StateGraph
        g = StateGraph(LoopState)
        g.add_node("intent", self.node_intent)
        g.add_node("perceive", self.node_perceive)
        g.add_node("select_lake", self.node_select_lake)
        g.add_node("refine", self.node_refine)
        g.add_node("plan", self.node_plan)
        g.add_node("verify", self.node_verify)
        g.add_node("output", self.node_output)
        g.set_entry_point("intent")
        g.add_edge("intent", "perceive")
        g.add_edge("perceive", "select_lake")
        g.add_conditional_edges("select_lake", self.route_after_select,
                                {"perceive": "perceive", "refine": "refine",
                                 "output": "output"})
        g.add_edge("refine", "plan")
        g.add_edge("plan", "verify")
        g.add_conditional_edges("verify", self.route_after_verify,
                                {"plan": "plan", "output": "output"})
        g.add_edge("output", END)
        return g.compile()


def run(task="绕湖走一圈", center=None, provider=DEFAULT_PROVIDER, llm=None):
    agent = LakeLoopAgent(llm=llm, provider=provider)
    graph = agent.build()
    init: LoopState = {
        "task": task,
        "center": center or DEFAULT_CENTER,
        "provider": provider,
        "plan_tries": 0,
        "perceive_tries": 0,
        "trace": [],
        "status": "start",
    }
    final = graph.invoke(init, {"recursion_limit": 30})
    return final
