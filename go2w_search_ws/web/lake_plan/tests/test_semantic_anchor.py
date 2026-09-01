"""M7.3 语义锚定测试: 规则退化 / VLM 解析宽容度 / 覆盖不足拒喂 VLM (封闭零网络)。

核心语义: 规划前必须确定 "本体在哪 / 湖是哪个"。VLM 缺席或卫星图不可得时
诚实降级到规则锚定并注明 why —— 绝不静默猜测。
"""
from __future__ import annotations

from PIL import Image

from go2w_brain.vlm import parse_json_loose
from lake_plan import plan_route, semantic_anchor
from lake_plan.semantic_anchor import anchor_semantics

CAMPUS_CENTER = (31.488192, 120.369486)  # 太科园园区 (M2.1 基准点)
ROBOT = {"lat": CAMPUS_CENTER[0], "lng": CAMPUS_CENTER[1]}


def _candidates(n: int = 3):
    return [{"centroid": (31.4880 + 0.001 * i, 120.3690 + 0.001 * i),
             "dist_km": 0.1 * (i + 1)} for i in range(n)]


class _FakeVlm:
    def __init__(self, text="", available=True):
        self._text = text
        self._avail = available
        self.prompt = None

    def available(self):
        return self._avail

    def vision(self, image, prompt, max_tokens=400):
        self.prompt = prompt
        return self._text


def _gray_tiles(monkeypatch):
    """离线/未命中: 占位灰底 (tiles_ok=0)。"""
    monkeypatch.setattr(
        semantic_anchor, "build_anchor_image",
        lambda *a, **k: (Image.new("RGB", (64, 64), (240, 240, 234)),
                         {"tiles_ok": 0}))


def _ok_tiles(monkeypatch):
    monkeypatch.setattr(
        semantic_anchor, "build_anchor_image",
        lambda *a, **k: (Image.new("RGB", (64, 64)), {"tiles_ok": 64}))


# ---------- 规则退化 ----------

def test_offline_real_path_degrades_to_rule():
    """真实路径: 离线且 esri 无缓存 → 全灰占位 → 覆盖率检查拒喂 VLM。"""
    out = anchor_semantics(None, "绕湖一周", ROBOT, _candidates(),
                           CAMPUS_CENTER, z=16, nx=4, ny=4)
    assert out["source"] == "rule"
    assert out["why"] == "satellite_unavailable"
    assert out["target"]["idx"] == 0
    assert out["target"]["name"] == "nearest_water"


def test_rule_anchor_picks_nearest():
    out = semantic_anchor._rule_anchor(ROBOT, _candidates(),
                                       why="vlm_unavailable")
    assert out["source"] == "rule"
    assert out["target"]["idx"] == 0  # dist_km 最小者
    assert out["target"]["centroid"] == _candidates()[0]["centroid"]


def test_no_candidates_rule():
    out = anchor_semantics(None, "绕湖", ROBOT, [], CAMPUS_CENTER)
    assert out["source"] == "rule"
    assert out["target"] is None
    assert out["why"] == "no_candidates"


def test_coverage_below_half_degrades(monkeypatch):
    _gray_tiles(monkeypatch)
    out = anchor_semantics(None, "绕湖", ROBOT, _candidates(),
                           CAMPUS_CENTER, nx=10, ny=8)
    assert out["source"] == "rule"
    assert out["why"] == "satellite_unavailable"


# ---------- VLM 路径 (宽容解析 + 任何失败退规则) ----------

def test_vlm_parse_fenced_and_filters_ambiguity(monkeypatch):
    _ok_tiles(monkeypatch)
    vlm = _FakeVlm("前置噪音 ```json {\"self_near_water\": true, "
                   "\"target_idx\": 1, \"target_name\": \"南湖\", "
                   "\"ambiguity\": [0, 9, 2], \"why\": \"红圈1最大\"} ``` 尾巴")
    out = anchor_semantics(vlm, "绕湖一周并巡查落水人员", ROBOT,
                           _candidates(), CAMPUS_CENTER)
    assert out["source"] == "vlm"
    assert out["target"]["idx"] == 1
    assert out["target"]["name"] == "南湖"
    assert out["ambiguity"] == [0, 2]  # 越界编号 9 被过滤
    assert out["self"]["lat"] == ROBOT["lat"]
    assert out["target"]["centroid"] == _candidates()[1]["centroid"]
    assert "raw" in out


def test_vlm_target_idx_out_of_range_degrades(monkeypatch):
    _ok_tiles(monkeypatch)
    vlm = _FakeVlm("{\"self_near_water\": true, \"target_idx\": 9, "
                   "\"ambiguity\": [], \"why\": \"x\"}")
    out = anchor_semantics(vlm, "绕湖", ROBOT, _candidates(), CAMPUS_CENTER)
    assert out["source"] == "rule"
    assert out["why"] == "vlm_unavailable"


def test_vlm_garbage_degrades_to_rule(monkeypatch):
    _ok_tiles(monkeypatch)
    out = anchor_semantics(_FakeVlm("抱歉, 这张图我看不清楚"),
                           "绕湖", ROBOT, _candidates(), CAMPUS_CENTER)
    assert out["source"] == "rule"
    assert out["why"] == "vlm_unavailable"


def test_vlm_unavailable_flag_degrades(monkeypatch):
    _ok_tiles(monkeypatch)
    out = anchor_semantics(_FakeVlm("{}", available=False),
                           "绕湖", ROBOT, _candidates(), CAMPUS_CENTER)
    assert out["source"] == "rule"
    assert out["why"] == "vlm_unavailable"


def test_task_text_reaches_vlm_prompt(monkeypatch):
    _ok_tiles(monkeypatch)
    vlm = _FakeVlm("{\"self_near_water\": false, \"target_idx\": 0, "
                   "\"ambiguity\": [], \"why\": \"ok\"}")
    anchor_semantics(vlm, "绕南湖巡查落水人员", ROBOT, _candidates(),
                     CAMPUS_CENTER)
    assert "绕南湖巡查落水人员" in vlm.prompt


# ---------- parse_json_loose ----------

def test_parse_json_loose_fences():
    assert parse_json_loose("```json\n{\"a\": 1}\n```") == {"a": 1}


def test_parse_json_loose_prefix_suffix():
    assert parse_json_loose("好的, 结果如下: {\"a\": 1} 以上。") == {"a": 1}


def test_parse_json_loose_plain():
    assert parse_json_loose("{\"a\": 1}") == {"a": 1}


def test_parse_json_loose_invalid():
    assert parse_json_loose("完全没有 json") is None
    assert parse_json_loose("[1,2]") is None
    assert parse_json_loose("") is None


# ---------- M7.3 语义纠正参数 (plan_route prefer) ----------

def test_plan_route_prefer_selects_nearest_to_prefer(water_route):
    """prefer 质心 → 选距其最近的合格水体 (而非距本体最近的)。
    夹具区域存在多个合格水体, 这是语义纠正的真实生效路径。"""
    base = plan_route(*CAMPUS_CENTER, kind="water")
    assert base["ok"], base.get("reason")
    centroid = tuple(base["target"]["centroid"])
    candidates = [base["target"]] + (base.get("nearby_candidates") or [])
    # prefer 指向当前湖质心 → 仍是它 (距离 0)
    same = plan_route(*CAMPUS_CENTER, kind="water", prefer=centroid)
    assert same["ok"], same.get("reason")
    assert tuple(same["target"]["centroid"]) == centroid
    # prefer 指向另一候选 → 换到距 prefer 最近的那个 (语义纠正生效)
    others = [c for c in candidates
              if tuple(c["centroid"]) != centroid]
    assert others, "夹具区域应存在多个合格水体"
    pick = others[0]
    out = plan_route(*CAMPUS_CENTER, kind="water",
                     prefer=tuple(pick["centroid"]))
    assert out["ok"], out.get("reason")
    # 同一水体但细化后质心估计可有 ~30m 差异 → 按邻近断言
    from lake_plan.geo import haversine_m
    got = out["target"]["centroid"]
    want = pick["centroid"]
    assert haversine_m(want[0], want[1], got[0], got[1]) < 150.0
    assert tuple(got) != centroid  # 确实换到了另一个水体
    # prefer 指向远处也不得报错 (选感知区域内距其最近的合格水体)
    far = plan_route(*CAMPUS_CENTER, kind="water", prefer=(30.0, 120.0))
    assert far["ok"], far.get("reason")
