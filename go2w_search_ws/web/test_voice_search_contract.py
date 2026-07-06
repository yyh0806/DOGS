"""Voice-driven room person search — NLU contract.

确保"去搜索这个房间,把所有人标注出来"这类语音指令被 parse_product_command
精确解析成 search_room task (room=__current__ + target_classes=[person] +
require_photos + mark_on_map + next_best_view)。

这是语音→任务链路的核心契约: 任何回归会让用户语音指令静默退化为
search_area (区域搜索, 不进房间) 或被 VLM 误解析为别的 task type。
前端 web/test_map_contract.js 和 panel.html 的"房间搜人"快捷按钮都依赖这个契约。
"""
import sys
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_product_command import parse_product_command, resolve_current_room


# ---- 用户核心场景: "去搜索这个房间，把所有人标注出来" ----

def test_core_command_current_room_all_people():
    """用户目标指令的精确命中 (语音 STT → NLU → search_room task)。"""
    result = parse_product_command("去搜索这个房间，把所有人标注出来")
    assert result is not None
    tasks = result["tasks"]
    assert len(tasks) == 1
    t = tasks[0]
    assert t["type"] == "search_room"
    assert t["params"]["room"] == "__current__"
    assert t["params"]["target_classes"] == ["person"]
    assert t["params"]["require_photos"] is True      # 拍照
    assert t["params"]["mark_on_map"] is True          # 地图标注
    assert t["params"]["search_strategy"] == "frontier_explore"
    assert t["params"]["use_lidar_person_range"] is True


# ---- 同义说法变体 (STT 产生的措辞差异 + 用户自然语言变化, 都应命中) ----

VARIANTS = [
    "搜索这个房间把所有人标注出来",
    "搜索当前房间标注所有人",
    "找一下这个房间里的人",
    "找这个房间的人并标记出来",
    "标记这个房间的人员",
    "搜寻本房间所有人",
    "把当前房间的人员标出来",
    "搜一下这间屋的人",
    "去搜索这个房间把所有人标记出来",
    "圈出当前房间里的全部人员",
]

def test_command_variants_hit_current_room_search():
    """所有变体都应命中 search_room + __current__ + [person]。"""
    for text in VARIANTS:
        result = parse_product_command(text)
        assert result is not None, f"应命中但返回 None: {text!r}"
        t = result["tasks"][0]
        assert t["type"] == "search_room", f"应为 search_room: {text!r}"
        assert t["params"]["room"] == "__current__", f"应为 __current__: {text!r}"
        assert t["params"]["target_classes"] == ["person"], f"应为 [person]: {text!r}"
        assert t["params"]["require_photos"] is True, f"应 require_photos: {text!r}"
        assert t["params"]["mark_on_map"] is True, f"应 mark_on_map: {text!r}"


# ---- 否定句不触发 (防止"别搜索"被误执行为搜索) ----

NEGATIVES = [
    "别搜索这个房间",
    "不要找这个房间的人",
    "不用标记所有人",
    "别去这个房间",
]

def test_negatives_return_none():
    for text in NEGATIVES:
        assert parse_product_command(text) is None, f"否定句不应触发: {text!r}"


# ---- 命名房间 (扩展场景: "去客厅搜索所有人") ----

def test_named_room_living_room():
    result = parse_product_command("去客厅搜索所有人")
    assert result is not None
    t = result["tasks"][0]
    assert t["type"] == "search_room"
    assert t["params"]["room"] == "客厅"
    assert t["params"]["target_classes"] == ["person"]


def test_named_room_lab():
    result = parse_product_command("搜索实验室里的人")
    assert result is not None
    assert result["tasks"][0]["params"]["room"] == "实验室"


def test_named_room_office():
    result = parse_product_command("去办公室找所有人标注出来")
    assert result is not None
    assert result["tasks"][0]["params"]["room"] == "办公室"


# ---- 非房间搜索指令不误判 (确保 fallback 到 VLM/关键词路径) ----

NON_ROOM = [
    "前进两米",
    "左转90度",
    "停下来",
    "返回起点",
    "",
    "   ",
    "你好",
    "跟着前面的人",   # follow 指令, 不是房间搜索
]

def test_non_room_commands_return_none():
    for text in NON_ROOM:
        assert parse_product_command(text) is None, f"非房间搜索指令不应命中: {text!r}"


# ---- resolve_current_room: __current__ → 实际房间名 ----

def test_resolve_current_room_point_inside_search_area():
    rooms = [{"name": "客厅",
              "search_area": {"origin_x": 0, "origin_y": 0, "width": 5, "height": 5},
              "nav_pose": {"x": 2.5, "y": 2.5}}]
    assert resolve_current_room(1.0, 1.0, rooms) == "客厅"


def test_resolve_current_room_falls_back_to_nearest_nav_pose():
    rooms = [
        {"name": "客厅", "search_area": None, "nav_pose": {"x": 0.0, "y": 0.0}},
        {"name": "卧室", "search_area": None, "nav_pose": {"x": 10.0, "y": 10.0}},
    ]
    assert resolve_current_room(9.0, 9.0, rooms) == "卧室"


def test_resolve_current_room_ignores_malformed_entries():
    rooms = [
        {"name": ""},                       # 空名
        {"foo": "bar"},                     # 无 name
        {"name": "客厅", "search_area": "bad", "nav_pose": None},  # 畸形数据
    ]
    assert resolve_current_room(0.0, 0.0, rooms) is None


def test_resolve_current_room_non_finite_returns_none():
    rooms = [{"name": "客厅", "search_area": {"origin_x": 0, "origin_y": 0,
              "width": 5, "height": 5}}]
    assert resolve_current_room(float("nan"), 1.0, rooms) is None
    assert resolve_current_room(float("inf"), 1.0, rooms) is None
