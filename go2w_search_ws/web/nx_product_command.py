"""Deterministic product command parsing for room person search.

This module is intentionally offline and model-free. It only recognizes the
product-grade room/person search command path and returns None for everything
else so the strict search-only VLM path may try generic target classes.
"""

import math
import re
import hashlib
import json

from nx_mission_schema import SearchMissionRequest


_CURRENT_ROOM = "__current__"

_CURRENT_ROOM_TERMS = (
    "全屋",
    "整个房间",
    "这个房间",
    "当前房间",
    "整间房",
    "房间",
    "这间房",
    "这间屋",
    "本房间",
    "本屋",
)
_CURRENT_ROOM_EXPLICIT_TERMS = tuple(
    term for term in _CURRENT_ROOM_TERMS if term != "房间"
)

_NEGATION_TERMS = (
    "别",
    "不要",
    "不用",
)

_SEARCH_TERMS = (
    "搜索",
    "探索",
    "搜寻",
    "搜",
    "寻找",
    "查找",
    "找",
)

_EXPLICIT_PERSON_TERMS = (
    "所有人",
    "全部人",
    "全部人员",
    "所有人员",
    "人员",
)

_TABLE_TERMS = (
    "所有桌子",
    "全部桌子",
    "桌子",
    "餐桌",
)

# 物品词典 (spec §2.2): NLU 扫描中文文本 → 英文 YOLO-World 类名.
# 与 nx_mission_schema._ALIASES 对称 (两处各加一行, 新词立即可检).
_OBJECT_TERM_ALIASES = {
    "椅子": "chair", "座椅": "chair", "凳子": "chair",
    "沙发": "couch", "床": "bed", "电视": "tv",
    "冰箱": "refrigerator", "微波炉": "microwave", "烤箱": "oven",
    "笔记本": "laptop", "杯子": "cup", "瓶子": "bottle",
    "书": "book", "钟": "clock", "花瓶": "vase",
    "绿植": "potted plant", "盆栽": "potted plant",
    "背包": "backpack", "碗": "bowl", "键盘": "keyboard",
}

# "所有物体"展开 (spec §2.4): 室内家具电器大件, 不含食物/动物/室外
_ALL_OBJECTS_TERMS = ("所有物体", "全部物体", "所有东西", "全部东西")
_ALL_OBJECTS_CLASSES = (
    "person", "chair", "couch", "dining table", "bed", "tv",
    "laptop", "refrigerator", "microwave", "oven", "book",
    "clock", "vase", "potted plant", "backpack", "bottle",
    "cup", "bowl",
)

_TABLE_CHAIR_TERMS = ("桌椅", "桌子和椅子", "桌子椅子", "餐桌椅")
_CHAIR_TERMS = ("所有椅子", "全部椅子", "椅子", "座椅", "凳子")

_MARK_TERMS = (
    "标注",
    "标记",
    "标出来",
    "标出",
    "圈出",
)

_KNOWN_ROOM_TERMS = (
    "会议室",
    "办公室",
    "实验室",
    "茶水间",
    "洗手间",
    "卫生间",
    "主卧室",
    "次卧室",
    "主卧",
    "次卧",
    "客厅",
    "卧室",
    "厨房",
    "餐厅",
    "书房",
    "阳台",
    "玄关",
    "走廊",
    "机房",
)

_ROOM_NAME_HINTS = (
    "房",
    "室",
    "厅",
    "间",
    "厨",
    "卫",
    "厕",
    "阳台",
    "玄关",
    "走廊",
    "机房",
    "仓库",
)

_NON_ROOM_CANDIDATES = (
    "前面",
    "后面",
    "左边",
    "右边",
    "旁边",
    "附近",
    "这里",
    "那里",
)

_TRAILING_ROOM_WORDS_RE = re.compile(r"(里|里面|内|中|的)$")
_GO_TERMS_RE = r"(?:去|到)"
_ROOM_SUFFIX_RE = r"(?:里|里面|内|中)?"
_ROOM_PERSON_SEPARATOR_RE = r"(?:里|里面|内|中|的)*"
_REQUIRED_ROOM_PERSON_SEPARATOR_RE = r"(?:里|里面|内|中|的)"
_SEARCH_TERMS_RE = r"(?:搜索|探索|搜寻|寻找|查找|搜|找)"
_PERSON_TARGET_RE = r"(?:所有人员|全部人员|所有人|全部人|人员|人)"
_PERSON_TARGET_FOLLOW_RE = r"(?=$|一下$|一遍$|吧$|吗$|并?(?:标注|标记|标出来|标出|圈出))"
_TARGET_FOLLOW_RE = _PERSON_TARGET_FOLLOW_RE


def _terms_re(terms: tuple[str, ...]) -> str:
    return "|".join(re.escape(term) for term in sorted(terms, key=len, reverse=True))


_CURRENT_ROOM_TERMS_RE = f"(?:{_terms_re(_CURRENT_ROOM_TERMS)})"
_KNOWN_ROOM_TERMS_RE = f"(?:{_terms_re(_KNOWN_ROOM_TERMS)})"
_BARE_CURRENT_ROOM_TARGET_TERMS = (
    "人",
    *_EXPLICIT_PERSON_TERMS,
    *_TABLE_TERMS,
    *_CHAIR_TERMS,
    *_ALL_OBJECTS_TERMS,
    *_TABLE_CHAIR_TERMS,
    *_OBJECT_TERM_ALIASES,
)
_BARE_CURRENT_ROOM_FOLLOW_RE = (
    rf"{_ROOM_PERSON_SEPARATOR_RE}(?=$|"
    rf"{_terms_re(_MARK_TERMS)}|{_terms_re(_BARE_CURRENT_ROOM_TARGET_TERMS)})"
)
_BARE_FRONTIER_SEARCH_COMMANDS = tuple(
    f"{verb}{area}{suffix}"
    for verb in ("搜索", "探索", "搜寻")
    for area in ("房间", "整个房间", "这个房间", "当前房间", "整间房", "全屋")
    for suffix in ("", "一下", "一遍")
)

# 运动指令触发词 (spec §1.1): 用完整词避免"搜索前面房间"误触发前进
_MOVE_FORWARD_TERMS = ("前进", "向前走", "往前走", "直走")
_MOVE_BACKWARD_TERMS = ("后退", "向后走", "往后走", "倒退")
_MOVE_LEFT_TERMS = (
    "左转", "向左转", "左转弯", "往左转",
    "向左扭头", "往左扭头", "左扭头",
)
_MOVE_RIGHT_TERMS = (
    "右转", "向右转", "右转弯", "往右转",
    "向右扭头", "往右扭头", "右扭头",
)
_BARE_HEAD_TURN_TERMS = (
    "扭头", "扭个头", "扭头一下", "扭个头一下",
)
_BARE_TURN_AROUND_TERMS = (
    "转身", "转个身", "回头", "掉头", "转过身",
    "转身一下", "转个身一下", "回头一下", "掉头一下",
)

_MOVE_DEFAULT_DISTANCE_M = 1.0
_MOVE_DEFAULT_ANGLE_DEG = 90.0
_MOVE_MAX_DISTANCE_M = 20.0

_CN_DIGIT = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}


def parse_product_command(text: str) -> dict | None:
    """Parse product-grade room/person search commands.

    Returns the canonical search_room task for current-room or named-room
    person search intent, otherwise None.
    """
    normalized = _normalize_text(text)
    if not normalized:
        return None
    if _contains_any(normalized, _NEGATION_TERMS):
        return None

    move = _parse_move_command(normalized)
    if move is not None:
        return _move_command_result(move)

    # 省略目标时，房间/全屋的搜索或探索都启动同一套全局 frontier
    # 算法，并按产品约定默认搜索、标注人。
    if normalized in _BARE_FRONTIER_SEARCH_COMMANDS:
        return _command_result(_CURRENT_ROOM)

    if not _contains_any(normalized, _CURRENT_ROOM_EXPLICIT_TERMS):
        named_room = _extract_named_room(normalized)
        if named_room:
            return _command_result(named_room)

        named_table_room = _extract_named_room_target(normalized, _TABLE_TERMS)
        if named_table_room:
            return _command_result(
                named_table_room,
                target_class="dining table",
                target_label="所有桌子",
            )

    if _is_current_room_person_search(normalized):
        return _command_result(_CURRENT_ROOM)

    if _is_current_room_target_search(normalized, _TABLE_TERMS):
        return _command_result(
            _CURRENT_ROOM, target_class="dining table", target_label="所有桌子")

    if (_contains_current_room_reference(normalized)
            and (_contains_any(normalized, _SEARCH_TERMS)
                 or _contains_any(normalized, _MARK_TERMS))):
        obj_targets = _extract_current_room_objects(normalized)
        if obj_targets is not None:
            target_classes, label = obj_targets
            # 单纯桌子已由上面 table 路径捕获; 这里处理椅/桌椅组合/物体/任意物品
            if target_classes != ("dining table",):
                return _multi_target_command_result(
                    _CURRENT_ROOM, target_classes, label)

    named_room = _extract_named_room(normalized)
    if named_room:
        return _command_result(named_room)

    named_table_room = _extract_named_room_target(normalized, _TABLE_TERMS)
    if named_table_room:
        return _command_result(
            named_table_room,
            target_class="dining table",
            target_label="所有桌子",
        )

    return None


def resolve_current_room(robot_x: float, robot_y: float, rooms: list[dict]) -> str | None:
    """Resolve the robot's current room from room details.

    First prefer a room whose search_area rectangle contains the robot point.
    If none contains it, return the room with the nearest usable nav_pose.
    Malformed room entries are ignored.
    """
    x = _finite_float(robot_x)
    y = _finite_float(robot_y)
    if x is None or y is None:
        return None

    nearest_name = None
    nearest_dist = None

    for room in rooms or []:
        if not isinstance(room, dict):
            continue
        name = room.get("name")
        if not isinstance(name, str) or not name.strip():
            continue
        name = name.strip()

        if _point_in_search_area(x, y, room.get("search_area")):
            return name

        dist = _nav_pose_distance(x, y, room.get("nav_pose"))
        if dist is not None and (nearest_dist is None or dist < nearest_dist):
            nearest_name = name
            nearest_dist = dist

    return nearest_name


def _command_result(
    room: str,
    *,
    target_class: str = "person",
    target_label: str = "所有人",
) -> dict:
    response = (
        f"搜索当前房间并标注{target_label}"
        if room == _CURRENT_ROOM
        else f"搜索{room}并标注{target_label}"
    )
    mission = build_search_mission(room, target_class)
    params = mission.to_task_params()
    if room != _CURRENT_ROOM:
        # Named-room orchestration already owns calibrated spatial bounds.  The
        # canonical nested request retains its total mission budget.
        params.pop("max_radius_m", None)
        params.pop("max_time", None)
    return {
        "response": response,
        "tasks": [{
            "type": "search_room",
            "priority": 8,
            "params": params,
        }],
    }


def build_search_mission(room: str, target_class: str) -> SearchMissionRequest:
    """Build the canonical, deterministic command template."""
    canonical_room = "current_room" if room == _CURRENT_ROOM else str(room)
    strategy = "frontier_explore" if room == _CURRENT_ROOM else "next_best_view"
    seed = json.dumps(
        {"room": canonical_room, "target_classes": [target_class],
         "search_strategy": strategy},
        ensure_ascii=False, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")
    request_id = f"command-{hashlib.sha256(seed).hexdigest()[:24]}"
    if room == _CURRENT_ROOM:
        return SearchMissionRequest.current_room(
            (target_class,), request_id=request_id)
    return SearchMissionRequest(
        request_id=request_id,
        room=canonical_room,
        target_classes=(target_class,),
        search_strategy=strategy,
        require_photos=True,
        mark_on_map=True,
        max_radius_m=12.0,
        # 2026-07-15 实测: 180s 预算太短, 狗探索 3-4 frontier 就 time_budget_exhausted,
        # 卡住的 goal 没来得及超时 abort. 提到 480s 给多 frontier + 单 goal 超时留余量.
        max_time_s=900.0,
    )


def _normalize_text(text: str) -> str:
    if not isinstance(text, str):
        return ""
    # 保留英文小数点 (运动指令 "后退1.5米" 需要, spec §1.1); 中文句号仍去除
    return re.sub(r"[\s，。！？、,!?；;：:]+", "", text)


def _contains_any(text: str, terms: tuple[str, ...]) -> bool:
    return any(term in text for term in terms)


def _contains_current_room_reference(text: str) -> bool:
    """Return whether text contains an unqualified current-room reference."""
    if _contains_any(text, _CURRENT_ROOM_EXPLICIT_TERMS):
        return True
    return bool(re.search(
        rf"(?:{_SEARCH_TERMS_RE}|{_terms_re(_MARK_TERMS)})房间"
        f"{_BARE_CURRENT_ROOM_FOLLOW_RE}",
        text,
    ))


def _is_current_room_person_search(text: str) -> bool:
    if not _contains_current_room_reference(text):
        return False
    if not (_contains_any(text, _SEARCH_TERMS) or _contains_any(text, _MARK_TERMS)):
        return False
    if _contains_any(text, _EXPLICIT_PERSON_TERMS):
        return True
    return bool(re.search(
        rf"{_CURRENT_ROOM_TERMS_RE}{_ROOM_PERSON_SEPARATOR_RE}"
        rf"(?:{_terms_re(_MARK_TERMS)})?人{_PERSON_TARGET_FOLLOW_RE}",
        text,
    ))


def _is_current_room_target_search(
    text: str, target_terms: tuple[str, ...]
) -> bool:
    return (
        _contains_current_room_reference(text)
        and (_contains_any(text, _SEARCH_TERMS) or _contains_any(text, _MARK_TERMS))
        and _contains_any(text, target_terms)
    )


def _extract_named_room(text: str) -> str | None:
    patterns = (
        rf"{_GO_TERMS_RE}(?P<room>{_KNOWN_ROOM_TERMS_RE}){_ROOM_SUFFIX_RE}"
        rf"{_SEARCH_TERMS_RE}{_PERSON_TARGET_RE}{_PERSON_TARGET_FOLLOW_RE}",
        rf"{_SEARCH_TERMS_RE}(?P<room>{_KNOWN_ROOM_TERMS_RE})"
        rf"{_ROOM_PERSON_SEPARATOR_RE}{_PERSON_TARGET_RE}{_PERSON_TARGET_FOLLOW_RE}",
        rf"{_GO_TERMS_RE}(?P<room>[\u4e00-\u9fffA-Za-z0-9_-]{{1,12}}){_ROOM_SUFFIX_RE}"
        rf"{_SEARCH_TERMS_RE}{_PERSON_TARGET_RE}{_PERSON_TARGET_FOLLOW_RE}",
        rf"{_SEARCH_TERMS_RE}(?P<room>[\u4e00-\u9fffA-Za-z0-9_-]{{1,12}})"
        rf"{_REQUIRED_ROOM_PERSON_SEPARATOR_RE}{_PERSON_TARGET_RE}{_PERSON_TARGET_FOLLOW_RE}",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            room = _clean_room_name(match.group("room"))
            if room and room not in _CURRENT_ROOM_TERMS:
                return room
    return None


def _extract_named_room_target(
    text: str, target_terms: tuple[str, ...]
) -> str | None:
    target_re = rf"(?:所有|全部)?(?:{_terms_re(target_terms)})"
    patterns = (
        rf"{_GO_TERMS_RE}(?P<room>{_KNOWN_ROOM_TERMS_RE}){_ROOM_SUFFIX_RE}"
        rf"{_SEARCH_TERMS_RE}{target_re}{_TARGET_FOLLOW_RE}",
        rf"{_SEARCH_TERMS_RE}(?P<room>{_KNOWN_ROOM_TERMS_RE})"
        rf"{_ROOM_PERSON_SEPARATOR_RE}{target_re}{_TARGET_FOLLOW_RE}",
        rf"{_GO_TERMS_RE}(?P<room>[\u4e00-\u9fffA-Za-z0-9_-]{{1,12}}){_ROOM_SUFFIX_RE}"
        rf"{_SEARCH_TERMS_RE}{target_re}{_TARGET_FOLLOW_RE}",
        rf"{_SEARCH_TERMS_RE}(?P<room>[\u4e00-\u9fffA-Za-z0-9_-]{{1,12}})"
        rf"{_REQUIRED_ROOM_PERSON_SEPARATOR_RE}{target_re}{_TARGET_FOLLOW_RE}",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            room = _clean_room_name(match.group("room"))
            if room and room not in _CURRENT_ROOM_TERMS:
                return room
    return None


def _clean_room_name(room: str) -> str | None:
    room = _TRAILING_ROOM_WORDS_RE.sub("", room.strip())
    if not room:
        return None
    if room in _CURRENT_ROOM_TERMS:
        return None
    if room in _NON_ROOM_CANDIDATES:
        return None
    if not any(hint in room for hint in _ROOM_NAME_HINTS):
        return None
    return room


def _point_in_search_area(x: float, y: float, search_area) -> bool:
    if not isinstance(search_area, dict):
        return False
    origin_x = _finite_float(search_area.get("origin_x"))
    origin_y = _finite_float(search_area.get("origin_y"))
    width = _finite_float(search_area.get("width"))
    height = _finite_float(search_area.get("height"))
    if origin_x is None or origin_y is None or width is None or height is None:
        return False
    if width <= 0.0 or height <= 0.0:
        return False
    return origin_x <= x <= origin_x + width and origin_y <= y <= origin_y + height


def _nav_pose_distance(x: float, y: float, nav_pose) -> float | None:
    if not isinstance(nav_pose, dict):
        return None
    nav_x = _finite_float(nav_pose.get("x"))
    nav_y = _finite_float(nav_pose.get("y"))
    if nav_x is None or nav_y is None:
        return None
    return math.hypot(x - nav_x, y - nav_y)


def _finite_float(value) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def _parse_chinese_number(text: str) -> float | None:
    """解析中文/阿拉伯数字。

    支持: 1, 1.5, 一, 两, 半, 十, 十二, 二十, 二十三, 四十五, 一百。
    用于运动指令的距离/角度数值抽取 (spec §1.1)。
    """
    text = (text or "").strip()
    if not text:
        return None
    if re.fullmatch(r"\d+(\.\d+)?", text):
        return float(text)
    if text == "半":
        return 0.5
    if text in ("一百", "壹百"):
        return 100.0
    if text == "十":
        return 10.0
    if text.startswith("十") and len(text) == 2 and text[1] in _CN_DIGIT:
        return float(10 + _CN_DIGIT[text[1]])
    if len(text) == 2 and text[0] in _CN_DIGIT and text[1] == "十":
        return float(_CN_DIGIT[text[0]] * 10)
    if (len(text) == 3 and text[0] in _CN_DIGIT
            and text[1] == "十" and text[2] in _CN_DIGIT):
        return float(_CN_DIGIT[text[0]] * 10 + _CN_DIGIT[text[2]])
    if len(text) == 1 and text in _CN_DIGIT:
        return float(_CN_DIGIT[text])
    return None


def _extract_amount(text: str, unit_chars: tuple[str, ...]) -> float | None:
    """从文本抽 "数字+单位" 的数值。

    unit_chars=("米","公尺") 抽距离; ("度","°","圈") 抽角度 (圈=360°)。
    """
    unit_alt = "|".join(re.escape(u) for u in unit_chars)
    m = re.search(rf"([\d.]+|[零一二两三四五六七八九十百半]+)\s*({unit_alt})", text)
    if not m:
        return None
    raw, unit = m.group(1), m.group(2)
    value = _parse_chinese_number(raw)
    if value is None:
        return None
    if unit == "圈":
        value *= 360.0
    return value


def _detect_move_direction(text: str) -> str | None:
    if _contains_any(text, _MOVE_FORWARD_TERMS):
        return "forward"
    if _contains_any(text, _MOVE_BACKWARD_TERMS):
        return "backward"
    if _contains_any(text, _MOVE_LEFT_TERMS):
        return "left"
    if _contains_any(text, _MOVE_RIGHT_TERMS):
        return "right"
    # 没有方向的"扭头"让整只狗默认左转 90°；"转身/回头/掉头"
    # 表示原地改变朝向，默认左转 180°。
    if text in _BARE_HEAD_TURN_TERMS or text in _BARE_TURN_AROUND_TERMS:
        return "left"
    return None


def _parse_move_command(text: str) -> dict | None:
    """解析运动指令 → {mode, direction, distance_m|angle_deg, clamped}。

    mode=linear 走 nav2 (前进/后退), mode=angular 走 cmd_vel+odom (左/右转)。
    距离上限 20m (截断标 clamped=True), 角度无上限。
    """
    direction = _detect_move_direction(text)
    if direction is None:
        return None
    if direction in ("forward", "backward"):
        amount = _extract_amount(text, ("米", "公尺")) or _MOVE_DEFAULT_DISTANCE_M
        clamped = False
        if amount > _MOVE_MAX_DISTANCE_M:
            amount = _MOVE_MAX_DISTANCE_M
            clamped = True
        return {"mode": "linear", "direction": direction,
                "distance_m": amount, "clamped": clamped}
    default_angle = (
        180.0 if text in _BARE_TURN_AROUND_TERMS
        else _MOVE_DEFAULT_ANGLE_DEG
    )
    amount = _extract_amount(text, ("度", "°", "圈")) or default_angle
    return {"mode": "angular", "direction": direction,
            "angle_deg": amount, "clamped": False}


def _move_command_result(move: dict) -> dict:
    params = {"mode": move["mode"], "direction": move["direction"],
              "clamped": move["clamped"]}
    if move["mode"] == "linear":
        params["distance_m"] = move["distance_m"]
    else:
        params["angle_deg"] = move["angle_deg"]
    dir_cn = {"forward": "前进", "backward": "后退",
              "left": "左转", "right": "右转"}[move["direction"]]
    amount = move.get("distance_m", move.get("angle_deg"))
    unit = "米" if move["mode"] == "linear" else "度"
    return {
        "response": f"{dir_cn}{amount}{unit}",
        "tasks": [{"type": "move_relative", "priority": 5, "params": params}],
    }


def _extract_current_room_objects(text: str) -> tuple[tuple[str, ...], str] | None:
    """从"搜索这个房间的X"抽取目标类清单 (spec §2.1/2.3/2.4).

    返回 (target_classes, label) 或 None. 顺序:
    所有物体 → 桌椅组合 → 桌/椅/任意词典物品(可组合).
    单纯"桌子"由调用方先行捕获, 这里不单独返回 ("dining table",).
    """
    if _contains_any(text, _ALL_OBJECTS_TERMS):
        return (tuple(_ALL_OBJECTS_CLASSES), "所有物体")
    if _contains_any(text, _TABLE_CHAIR_TERMS):
        return (("dining table", "chair"), "桌椅")
    found: list[str] = []
    label_parts: list[str] = []
    if _contains_any(text, _TABLE_TERMS):
        found.append("dining table")
        label_parts.append("桌子")
    if _contains_any(text, _CHAIR_TERMS):
        found.append("chair")
        label_parts.append("椅子")
    for cn, en in _OBJECT_TERM_ALIASES.items():
        if cn in text and en not in found:
            found.append(en)
            label_parts.append(cn)
    if not found:
        return None
    seen: set[str] = set()
    unique = [c for c in found if not (c in seen or seen.add(c))]
    return (tuple(unique), "、".join(label_parts))


def _multi_target_command_result(
    room: str, target_classes: tuple[str, ...], label: str
) -> dict:
    """构造多类 search_room 任务 (spec §2.3)."""
    canonical_room = "current_room" if room == _CURRENT_ROOM else str(room)
    strategy = "frontier_explore" if room == _CURRENT_ROOM else "next_best_view"
    targets = tuple(target_classes)
    seed = json.dumps(
        {"room": canonical_room, "target_classes": list(targets),
         "search_strategy": strategy},
        ensure_ascii=False, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")
    request_id = f"command-{hashlib.sha256(seed).hexdigest()[:24]}"
    if room == _CURRENT_ROOM:
        mission = SearchMissionRequest.current_room(targets, request_id=request_id)
    else:
        mission = SearchMissionRequest(
            request_id=request_id,
            room=canonical_room,
            target_classes=targets,
            search_strategy=strategy,
            require_photos=True,
            mark_on_map=True,
            max_radius_m=12.0,
            max_time_s=900.0,
        )
    params = mission.to_task_params()
    if room != _CURRENT_ROOM:
        params.pop("max_radius_m", None)
        params.pop("max_time", None)
    response = (f"搜索当前房间并标注{label}" if room == _CURRENT_ROOM
                else f"搜索{room}并标注{label}")
    return {
        "response": response,
        "tasks": [{"type": "search_room", "priority": 8, "params": params}],
    }
