"""Deterministic product command parsing for room person search.

This module is intentionally offline and model-free. It only recognizes the
product-grade room/person search command path and returns None for everything
else so existing VLM/fallback parsing can continue unchanged.
"""

import math
import re


_CURRENT_ROOM = "__current__"

_CURRENT_ROOM_TERMS = (
    "这个房间",
    "当前房间",
    "这间房",
    "这间屋",
    "本房间",
    "本屋",
)

_NEGATION_TERMS = (
    "别",
    "不要",
    "不用",
)

_SEARCH_TERMS = (
    "搜索",
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
_ROOM_PERSON_SEPARATOR_RE = r"(?:里|里面|内|中|的)?"
_REQUIRED_ROOM_PERSON_SEPARATOR_RE = r"(?:里|里面|内|中|的)"
_SEARCH_TERMS_RE = r"(?:搜索|搜寻|寻找|查找|搜|找)"
_PERSON_TARGET_RE = r"(?:所有人员|全部人员|所有人|全部人|人员|人)"
_PERSON_TARGET_FOLLOW_RE = r"(?=$|一下$|一遍$|吧$|吗$|并?(?:标注|标记|标出来|标出|圈出))"


def _terms_re(terms: tuple[str, ...]) -> str:
    return "|".join(re.escape(term) for term in sorted(terms, key=len, reverse=True))


_CURRENT_ROOM_TERMS_RE = f"(?:{_terms_re(_CURRENT_ROOM_TERMS)})"
_KNOWN_ROOM_TERMS_RE = f"(?:{_terms_re(_KNOWN_ROOM_TERMS)})"


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

    if _is_current_room_person_search(normalized):
        return _command_result(_CURRENT_ROOM)

    named_room = _extract_named_room(normalized)
    if named_room:
        return _command_result(named_room)

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


def _command_result(room: str) -> dict:
    return {
        "response": "搜索当前房间并标注所有人",
        "tasks": [{
            "type": "search_room",
            "priority": 8,
            "params": {
                "room": room,
                "target_classes": ["person"],
                "require_photos": True,
                "mark_on_map": True,
                "search_strategy": "next_best_view",
                "use_lidar_person_range": True,
            },
        }],
    }


def _normalize_text(text: str) -> str:
    if not isinstance(text, str):
        return ""
    return re.sub(r"[\s，。！？、,.!?；;：:]+", "", text)


def _contains_any(text: str, terms: tuple[str, ...]) -> bool:
    return any(term in text for term in terms)


def _is_current_room_person_search(text: str) -> bool:
    if not _contains_any(text, _CURRENT_ROOM_TERMS):
        return False
    if not (_contains_any(text, _SEARCH_TERMS) or _contains_any(text, _MARK_TERMS)):
        return False
    if _contains_any(text, _EXPLICIT_PERSON_TERMS):
        return True
    return bool(re.search(
        rf"{_CURRENT_ROOM_TERMS_RE}{_ROOM_PERSON_SEPARATOR_RE}人{_PERSON_TARGET_FOLLOW_RE}",
        text,
    ))


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
            if room and not _contains_any(room, _CURRENT_ROOM_TERMS):
                return room
    return None


def _clean_room_name(room: str) -> str | None:
    room = _TRAILING_ROOM_WORDS_RE.sub("", room.strip())
    if not room:
        return None
    if room in ("这个房间", "当前房间", "这间房", "这间屋", "本房间", "本屋"):
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
