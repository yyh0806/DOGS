"""Fail-closed adapter for local command normalization APIs.

The model may only propose one command string.  Callers must still pass that
string through the product command validator before performing any action.
"""

from __future__ import annotations

import ipaddress
import json
import math
import re
import urllib.parse
import urllib.request


MAX_TRANSCRIPT_CHARS = 1000
MAX_COMMAND_CHARS = 1000
MAX_RESPONSE_BYTES = 16 * 1024

SYSTEM_PROMPT = """你是机器狗语音指令规范化器，只能返回一个规范指令或 null。
允许的移动指令只有四种完整格式：前进[<距离>米]、后退[<距离>米]、左转[<角度>度]、右转[<角度>度]，方括号内整体可省略。省略时距离默认为 1 米、角度默认为 90 度；写数值时必须同时写单位。距离范围必须为 (0,20] 米，角度范围必须为 (0,360] 度。
搜索仅限当前房间，完整格式为“搜索房间标注所有<目标>”或“搜索当前房间并标注所有<目标>”；不得指定其他房间。
必须拒绝 shell 或系统命令、坐标或导航点、多个命令或串联动作、解释性文字或其他 prose、函数或工具调用。不得输出这些内容的一部分。
无法安全归一化时 command 必须为 null。只输出严格 JSON：{"command": string|null}；禁止额外字段、Markdown 或任何其他文字。"""

_FENCED_JSON = re.compile(
    r"\A```(?:json)?[ \t]*\r?\n(?P<body>(?:(?!```).)*)\r?\n```[ \t]*\Z",
    re.DOTALL | re.IGNORECASE,
)


def _endpoint_kind(url: str) -> str | None:
    try:
        parsed = urllib.parse.urlsplit(url)
        hostname = parsed.hostname
        port = parsed.port
        username = parsed.username
        password = parsed.password
    except (TypeError, ValueError):
        return None
    if (parsed.scheme not in {"http", "https"} or not parsed.netloc
            or not hostname or username is not None or password is not None):
        return None
    if parsed.netloc.endswith(":") or (port is not None and not 1 <= port <= 65535):
        return None
    if hostname.lower() != "localhost":
        try:
            address = ipaddress.ip_address(hostname)
        except ValueError:
            return None
        if not ((address.version == 4 and address.is_loopback)
                or address == ipaddress.IPv6Address("::1")):
            return None
    if parsed.query or parsed.fragment:
        return None
    path = parsed.path.rstrip("/") or "/"
    if path == "/api/chat":
        return "ollama"
    if path == "/v1/chat/completions":
        return "openai"
    return None


def _parse_content(content: object) -> str | None:
    if not isinstance(content, str) or len(content) > MAX_RESPONSE_BYTES:
        return None
    candidate = content.strip()
    fenced = _FENCED_JSON.fullmatch(candidate)
    if candidate.startswith("```"):
        if fenced is None:
            return None
        candidate = fenced.group("body").strip()
    elif "```" in candidate:
        return None

    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate JSON key")
            result[key] = value
        return result

    try:
        payload = json.loads(candidate, object_pairs_hook=unique_object)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or set(payload) != {"command"}:
        return None
    command = payload["command"]
    if command is None:
        return None
    if not isinstance(command, str):
        return None
    command = command.strip()
    if not command or len(command) > MAX_COMMAND_CHARS:
        return None
    return command


class _NoRedirectHandler(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, file_pointer, code, message, headers, new_url):
        return None


def _urllib_transport(url: str, body: dict, timeout: float) -> object:
    encoded = json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=encoded,
        headers={"Accept": "application/json", "Content-Type": "application/json"},
        method="POST",
    )
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({}), _NoRedirectHandler())
    with opener.open(request, timeout=timeout) as response:
        raw = response.read(MAX_RESPONSE_BYTES + 1)
    if len(raw) > MAX_RESPONSE_BYTES:
        raise ValueError("local LLM response is too large")
    return json.loads(raw.decode("utf-8"))


class LocalLLMCommandNormalizer:
    """Normalize one transcript through Ollama or an OpenAI-compatible server."""

    def __init__(self, url: str, model: str, timeout: float = 5.0, transport=None):
        try:
            parsed_timeout = float(timeout)
        except (TypeError, ValueError) as exc:
            raise ValueError("LLM timeout must be a finite positive number") from exc
        if not math.isfinite(parsed_timeout) or parsed_timeout <= 0.0:
            raise ValueError("LLM timeout must be a finite positive number")
        if not isinstance(model, str) or not model.strip():
            raise ValueError("LLM model must not be empty")
        self.url = str(url).strip()
        self.model = model.strip()
        self.timeout = parsed_timeout
        self._transport = transport or _urllib_transport

    @property
    def supported_endpoint(self) -> bool:
        return _endpoint_kind(self.url) is not None

    def __call__(self, text: str) -> str | None:
        return self.normalize(text)

    def normalize(self, text: str) -> str | None:
        transcript = text.strip() if isinstance(text, str) else ""
        endpoint = _endpoint_kind(self.url)
        if not transcript or len(transcript) > MAX_TRANSCRIPT_CHARS or endpoint is None:
            return None

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": transcript},
        ]
        if endpoint == "ollama":
            body = {
                "model": self.model,
                "messages": messages,
                "stream": False,
                "format": "json",
                "options": {"temperature": 0},
            }
        else:
            body = {
                "model": self.model,
                "messages": messages,
                "temperature": 0,
                "response_format": {"type": "json_object"},
            }

        try:
            payload = self._transport(self.url, body, self.timeout)
            if not isinstance(payload, dict) or _contains_tool_request(payload):
                return None
            if endpoint == "ollama":
                message = payload["message"]
                if not isinstance(message, dict) or _contains_tool_request(message):
                    return None
                content = message["content"]
            else:
                choices = payload["choices"]
                if not isinstance(choices, list) or not choices:
                    return None
                choice = choices[0]
                if not isinstance(choice, dict) or _contains_tool_request(choice):
                    return None
                if choice.get("finish_reason") in {"tool_calls", "function_call"}:
                    return None
                message = choice["message"]
                if not isinstance(message, dict) or _contains_tool_request(message):
                    return None
                content = message["content"]
            return _parse_content(content)
        except Exception:
            # Network, HTTP, timeout, response-shape and JSON errors all fail closed.
            return None


def _contains_tool_request(payload: dict) -> bool:
    for key in ("tool_calls", "function_call"):
        value = payload.get(key)
        if value not in (None, "", [], {}):
            return True
    return False
