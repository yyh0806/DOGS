"""PC 端语音对话控制台 — Vosk STT + TTS 双向语音 (全离线)。

发指令: 麦克风 → Vosk STT → 确定性解析/可选本地 LLM 归一化 → POST /api/command → NX
收反馈: NX WebSocket(8001) → 关键状态 (mission_report / FAILED / ARRIVED) → pyttsx3 TTS 播报

绕开浏览器 Web Speech API 的安全上下文限制; PC 本地全离线 (Vosk + pyttsx3).
本地 LLM 只归一化文本；每个结果仍须通过 PC 确定性解析器和 NX 安全门。

依赖:
    pip install vosk sounddevice requests pyttsx3 websocket-client
    下载 Vosk 中文模型 (~50MB):
      https://alphacephei.com/vosk/models → vosk-model-small-cn-0.22
      解压到 ./vosk-model-small-cn-0.22 或设 VOSK_MODEL_PATH 指向它

跑法:
    python tools/voice_console.py --nx 192.168.43.41
    (说一句话 → 自动发送 → NX 执行 → TTS 播报结果. Ctrl+C 退出)

可选:
    --no-auto-send   识别后只显示不发送 (防 STT 误识别让狗乱跑)
    --no-tts         关闭 TTS 播报 (只看文字)
    --model PATH     Vosk 模型路径 (默认 VOSK_MODEL_PATH 或 ./vosk-model-small-cn-0.22)
    --token-file PATH 读取部署时生成的控制 Token
    --llm-url URL    Ollama /api/chat 或 OpenAI 兼容 /v1/chat/completions
    --llm-mode MODE  off、fallback 或 always
    --port 8000      NX HTTP 端口
    --ws-port 8001   NX WebSocket 端口
"""
import argparse
import ipaddress
import json
import math
import os
import queue
import re
import socket
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

SAMPLE_RATE = 16000
PROJECT_ROOT = Path(__file__).resolve().parents[1]
WEB_DIR = PROJECT_ROOT / "web"
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_product_command import parse_product_command
from nx_mission_schema import MissionValidationError, SearchMissionRequest


def default_model_path() -> str:
    return os.environ.get(
        "VOSK_MODEL_PATH",
        str(PROJECT_ROOT / "models" / "vosk-model-small-cn-0.22"),
    )


def load_control_token(path=None, *, environ=None) -> str:
    """Load one URL-safe control token without silently joining lines."""
    environ = os.environ if environ is None else environ
    try:
        raw = (Path(path).read_text(encoding="ascii") if path
               else str(environ.get("GO2W_CONTROL_TOKEN", "")))
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"control token cannot be read: {exc}") from exc
    lines = raw.splitlines()
    if not raw:
        return ""
    if len(lines) != 1:
        raise ValueError("control token must be one URL-safe line")
    token = lines[0]
    allowed = set(
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        "0123456789._~-"
    )
    if not token or any(character not in allowed for character in token):
        raise ValueError("control token must be one URL-safe line")
    if len(token) < 32:
        raise ValueError("control token must contain at least 32 URL-safe characters")
    return token


def validate_voice_command(text: str) -> dict:
    """放行所有确定性产品指令 (search_room OR move_relative).

    search_room 仍校验嵌套 mission_request 合法; move_relative 由 parse_product_command
    + canonicalize_move_tasks 保证合法, 这里直接放行. fingerprint 含 direction/distance/angle,
    所以"前进两米"和"前进一米"互不压制 (dedupe 不误杀).
    """
    raw_text = text.strip() if isinstance(text, str) else ""
    result = parse_product_command(raw_text)
    tasks = result.get("tasks", []) if isinstance(result, dict) else []
    task = tasks[0] if len(tasks) == 1 and isinstance(tasks[0], dict) else None
    if task is None:
        return {
            "ok": False,
            "reason": "unsupported_voice_command",
            "text": raw_text,
        }
    if task.get("type") == "search_room":
        try:
            SearchMissionRequest.from_dict(task["params"]["mission_request"])
        except (MissionValidationError, TypeError, KeyError):
            return {
                "ok": False,
                "reason": "unsupported_voice_command",
                "text": raw_text,
            }
    fingerprint = json.dumps(task, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    return {
        "ok": True,
        "text": raw_text,
        "response": result.get("response", ""),
        "task": task,
        "fingerprint": fingerprint,
    }


# 向后兼容: 旧调用方 (SearchCommandDispatcher.dispatch 等) 仍可用旧名
validate_search_command = validate_voice_command


_CANONICAL_MOVE_PROPOSAL = re.compile(
    r"(?P<direction>前进|后退|左转|右转)"
    r"(?:(?P<amount>\d+(?:\.\d+)?|[零一二两三四五六七八九十百半]+)"
    r"(?P<unit>米|度))?"
)
_CN_DIGITS = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3,
              "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}


def _canonical_number(text: str) -> float | None:
    if re.fullmatch(r"\d+(?:\.\d+)?", text):
        return float(text)
    if text == "半":
        return 0.5
    if text == "一百":
        return 100.0
    if text == "十":
        return 10.0
    if len(text) == 1 and text in _CN_DIGITS:
        return float(_CN_DIGITS[text])
    if len(text) == 2 and text[0] == "十" and text[1] in _CN_DIGITS:
        return float(10 + _CN_DIGITS[text[1]])
    if len(text) == 2 and text[0] in _CN_DIGITS and text[1] == "十":
        return float(_CN_DIGITS[text[0]] * 10)
    if (len(text) == 3 and text[0] in _CN_DIGITS and text[1] == "十"
            and text[2] in _CN_DIGITS):
        return float(_CN_DIGITS[text[0]] * 10 + _CN_DIGITS[text[2]])
    return None


def validate_model_proposal(text: str) -> dict:
    """Apply the narrow canonical grammar to an already-untrusted proposal."""
    validation = validate_voice_command(text)
    if not validation.get("ok"):
        return validation

    proposal = validation["text"]
    task = validation["task"]
    params = task.get("params", {})
    task_type = task.get("type")

    if task_type == "search_room":
        if params.get("room") != "__current__":
            return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}
        response = validation.get("response", "")
        prefix = "搜索当前房间并标注"
        if not isinstance(response, str) or not response.startswith(prefix):
            return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}
        target = response[len(prefix):]
        if target.startswith("全部"):
            target = f"所有{target[2:]}"
        elif not target.startswith("所有"):
            target = f"所有{target}"
        canonical_forms = {
            f"搜索房间标注{target}",
            f"搜索当前房间并标注{target}",
        }
        if proposal not in canonical_forms:
            return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}
        return validation

    if task_type == "move_relative":
        match = _CANONICAL_MOVE_PROPOSAL.fullmatch(proposal)
        if match is None or params.get("clamped") is True:
            return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}
        direction = match.group("direction")
        amount_text = match.group("amount")
        unit = match.group("unit")
        if direction in {"前进", "后退"}:
            expected_direction = "forward" if direction == "前进" else "backward"
            admitted_amount = params.get("distance_m")
            amount = 1.0 if amount_text is None else _canonical_number(amount_text)
            valid_range = amount is not None and 0.0 < amount <= 20.0
            valid_unit = unit is None if amount_text is None else unit == "米"
        else:
            expected_direction = "left" if direction == "左转" else "right"
            admitted_amount = params.get("angle_deg")
            amount = 90.0 if amount_text is None else _canonical_number(amount_text)
            valid_range = amount is not None and 0.0 < amount <= 360.0
            valid_unit = unit is None if amount_text is None else unit == "度"
        if not valid_range or not valid_unit or params.get("direction") != expected_direction:
            return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}
        try:
            same_amount = math.isclose(float(admitted_amount), amount)
        except (TypeError, ValueError):
            same_amount = False
        if not same_amount:
            return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}
        return validation

    return {"ok": False, "reason": "unsupported_voice_command", "text": proposal}


class SearchCommandDispatcher:
    """Validate, optionally normalize, submit, and de-duplicate commands."""

    def __init__(self, *, sender=None, dedupe_seconds: float = 15.0,
                 monotonic=time.monotonic, normalizer=None,
                 normalizer_mode: str | None = None):
        try:
            dedupe = float(dedupe_seconds)
        except (TypeError, ValueError) as exc:
            raise ValueError("dedupe_seconds must be a finite non-negative number") from exc
        if not math.isfinite(dedupe) or dedupe < 0.0:
            raise ValueError("dedupe_seconds must be a finite non-negative number")
        mode = ("fallback" if normalizer is not None else "off") \
            if normalizer_mode is None else str(normalizer_mode).strip().lower()
        if mode not in {"off", "fallback", "always"}:
            raise ValueError("normalizer_mode must be one of: off, fallback, always")
        self._sender = sender or send_command
        self._dedupe_seconds = dedupe
        self._monotonic = monotonic
        self._normalizer = normalizer
        self._normalizer_mode = mode
        self._last_fingerprint = None
        self._last_accepted_at = None

    def _normalize(self, text: str) -> str | None:
        if self._normalizer is None:
            return None
        try:
            if callable(self._normalizer):
                normalized = self._normalizer(text)
            else:
                normalized = self._normalizer.normalize(text)
        except Exception:
            return None
        return normalized if isinstance(normalized, str) else None

    @staticmethod
    def _with_admission_metadata(validation: dict, original_text: str,
                                 normalized_text: str | None, *,
                                 normalizer_attempted: bool,
                                 normalizer_status: str) -> dict:
        result = dict(validation)
        result["original_text"] = original_text
        result["normalized_text"] = normalized_text
        result["normalizer_attempted"] = bool(normalizer_attempted)
        result["normalizer_status"] = str(normalizer_status)
        return result

    def admit(self, text: str) -> dict:
        """Return the single parser-admitted command without sending it."""
        original_text = text.strip() if isinstance(text, str) else ""
        deterministic = validate_voice_command(original_text)

        if self._normalizer_mode == "off":
            return self._with_admission_metadata(
                deterministic, original_text, original_text,
                normalizer_attempted=False,
                normalizer_status="disabled")

        if self._normalizer_mode == "fallback" and deterministic.get("ok"):
            return self._with_admission_metadata(
                deterministic, original_text, original_text,
                normalizer_attempted=False,
                normalizer_status="not_needed")

        normalized_text = self._normalize(original_text)
        normalizer_attempted = self._normalizer is not None
        normalizer_status = (
            "no_proposal" if normalizer_attempted else "unavailable")
        if self._normalizer_mode == "always" and deterministic.get("ok"):
            metadata_text = original_text
            if normalized_text is not None:
                normalized = validate_model_proposal(normalized_text)
                if (normalized.get("ok")
                        and normalized.get("fingerprint") == deterministic.get("fingerprint")):
                    metadata_text = normalized["text"]
                    normalizer_status = "admitted"
                else:
                    normalizer_status = "rejected_by_safety_gate"
            return self._with_admission_metadata(
                deterministic, original_text, metadata_text,
                normalizer_attempted=normalizer_attempted,
                normalizer_status=normalizer_status)

        if normalized_text is not None:
            normalized = validate_model_proposal(normalized_text)
            if normalized.get("ok"):
                return self._with_admission_metadata(
                    normalized, original_text, normalized["text"],
                    normalizer_attempted=normalizer_attempted,
                    normalizer_status="admitted")
            if not deterministic.get("ok"):
                return self._with_admission_metadata(
                    normalized, original_text, normalized_text,
                    normalizer_attempted=normalizer_attempted,
                    normalizer_status="rejected_by_safety_gate")

        # In always mode a known product command remains safe if the local model
        # is unavailable or proposes anything the deterministic parser rejects.
        if deterministic.get("ok"):
            return self._with_admission_metadata(
                deterministic, original_text, original_text,
                normalizer_attempted=normalizer_attempted,
                normalizer_status=normalizer_status)
        return self._with_admission_metadata(
            deterministic, original_text, normalized_text,
            normalizer_attempted=normalizer_attempted,
            normalizer_status=normalizer_status)

    def dispatch(self, api_url: str, text: str) -> dict:
        validation = self.admit(text)
        if not validation.get("ok"):
            return validation

        now = self._monotonic()
        fingerprint = validation["fingerprint"]
        if (
            self._last_fingerprint == fingerprint
            and self._last_accepted_at is not None
            and 0.0 <= now - self._last_accepted_at < self._dedupe_seconds
        ):
            return {
                "ok": False,
                "accepted": False,
                "reason": "duplicate_voice_command",
                "text": validation["text"],
                "task": validation["task"],
                "original_text": validation["original_text"],
                "normalized_text": validation["normalized_text"],
                "normalizer_attempted": validation["normalizer_attempted"],
                "normalizer_status": validation["normalizer_status"],
            }

        result = dict(self._sender(api_url, validation["text"]) or {})
        result["text"] = validation["text"]
        result["task"] = validation["task"]
        result["original_text"] = validation["original_text"]
        result["normalized_text"] = validation["normalized_text"]
        result["normalizer_attempted"] = validation["normalizer_attempted"]
        result["normalizer_status"] = validation["normalizer_status"]
        if result.get("ok") is True and result.get("accepted") is True:
            self._last_fingerprint = fingerprint
            self._last_accepted_at = now
        return result


def normalizer_feedback(result: dict) -> str | None:
    """Return one concise, user-visible explanation of the LLM fallback."""
    if not isinstance(result, dict) or not result.get("normalizer_attempted"):
        return None
    status = result.get("normalizer_status")
    if status == "admitted":
        return f"已归一化为：{result.get('normalized_text') or ''}"
    if status == "rejected_by_safety_gate":
        return (
            "兜底已调用，但输出未通过安全校验："
            f"{result.get('normalized_text') or '空'}"
        )
    if status == "no_proposal":
        return "兜底已调用，但未返回可用规范指令"
    return "兜底不可用"


# 可选依赖: TTS + WebSocket (缺失则降级, 不阻塞 STT 主功能)
try:
    import pyttsx3
    _HAS_TTS = True
except Exception:
    _HAS_TTS = False

try:
    import websocket  # websocket-client
    _HAS_WS = True
except Exception:
    _HAS_WS = False


# ============================================================================
# TTS: pyttsx3 离线播报 (单线程 + 队列, 避免并发 say)
# ============================================================================
_tts_queue: "queue.Queue[str | None]" = queue.Queue()
_tts_engine = None


def _tts_worker() -> None:
    """后台线程: 顺序播报 (pyttsx3 不能并发 say)."""
    global _tts_engine
    try:
        _tts_engine = pyttsx3.init()
        # 找中文语音 (Windows: Huihui/Yaoyao; Linux/Mac 也可能装了)
        for v in _tts_engine.getProperty("voices"):
            name = (getattr(v, "name", "") + " " + getattr(v, "id", "")).lower()
            if any(k in name for k in ("chinese", "zh", "huihui", "yaoyao", "tingting")):
                _tts_engine.setProperty("voice", v.id)
                break
        _tts_engine.setProperty("rate", 180)  # 语速 (默认 200 偏快)
    except Exception as e:
        print(f"[TTS] 初始化失败 (降级为只打印): {e}")
        _tts_engine = None

    while True:
        text = _tts_queue.get()
        if text is None:
            break
        try:
            if _tts_engine:
                _tts_engine.say(text)
                _tts_engine.runAndWait()
            else:
                print(f"[TTS-TEXT] {text}")
        except Exception as e:
            print(f"[TTS] 播报失败: {e}")


def speak(text: str) -> None:
    """主线程/WS 线程调: 把文本塞进队列让 TTS 线程播报."""
    print(f"[TTS] {text}")
    _tts_queue.put(text)


# ============================================================================
# WebSocket: 接收 NX 反馈, 关键状态 → TTS
# ============================================================================
def _on_ws_message(ws, message: str) -> None:
    """NX 广播 → 过滤关键状态 → TTS. 不播报视频帧/雷达/partial (太吵)."""
    try:
        data = json.loads(message)
    except Exception:
        return
    mtype = data.get("type")
    payload = data.get("data") or {}

    if mtype == "mission_report":
        dets = payload.get("detections") or []
        room = payload.get("room") or payload.get("room_name") or ""
        n = len(dets)
        if n > 0:
            speak(f"任务完成，在{room}找到{n}人" if room else f"任务完成，找到{n}人")
        else:
            speak(f"任务完成，但{room or '房间'}未发现人")
    elif mtype == "search_room":
        phase = payload.get("phase")
        room = payload.get("room") or ""
        if phase == "FAILED":
            reason = payload.get("reason") or payload.get("msg") or "未知原因"
            speak(f"任务失败：{reason}")
        elif phase == "ARRIVED" and room and room != "__frontier__":
            # 有图 (next_best_view) 路径才 ARRIVED; 无图路径 (frontier_explore) 不发此 phase
            speak(f"已到达{room}")
        elif phase == "INIT_SLAM":
            # 无图 (frontier_explore) 路径开始: SLAM 建图初始化, 给用户"任务启动"反馈
            speak("开始探索房间")
    elif mtype == "move_result":
        phase = payload.get("phase")
        direction = payload.get("direction")
        dir_cn = {"forward": "前进", "backward": "后退",
                  "left": "左转", "right": "右转"}.get(direction, direction or "")
        amount = payload.get("distance_m")
        if amount is None:
            amount = payload.get("angle_deg")
        unit = "米" if payload.get("distance_m") is not None else "度"
        if phase == "succeeded" and amount is not None:
            speak(f"已{dir_cn}{amount}{unit}")
        elif phase == "aborted":
            reason = payload.get("reason") or ""
            speak(f"无法{dir_cn}：{reason}" if reason else f"无法{dir_cn}")
        elif phase == "timed_out":
            speak(f"{dir_cn}超时，已停止")
    # 其他 type (frame/scan/locate/vlm/partial) 不播报


def validate_bind_address(value: str | None) -> str | None:
    """Return a normalized IPv4 source address for direct NX connections."""
    address = str(value or "").strip()
    if not address:
        return None
    try:
        parsed = ipaddress.ip_address(address)
    except ValueError as exc:
        raise ValueError("bind address must be a valid IPv4 address") from exc
    if parsed.version != 4:
        raise ValueError("bind address must be an IPv4 address")
    return str(parsed)


def _connect_bound_websocket(nx_ws_url: str, bind_address: str,
                             *, timeout: float = 5.0):
    """Open one websocket over a TCP socket bound to the selected PC NIC."""
    parsed = urlsplit(nx_ws_url)
    if parsed.scheme != "ws" or not parsed.hostname:
        raise ValueError("bound websocket URL must use ws:// with a host")
    port = parsed.port or 80
    stream = socket.create_connection(
        (parsed.hostname, port),
        timeout=timeout,
        source_address=(validate_bind_address(bind_address), 0),
    )
    try:
        return websocket.create_connection(
            nx_ws_url, timeout=timeout, socket=stream)
    except Exception:
        stream.close()
        raise


def _ws_worker(nx_ws_url: str, bind_address: str | None = None) -> None:
    """后台线程: 连 NX WS, 断开后 2s 自动重连."""
    while True:
        ws = None
        try:
            if bind_address:
                ws = _connect_bound_websocket(nx_ws_url, bind_address)
                try:
                    while True:
                        message = ws.recv()
                        if message is None:
                            break
                        _on_ws_message(ws, message)
                finally:
                    ws.close()
                time.sleep(2)
                continue
            ws = websocket.WebSocketApp(
                nx_ws_url,
                on_message=_on_ws_message,
                on_error=lambda w, e: print(f"[WS] error: {e}"),
                on_close=lambda w, *a: print(f"[WS] closed, 2s 后重连..."),
            )
            ws.run_forever(ping_interval=10, ping_timeout=5)
        except Exception as e:
            print(f"[WS] 异常: {e}, 2s 后重连")
        time.sleep(2)


# ============================================================================
# 发指令
# ============================================================================
def _bound_requests_session(requests_module, bind_address: str):
    """Build a requests Session whose sockets originate from one local IP."""
    address = validate_bind_address(bind_address)

    class SourceAddressAdapter(requests_module.adapters.HTTPAdapter):
        def init_poolmanager(self, connections, maxsize, block=False,
                             **pool_kwargs):
            pool_kwargs["source_address"] = (address, 0)
            return super().init_poolmanager(
                connections, maxsize, block=block, **pool_kwargs)

    session = requests_module.Session()
    session.trust_env = False
    adapter = SourceAddressAdapter()
    session.mount("http://", adapter)
    session.mount("https://", adapter)
    return session


def send_command(nx_url: str, text: str, *, token: str | None = None,
                 bind_address: str | None = None) -> dict:
    """POST /api/command and require feedback-confirmed task admission."""
    try:
        import requests
    except ImportError as e:
        print("  [FAIL] requests 未安装 (pip install -r requirements-voice.txt)")
        return {"ok": False, "reason": "requests_missing", "error": str(e)}
    try:
        token = load_control_token(environ=os.environ) if token is None else token
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        session = None
        client = requests
        try:
            if bind_address:
                session = _bound_requests_session(requests, bind_address)
                client = session
            r = client.post(
                nx_url, json={"text": text}, headers=headers, timeout=5)
        finally:
            if session is not None:
                session.close()
        try:
            payload = r.json()
        except (TypeError, ValueError):
            payload = {}
        if not isinstance(payload, dict):
            payload = {}

        transport_ok = bool(r.ok)
        confirmed = (
            transport_ok
            and payload.get("ok") is True
            and payload.get("accepted") is True
        )
        result = dict(payload)
        result.update({
            "ok": confirmed,
            "transport_ok": transport_ok,
            "status_code": int(r.status_code),
        })
        result.setdefault("accepted", False)
        if confirmed:
            print("  [OK] NX 已确认接收任务")
            return result

        reason = payload.get("reason")
        if not reason:
            reason = "admission_unconfirmed" if transport_ok else "nx_rejected"
        result["reason"] = reason
        if r.text:
            result.setdefault("body", r.text[:240])
        print(f"  [FAIL] NX 未接收任务 ({r.status_code}): {reason}")
        return result
    except requests.exceptions.RequestException as e:
        print(f"  [FAIL] 连接 NX 失败 (NX 在线? {nx_url}): {e.__class__.__name__}")
        return {"ok": False, "reason": "nx_unreachable", "error": e.__class__.__name__}


def accepted_acknowledgement(result: dict) -> str:
    """Choose the confirmed acknowledgement from the admitted task type."""
    task = result.get("task") if isinstance(result, dict) else None
    if isinstance(task, dict) and task.get("type") == "move_relative":
        return "移动任务已接收"
    return "搜索任务已接收"


def rejected_acknowledgement(result: dict) -> str:
    """Choose a failure acknowledgement when an admitted task is available."""
    task = result.get("task") if isinstance(result, dict) else None
    if isinstance(task, dict) and task.get("type") == "move_relative":
        return "移动任务未接收"
    if isinstance(task, dict) and task.get("type") == "search_room":
        return "搜索任务未接收"
    return "任务未接收"


# ============================================================================
# 主循环
# ============================================================================
def main() -> int:
    p = argparse.ArgumentParser(
        description="PC 端离线语音房间目标搜索 → NX (Vosk STT + TTS)")
    p.add_argument("--nx", default=os.environ.get("GO2W_NX_HOST", "192.168.43.41"),
                   help="NX IP (默认 GO2W_NX_HOST 或 192.168.43.41)")
    p.add_argument("--port", default="8000", help="NX HTTP 端口")
    p.add_argument("--ws-port", default="8001", help="NX WebSocket 端口")
    p.add_argument(
        "--bind-address",
        default=os.environ.get("GO2W_BIND_ADDRESS", ""),
        help="PC local IPv4 source address for direct NX connections "
             "(GO2W_BIND_ADDRESS)",
    )
    p.add_argument("--model", default=default_model_path(),
                   help="Vosk 中文模型路径")
    p.add_argument("--text", default=None,
                   help="不用麦克风，直接验证或发送这段文本")
    p.add_argument("--dedupe-seconds", type=float, default=15.0,
                   help="相同任务成功下发后的防重复秒数 (默认 15)")
    p.add_argument("--token-file", default=None,
                   help="控制 Token 文件；默认读取 GO2W_CONTROL_TOKEN")
    p.add_argument("--llm-url", default=os.environ.get("GO2W_LOCAL_LLM_URL", ""),
                   help="本地 Ollama/OpenAI 兼容端点；空值禁用 (GO2W_LOCAL_LLM_URL)")
    p.add_argument("--llm-model", default=os.environ.get(
        "GO2W_LOCAL_LLM_MODEL", "qwen2.5:3b"),
        help="本地 LLM 模型名 (默认 qwen2.5:3b)")
    p.add_argument("--llm-mode", choices=("off", "fallback", "always"),
                   default=os.environ.get("GO2W_LOCAL_LLM_MODE", "fallback"),
                   help="归一化模式 (默认 fallback)")
    p.add_argument("--llm-timeout", type=float,
                   default=os.environ.get("GO2W_LOCAL_LLM_TIMEOUT", "5"),
                   help="本地 LLM 请求超时秒数，必须为有限正数 (默认 5)")
    p.add_argument("--no-auto-send", action="store_true",
                   help="识别后只验证不发送")
    p.add_argument("--no-tts", action="store_true", help="关闭 TTS 播报")
    args = p.parse_args()

    nx_url = f"http://{args.nx}:{args.port}/api/command"
    nx_ws_url = f"ws://{args.nx}:{args.ws_port}"
    try:
        bind_address = validate_bind_address(args.bind_address)
        control_token = load_control_token(args.token_file)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2
    if not args.no_auto_send and not control_token:
        print("[FAIL] 自动发送需要 --token-file 或 GO2W_CONTROL_TOKEN")
        return 2
    try:
        if not math.isfinite(args.llm_timeout) or args.llm_timeout <= 0.0:
            raise ValueError("LLM timeout must be a finite positive number")
        normalizer = None
        llm_url = args.llm_url.strip()
        if args.llm_mode != "off" and llm_url:
            from local_llm_nlu import LocalLLMCommandNormalizer
            normalizer = LocalLLMCommandNormalizer(
                llm_url, args.llm_model, timeout=args.llm_timeout)
            if not normalizer.supported_endpoint:
                raise ValueError(
                    "LLM URL is loopback-only (localhost, 127.0.0.0/8, or ::1) "
                    "and must use /api/chat or /v1/chat/completions")
        dispatcher = SearchCommandDispatcher(
            sender=lambda url, command: send_command(
                url, command, token=control_token,
                bind_address=bind_address),
            dedupe_seconds=args.dedupe_seconds,
            normalizer=normalizer,
            normalizer_mode=args.llm_mode)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2

    # 文本模式与麦克风使用同一个安全门；不加载模型、不打开音频设备。
    if args.text is not None:
        validation = dispatcher.admit(args.text)
        if args.no_auto_send:
            print(json.dumps(validation, ensure_ascii=False, indent=2))
            if validation.get("ok"):
                task_type = validation.get("task", {}).get("type", "voice")
                print(f"[OK] 仅验证：语音文本将生成 {task_type} 任务，未发送到 NX")
                return 0
            print("[FAIL] 仅支持单个相对移动或当前房间目标搜索指令")
            return 2
        result = dispatcher.dispatch(nx_url, args.text)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 1

    # 1. 模型存在性检查
    if not os.path.isdir(args.model):
        print(f"[FAIL] Vosk 模型未找到: {args.model}")
        print("下载: https://alphacephei.com/vosk/models -> vosk-model-small-cn-0.22 (~50MB)")
        print(f"解压到 '{args.model}' 或设 VOSK_MODEL_PATH 环境变量")
        return 1

    try:
        import sounddevice as sd
        from vosk import KaldiRecognizer, Model
    except ImportError as e:
        print(f"[FAIL] 语音依赖未安装: {e}")
        print("安装: pip install -r requirements-voice.txt")
        return 1

    # 2. 启动 TTS 线程 (可选依赖降级)
    use_tts = _HAS_TTS and not args.no_tts
    if use_tts:
        threading.Thread(target=_tts_worker, daemon=True).start()
        print("[TTS] 启动 (pyttsx3 离线)")
    elif not _HAS_TTS:
        print("[TTS] pyttsx3 未装 (pip install pyttsx3), 降级为只打印")
    else:
        print("[TTS] 已用 --no-tts 关闭")

    # 3. 启动 WS 监听线程 (接收 NX 反馈 → TTS)
    if _HAS_WS:
        threading.Thread(
            target=_ws_worker,
            args=(nx_ws_url, bind_address),
            daemon=True,
        ).start()
        print(f"[WS]  连 {nx_ws_url} (接收 NX 反馈)")
    else:
        print("[WS]  websocket-client 未装 (pip install websocket-client), 无 TTS 反馈")

    # 4. 加载 Vosk 模型
    print(f"加载 Vosk 模型 {args.model} ...")
    try:
        model = Model(args.model)
    except Exception as e:
        print(f"[FAIL] 模型加载失败: {e}")
        return 1
    rec = KaldiRecognizer(model, SAMPLE_RATE)
    rec.SetWords(True)

    print(f"\n[OK] 就绪. NX = {nx_url}")
    print(f"自动发送: {'否 (只显示)' if args.no_auto_send else '是'} | "
          f"TTS: {'开' if use_tts else '关'}")
    print("说一句话, 检测到静音自动结束. Ctrl+C 退出.\n")

    # 5. Vosk STT 主循环
    aq: "queue.Queue[bytes]" = queue.Queue()

    def audio_callback(indata, frames, time_info, status):
        if status:
            print(f"[audio] {status}", file=sys.stderr)
        aq.put(bytes(indata))

    try:
        with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=8000, dtype="int16",
                               channels=1, callback=audio_callback):
            while True:
                data = aq.get()
                if rec.AcceptWaveform(data):
                    # Vosk 检测到句尾 (静音) 返回 final result
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip()
                    if text:
                        print(f"\n[VOICE] 识别: {text}")
                        if args.no_auto_send:
                            validation = dispatcher.admit(text)
                            print(json.dumps(validation, ensure_ascii=False))
                            print("  (no-auto-send 模式，仅验证，不发送)")
                        else:
                            result = dispatcher.dispatch(nx_url, text)
                            reason = result.get("reason")
                            fallback_feedback = normalizer_feedback(result)
                            if fallback_feedback:
                                print(f"  [LLM] {fallback_feedback}")
                            if result.get("ok"):
                                if use_tts:
                                    speak(accepted_acknowledgement(result))
                            elif reason == "unsupported_voice_command":
                                print("  [IGNORED] 不是受支持的单个移动或当前房间搜索指令")
                            elif reason == "duplicate_voice_command":
                                print("  [IGNORED] 重复语音，未再次下发")
                            elif use_tts:
                                failure_ack = rejected_acknowledgement(result)
                                speak(f"{failure_ack}：{reason or '未知原因'}")
                    # 空文本 = 纯静音, 忽略
                else:
                    # partial: 实时显示正在说的话
                    partial = json.loads(rec.PartialResult()).get("partial", "")
                    if partial:
                        print(f"\r  ... {partial}", end="", flush=True)
    except KeyboardInterrupt:
        print("\n\n[bye] 退出.")
        return 0
    except sd.PortAudioError as e:
        print(f"[FAIL] 麦克风初始化失败 (无设备/权限被拒?): {e}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
