"""PC 端语音对话控制台 — Vosk STT + TTS 双向语音 (全离线)。

发指令: 麦克风 → Vosk STT → POST /api/command → NX
收反馈: NX WebSocket(8001) → 关键状态 (mission_report / FAILED / ARRIVED) → pyttsx3 TTS 播报

绕开浏览器 Web Speech API 的安全上下文限制; PC 本地全离线 (Vosk + pyttsx3).
NLU (parse_product_command + VLM 兜底) 全在 NX 侧, PC 只做"听 + 发原文 + 播报".

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
    --port 8000      NX HTTP 端口
    --ws-port 8001   NX WebSocket 端口
"""
import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from pathlib import Path

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


def validate_search_command(text: str) -> dict:
    """Allow only product-grade room target-search voice intents."""
    raw_text = text.strip() if isinstance(text, str) else ""
    result = parse_product_command(raw_text)
    tasks = result.get("tasks", []) if isinstance(result, dict) else []
    task = tasks[0] if len(tasks) == 1 and isinstance(tasks[0], dict) else None
    params = task.get("params", {}) if isinstance(task, dict) else {}
    try:
        mission = SearchMissionRequest.from_dict(params.get("mission_request"))
    except (MissionValidationError, TypeError):
        mission = None
    valid = (
        task is not None
        and task.get("type") == "search_room"
        and mission is not None
        and params == mission.to_task_params()
        or (
            mission is not None
            and mission.room != "current_room"
            and params.get("room") == mission.room
            and params.get("target_classes") == list(mission.target_classes)
            and params.get("search_strategy") == mission.search_strategy
            and params.get("require_photos") is mission.require_photos
            and params.get("mark_on_map") is mission.mark_on_map
        )
    )
    if not valid:
        return {
            "ok": False,
            "reason": "unsupported_voice_command",
            "text": raw_text,
        }
    fingerprint = json.dumps(mission.to_dict(), ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    return {
        "ok": True,
        "text": raw_text,
        "response": result.get("response", ""),
        "task": task,
        "fingerprint": fingerprint,
    }


class SearchCommandDispatcher:
    """Validate, submit, and de-duplicate confirmed room-search commands."""

    def __init__(self, *, sender=None, dedupe_seconds: float = 15.0,
                 monotonic=time.monotonic):
        try:
            dedupe = float(dedupe_seconds)
        except (TypeError, ValueError) as exc:
            raise ValueError("dedupe_seconds must be a finite non-negative number") from exc
        if not math.isfinite(dedupe) or dedupe < 0.0:
            raise ValueError("dedupe_seconds must be a finite non-negative number")
        self._sender = sender or send_command
        self._dedupe_seconds = dedupe
        self._monotonic = monotonic
        self._last_fingerprint = None
        self._last_accepted_at = None

    def dispatch(self, api_url: str, text: str) -> dict:
        validation = validate_search_command(text)
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
            }

        result = dict(self._sender(api_url, validation["text"]) or {})
        result.setdefault("text", validation["text"])
        result.setdefault("task", validation["task"])
        if result.get("ok") is True and result.get("accepted") is True:
            self._last_fingerprint = fingerprint
            self._last_accepted_at = now
        return result

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
    # 其他 type (frame/scan/locate/vlm/partial) 不播报


def _ws_worker(nx_ws_url: str) -> None:
    """后台线程: 连 NX WS, 断开后 2s 自动重连."""
    while True:
        try:
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
def send_command(nx_url: str, text: str, *, token: str | None = None) -> dict:
    """POST /api/command and require feedback-confirmed task admission."""
    try:
        import requests
    except ImportError as e:
        print("  [FAIL] requests 未安装 (pip install -r requirements-voice.txt)")
        return {"ok": False, "reason": "requests_missing", "error": str(e)}
    try:
        token = load_control_token(environ=os.environ) if token is None else token
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        r = requests.post(
            nx_url, json={"text": text}, headers=headers, timeout=5)
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
            print("  [OK] NX 已确认接收搜索任务")
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
    p.add_argument("--model", default=default_model_path(),
                   help="Vosk 中文模型路径")
    p.add_argument("--text", default=None,
                   help="不用麦克风，直接验证或发送这段文本")
    p.add_argument("--dedupe-seconds", type=float, default=15.0,
                   help="相同搜索任务成功下发后的防重复秒数 (默认 15)")
    p.add_argument("--token-file", default=None,
                   help="控制 Token 文件；默认读取 GO2W_CONTROL_TOKEN")
    p.add_argument("--no-auto-send", action="store_true",
                   help="识别后只验证不发送")
    p.add_argument("--no-tts", action="store_true", help="关闭 TTS 播报")
    args = p.parse_args()

    nx_url = f"http://{args.nx}:{args.port}/api/command"
    nx_ws_url = f"ws://{args.nx}:{args.ws_port}"
    try:
        control_token = load_control_token(args.token_file)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2
    if not args.no_auto_send and not control_token:
        print("[FAIL] 自动发送需要 --token-file 或 GO2W_CONTROL_TOKEN")
        return 2
    try:
        dispatcher = SearchCommandDispatcher(
            sender=lambda url, command: send_command(
                url, command, token=control_token),
            dedupe_seconds=args.dedupe_seconds)
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return 2

    # 文本模式与麦克风使用同一个安全门；不加载模型、不打开音频设备。
    if args.text is not None:
        validation = validate_search_command(args.text)
        if args.no_auto_send:
            print(json.dumps(validation, ensure_ascii=False, indent=2))
            if validation.get("ok"):
                print("[OK] 仅验证：语音文本将生成 search_room 任务，未发送到 NX")
                return 0
            print("[FAIL] 仅支持已配置目标的房间搜索并标注指令")
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
        threading.Thread(target=_ws_worker, args=(nx_ws_url,), daemon=True).start()
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
                            validation = validate_search_command(text)
                            print(json.dumps(validation, ensure_ascii=False))
                            print("  (no-auto-send 模式，仅验证，不发送)")
                        else:
                            result = dispatcher.dispatch(nx_url, text)
                            reason = result.get("reason")
                            if result.get("ok"):
                                if use_tts:
                                    speak("搜索任务已接收")
                            elif reason == "unsupported_voice_command":
                                print("  [IGNORED] 不是受支持的房间目标搜索指令")
                            elif reason == "duplicate_voice_command":
                                print("  [IGNORED] 重复语音，未再次下发")
                            elif use_tts:
                                speak(f"搜索任务未接收：{reason or '未知原因'}")
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
