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
    --port 8000      NX HTTP 端口
    --ws-port 8001   NX WebSocket 端口
"""
import argparse
import json
import os
import queue
import sys
import threading
import time

import requests
import sounddevice as sd
from vosk import Model, KaldiRecognizer

SAMPLE_RATE = 16000

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
def send_command(nx_url: str, text: str) -> None:
    """POST /api/command 到 NX. NX 离线时不崩 (继续监听下一句)."""
    try:
        r = requests.post(nx_url, json={"text": text}, timeout=5)
        if r.ok:
            print(f"  [OK] 已发送到 NX")
        else:
            print(f"  [FAIL] NX 返回 {r.status_code}: {r.text[:120]}")
    except requests.exceptions.RequestException as e:
        print(f"  [FAIL] 连接 NX 失败 (NX 在线? {nx_url}): {e.__class__.__name__}")


# ============================================================================
# 主循环
# ============================================================================
def main() -> None:
    p = argparse.ArgumentParser(description="PC 端语音对话 → NX (Vosk STT + TTS)")
    p.add_argument("--nx", default=os.environ.get("GO2W_NX_HOST", "192.168.43.41"),
                   help="NX IP (默认 GO2W_NX_HOST 或 192.168.43.41)")
    p.add_argument("--port", default="8000", help="NX HTTP 端口")
    p.add_argument("--ws-port", default="8001", help="NX WebSocket 端口")
    p.add_argument("--model", default=os.environ.get("VOSK_MODEL_PATH",
                   "vosk-model-small-cn-0.22"), help="Vosk 中文模型路径")
    p.add_argument("--no-auto-send", action="store_true",
                   help="识别后只显示不发送 (防 STT 误识别让狗乱跑)")
    p.add_argument("--no-tts", action="store_true", help="关闭 TTS 播报")
    args = p.parse_args()

    # 1. 模型存在性检查
    if not os.path.isdir(args.model):
        print(f"[FAIL] Vosk 模型未找到: {args.model}")
        print("下载: https://alphacephei.com/vosk/models -> vosk-model-small-cn-0.22 (~50MB)")
        print(f"解压到 '{args.model}' 或设 VOSK_MODEL_PATH 环境变量")
        sys.exit(1)

    nx_url = f"http://{args.nx}:{args.port}/api/command"
    nx_ws_url = f"ws://{args.nx}:{args.ws_port}"

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
        sys.exit(1)
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
                            print("  (no-auto-send 模式, 不发送)")
                        else:
                            send_command(nx_url, text)
                    # 空文本 = 纯静音, 忽略
                else:
                    # partial: 实时显示正在说的话
                    partial = json.loads(rec.PartialResult()).get("partial", "")
                    if partial:
                        print(f"\r  ... {partial}", end="", flush=True)
    except KeyboardInterrupt:
        print("\n\n[bye] 退出.")
    except sd.PortAudioError as e:
        print(f"[FAIL] 麦克风初始化失败 (无设备/权限被拒?): {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
