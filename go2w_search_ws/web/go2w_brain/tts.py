"""tts — 语音播报后端 (M7.2 接入点)。

speak 工具的说话能力到此落地: 后端协议 TtsBackend.speak(text) -> dict;
- ConsoleTtsBackend: 演练/无音频环境 —— 打印+留痕, spoken 语义真实;
- EdgeTtsBackend: NX/PC 真发声 (edge-tts 子进程, 无第三方 Python 依赖,
  只需系统装有 edge-tts CLI; 缺失时 fail-soft 返回 unavailable)。
选择: 环境变量 GO2W_TTS=console|edge (默认 console)。
"""
from __future__ import annotations

import shutil
import subprocess
from typing import Any


class ConsoleTtsBackend:
    name = "console"

    def speak(self, text: str) -> dict[str, Any]:
        print(f"[TTS] {text}")
        return {"ok": True, "spoken": True, "backend": self.name}


class EdgeTtsBackend:
    name = "edge"

    def speak(self, text: str) -> dict[str, Any]:
        exe = shutil.which("edge-tts")
        if not exe:
            return {"ok": False, "reason": "edge_tts_not_installed",
                    "hint": "pip install edge-tts (或 apt 安装后重试)"}
        try:
            result = subprocess.run(
                [exe, "--voice", "zh-CN-XiaoxiaoNeural", "--text", text,
                 "--write-media", "-"],
                capture_output=True, timeout=30.0)
        except subprocess.TimeoutExpired:
            return {"ok": False, "reason": "tts_timeout"}
        if result.returncode != 0:
            return {"ok": False, "reason": "tts_failed",
                    "detail": result.stderr.decode(errors="ignore")[:160]}
        # 写临时 mp3 并请求系统播放 (无音频设备的干跑机器上静默成功)
        import tempfile
        from pathlib import Path
        tmp = Path(tempfile.gettempdir()) / "go2w_tts.mp3"
        tmp.write_bytes(result.stdout)
        _try_play(tmp)
        return {"ok": True, "spoken": True, "backend": self.name,
                "media": str(tmp)}


def _try_play(path) -> None:
    player = shutil.which("ffplay") or shutil.which("aplay")
    if not player:
        return
    try:
        subprocess.Popen([player, "-nodisp", "-autoexit", str(path)],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    except OSError:
        pass


def build_backend(choice: str = ""):
    choice = (choice or "console").strip().lower()
    if choice == "edge":
        return EdgeTtsBackend()
    return ConsoleTtsBackend()
