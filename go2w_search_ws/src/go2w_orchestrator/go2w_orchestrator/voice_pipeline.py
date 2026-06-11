"""语音管线: whisper.cpp STT + VAD 端点检测。

使用 whisper.cpp 在 CPU 上做语音转文本，
复用 audio/capture.py 的 WebRTC VAD 端点检测。
"""

import io
import os
import wave
import time
import logging
import tempfile
import subprocess
import threading
from typing import Optional, Callable

import numpy as np

from audio.capture import AudioCapture
from ai.config import AUDIO_SAMPLE_RATE

logger = logging.getLogger(__name__)


class WhisperSTT:
    """whisper.cpp 语音转文本。"""

    def __init__(self, model_path: str = "models/ggml-base.bin",
                 language: str = "zh",
                 whisper_binary: str = "whisper-cpp"):
        self._model_path = model_path
        self._language = language
        self._whisper_binary = whisper_binary
        self._available = False
        self._check_available()

    def _check_available(self):
        """检查 whisper.cpp 是否可用。"""
        try:
            result = subprocess.run(
                [self._whisper_binary, "--help"],
                capture_output=True, timeout=5
            )
            if result.returncode == 0:
                self._available = True
                logger.info("whisper.cpp 可用")
            else:
                logger.warning("whisper.cpp 不可用")
        except (FileNotFoundError, subprocess.TimeoutExpired):
            logger.warning(f"whisper.cpp 未找到 ({self._whisper_binary})，尝试 Python whisper")
            self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def transcribe(self, audio: np.ndarray, sample_rate: int = AUDIO_SAMPLE_RATE) -> str:
        """将音频转为文本。

        Args:
            audio: numpy int16 数组
            sample_rate: 采样率
        Returns:
            识别出的文本
        """
        if self._available:
            return self._transcribe_whisper_cpp(audio, sample_rate)
        else:
            return self._transcribe_python_whisper(audio, sample_rate)

    def _transcribe_whisper_cpp(self, audio: np.ndarray, sample_rate: int) -> str:
        """使用 whisper.cpp 命令行工具转写。"""
        # 写入临时 WAV 文件
        audio_float = audio.astype(np.float32) / 32768.0
        if audio_float.ndim == 2:
            audio_float = audio_float.squeeze()

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            wav_path = f.name
            with wave.open(f, 'w') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(sample_rate)
                wf.writeframes(audio.tobytes())

        try:
            cmd = [
                self._whisper_binary,
                '-m', self._model_path,
                '-l', self._language,
                '-f', wav_path,
                '--no-timestamps',
                '-nt',
            ]
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10
            )
            text = result.stdout.strip()
            # whisper.cpp 输出可能有前缀 [BLANK_AUDIO] 等
            if text.startswith('['):
                lines = text.split('\n')
                text = lines[-1].strip() if lines else ""
            return text
        except subprocess.TimeoutExpired:
            logger.warning("whisper.cpp 超时")
            return ""
        except Exception as e:
            logger.error(f"whisper.cpp 错误: {e}")
            return ""
        finally:
            os.unlink(wav_path)

    @staticmethod
    def _transcribe_python_whisper(audio: np.ndarray, sample_rate: int) -> str:
        """降级: 使用 Python whisper 包。"""
        try:
            import whisper
            model = whisper.load_model("tiny")
            audio_float = audio.astype(np.float32) / 32768.0
            if audio_float.ndim == 2:
                audio_float = audio_float.squeeze()
            result = model.transcribe(audio_float, language="zh")
            return result.get("text", "")
        except ImportError:
            logger.warning("whisper 包也未安装")
            return ""
        except Exception as e:
            logger.error(f"Python whisper 错误: {e}")
            return ""


class VoicePipeline:
    """语音管线: VAD 检测 → whisper STT → 文本输出。

    在后台线程中持续监听麦克风，检测到语音后转写为文本，
    通过回调函数传递给编排器。
    """

    def __init__(self, on_text: Callable[[str], None],
                 stt: Optional[WhisperSTT] = None):
        """
        Args:
            on_text: 文本回调，接收识别出的文本
            stt: WhisperSTT 实例（默认自动创建）
        """
        self._on_text = on_text
        self._stt = stt or WhisperSTT()
        self._capture = AudioCapture()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._running

    def start(self):
        """启动语音监听。"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("语音管线已启动")

    def stop(self):
        """停止语音监听。"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("语音管线已停止")

    def _loop(self):
        """后台监听循环。"""
        while self._running:
            try:
                audio = self._capture.get_utterance(timeout=5.0)
                if audio is None or len(audio) < 1600:  # < 0.1s
                    continue

                text = self._stt.transcribe(audio)
                if text and len(text.strip()) > 1:
                    logger.info(f"语音识别: {text}")
                    self._on_text(text.strip())

            except Exception as e:
                logger.debug(f"语音管线错误: {e}")
                time.sleep(0.5)
