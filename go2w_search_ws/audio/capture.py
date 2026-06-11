"""
音频采集 + VAD (语音活动检测)
==============================
用 sounddevice 录音，webrtcvad 做端点检测。
平台无关：笔记本 USB 麦克风 / NX USB 麦克风都一样用。
"""

import logging
import queue
import threading
import time
import numpy as np

from ai.config import AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, VAD_SILENCE_TIMEOUT

logger = logging.getLogger("go2w.audio")


class AudioCapture:
    """麦克风录音 + VAD 端点检测。

    使用方式：
        cap = AudioCapture()
        cap.start()
        while running:
            audio = cap.get_utterance()  # 阻塞等待一句完整的话
            if audio is not None:
                text = voice_model.transcribe(audio)
    """

    def __init__(self, device=None, sample_rate=AUDIO_SAMPLE_RATE,
                 channels=AUDIO_CHANNELS, silence_timeout=VAD_SILENCE_TIMEOUT):
        self._device = device
        self._sample_rate = sample_rate
        self._channels = channels
        self._silence_timeout = silence_timeout
        self._stream = None
        self._running = False
        # VAD 状态
        self._vad = None
        self._vad_sensitivity = 2  # 0-3
        # 音频缓冲
        self._audio_buffer = []
        self._utterance_queue = queue.Queue(maxsize=10)
        # 录音状态
        self._is_speaking = False
        self._silence_start = None
        self._frame_duration_ms = 30  # webrtcvad 要求 10/20/30ms

    def start(self):
        """启动麦克风录音。"""
        try:
            import webrtcvad
            self._vad = webrtcvad.Vad(self._vad_sensitivity)
            logger.info(f"VAD 初始化成功，灵敏度: {self._vad_sensitivity}")
        except ImportError:
            logger.warning("webrtcvad 未安装，使用能量检测代替 VAD")
            self._vad = None

        try:
            import sounddevice as sd
            # webrtcvad 要求 16kHz, 16bit, mono
            frame_size = int(self._sample_rate * self._frame_duration_ms / 1000)
            self._frame_size = frame_size

            self._stream = sd.InputStream(
                device=self._device,
                samplerate=self._sample_rate,
                channels=self._channels,
                dtype='int16',
                blocksize=frame_size,
                callback=self._audio_callback,
            )
            self._stream.start()
            self._running = True
            logger.info(f"麦克风录音启动 (设备={self._device}, {self._sample_rate}Hz, "
                        f"帧长={self._frame_duration_ms}ms)")

            # 启动 VAD 处理线程
            self._vad_thread = threading.Thread(target=self._vad_worker, daemon=True)
            self._vad_thread.start()

        except Exception as e:
            logger.error(f"麦克风启动失败: {e}")
            logger.info("提示: pip install sounddevice webrtcvad")

    def stop(self):
        """停止录音。"""
        self._running = False
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        logger.info("麦克风录音停止")

    def _audio_callback(self, indata, frames, time_info, status):
        """sounddevice 回调，每次送入一帧音频。"""
        if status:
            return
        # 转为 bytes 用于 VAD
        frame_bytes = indata.tobytes()
        self._audio_buffer.append((frame_bytes, indata.copy()))

    def _is_speech(self, frame_bytes: bytes) -> bool:
        """检测一帧是否包含语音。"""
        if self._vad is not None:
            try:
                return self._vad.is_speech(frame_bytes, self._sample_rate)
            except Exception:
                pass
        # 降级方案：能量检测
        samples = np.frombuffer(frame_bytes, dtype=np.int16).astype(np.float32)
        energy = np.sqrt(np.mean(samples ** 2))
        return energy > 300  # 经验阈值

    def _vad_worker(self):
        """VAD 处理线程：检测语音端点，拼出完整语句。"""
        utterance_frames = []
        silence_count = 0
        max_silence_frames = int(self._silence_timeout * 1000 / self._frame_duration_ms)

        while self._running:
            if not self._audio_buffer:
                time.sleep(0.01)
                continue

            frame_bytes, frame_data = self._audio_buffer.pop(0)
            is_speech = self._is_speech(frame_bytes)

            if is_speech:
                utterance_frames.append(frame_data)
                self._is_speaking = True
                silence_count = 0
            elif self._is_speaking:
                # 说话中的静音
                utterance_frames.append(frame_data)
                silence_count += 1
                if silence_count >= max_silence_frames:
                    # 一句话结束，拼接输出
                    if len(utterance_frames) > 3:  # 至少 ~90ms
                        full_audio = np.concatenate(utterance_frames, axis=0)
                        try:
                            self._utterance_queue.put_nowait(full_audio)
                        except queue.Full:
                            # 丢掉最旧的
                            try:
                                self._utterance_queue.get_nowait()
                            except queue.Empty:
                                pass
                            self._utterance_queue.put_nowait(full_audio)
                    utterance_frames = []
                    self._is_speaking = False
                    silence_count = 0

    def get_utterance(self, timeout=10.0):
        """阻塞等待一句完整的语音，返回 numpy 数组 (int16)。

        Args:
            timeout: 最长等待时间（秒）
        Returns:
            numpy array (int16) 或 None（超时）
        """
        try:
            return self._utterance_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def is_speaking(self) -> bool:
        return self._is_speaking

    @staticmethod
    def list_devices():
        """列出所有可用的音频设备。"""
        try:
            import sounddevice as sd
            print(sd.query_devices())
        except ImportError:
            print("sounddevice 未安装: pip install sounddevice")
