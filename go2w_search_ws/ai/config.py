"""
AI 模块全局配置
===============
所有硬件相关、平台相关的配置集中在这里。
迁移到 Orin NX 时只需修改本文件。
"""

import torch
import os

# ============================================================================
# 设备配置
# ============================================================================

# CUDA 设备：Orin NX 上只有一个 GPU (cuda:0)，笔记本上也一样
CUDA_DEVICE = os.environ.get("GO2W_CUDA_DEVICE", "cuda:0")

# 是否有可用的 CUDA
CUDA_AVAILABLE = torch.cuda.is_available()

# 推理设备
DEVICE = torch.device(CUDA_DEVICE) if CUDA_AVAILABLE else torch.device("cpu")

# GPU 显存 (MB)，用于自动选择模型精度
GPU_MEM_MB = torch.cuda.get_device_properties(0).total_memory // (1024 * 1024) if CUDA_AVAILABLE else 0

# ============================================================================
# 模型配置
# ============================================================================

# --- Whisper.cpp (语音转文本，CPU 运行，不占 VRAM) ---
WHISPER_MODEL_PATH = os.environ.get("GO2W_WHISPER_MODEL", "models/ggml-base.bin")
WHISPER_LANGUAGE = os.environ.get("GO2W_WHISPER_LANG", "zh")
WHISPER_BINARY = os.environ.get("GO2W_WHISPER_BIN", "whisper-cpp")

# --- Qwen2.5-VL (视觉定位) ---
VLM_MODEL_NAME = os.environ.get(
    "GO2W_VLM_MODEL",
    "/home/nhy/ZCodeProject/models/Qwen/Qwen2___5-VL-3B-Instruct"
)
VLM_QUANT = os.environ.get("GO2W_VLM_QUANT", "fp16")

# --- YOLO (目标检测) ---
YOLO_MODEL_PATH = os.environ.get("GO2W_YOLO_MODEL", "yolov8n.pt")
YOLO_CONFIDENCE = float(os.environ.get("GO2W_YOLO_CONF", "0.45"))

# ============================================================================
# 音频配置
# ============================================================================

# 音频采样率
AUDIO_SAMPLE_RATE = int(os.environ.get("GO2W_AUDIO_RATE", "16000"))
# 音频通道数
AUDIO_CHANNELS = int(os.environ.get("GO2W_AUDIO_CHANNELS", "1"))
# 每次录音时长（秒），用于分块处理
AUDIO_CHUNK_DURATION = float(os.environ.get("GO2W_AUDIO_CHUNK", "3.0"))
# VAD 灵敏度 (0-3, 0=最不灵敏, 3=最灵敏)
VAD_SENSITIVITY = int(os.environ.get("GO2W_VAD_SENSITIVITY", "2"))
# 静音超时（秒），超过后认为一句话结束
VAD_SILENCE_TIMEOUT = float(os.environ.get("GO2W_VAD_SILENCE", "1.5"))

# ============================================================================
# 辅助函数
# ============================================================================

def should_use_fp16() -> bool:
    """根据显存自动决定是否使用 fp16。"""
    if not CUDA_AVAILABLE:
        return False
    # < 10GB 显存用 fp16，>= 10GB 可以考虑 fp32
    return GPU_MEM_MB < 10000


def memory_summary() -> str:
    """返回当前 GPU 内存使用摘要。"""
    if not CUDA_AVAILABLE:
        return "CUDA 不可用，将使用 CPU"
    total = GPU_MEM_MB
    allocated = torch.cuda.memory_allocated(0) // (1024 * 1024)
    reserved = torch.cuda.memory_reserved(0) // (1024 * 1024)
    return f"GPU: {total}MB total, {allocated}MB used, {reserved}MB reserved"
