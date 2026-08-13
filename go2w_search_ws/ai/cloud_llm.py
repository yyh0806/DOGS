"""DeepSeek 云端 LLM 客户端 (OpenAI 兼容 /chat/completions)。

设计原则 (延续项目哲学):
  - fail-closed: 任何网络/超时/JSON/HTTP 错误 → 返回 None, 不影响确定性解析
  - 思考过程透出: deepseek-reasoner 的 reasoning_content 与最终 answer 分离返回,
    供前端展示, 不参与执行
  - 配置走环境变量 (key 不进仓库, 部署时写入 /etc/go2w/llm.env):
      GO2W_LLM_API_URL   (默认 https://api.deepseek.com/v1/chat/completions)
      GO2W_LLM_API_KEY
      GO2W_LLM_MODEL     (默认 deepseek-chat; deepseek-reasoner 可选)
      GO2W_LLM_TIMEOUT   (默认 15s, 控制回路外单次调用有硬超时)
"""
from __future__ import annotations

import json
import logging
import os
import urllib.error
import urllib.request
from typing import Optional, Tuple

logger = logging.getLogger("go2w.cloud_llm")

_DEFAULT_URL = "https://api.deepseek.com/v1/chat/completions"
_DEFAULT_MODEL = "deepseek-chat"
_DEFAULT_TIMEOUT = 15.0


class CloudLLM:
    """DeepSeek 聊天客户端。"""

    def __init__(self, api_url: Optional[str] = None,
                 api_key: Optional[str] = None,
                 model: Optional[str] = None,
                 timeout: Optional[float] = None):
        self.api_url = (api_url or os.environ.get("GO2W_LLM_API_URL")
                        or _DEFAULT_URL)
        self.api_key = api_key or os.environ.get("GO2W_LLM_API_KEY", "")
        self.model = model or os.environ.get("GO2W_LLM_MODEL") or _DEFAULT_MODEL
        try:
            self.timeout = float(timeout
                                 or os.environ.get("GO2W_LLM_TIMEOUT")
                                 or _DEFAULT_TIMEOUT)
        except (TypeError, ValueError):
            self.timeout = _DEFAULT_TIMEOUT

    @property
    def configured(self) -> bool:
        return bool(self.api_key.strip())

    def chat(self, system_prompt: str, user_text: str) -> Tuple[Optional[str], Optional[str]]:
        """单轮对话 → (answer, reasoning)。任何错误返回 (None, None)。"""
        if not self.configured:
            logger.warning("[cloud_llm] 未配置 API key, 跳过云端调用")
            return None, None
        body = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_text},
            ],
            "temperature": 0.2,   # 低温度: 计划输出尽量确定
            "max_tokens": 600,
        }
        req = urllib.request.Request(
            self.api_url,
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            logger.warning(f"[cloud_llm] HTTP {e.code}: {e.read()[:200]!r}")
            return None, None
        except (urllib.error.URLError, TimeoutError, OSError,
                json.JSONDecodeError) as e:
            logger.warning(f"[cloud_llm] 请求失败: {type(e).__name__}")
            return None, None
        try:
            choice = data["choices"][0]["message"]
            answer = (choice.get("content") or "").strip() or None
            # deepseek-reasoner 专有字段: 思考链 (可选)
            reasoning = (choice.get("reasoning_content") or "").strip() or None
            return answer, reasoning
        except (KeyError, IndexError, TypeError) as e:
            logger.warning(f"[cloud_llm] 响应结构异常: {e}")
            return None, None
