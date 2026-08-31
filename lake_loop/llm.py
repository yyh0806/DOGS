"""DeepSeek 大模型客户端（OpenAI 兼容协议）。

- chat(): 文本推理，返回 (内容, 思维链 reasoning_content)
- vision(): 图像输入（瓦片拼接图直接作为大模型输入）
- chat_json(): 要求 JSON 输出并稳健解析
- key 解析顺序: 环境变量 DEEPSEEK_API_KEY -> ~/.dsh/.credentials.yaml
"""
import base64
import io
import json
import os
import re
from pathlib import Path

import requests

from config import LLM_BASE_URL, TEXT_MODEL, VISION_MODEL, CREDENTIALS_FILE

FENCE = "\x60" * 3  # markdown 代码围栏


def resolve_api_key():
    k = os.environ.get("DEEPSEEK_API_KEY")
    if k:
        return k.strip()
    try:
        m = re.search(r"DEEPSEEK_API_KEY:\s*(\S+)",
                      CREDENTIALS_FILE.read_text(encoding="utf-8"))
        if m:
            return m.group(1).strip()
    except OSError:
        pass
    return None


class LLM:
    def __init__(self, api_key=None, base_url=LLM_BASE_URL, text_model=TEXT_MODEL,
                 vision_model=VISION_MODEL, timeout=240):
        self.key = api_key or resolve_api_key()
        self.base_url = base_url.rstrip("/")
        self.text_model = text_model
        self.vision_model = vision_model
        self.timeout = timeout
        self.available = bool(self.key)

    # ---------- 基础 ----------
    def _post(self, payload: dict) -> dict:
        r = requests.post(
            f"{self.base_url}/chat/completions",
            headers={"Authorization": f"Bearer {self.key}",
                     "Content-Type": "application/json"},
            data=json.dumps(payload), timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def chat(self, system: str, user: str, model=None, temperature=0.2,
             max_tokens=3000, with_reasoning=True):
        payload = {
            "model": model or self.text_model,
            "messages": [{"role": "system", "content": system},
                         {"role": "user", "content": user}],
            "temperature": temperature,
            "max_tokens": max_tokens,
        }
        out = self._post(payload)
        msg = out["choices"][0]["message"]
        content = msg.get("content") or ""
        reasoning = (msg.get("reasoning_content") or "") if with_reasoning else ""
        return content, reasoning

    def vision(self, image, prompt: str, model=None, temperature=0.2,
               max_tokens=4000):
        """image: PIL.Image 或路径。返回 (内容, 思维链)。"""
        from PIL import Image
        if isinstance(image, (str, Path)):
            image = Image.open(image)
        img = image.convert("RGB")
        long_side = max(img.size)
        if long_side > 1400:
            scale = 1400 / long_side
            img = img.resize((int(img.width * scale), int(img.height * scale)))
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        b64 = base64.b64encode(buf.getvalue()).decode()
        payload = {
            "model": model or self.vision_model,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": "data:image/png;base64," + b64}},
                ],
            }],
            "temperature": temperature,
            "max_tokens": max_tokens,
        }
        out = self._post(payload)
        msg = out["choices"][0]["message"]
        return msg.get("content") or "", msg.get("reasoning_content") or ""

    # ---------- 结构化 ----------
    @staticmethod
    def extract_json(text: str):
        """从模型回复里稳健地抽出第一个 JSON 对象/数组。"""
        if not text:
            return None
        lines = [ln for ln in text.strip().splitlines()
                 if not ln.strip().startswith(FENCE)]
        t = "\n".join(lines).strip()
        for opener, closer in (("{", "}"), ("[", "]")):
            start = t.find(opener)
            if start < 0:
                continue
            depth = 0
            in_str = False
            esc = False
            for i in range(start, len(t)):
                ch = t[i]
                if in_str:
                    if esc:
                        esc = False
                    elif ch == "\\":
                        esc = True
                    elif ch == '"':
                        in_str = False
                    continue
                if ch == '"':
                    in_str = True
                elif ch == opener:
                    depth += 1
                elif ch == closer:
                    depth -= 1
                    if depth == 0:
                        try:
                            return json.loads(t[start:i + 1])
                        except json.JSONDecodeError:
                            break
        m = re.search(r"[\[{].*[\]}]", t, re.S)
        if m:
            try:
                return json.loads(m.group(0))
            except json.JSONDecodeError:
                return None
        return None

    def chat_json(self, system, user, **kw):
        content, reasoning = self.chat(system, user, **kw)
        obj = self.extract_json(content)
        return obj, content, reasoning
