import importlib.util
import http.server
import json
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "local_llm_nlu.py"


def load_module():
    spec = importlib.util.spec_from_file_location("local_llm_nlu_for_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ollama_response_returns_command_and_disables_streaming():
    module = load_module()
    seen = {}

    def transport(url, body, timeout):
        seen.update(url=url, body=body, timeout=timeout)
        return {"message": {"content": '{"command":"搜索房间标注所有椅子"}'}}

    normalizer = module.LocalLLMCommandNormalizer(
        url="http://127.0.0.1:11434/api/chat", model="qwen2.5:3b", timeout=2,
        transport=transport,
    )

    assert normalizer.normalize("帮我看看这里的椅子") == "搜索房间标注所有椅子"
    assert seen["url"].endswith("/api/chat")
    assert seen["body"]["model"] == "qwen2.5:3b"
    assert seen["body"]["stream"] is False
    assert seen["body"]["format"] == "json"
    assert seen["body"]["options"]["temperature"] == 0
    prompt = seen["body"]["messages"][0]["content"]
    for canonical_form in ("前进[<距离>米]", "后退[<距离>米]", "左转[<角度>度]", "右转[<角度>度]"):
        assert canonical_form in prompt
    assert "(0,20]" in prompt
    assert "(0,360]" in prompt
    assert "仅限当前房间" in prompt
    for semantic_example in (
        "搜索全屋", "探索房间", "扭头", "转身", "回头",
        "搜索房间标注所有人", "左转90度", "左转180度",
    ):
        assert semantic_example in prompt
    assert "不得原样输出方括号" in prompt
    for forbidden in ("系统命令", "shell", "坐标", "多个命令", "解释性文字", "工具调用"):
        assert forbidden in prompt


def test_openai_compatible_shape_and_body_work():
    module = load_module()
    seen = {}

    def transport(url, body, timeout):
        seen.update(url=url, body=body, timeout=timeout)
        return {"choices": [{"message": {"content": '{"command":"前进一米"}'}}]}

    normalizer = module.LocalLLMCommandNormalizer(
        url="http://127.0.0.1:8080/v1/chat/completions", model="local", timeout=3,
        transport=transport,
    )

    assert normalizer.normalize("往前走一点") == "前进一米"
    assert seen["body"]["temperature"] == 0
    assert seen["body"]["response_format"] == {"type": "json_object"}
    assert "stream" not in seen["body"]


def test_openai_extracts_first_choice_and_fenced_object_is_accepted():
    module = load_module()
    normalizer = module.LocalLLMCommandNormalizer(
        url="http://127.0.0.1:8080/v1/chat/completions", model="local",
        transport=lambda *_args: {
            "choices": [
                {"message": {"content": '```json\n{"command":"前进一米"}\n```'}},
                {"message": {"content": '{"command":"右转45度"}'}},
            ]
        },
    )

    assert normalizer.normalize("向前走") == "前进一米"


def test_normalizer_fails_closed_for_invalid_content_and_transport_errors():
    module = load_module()
    invalid_contents = [
        '{"commands":["前进","右转"]}',
        '["前进"]',
        '{"command":"前进", "extra": true}',
        '说明：{"command":"前进"}',
        '```json\n{"command":"前进"}\n```\n```json\n{"command":"右转"}\n```',
        '{"command":"' + "前" * 1001 + '"}',
        '{"command":{"text":"前进"}}',
        '{"command":"前进一米","command":"后退一米"}',
        '```json {"command":"前进一米"}```',
    ]

    for content in invalid_contents:
        normalizer = module.LocalLLMCommandNormalizer(
            url="http://127.0.0.1:11434/api/chat", model="local", timeout=1,
            transport=lambda *_args, value=content: {"message": {"content": value}},
        )
        assert normalizer.normalize("测试") is None

    for error in (TimeoutError(), OSError("offline"), ValueError("bad response")):
        normalizer = module.LocalLLMCommandNormalizer(
            url="http://127.0.0.1:11434/api/chat", model="local", timeout=1,
            transport=lambda *_args, value=error: (_ for _ in ()).throw(value),
        )
        assert normalizer.normalize("测试") is None


def test_normalizer_rejects_oversized_transcript_and_unsupported_response_shape():
    module = load_module()
    calls = []
    normalizer = module.LocalLLMCommandNormalizer(
        url="http://127.0.0.1:9999/unknown", model="local", timeout=1,
        transport=lambda *args: calls.append(args) or {"message": {"content": '{"command":null}'}},
    )

    assert normalizer.normalize("测" * 1001) is None
    assert calls == []
    assert normalizer.normalize("测试") is None


def test_endpoint_policy_accepts_only_loopback_hosts_and_valid_ports():
    module = load_module()
    accepted = [
        "http://localhost:11434/api/chat",
        "http://127.0.0.1:11434/api/chat",
        "http://127.42.3.9/v1/chat/completions",
        "http://[::1]:8080/v1/chat/completions",
    ]
    rejected = [
        "http://192.168.1.2:11434/api/chat",
        "https://api.example.com/v1/chat/completions",
        "http://user@localhost:11434/api/chat",
        "http://user:pass@127.0.0.1:11434/api/chat",
        "http://localhost:99999/api/chat",
        "http://localhost:0/api/chat",
        "http://localhost:/api/chat",
        "http:///api/chat",
    ]

    for url in accepted:
        assert module.LocalLLMCommandNormalizer(url, "local").supported_endpoint is True
    for url in rejected:
        assert module.LocalLLMCommandNormalizer(url, "local").supported_endpoint is False


def test_default_transport_does_not_follow_http_redirects():
    module = load_module()
    paths = []

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            paths.append(self.path)
            if self.path == "/api/chat":
                self.send_response(302)
                self.send_header("Location", "/redirected")
                self.end_headers()
                return
            body = b'{"message":{"content":"{\\"command\\":\\"forward\\"}"}}'
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            paths.append(self.path)
            body = b'{"message":{"content":"{\\"command\\":\\"forward\\"}"}}'
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        normalizer = module.LocalLLMCommandNormalizer(
            f"http://127.0.0.1:{server.server_port}/api/chat", "local")
        assert normalizer.normalize("测试") is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert paths == ["/api/chat"]


def test_ollama_rejects_tool_or_function_responses_even_with_valid_content():
    module = load_module()
    base_message = {"content": '{"command":"前进一米"}'}
    for extra in (
        {"tool_calls": [{"function": {"name": "move"}}]},
        {"function_call": {"name": "move"}},
    ):
        message = dict(base_message, **extra)
        normalizer = module.LocalLLMCommandNormalizer(
            "http://127.0.0.1:11434/api/chat", "local",
            transport=lambda *_args, value=message: {"message": value},
        )
        assert normalizer.normalize("测试") is None


def test_openai_rejects_tool_or_function_responses_even_with_valid_content():
    module = load_module()
    cases = [
        {"message": {"content": '{"command":"前进一米"}', "tool_calls": [{}]}},
        {"message": {"content": '{"command":"前进一米"}', "function_call": {"name": "move"}}},
        {"message": {"content": '{"command":"前进一米"}'}, "finish_reason": "tool_calls"},
        {"message": {"content": '{"command":"前进一米"}'}, "finish_reason": "function_call"},
    ]
    for choice in cases:
        normalizer = module.LocalLLMCommandNormalizer(
            "http://127.0.0.1:8080/v1/chat/completions", "local",
            transport=lambda *_args, value=choice: {"choices": [value]},
        )
        assert normalizer.normalize("测试") is None
