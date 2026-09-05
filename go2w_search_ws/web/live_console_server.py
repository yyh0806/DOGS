#!/usr/bin/env python3
"""live_console_server.py — 实时任务控制台 (M5 演示, 单一交互页)。

架构:
  浏览器 (EventSource) ──GET /events──▶ SSE 流 (实时步骤广播)
       │
       └──POST /api/run {task}──▶ 后台线程跑真实 go2w_brain
                                  (真 LLM / 真 lake_plan / 真守卫与检测链,
                                  运动指令强制干跑 —— 控制层不在本页范围)

每一个 SessionLog 轨迹条目经 BroadcastLog 即时推给所有客户端;
规划完成时附带完整几何 (环线/扫描点/水域) 供地图实时绘制;
scan_water 结果中的告警坐标随 tool_result 直达前端。

纯 stdlib (http.server + threading + json), 无第三方依赖。
"""
from __future__ import annotations

import json
import queue
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(WEB_DIR))

from go2w_brain.config import BrainConfig  # noqa: E402
from go2w_brain.run_brain import build_session  # noqa: E402
from go2w_brain.session_log import SessionLog  # noqa: E402

HTML_PATH = WEB_DIR / "live_console.html"
PORT = 8088
_PLAN_TOOLS = ("plan_lake_loop", "plan_campus_loop")


def _tile_content_type(data: bytes) -> str:
    """按魔数识别瓦片格式 (esri 卫星影像实为 JPEG, osm 为 PNG)。"""
    if data[:3] == b"\xff\xd8\xff":
        return "image/jpeg"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png"
    return "application/octet-stream"


class Hub:
    """SSE 广播中枢: 每客户端一个队列, 断连自动清理。"""

    def __init__(self):
        self._clients: list[queue.Queue] = []
        self._lock = threading.Lock()

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=500)
        with self._lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def publish(self, event: str, data) -> None:
        payload = f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False, default=str)}\n\n"
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                pass  # 慢客户端丢事件 (保最新性, 不阻塞大脑)


class ConsoleState:
    def __init__(self):
        self.running = False
        self.lock = threading.Lock()
        self.last_task = ""
        self.last_done = None

    def try_start(self) -> bool:
        with self.lock:
            if self.running:
                return False
            self.running = True
            return True

    def finish(self):
        with self.lock:
            self.running = False


STATE = ConsoleState()
HUB = Hub()
_CURRENT_MEMORY = []  # 供 /api/memory 读取 (build_session 内构造)


def _current_memory():
    if _CURRENT_MEMORY:
        return _CURRENT_MEMORY[0]
    # 重启后懒加载磁盘快照: 任务未跑也能在记忆面板看历史经验 (M7.3)
    try:
        from go2w_brain.memory import MemoryStore
        path = BrainConfig.from_env().memory_dir / "memory.jsonl"
        if path.exists():
            store = MemoryStore(path)
            _CURRENT_MEMORY.append(store)
            return store
    except Exception:  # noqa: BLE001
        pass
    return None


def run_mission(task: str):
    """后台线程: 跑真实大脑, 全程广播。"""
    config = BrainConfig.from_env()
    config.dry_run = True  # 本页只演示任务层; 运动指令一律不下发
    config.platform = "mock"
    log_path = WEB_DIR / "runs" / "brain" / (
        datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_live.jsonl")

    def broadcast(entry):
        HUB.publish("log", entry)
        # 规划完成后附送完整几何 (地图实时绘制)
        if (entry.get("kind") == "tool_result"
                and entry.get("name") in _PLAN_TOOLS
                and session is not None):
            plan = session.plan_snapshot()
            if plan.get("waypoints"):
                HUB.publish("plan", {"kind": plan.get("kind"),
                                     "waypoints": plan["waypoints"],
                                     "scan_points": plan.get("scan_points") or [],
                                     "water_polygon": plan.get("water_polygon"),
                                     "campus_polygon": plan.get("campus_polygon"),
                                     "target": plan.get("target")})

    session = None  # 闭包内延迟赋值 (broadcast 只在 run 后触发)
    try:
        log = SessionLog(log_path, on_append=broadcast)
        session = build_session(config, log)
        _CURRENT_MEMORY[:] = [getattr(session, "_memory", None)]
        HUB.publish("status", {"running": True, "task": task})
        result = session.run(task)
        STATE.last_done = result
        HUB.publish("done", {"answer": result["answer"],
                             "steps": result["steps"],
                             "llm_used": result["llm_used"],
                             "trace": result["trace"]})
    except Exception as exc:  # noqa: BLE001
        HUB.publish("error", {"reason": f"{type(exc).__name__}: {exc}"})
    finally:
        STATE.finish()
        HUB.publish("status", {"running": False, "task": task})


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            body = HTML_PATH.read_text(encoding="utf-8")
            self._respond(200, "text/html; charset=utf-8", body)
        elif self.path == "/events":
            self._sse()
        elif self.path.startswith("/tiles/"):
            # M7.2.1: 本地底图瓦片 (OSM 缓存优先, 零外网依赖)
            self._serve_tile(self.path)
        elif self.path.startswith("/static/"):
            self._serve_static(self.path)
        elif self.path.startswith("/runs/"):
            self._serve_static(self.path)
        elif self.path == "/api/memory":
            # M7.1: 记忆库只读快照 (地图叠加层用)
            try:
                memory = _current_memory()
                entries = memory.entries() if memory else []
                self._respond(200, "application/json",
                              json.dumps({"ok": True,
                                          "entries": entries,
                                          "summary": (memory.summary()
                                                      if memory else {})},
                                         ensure_ascii=False))
            except Exception as exc:  # noqa: BLE001
                self._respond(500, "application/json",
                              json.dumps({"ok": False,
                                          "reason": str(exc)}))
        else:
            self._respond(404, "text/plain; charset=utf-8", "not found")

    def _serve_static(self, path):
        rel = path.lstrip("/").replace("\\", "/")
        if ".." in rel.split("/"):
            self._respond(403, "text/plain; charset=utf-8", "forbidden")
            return
        file_path = (WEB_DIR / rel).resolve()
        if not str(file_path).startswith(str(WEB_DIR.resolve())):
            self._respond(403, "text/plain; charset=utf-8", "forbidden")
            return
        if not file_path.is_file():
            self._respond(404, "text/plain; charset=utf-8", "not found")
            return
        content_type = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".json": "application/json",
            ".png": "image/png",
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
        }.get(file_path.suffix, "application/octet-stream")
        self._respond(200, content_type, file_path.read_bytes())

    def _serve_tile(self, path):
        # /tiles/{provider}/{z}/{x}/{y}.png  (provider: esri|osm|carto|amap)
        parts = path.strip("/").split("/")
        if len(parts) != 5:
            self._respond(404, "text/plain; charset=utf-8", "not found")
            return
        provider, z_s, x_s, y_s = parts[1], parts[2], parts[3], parts[4]
        try:
            z, x, y = (int(z_s), int(x_s), int(y_s.split(".")[0]))
        except ValueError:
            self._respond(404, "text/plain; charset=utf-8", "not found")
            return
        sys.path.insert(0, str(WEB_DIR))
        from lake_plan import tiles as tile_lib
        cache = tile_lib.tile_cache_path(provider, z, x, y)
        if cache.exists() and cache.stat().st_size > 100:
            data = cache.read_bytes()
            self._respond(200, _tile_content_type(data), data)
            return
        try:
            data = tile_lib.fetch_tile(provider, z, x, y)
            self._respond(200, _tile_content_type(data), data)
        except tile_lib.TileError:
            self._respond(404, "text/plain; charset=utf-8", "tile missing")

    def do_POST(self):
        if self.path == "/api/run":
            length = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self._respond(400, "application/json",
                              json.dumps({"ok": False,
                                          "reason": "invalid_json"}))
                return
            task = str((payload or {}).get("task", "")).strip()
            if not task:
                self._respond(400, "application/json",
                              json.dumps({"ok": False,
                                          "reason": "empty_task"}))
                return
            if not STATE.try_start():
                self._respond(409, "application/json",
                              json.dumps({"ok": False,
                                          "reason": "already_running"}))
                return
            threading.Thread(target=run_mission, args=(task,),
                             daemon=True).start()
            self._respond(200, "application/json",
                          json.dumps({"ok": True, "task": task}))
        else:
            self._respond(404, "application/json",
                          json.dumps({"ok": False, "reason": "not_found"}))

    def _respond(self, status, content_type, body):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = HUB.subscribe()
        try:
            self.wfile.write(f"event: hello\ndata: {json.dumps({'running': STATE.running, 'task': STATE.last_task})}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    payload = q.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")  # 心跳保活
                    self.wfile.flush()
                    continue
                self.wfile.write(payload.encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            HUB.unsubscribe(q)

    def log_message(self, *args):
        pass


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"live mission console: http://127.0.0.1:{PORT}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
