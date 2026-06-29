#!/usr/bin/env python3
"""round-3 解耦验证: 视频流与 YOLO 解耦 (不依赖 ultralytics/torch, 纯逻辑测试)。

验证点 (对应 round-3 需求):
  1. detector=None 时 _run_detector(frame) 返回 [] (检测器抽象 + 纯视频流路径)
  2. detector=None 时 _video_yolo_loop 单步 (取帧→缓存) 仍工作:
     get_frame_jpeg() 返回 (b64_jpeg, int=0) — detections 是整数 0 (C1.4 契约保持)
  3. get_detections_world 在 detector=None 时返回 [] (slam.data.detections 数组契约)
  4. GO2W_AI_NO_DETECT=1: _init_detector 直接 detector=None, 不尝试加载 YOLO
     (即使本机装了 ultralytics 也跳过; 通过子进程 + env 验证模块级开关)
  5. 无 ultralytics (本测试机 ultralytics 缺失, 真实模拟 NX): _init_detector
     detector=None, 不抛异常, 视频流路径可用
  6. _run_detector 是检测器抽象入口: 替换它即可接入 locateanything (签名契约)
  7. type=frame 契约: get_frame_jpeg 返回 (str_b64, int) — int 不是 list (红线)

不启动真视频线程 (避免 SDK/网络依赖), 用 MockFrameGenerator 直接喂数据进缓存,
模拟 _video_yolo_loop 在 detector=None 时的写入行为。
"""
import os
import sys
import subprocess
import base64

# 让 web/ 可 import
WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")
sys.path.insert(0, WEB)

import nx_ai_node as ai

PASS = 0
FAIL = 0


def ok(name, detail=""):
    global PASS
    print(f"  PASS: {name} {detail}")
    PASS += 1


def no(name, detail=""):
    global FAIL
    print(f"  FAIL: {name} {detail}")
    FAIL += 1


print("===== 1. _run_detector 抽象: detector=None → [] (round-3 检测器抽象) =====")
eng = ai.NxAiEngine()
# detector 默认 None (未 _init_detector); _run_detector 应返回 []
eng._detector = None
import numpy as np
dummy_frame = np.full((720, 1280, 3), 80, dtype=np.uint8)
r = eng._run_detector(dummy_frame)
if isinstance(r, list) and len(r) == 0:
    ok("_run_detector(detector=None) 返回空 list", "")
else:
    no("_run_detector(detector=None) 应返回 []", str(r))

# frame=None 也应返回 [] (防御)
r2 = eng._run_detector(None)
if isinstance(r2, list) and len(r2) == 0:
    ok("_run_detector(None) 返回空 list (防御)", "")
else:
    no("_run_detector(None) 应返回 []", str(r2))


print()
print("===== 2. 纯视频流路径: detector=None 时 get_frame_jpeg → (b64, int=0) =====")
# 模拟 _video_yolo_loop 在 detector=None 时的写入: 取 mock 帧 → resize → 缓存
gen = ai.MockFrameGenerator()
frame = gen.next_frame()
# 复刻 _video_yolo_loop 的 detector=None 分支: dets=_run_detector(frame)=[] + resize + 缓存
dets = eng._run_detector(frame)  # [] (detector=None)
# 模拟 resize 到 720p + 写缓存 (与 _video_yolo_loop 一致)
try:
    import cv2
    if frame.shape[1] != 1280 or frame.shape[0] != 720:
        frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
except Exception:
    pass
with eng._lock:
    eng._latest_frame = frame
    eng._latest_dets = dets
    eng._detect_frame_w = frame.shape[1]
    eng._frame_count += 1

r = eng.get_frame_jpeg()
if r is not None and isinstance(r, tuple) and len(r) == 2:
    b64, det_count = r
    b64_valid = isinstance(b64, str) and len(b64) > 0
    # 红线 C1.4: detections 必须是 int, 不是 list
    count_is_int = isinstance(det_count, int)
    count_is_zero = (det_count == 0)
    # 验证 b64 真能解码成 jpeg (前端能渲染)
    try:
        raw = base64.b64decode(b64)
        # JPEG SOI 头 ff d8
        jpeg_ok = raw[:2] == b"\xff\xd8"
    except Exception:
        jpeg_ok = False
    if b64_valid and count_is_int and count_is_zero and jpeg_ok:
        ok("detector=None 时 get_frame_jpeg 返回 (b64_jpeg, int=0)",
           f"b64_len={len(b64)} jpeg_ok={jpeg_ok}")
    else:
        no("get_frame_jpeg 契约错",
           f"b64_valid={b64_valid} count_is_int={count_is_int} "
           f"count_is_zero={count_is_zero} jpeg_ok={jpeg_ok}")
else:
    no("get_frame_jpeg 返回格式错", str(r))


print()
print("===== 3. detector=None 时 get_detections_world → [] (slam 数组契约) =====")
dw = eng.get_detections_world(robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
if isinstance(dw, list) and len(dw) == 0:
    ok("detector=None 时 get_detections_world 返回 [] (无检测不打点)", "")
else:
    no("get_detections_world 应返回空 list", str(dw))


print()
print("===== 4. GO2W_AI_NO_DETECT=1: 模块级禁检测开关 (子进程验证) =====")
# 子进程 import nx_ai_node, 验证 _DETECT_ALLOWED/_DETECT_DISABLED_BY_ENV/_ULTRALYTICS_AVAILABLE
# 通过 env GO2W_AI_NO_DETECT=1 验证开关生效 (不受本进程已 import 影响)
# 注意: 子进程禁用 logging (避免 CJK 日志在 Windows GBK 下解码崩 _readerthread),
# 只打印 ASCII KEY=VALUE 行, 用二进制 + utf-8 解码读取。
code = (
    "import os,sys,logging; logging.disable(logging.CRITICAL); "
    "sys.path.insert(0, %r); "
    "import nx_ai_node as ai; "
    "print('DISABLED_BY_ENV=' + str(ai._DETECT_DISABLED_BY_ENV)); "
    "print('ULTRALYTICS_AVAILABLE=' + str(ai._ULTRALYTICS_AVAILABLE)); "
    "print('DETECT_ALLOWED=' + str(ai._DETECT_ALLOWED)); "
    "eng = ai.NxAiEngine(); eng._init_detector(); "
    "print('DETECTOR_AFTER_INIT=' + str(eng._detector)); "
    "print('DETECTOR_INITED=' + str(eng._detector_inited)); "
    "print('NO_EXCEPTION=1')"
) % WEB

env = dict(os.environ)
env["GO2W_AI_NO_DETECT"] = "1"
env["PYTHONIOENCODING"] = "utf-8"
out = subprocess.run([sys.executable, "-c", code], env=env,
                     capture_output=True, timeout=30)
txt = out.stdout.decode("utf-8", errors="replace").strip()
lines = dict(l.split("=", 1) for l in txt.splitlines() if "=" in l)
disabled = lines.get("DISABLED_BY_ENV", "").strip() == "True"
detect_allowed = lines.get("DETECT_ALLOWED", "").strip() == "False"  # 禁了就 False
detector_none = lines.get("DETECTOR_AFTER_INIT", "").strip() == "None"
detector_inited = lines.get("DETECTOR_INITED", "").strip() == "True"
if disabled and detect_allowed and detector_none and detector_inited:
    ok("GO2W_AI_NO_DETECT=1: _DETECT_ALLOWED=False, _init_detector→detector=None (跳过 YOLO)", "")
else:
    no("GO2W_AI_NO_DETECT=1 开关未生效",
       f"out={txt!r} err={out.stderr.decode('utf-8', errors='replace').strip()[:200]!r}")


print()
print("===== 5. 无 ultralytics (本机真实模拟 NX): _init_detector → None, 不抛 =====")
# 本测试机 ultralytics 缺失 (与 NX 一致), 不设 GO2W_AI_NO_DETECT, 验证自动禁检测路径
code2 = (
    "import os,sys,logging; logging.disable(logging.CRITICAL); "
    "sys.path.insert(0, %r); "
    "os.environ.pop('GO2W_AI_NO_DETECT', None); "
    "import nx_ai_node as ai; "
    "print('ULTRALYTICS_AVAILABLE=' + str(ai._ULTRALYTICS_AVAILABLE)); "
    "print('DETECT_ALLOWED=' + str(ai._DETECT_ALLOWED)); "
    "eng = ai.NxAiEngine(); eng._init_detector(); "
    "print('DETECTOR_AFTER_INIT=' + str(eng._detector)); "
    "print('NO_EXCEPTION=1')"
) % WEB
env2 = dict(os.environ)
env2.pop("GO2W_AI_NO_DETECT", None)
env2["PYTHONIOENCODING"] = "utf-8"
out2 = subprocess.run([sys.executable, "-c", code2], env=env2,
                      capture_output=True, timeout=30)
txt2 = out2.stdout.decode("utf-8", errors="replace").strip()
lines2 = dict(l.split("=", 1) for l in txt2.splitlines() if "=" in l)
ultra_avail = lines2.get("ULTRALYTICS_AVAILABLE", "").strip() == "False"  # 本机缺失
detect_allowed2 = lines2.get("DETECT_ALLOWED", "").strip() == "False"
detector_none2 = lines2.get("DETECTOR_AFTER_INIT", "").strip() == "None"
no_exc = lines2.get("NO_EXCEPTION", "").strip() == "1"
if ultra_avail and detect_allowed2 and detector_none2 and no_exc:
    ok("无 ultralytics: _init_detector→None 不抛 (NX 纯视频流路径可用)", "")
else:
    no("无 ultralytics 路径异常",
       f"out={txt2!r} err={out2.stderr.decode('utf-8', errors='replace').strip()[:200]!r}")


print()
print("===== 6. _run_detector 抽象可替换 (locateanything 接口契约) =====")
# 验证: 子类覆盖 _run_detector 即可接入新检测器, 视频流路径不动
class LocateAnythingStub(ai.NxAiEngine):
    """模拟 locateanything 接入: 覆盖 _run_detector, 返回固定检测。"""
    def _run_detector(self, frame):
        # 同签名 (frame → list[dict]), 视频流路径 (get_frame_jpeg 等) 不动
        if frame is None:
            return []
        return [{"class": "red_cup", "confidence": 0.88, "bbox": [100, 100, 200, 200]}]

stub = LocateAnythingStub()
stub._detector = None  # 哪怕底层 detector=None, 抽象层也能独立返回检测
dets_stub = stub._run_detector(dummy_frame)
if (isinstance(dets_stub, list) and len(dets_stub) == 1
        and dets_stub[0]["class"] == "red_cup"):
    # 验证 get_detections_world 能消费抽象层返回的检测 (视频流下游不动)
    with stub._lock:
        stub._latest_dets = dets_stub
        stub._detect_frame_w = 1280
    dw_stub = stub.get_detections_world(0.0, 0.0, 0.0)
    if isinstance(dw_stub, list) and len(dw_stub) == 1 and "x" in dw_stub[0]:
        ok("_run_detector 可被子类覆盖 (locateanything 同签名替换, 下游不变)",
           f"det={dets_stub[0]['class']} world_x={dw_stub[0]['x']}")
    else:
        no("抽象层返回的检测未被 get_detections_world 正确消费", str(dw_stub))
else:
    no("_run_detector 子类覆盖未生效", str(dets_stub))


print()
print("===== 7. type=frame 契约红线: detections 是 int 不是 list =====")
# 再独立断言: get_frame_jpeg 的第 2 个返回值类型必须是 int (C1.4)
r = eng.get_frame_jpeg()
if r is not None:
    _, count = r
    if type(count) is int:  # 严格 int, 不是 list
        ok("type=frame.detections 是 int (C1.4 红线保持)", f"type={type(count).__name__}")
    else:
        no("type=frame.detections 必须是 int", f"got {type(count).__name__}: {count!r}")
else:
    no("get_frame_jpeg 返回 None (缓存未就绪?)", "")


print()
print(f"===== 结果: {PASS} PASS, {FAIL} FAIL =====")
sys.exit(0 if FAIL == 0 else 1)
