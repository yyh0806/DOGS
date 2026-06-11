#!/usr/bin/env python3
"""
Go2W 搜索测试脚本 — 纯 Python，无需 ROS2
==============================================

直接通过网线连接笔记本和 Go2W，一条命令测试完整的搜索+检测流程。

已验证的 SDK API (unitree_sdk2py 1.0.1):
  - ChannelFactory().Init(id, interface)
  - SportClient: BalanceStand, StandDown, Sit, Move(vx,vy,vyaw), StopMove
  - VideoClient: GetImageSample() -> (code, list[int])  JPEG 1920x1080

用法:
  # 搜索 4x4 米区域
  python3 test_standalone.py --width 4 --height 4

  # 搜索并寻找人
  python3 test_standalone.py --width 6 --height 4 --target person

  # 只测摄像头+检测 (不连狗运动)
  python3 test_standalone.py --skip-robot

  # 手动控制测试 (键盘 WASD)
  python3 test_standalone.py --manual

  # 模拟模式 (不需要狗也不需要摄像头)
  python3 test_standalone.py --width 4 --height 4 --sim
"""

import argparse
import json
import math
import os
import sys
import time
import threading
import logging
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger("go2w_test")


# ============================================================================
# Go2W 运动控制 (封装 SDK)
# ============================================================================

def init_dds(interface: str) -> bool:
    """全局初始化 DDS ChannelFactory（单例，只调一次）。"""
    try:
        from unitree_sdk2py.core.channel import ChannelFactory
        factory = ChannelFactory()
        factory.Init(0, interface)
        logger.info(f"DDS 初始化完成 (网卡: {interface})")
        return True
    except Exception as e:
        logger.error(f"DDS 初始化失败: {e}")
        return False


class Go2WRobot:
    """Go2W 机器狗控制。SDK 不可用时自动降级为模拟模式。"""

    def __init__(self, interface: str = "enp65s0"):
        self._client = None
        self._connected = False
        self._sim_mode = False
        self._interface = interface
        self._sim_x = 0.0
        self._sim_y = 0.0
        self._sim_yaw = 0.0
        self._dds_initialized = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def sim_mode(self) -> bool:
        return self._sim_mode

    def connect(self) -> bool:
        try:
            from unitree_sdk2py.go2.sport.sport_client import SportClient

            logger.info(f"连接 Go2W (网卡: {self._interface})...")
            if not init_dds(self._interface):
                raise RuntimeError("DDS 初始化失败")
            self._dds_initialized = True

            self._client = SportClient()
            self._client.SetTimeout(10.0)
            self._client.Init()

            self._connected = True
            logger.info("Go2W 连接成功!")
            return True
        except ImportError:
            logger.warning("unitree_sdk2py 未安装，进入模拟模式")
            self._sim_mode = True
            self._connected = True
            return True
        except Exception as e:
            logger.error(f"连接失败: {e}，切换模拟模式")
            self._sim_mode = True
            self._connected = True
            return True

    def balance_stand(self):
        if self._sim_mode:
            logger.info("[模拟] 站立")
            return True
        try:
            self._client.BalanceStand()
            return True
        except Exception as e:
            logger.error(f"站立失败: {e}")
            return False

    def stand_down(self):
        """趴下。"""
        if self._sim_mode:
            logger.info("[模拟] 趴下")
            return True
        try:
            self._client.StandDown()
            return True
        except Exception as e:
            logger.error(f"趴下失败: {e}")
            return False

    def sit(self):
        """坐下。"""
        if self._sim_mode:
            logger.info("[模拟] 坐下")
            return True
        try:
            self._client.Sit()
            return True
        except Exception as e:
            logger.error(f"坐下失败: {e}")
            return False

    def stop(self):
        """停止运动。Go2W 轮式模式 StopMove() 无效，用 Move(0,0,0) 归零速度。"""
        if self._sim_mode:
            return True
        try:
            self._client.Move(0, 0, 0)
            return True
        except Exception as e:
            logger.error(f"停止失败: {e}")
            return False

    def move(self, vx: float, vy: float, vyaw: float):
        """设置速度指令 (body frame)。"""
        if self._sim_mode:
            dt = 0.1
            self._sim_x += (vx * math.cos(self._sim_yaw) - vy * math.sin(self._sim_yaw)) * dt
            self._sim_y += (vx * math.sin(self._sim_yaw) + vy * math.cos(self._sim_yaw)) * dt
            self._sim_yaw += vyaw * dt
            logger.debug(f"[模拟] move vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}")
            return True
        try:
            self._client.Move(vx, vy, vyaw)
            return True
        except Exception as e:
            logger.error(f"Move 失败: {e}")
            return False

    def get_position(self) -> Tuple[float, float, float]:
        if self._sim_mode:
            return self._sim_x, self._sim_y, self._sim_yaw
        return 0.0, 0.0, 0.0


# ============================================================================
# 摄像头 (SDK VideoClient)
# ============================================================================

class Camera:
    """Go2W 摄像头，通过 SDK VideoClient 获取 JPEG 帧。"""

    def __init__(self, interface: str = "enp65s0"):
        self._interface = interface
        self._video_client = None
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._connected = False
        self._thread = None

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> bool:
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient

            logger.info("初始化摄像头 (VideoClient)...")
            # ChannelFactory 已在 Go2WRobot.connect() 中初始化，不再重复调用

            self._video_client = VideoClient()
            self._video_client.SetTimeout(10.0)
            self._video_client.Init()

            # 测试一帧
            code, data = self._video_client.GetImageSample()
            if code == 0 and data and len(data) > 0:
                self._connected = True
                self._running = True
                self._thread = threading.Thread(target=self._capture_loop, daemon=True)
                self._thread.start()
                logger.info("摄像头已连接 (1920x1080 JPEG)")
                return True
            else:
                logger.warning(f"摄像头获取失败: code={code}")
                return False
        except Exception as e:
            logger.warning(f"摄像头初始化失败: {e}")
            return False

    def _capture_loop(self):
        while self._running:
            try:
                code, data = self._video_client.GetImageSample()
                if code == 0 and data and len(data) > 0:
                    raw = bytes(data)
                    img_array = np.frombuffer(raw, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._lock:
                            self._frame = frame
            except Exception:
                pass
            time.sleep(0.067)  # ~15fps

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)


# ============================================================================
# 目标检测
# ============================================================================

class Detector:
    def __init__(self, model: str = "yolov8n.pt", confidence: float = 0.45,
                 target_classes: Optional[List[str]] = None):
        self._confidence = confidence
        self._target_classes = set(target_classes) if target_classes else None
        self._model = None

        try:
            from ultralytics import YOLO
            self._model = YOLO(model)
            logger.info(f"YOLO 加载成功: {model}")
        except Exception as e:
            logger.error(f"YOLO 加载失败: {e}")

    @property
    def available(self) -> bool:
        return self._model is not None

    def detect(self, frame: np.ndarray) -> List[dict]:
        if self._model is None:
            return []
        results = self._model(frame, conf=self._confidence, iou=0.45,
                              verbose=False, imgsz=640)
        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for i in range(len(result.boxes)):
                cls_id = int(result.boxes.cls[i])
                conf = float(result.boxes.conf[i])
                xyxy = result.boxes.xyxy[i].cpu().numpy()
                name = self._model.names.get(cls_id, f"cls_{cls_id}")
                if self._target_classes and name not in self._target_classes:
                    continue
                detections.append({
                    "class": name,
                    "confidence": round(conf, 3),
                    "bbox": [round(float(v), 1) for v in xyxy],
                })
        return detections

    def annotate(self, frame: np.ndarray, detections: List[dict]) -> np.ndarray:
        out = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det['class']} {det['confidence']:.2f}"
            cv2.putText(out, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return out


# ============================================================================
# 路径规划
# ============================================================================

def plan_lawnmower(width: float, height: float, spacing: float = 1.5) -> List[dict]:
    waypoints = []
    num_rows = max(1, int(math.ceil(height / spacing)))
    for row in range(num_rows + 1):
        y = min(row * spacing, height)
        if row % 2 == 0:
            waypoints.append({"x": 0.0, "y": y})
            waypoints.append({"x": width, "y": y})
        else:
            waypoints.append({"x": width, "y": y})
            waypoints.append({"x": 0.0, "y": y})
    logger.info(f"路径规划: {len(waypoints)} 航点, 区域 {width}x{height}m, 间距 {spacing}m")
    return waypoints


# ============================================================================
# 导航 (速度控制方式)
# ============================================================================

def navigate_to_waypoint(robot: Go2WRobot, target: dict,
                         speed: float = 0.3, move_duration_scale: float = 2.5) -> bool:
    """用速度控制导航到航点。

    实机模式下:
      - 计算与目标的距离和方向
      - 发送 Move(vx, 0, vyaw) 持续一段时间
      - 距离越近速度越慢
    """
    x, y = target["x"], target["y"]
    rx, ry, _ = robot.get_position()
    dx = x - rx
    dy = y - ry
    distance = math.sqrt(dx * dx + dy * dy)

    if distance < 0.2:
        logger.info(f"  已在 ({x:.1f}, {y:.1f}) 附近")
        return True

    logger.info(f"  前往 ({x:.1f}, {y:.1f}), 距离 {distance:.2f}m")

    if robot.sim_mode:
        robot._sim_x = x
        robot._sim_y = y
        time.sleep(0.5)
        return True

    # 实机: 按距离估算运动时间
    move_time = distance / speed
    vx = speed
    vyaw = math.atan2(dy, dx) * 0.3  # 简单修正偏航

    robot.move(vx, 0.0, vyaw)
    time.sleep(move_time)
    robot.stop()
    time.sleep(0.3)

    return True


# ============================================================================
# 测试模式
# ============================================================================

def test_manual(robot: Go2WRobot, camera: Camera, detector: Detector):
    """手动键盘控制测试。"""
    logger.info("=" * 50)
    logger.info("手动控制模式")
    logger.info("  W/S: 前进/后退    A/D: 左转/右转")
    logger.info("  Q/E: 左移/右移    Space: 停止")
    logger.info("  P: 拍照检测       X: 退出")
    logger.info("=" * 50)

    speed = 0.3
    vyaw_speed = 0.3

    def show_camera():
        while True:
            frame = camera.get_frame()
            if frame is not None:
                h, w = frame.shape[:2]
                scale = 640 / max(w, 1)
                small = cv2.resize(frame, (int(w * scale), int(h * scale)))
                cv2.imshow("Go2W Camera (X退出)", small)
            if cv2.waitKey(1) & 0xFF == ord('x'):
                break

    if camera.connected:
        cam_thread = threading.Thread(target=show_camera, daemon=True)
        cam_thread.start()

    import tty, termios, select
    while True:
        if not sys.stdin.isatty():
            time.sleep(0.1)
            continue
        old = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            ch = ''
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

        if ch == 'w':
            robot.move(speed, 0, 0)
        elif ch == 's':
            robot.move(-speed, 0, 0)
        elif ch == 'a':
            robot.move(0, 0, vyaw_speed)
        elif ch == 'd':
            robot.move(0, 0, -vyaw_speed)
        elif ch == 'q':
            robot.move(0, speed, 0)
        elif ch == 'e':
            robot.move(0, -speed, 0)
        elif ch == ' ':
            robot.stop()
            logger.info("已停止")
        elif ch == 'p':
            frame = camera.get_frame()
            if frame is not None:
                dets = detector.detect(frame)
                logger.info(f"检测到 {len(dets)} 个目标")
                for d in dets:
                    logger.info(f"  {d['class']} ({d['confidence']:.2f})")
                annotated = detector.annotate(frame, dets)
                os.makedirs("output", exist_ok=True)
                path = f"output/capture_{datetime.now().strftime('%H%M%S')}.jpg"
                cv2.imwrite(path, annotated)
                logger.info(f"已保存: {path}")
        elif ch in ('x', '\x03'):
            break

    robot.stop()
    cv2.destroyAllWindows()


def test_search(robot: Go2WRobot, camera: Camera, detector: Detector,
                width: float, height: float, spacing: float,
                target_classes: Optional[List[str]]):
    """自动搜索测试。"""
    mode_str = "模拟" if robot.sim_mode else "实机"
    logger.info("=" * 60)
    logger.info(f"  自动搜索任务 [{mode_str}]")
    logger.info(f"  区域: {width}m x {height}m, 间距: {spacing}m")
    logger.info(f"  目标: {target_classes or '所有'}")
    logger.info("=" * 60)

    # 1. 站立
    logger.info("[1] 站立...")
    robot.balance_stand()
    time.sleep(2.0)

    # 2. 规划路径
    logger.info("[2] 规划路径...")
    waypoints = plan_lawnmower(width, height, spacing)
    for i, wp in enumerate(waypoints):
        logger.info(f"  [{i:2d}] ({wp['x']:.1f}, {wp['y']:.1f})")

    # 3. 逐航点导航 + 检测
    logger.info("[3] 开始搜索...")
    all_detections = []
    output_dir = Path("output/detections")
    output_dir.mkdir(parents=True, exist_ok=True)

    for i, wp in enumerate(waypoints):
        logger.info(f"\n--- 航点 {i+1}/{len(waypoints)} ---")

        navigate_to_waypoint(robot, wp, speed=0.3)

        # 拍照检测
        frame = camera.get_frame()
        if frame is not None and detector.available:
            dets = detector.detect(frame)
            if dets:
                rx, ry, ryaw = robot.get_position()
                for det in dets:
                    det["robot_x"] = round(rx, 2)
                    det["robot_y"] = round(ry, 2)
                    det["waypoint"] = i
                    all_detections.append(det)
                logger.info(f"  发现 {len(dets)} 个目标!")
                for d in dets:
                    logger.info(f"    {d['class']} ({d['confidence']:.2f})")
                annotated = detector.annotate(frame, dets)
                ts = datetime.now().strftime("%H%M%S")
                path = str(output_dir / f"det_{ts}_{i}.jpg")
                cv2.imwrite(path, annotated)
                logger.info(f"  保存: {path}")
            else:
                logger.info("  无目标")
        else:
            logger.info("  (摄像头/检测器不可用，跳过)")

        time.sleep(0.3)

    # 4. 停止
    robot.stop()

    # 5. 报告
    logger.info("\n" + "=" * 60)
    logger.info(f"  搜索完成! [{mode_str}]")
    logger.info(f"  访问航点: {len(waypoints)}")
    logger.info(f"  发现目标: {len(all_detections)}")
    for d in all_detections:
        logger.info(f"    {d['class']} @ ({d.get('robot_x','?')}, {d.get('robot_y','?')})")
    logger.info("=" * 60)

    report = {
        "timestamp": datetime.now().isoformat(),
        "area": {"width": width, "height": height, "spacing": spacing},
        "waypoints": len(waypoints),
        "detections": all_detections,
        "mode": mode_str,
    }
    os.makedirs("output", exist_ok=True)
    report_path = f"output/search_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    logger.info(f"报告: {report_path}")

    # 6. 趴下
    logger.info("趴下...")
    robot.stand_down()
    time.sleep(2.0)


# ============================================================================
# 主入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Go2W 搜索测试 (纯 Python, 无需 ROS2)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--width", type=float, default=4.0, help="搜索区域宽度 (米)")
    parser.add_argument("--height", type=float, default=4.0, help="搜索区域高度 (米)")
    parser.add_argument("--spacing", type=float, default=1.5, help="搜索行间距 (米)")
    parser.add_argument("--target", type=str, default=None, help="目标类别 (person, car...)")
    parser.add_argument("--interface", type=str, default="enp65s0", help="有线网卡名")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="YOLO 模型")
    parser.add_argument("--confidence", type=float, default=0.45, help="检测置信度")
    parser.add_argument("--skip-robot", action="store_true", help="跳过机器人，只测摄像头+检测")
    parser.add_argument("--skip-camera", action="store_true", help="跳过摄像头")
    parser.add_argument("--manual", action="store_true", help="手动键盘控制")
    parser.add_argument("--sim", action="store_true", help="强制模拟模式")

    args = parser.parse_args()
    target_classes = [args.target] if args.target else None

    # ---- 机器人 ----
    robot = Go2WRobot(interface=args.interface)
    if not args.skip_robot:
        if args.sim:
            robot._sim_mode = True
            robot._connected = True
        else:
            robot.connect()
    else:
        robot._sim_mode = True
        robot._connected = True

    # ---- 摄像头 ----
    camera = Camera(interface=args.interface)
    if not args.skip_camera and not robot.sim_mode:
        camera.start()

    # ---- 检测器 ----
    detector = Detector(args.model, args.confidence, target_classes)

    # ---- 运行 ----
    try:
        if args.manual:
            test_manual(robot, camera, detector)
        else:
            test_search(robot, camera, detector,
                        args.width, args.height, args.spacing,
                        target_classes)
    except KeyboardInterrupt:
        logger.info("\n用户中断，停止...")
        robot.stop()
    finally:
        camera.stop()
        if robot.connected and not robot.sim_mode:
            robot.stop()


if __name__ == "__main__":
    main()
