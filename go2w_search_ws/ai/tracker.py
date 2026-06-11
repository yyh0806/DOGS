"""
目标跟踪器
==========
整合 VLM 定位 + 运动控制的跟踪状态机。
支持"跟着某人"类型的持续跟踪任务。
"""

import logging
import threading
import time
from typing import Optional

import numpy as np

logger = logging.getLogger("go2w.tracker")


class TargetTracker:
    """目标跟踪状态机。

    状态流程：
      IDLE → (收到 follow 指令) → SEARCHING → (VLM 找到目标) → TRACKING
      TRACKING → (目标丢失) → RECOVERING → (重新找到) → TRACKING
      TRACKING → (收到 stop 指令) → IDLE
      RECOVERING → (超时) → IDLE

    用法：
        tracker = TargetTracker(vlm_engine, robot)
        tracker.start_follow("穿蓝色衣服的人")
        # 在视频循环中：
        tracker.update(frame)
    """

    # 跟踪状态
    IDLE = "idle"
    SEARCHING = "searching"
    TRACKING = "tracking"
    RECOVERING = "recovering"

    def __init__(self, vlm_engine, robot_sdk, detector=None):
        """
        Args:
            vlm_engine: VLMEngine 实例
            robot_sdk: RobotSDK 实例
            detector: Detector 实例（可选，用于辅助检测）
        """
        self._vlm = vlm_engine
        self._robot = robot_sdk
        self._detector = detector
        self._lock = threading.Lock()
        # 跟踪状态
        self._state = self.IDLE
        self._target_desc = ""
        self._thread = None
        self._running = False
        # 跟踪参数
        self._search_interval = 2.0   # SEARCHING 时 VLM 推理间隔（秒）
        self._track_interval = 0.5    # TRACKING 时 VLM 推理间隔
        self._recover_timeout = 5.0   # RECOVERING 超时
        self._lost_frames = 0         # 连续丢失帧数
        self._max_lost = 3            # 连续丢失多少帧进入 RECOVERING
        # 跟踪结果
        self._last_bbox = None
        self._last_update = 0.0

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def target(self) -> str:
        with self._lock:
            return self._target_desc

    @property
    def last_bbox(self):
        with self._lock:
            return self._last_bbox

    def start_follow(self, target_description: str):
        """开始跟踪目标。"""
        with self._lock:
            if self._state != self.IDLE:
                logger.warning(f"跟踪器状态 {self._state}，无法启动新跟踪")
                return False
            self._target_desc = target_description
            self._state = self.SEARCHING
            self._running = True
            self._lost_frames = 0
            self._last_bbox = None

        self._thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self._thread.start()
        logger.info(f"开始跟踪目标: {target_description}")
        return True

    def stop(self):
        """停止跟踪。"""
        self._running = False
        self._robot.stop()
        with self._lock:
            self._state = self.IDLE
            self._target_desc = ""
            self._last_bbox = None
        logger.info("跟踪停止")

    def _tracking_loop(self):
        """跟踪主循环，在独立线程中运行。"""
        recover_start = None

        while self._running:
            try:
                # 获取最新视频帧
                frame = self._robot.get_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue

                state = self.state
                target = self.target

                if state == self.SEARCHING:
                    # 慢速旋转搜索目标
                    self._robot.move(0.0, 0.0, 0.3)  # 原地旋转
                    # 定期用 VLM 检测
                    if time.time() - self._last_update >= self._search_interval:
                        self._search_vlm(frame, target)

                elif state == self.TRACKING:
                    # 跟踪目标
                    if time.time() - self._last_update >= self._track_interval:
                        result = self._vlm.track_target(
                            frame, target,
                            img_w=frame.shape[1], img_h=frame.shape[0]
                        )
                        if result["found"]:
                            self._lost_frames = 0
                            with self._lock:
                                self._last_bbox = result.get("bbox")
                            # 发送跟踪控制
                            vx = result.get("vx", 0)
                            vyaw = result.get("vyaw", 0)
                            self._robot.move(vx, 0.0, vyaw)
                            self._last_update = time.time()
                        else:
                            self._lost_frames += 1
                            if self._lost_frames >= self._max_lost:
                                with self._lock:
                                    self._state = self.RECOVERING
                                recover_start = time.time()
                                logger.info("目标丢失，进入恢复模式")

                elif state == self.RECOVERING:
                    # 恢复策略：转回寻找
                    self._robot.move(0.0, 0.0, 0.4)  # 旋转搜索
                    if time.time() - self._last_update >= self._search_interval:
                        self._search_vlm(frame, target)
                    # 超时检查
                    if recover_start and time.time() - recover_start > self._recover_timeout:
                        logger.info("恢复超时，停止跟踪")
                        self.stop()
                        return

                time.sleep(0.05)

            except Exception as e:
                logger.error(f"跟踪循环异常: {e}")
                time.sleep(0.5)

    def _search_vlm(self, frame: np.ndarray, target: str):
        """用 VLM 搜索目标，找到后切换到 TRACKING。"""
        result = self._vlm.locate(frame, target)
        if result["found"]:
            with self._lock:
                self._state = self.TRACKING
                self._last_bbox = result.get("bbox")
            self._lost_frames = 0
            self._last_update = time.time()
            self._robot.stop()
            logger.info(f"找到目标: {result.get('description', target)}")
        else:
            self._last_update = time.time()
            logger.debug(f"未找到目标: {target}")
