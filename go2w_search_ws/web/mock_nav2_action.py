#!/usr/bin/env python3
"""Go2W 阶段E — Mock Nav2 action server (spec-stage-e §7.1, 决策 5)。

== 职责 ==
Fake `/navigate_to_pose` action server, 让阶段E 编排状态机在没有真 Nav2/SLAM/狗时
端到端可验证 (spec 决策 5)。独立 rclpy 节点进程, 与 nx_web_server 进程隔离。

== 行为 (env 可配) ==
  - GO2W_MOCK_NAV_DELAY (默认 0.5s): 收到 goal 后模拟导航耗时, 期间发 feedback
  - GO2W_MOCK_NAV_FAIL (默认 ""): 空格分隔 "x,y" 列表, 这些坐标 goal 会 abort
  - GO2W_MOCK_NAV_REJECT (默认 ""): 这些坐标 goal 会被 reject (测试 rejected 路径)
默认: 收到 goal → 0.5s 后 status=SUCCEEDED (立即到达, 验证状态机用)

== 启动 (NX) ==
  source /opt/ros/humble/setup.bash
  python3 web/mock_nav2_action.py
  # 或配 env:
  GO2W_MOCK_NAV_DELAY=2.0 GO2W_MOCK_NAV_FAIL="2.5,1.8" python3 web/mock_nav2_action.py

== spec 反模式 (§12) ==
  - 不模拟真实物理 (简单 delay + feedback 递减即可, 反模式 14)
  - feedback 字段: distance_remaining (递减 5m→0m) + estimated_time_remaining
"""

import logging
import os
import time

logger = logging.getLogger("go2w.mock_nav2")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")


def _parse_xy_set(s: str) -> set:
    """"1.0,2.0 3.0,4.0" → {(1.0,2.0), (3.0,4.0)}。
    容错: 解析失败的 token 跳过 (不崩)。
    """
    out = set()
    if not s:
        return out
    # 真解析: "x,y" 形式配对, 空格分隔多条
    parts = s.split()
    for p in parts:
        try:
            xy = p.split(",")
            if len(xy) == 2:
                out.add((round(float(xy[0]), 3), round(float(xy[1]), 3)))
        except (ValueError, IndexError):
            continue
    return out


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionServer, GoalResponse, CancelResponse
        from nav2_msgs.action import NavigateToPose
    except Exception as e:
        logger.error(f"无法导入 rclpy/nav2_msgs: {e}")
        logger.error("mock_nav2_action 必须在 NX (ROS2 Humble) 上运行, 当前环境无 ROS2")
        raise

    class MockNav2ActionServer(Node):
        """Fake NavigateToPose action server (spec 决策 5)。"""

        def __init__(self):
            super().__init__('mock_nav2_action_server')
            self._delay = float(os.environ.get('GO2W_MOCK_NAV_DELAY', '0.5'))
            self._fail_set = _parse_xy_set(os.environ.get('GO2W_MOCK_NAV_FAIL', ''))
            self._reject_set = _parse_xy_set(os.environ.get('GO2W_MOCK_NAV_REJECT', ''))
            # feedback 发送次数 (按 delay 等分)
            self._fb_steps = 5
            self._action = ActionServer(
                self, NavigateToPose, '/navigate_to_pose',
                execute_callback=self._execute,
                goal_callback=self._goal_cb,
                cancel_callback=self._cancel_cb,
            )
            self.get_logger().info(
                f"Mock Nav2 action server 就绪: /navigate_to_pose "
                f"(delay={self._delay}s, fail={self._fail_set}, reject={self._reject_set})")

        def _goal_xy(self, goal_request):
            """从 goal_request 取 (x, y) 用于 reject/fail 匹配。"""
            try:
                pos = goal_request.pose.pose.position
                return (round(float(pos.x), 3), round(float(pos.y), 3))
            except Exception:
                return None

        def _goal_cb(self, goal_request):
            """reject 列表内的坐标拒绝, 其余接受。
            spec §7.1: reject 是 _goal_cb 返回 GoalResponse.REJECT (goal 根本没接受)。
            """
            xy = self._goal_xy(goal_request)
            if xy is not None and xy in self._reject_set:
                self.get_logger().info(f"MOCK REJECT goal @ {xy} (在 reject 列表)")
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _cancel_cb(self, goal_handle):
            """spec §7.1: cancel 总是 ACCEPT, _execute 内部检查 is_cancel_requested。"""
            self.get_logger().info(f"MOCK 收到 cancel 请求: ACCEPT")
            return CancelResponse.ACCEPT

        def _make_feedback(self, distance_remaining, eta_sec):
            """构造 NavigateToPose.Feedback (distance_remaining + estimated_time_remaining)。
            rclpy duration 用 Duration(msg={'sec': N, 'nanosec': M})。
            """
            fb = NavigateToPose.Feedback()
            try:
                fb.distance_remaining = float(distance_remaining)
                # estimated_time_remaining 是 builtin_interfaces/Duration
                fb.estimated_time_remaining.sec = int(eta_sec)
                fb.estimated_time_remaining.nanosec = int((eta_sec - int(eta_sec)) * 1e9)
            except Exception as e:
                self.get_logger().debug(f"feedback 构造异常: {e}")
            return fb

        def _execute(self, goal_handle):
            """模拟导航: 发 feedback (distance_remaining 递减) → delay → succeed/abort。
            spec §7.1 实现要点:
              - reject 列表: _goal_cb 已拦, 这里不会进
              - fail 列表: 发几次 feedback → goal_handle.abort()
              - 正常: 按 delay 分 fb_steps 次发 feedback (distance 5m→0m 递减) → succeed
              - cancel: 检查 is_cancel_requested → abort
            """
            xy = self._goal_xy(goal_handle)
            self.get_logger().info(f"MOCK execute 开始, goal @ {xy}")

            # 检查 cancel (cancel 可能在执行中到达)
            if goal_handle.is_cancel_requested:
                self.get_logger().info("MOCK execute: cancel 已请求, abort")
                goal_handle.abort()
                return NavigateToPose.Result()

            # fail 列表 → abort (发一次 feedback 后 abort)
            if xy is not None and xy in self._fail_set:
                self.get_logger().info(f"MOCK FAIL goal @ {xy} (在 fail 列表) → abort")
                # 发一次 feedback 让进度推送有数据
                goal_handle.publish_feedback(self._make_feedback(0.5, 1.0))
                time.sleep(min(0.1, self._delay))
                goal_handle.abort()
                return NavigateToPose.Result()

            # 正常: 按 delay 分 fb_steps 次发 feedback (distance 5m → 0m 递减)
            step_delay = self._delay / max(1, self._fb_steps)
            start_distance = 5.0
            for i in range(self._fb_steps):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("MOCK execute 中途 cancel, abort")
                    goal_handle.abort()
                    return NavigateToPose.Result()
                frac = (i + 1) / self._fb_steps
                dist = start_distance * (1.0 - frac)   # 5 → 0
                eta = max(0.0, self._delay * (1.0 - frac))
                goal_handle.publish_feedback(self._make_feedback(dist, eta))
                time.sleep(step_delay)

            # 到达 → succeed
            goal_handle.succeed()
            self.get_logger().info(f"MOCK SUCCEED goal @ {xy}")
            result = NavigateToPose.Result()
            return result

    rclpy.init()
    node = MockNav2ActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
