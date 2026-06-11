"""Go2W 任务编排器节点。

中央调度节点，管理任务队列、调用 VLM 做任务分解、
通过 Nav2 Action 发送导航目标。

节点名: go2w_orchestrator
话题:
  发布: /go2w/task_queue  (std_msgs/String JSON)
  订阅: /go2w/voice_text  (std_msgs/String)
  订阅: /go2w/detections  (std_msgs/String JSON)
服务:
  /go2w/start_search  (go2w_interfaces/StartSearch)
  /go2w/stop_search   (go2w_interfaces/StopSearch)
  /go2w/send_command  (std_srvs/Trigger 或自定义)
Action Client:
  /navigate_to_pose   (nav2_msgs/NavigateToPose)
"""

import json
import math
import time
import logging
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

from .task_queue import TaskQueue, TaskItem
from .planner import plan_lawnmower, plan_spiral, compute_path_length
from .vlm_integration import VLMIntegration
from .voice_pipeline import VoicePipeline, WhisperSTT

logger = logging.getLogger(__name__)


class OrchestratorNode(Node):
    """Go2W 任务编排器。"""

    def __init__(self):
        super().__init__('go2w_orchestrator')

        # 参数
        self.declare_parameter('vlm_model', 'Qwen/Qwen2.5-VL-3B-Instruct')
        self.declare_parameter('whisper_model', 'models/ggml-base.bin')
        self.declare_parameter('whisper_language', 'zh')
        self.declare_parameter('default_search_width', 10.0)
        self.declare_parameter('default_search_height', 10.0)
        self.declare_parameter('default_search_spacing', 2.5)
        self.declare_parameter('nav_goal_timeout', 120.0)
        self.declare_parameter('auto_approach_detections', False)

        # 核心组件
        self._queue = TaskQueue()
        self._vlm = VLMIntegration()
        self._voice: Optional[VoicePipeline] = None

        # 导航状态
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_active = False
        self._start_position = None  # 记录起始位置

        # 检测缓冲
        self._detections = []
        self._detection_lock = threading.Lock()

        # ROS2 接口
        self._queue_pub = self.create_publisher(String, '/go2w/task_queue', 10)
        self._response_pub = self.create_publisher(String, '/go2w/vlm_response', 10)

        self.create_subscription(String, '/go2w/voice_text', self._voice_text_cb, 10)
        self.create_subscription(String, '/go2w/detections', self._detection_cb, 10)

        # 主循环 10Hz
        self._tick_timer = self.create_timer(0.1, self._tick)

        # 状态发布 2Hz
        self._status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info("编排器就绪，等待指令...")

    # ---- 语音/文本指令处理 ----

    def _voice_text_cb(self, msg: String):
        """处理语音转写文本。"""
        self._process_text_command(msg.data)

    def _process_text_command(self, text: str):
        """通过 VLM 解析文本指令并加入任务队列。"""
        self.get_logger().info(f"收到指令: {text}")

        # 在后台线程中调用 VLM（推理可能耗时数秒）
        thread = threading.Thread(
            target=self._vlm_process_command, args=(text,), daemon=True
        )
        thread.start()

    def _vlm_process_command(self, text: str):
        """VLM 处理指令（后台线程）。"""
        try:
            if not self._vlm.loaded:
                self.get_logger().info("加载 VLM 模型...")
                if not self._vlm.load():
                    self.get_logger().error("VLM 加载失败，使用降级解析")
                    result = self._vlm._fallback_parse(text)
                else:
                    result = self._vlm.process_command(text)
            else:
                result = self._vlm.process_command(text)

            # 发布 VLM 回复
            response_msg = String()
            response_msg.data = json.dumps(result, ensure_ascii=False)
            self._response_pub.publish(response_msg)

            if result.get("response"):
                self.get_logger().info(f"VLM 回复: {result['response']}")

            # 将任务加入队列
            tasks = result.get("tasks", [])
            if tasks:
                task_items = []
                for i, t in enumerate(tasks):
                    task_type = t.get("type", "navigate")
                    params = t.get("params", {})
                    priority = t.get("priority", max(1, 8 - i))
                    task_items.append(TaskItem(task_type, params, priority))
                self._queue.push_list(task_items)
                self.get_logger().info(f"已加入 {len(task_items)} 个任务")
            else:
                # 立即处理特殊指令
                text_lower = text.lower()
                if "停" in text:
                    self._handle_stop()

        except Exception as e:
            self.get_logger().error(f"VLM 处理失败: {e}")

    # ---- 检测处理 ----

    def _detection_cb(self, msg: String):
        """处理检测结果。"""
        try:
            det = json.loads(msg.data)
            with self._detection_lock:
                self._detections.append(det)
                if len(self._detections) > 100:
                    self._detections = self._detections[-50:]
        except Exception:
            pass

    # ---- Nav2 Action 处理 ----

    def _send_nav_goal(self, x: float, y: float, yaw: float):
        """发送导航目标到 Nav2。"""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server 不可用")
            self._queue.fail_active("Nav2 不可用")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        half = yaw / 2.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(half)
        goal_msg.pose.pose.orientation.w = math.cos(half)

        self.get_logger().info(f"发送导航目标: ({x:.2f}, {y:.2f}, {yaw:.2f})")
        self._nav_active = True

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        """Nav2 goal 接受/拒绝回调。"""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav2 拒绝了导航目标")
            self._nav_active = False
            self._queue.fail_active("目标被拒绝")
            return

        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        """Nav2 导航完成回调。"""
        self._nav_active = False
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("导航目标到达")
            self._queue.complete_active({"status": "success"})
        else:
            self.get_logger().warn(f"导航失败 (status={result.status})")
            self._queue.fail_active(f"Nav2 status={result.status}")

    def _nav_feedback_cb(self, feedback_msg):
        """Nav2 导航反馈。"""
        pass  # 可以用来发布进度

    # ---- 主循环 ----

    def _tick(self):
        """主循环 10Hz。"""
        if self._nav_active:
            return  # 正在导航，等待完成

        # 取下一个任务
        task = self._queue.pop_next()
        if task is None:
            return

        self.get_logger().info(
            f"执行任务: {task.task_type} (优先级 {task.priority})"
        )

        if task.task_type == "navigate":
            params = task.params
            self._send_nav_goal(
                params.get("x", 0.0),
                params.get("y", 0.0),
                params.get("yaw", 0.0),
            )

        elif task.task_type == "search_area":
            self._execute_search(task)

        elif task.task_type == "follow":
            self.get_logger().info(f"开始跟踪: {task.params.get('target', '')}")
            # follow 模式由单独的跟踪节点处理
            self._queue.complete_active({"status": "delegated"})

        elif task.task_type == "return_home":
            if self._start_position:
                self._send_nav_goal(*self._start_position)
            else:
                self._send_nav_goal(0.0, 0.0, 0.0)

        elif task.task_type == "stop":
            self._handle_stop()

        elif task.task_type == "observe":
            self.get_logger().info(f"观察目标: {task.params.get('target', '')}")
            # TODO: 调用 VLM 观察并记录
            self._queue.complete_active({"status": "observed"})

        elif task.task_type == "wait":
            wait_time = task.params.get("duration", 1.0)
            self.get_logger().info(f"等待 {wait_time}s")
            time.sleep(wait_time)
            self._queue.complete_active()

        else:
            self.get_logger().warn(f"未知任务类型: {task.task_type}")
            self._queue.fail_active(f"未知类型: {task.task_type}")

    def _execute_search(self, task: TaskItem):
        """执行搜索任务：生成航点并加入队列。"""
        params = task.params
        width = params.get("width", self.get_parameter('default_search_width')
                           .get_parameter_value().double_value)
        height = params.get("height", self.get_parameter('default_search_height')
                            .get_parameter_value().double_value)
        spacing = params.get("spacing", self.get_parameter('default_search_spacing')
                             .get_parameter_value().double_value)
        pattern = params.get("pattern", "lawnmower")

        # 记录起始位置
        self._start_position = (0.0, 0.0, 0.0)  # 从当前位置开始

        # 生成航点
        if pattern == "spiral":
            waypoints = plan_spiral(width, height, spacing)
        else:
            waypoints = plan_lawnmower(width, height, spacing)

        total_dist = compute_path_length(waypoints)
        self.get_logger().info(
            f"搜索路径: {len(waypoints)} 个航点, 总距离 {total_dist:.1f}m"
        )

        # 完成搜索编排任务
        self._queue.complete_active({"waypoints": len(waypoints)})

        # 将航点转为 navigate 任务
        nav_tasks = []
        for i, wp in enumerate(waypoints):
            nav_tasks.append(TaskItem(
                "navigate",
                {"x": wp["x"], "y": wp["y"], "yaw": wp["yaw"]},
                priority=max(1, 7 - i // 3),
                parent_id=task.task_id,
            ))

        # 最后加一个返回起点的任务
        if self._start_position:
            nav_tasks.append(TaskItem(
                "return_home", {"priority": 1}, priority=1, parent_id=task.task_id
            ))

        self._queue.push_list(nav_tasks)

    def _handle_stop(self):
        """停止当前所有任务。"""
        # 取消 Nav2 导航
        if self._nav_active and self._nav_goal_handle:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_active = False

        self._queue.cancel_all()
        self.get_logger().info("所有任务已停止")

    # ---- 状态发布 ----

    def _publish_status(self):
        """发布任务队列状态。"""
        state = self._queue.get_state()
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False)
        self._queue_pub.publish(msg)

    # ---- 语音管线控制 ----

    def start_voice(self):
        """启动语音监听。"""
        if self._voice and self._voice.running:
            return
        whisper_model = self.get_parameter('whisper_model').get_parameter_value().string_value
        whisper_lang = self.get_parameter('whisper_language').get_parameter_value().string_value
        stt = WhisperSTT(model_path=whisper_model, language=whisper_lang)
        self._voice = VoicePipeline(
            on_text=lambda text: self._process_text_command(text),
            stt=stt,
        )
        self._voice.start()

    def stop_voice(self):
        """停止语音监听。"""
        if self._voice:
            self._voice.stop()

    def destroy_node(self):
        self.stop_voice()
        self._vlm.unload()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
