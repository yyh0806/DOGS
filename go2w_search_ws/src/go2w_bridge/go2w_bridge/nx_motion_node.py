"""NX-local Go2W motion node with one feedback-driven state owner.

ROS callbacks only enqueue events.  One actor thread owns the state machine
and sends serialized effects to the stable local Sport lease gateway.
Startup never blindly changes posture: feedback decides whether the robot is
already parked, needs one stationary parking transition, or must remain in a
zero-velocity fault hold.
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    from .build_info import release_id
    from .motion_controller import MotionController
    from .motion_machine import Go2WMotionMachine
    from .motion_protocol import (
        MotionIntentEnvelope,
        MotionProtocolError,
        motion_status_dict,
    )
    from .motion_safety import DriveExecutionWatchdog, ScanFreshnessWatchdog
    from .motion_types import ActualMotionState, SessionState, StopProfile
    from .sport_gateway_client import SportGatewayClient
except ImportError:  # Direct-file compatibility deployment on the NX.
    from build_info import release_id
    from motion_controller import MotionController
    from motion_machine import Go2WMotionMachine
    from motion_protocol import (
        MotionIntentEnvelope,
        MotionProtocolError,
        motion_status_dict,
    )
    from motion_safety import DriveExecutionWatchdog, ScanFreshnessWatchdog
    from motion_types import ActualMotionState, SessionState, StopProfile
    from sport_gateway_client import SportGatewayClient


RELEASE_ID = release_id()

def _parameter_value(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def _stop_profile_from_environment():
    raw = os.environ.get(
        "GO2W_STOP_PROFILE", StopProfile.MOVE_ZERO_ONLY.value).strip()
    try:
        return StopProfile(raw)
    except ValueError:
        return StopProfile.MOVE_ZERO_ONLY


class NxMotionNode(Node):
    """Thin ROS adapter around the single-thread MotionController."""

    _POSE_TO_INTENT = {
        "stand": "park",
        "balance": "start_manual",
        "estop": "estop",
        "reset_drive_fault": "clear_estop",
    }

    def __init__(self):
        super().__init__("nx_motion_node")
        default_iface = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")
        self._dog_interface = str(
            _parameter_value(self, "dog_interface", default_iface))
        default_gateway_socket = os.environ.get(
            "GO2W_SPORT_GATEWAY_SOCKET",
            "/run/go2w-sport-gateway/sport.sock",
        )
        self._gateway_socket = str(_parameter_value(
            self, "sport_gateway_socket", default_gateway_socket))
        self._move_rate = float(_parameter_value(self, "move_rate", 20.0))
        self._sdk_call_timeout = float(
            _parameter_value(self, "sdk_call_timeout", 0.8))
        self._sdk_retry_sec = float(
            _parameter_value(self, "sdk_retry_sec", 1.0))
        self._nav_scan_timeout = float(
            _parameter_value(self, "nav_scan_timeout", 1.8))
        self._manual_cmd_timeout = float(
            _parameter_value(self, "manual_cmd_timeout", 0.5))
        self._nav_cmd_timeout = float(
            _parameter_value(self, "nav_cmd_timeout", 0.3))
        self._drive_response_timeout = float(
            _parameter_value(self, "drive_response_timeout", 0.6))
        self._min_wheel_response = float(
            _parameter_value(self, "min_wheel_response", 0.15))
        self._minimum_battery_soc = float(
            _parameter_value(self, "min_drive_battery_soc", 20.0))
        self._transition_timeout = float(
            _parameter_value(self, "pose_feedback_timeout", 4.0))
        self._max_vx = float(_parameter_value(self, "max_vx", 0.30))
        self._max_vy = float(_parameter_value(self, "max_vy", 0.0))
        self._max_vyaw = float(_parameter_value(self, "max_vyaw", 0.15))
        self._turn_creep_gain = float(
            _parameter_value(self, "turn_creep_compensation_gain", 1.0))
        self._turn_creep_maximum = float(
            _parameter_value(self, "turn_creep_compensation_max", 0.15))
        self._turn_linear_epsilon = float(
            _parameter_value(self, "pure_turn_linear_epsilon", 0.02))
        self._turn_angular_threshold = float(
            _parameter_value(self, "pure_turn_angular_threshold", 0.05))
        self._pure_turn_clearance = float(
            _parameter_value(self, "pure_turn_clearance", 0.95))
        self._turn_flip_window = float(
            _parameter_value(self, "nav_turn_flip_window", 3.0))
        self._max_turn_flips = int(
            _parameter_value(self, "nav_max_turn_flips", 3))

        clock = time.monotonic
        machine = Go2WMotionMachine(
            now=clock,
            stop_profile=_stop_profile_from_environment(),
            minimum_battery_soc=self._minimum_battery_soc,
            transition_timeout=self._transition_timeout,
        )
        scan_watchdog = ScanFreshnessWatchdog(
            timeout=self._nav_scan_timeout,
            clock=clock,
            pure_turn_clearance=self._pure_turn_clearance,
            pure_turn_linear_epsilon=self._turn_linear_epsilon,
            pure_turn_angular_threshold=self._turn_angular_threshold,
            turn_flip_window=self._turn_flip_window,
            max_turn_flips=self._max_turn_flips,
        )
        drive_watchdog = DriveExecutionWatchdog(
            timeout=self._drive_response_timeout,
            min_wheel_speed=self._min_wheel_response,
            clock=clock,
        )
        self._controller = MotionController(
            machine=machine,
            scan_watchdog=scan_watchdog,
            drive_watchdog=drive_watchdog,
            clock=clock,
            manual_timeout=self._manual_cmd_timeout,
            nav_timeout=self._nav_cmd_timeout,
            max_vx=self._max_vx,
            max_vy=self._max_vy,
            max_vyaw=self._max_vyaw,
            turn_creep_gain=self._turn_creep_gain,
            turn_creep_maximum=self._turn_creep_maximum,
            turn_linear_epsilon=self._turn_linear_epsilon,
            turn_angular_threshold=self._turn_angular_threshold,
        )

        self._events = queue.Queue()
        self._status_lock = threading.Lock()
        self._status = self._initial_status()
        self._last_command = (0.0, 0.0, 0.0)
        self._last_command_source = None
        self._invalid_feedback_count = 0
        self._invalid_intent_count = 0
        self._scan_valid_count = 0
        self._scan_invalid_count = 0
        self._sdk_initialized = False
        self._shutdown_enqueued = False
        self._actor_thread = threading.Thread(
            target=self._actor_loop,
            name="go2w-motion-actor",
            daemon=True,
        )
        self._actor_thread.start()

        self._state_pub = self.create_publisher(String, "/dog_state", 10)
        self._drive_feedback_sub = self.create_subscription(
            String, "/wheel_feedback", self._on_drive_feedback, 10)
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan_mid360", self._on_scan, 10)
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._nav_cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel_nav", self._on_nav_cmd_vel, 10)
        self._motion_session_sub = self.create_subscription(
            String, "/motion_session", self._on_motion_session, 10)
        self._cmd_pose_sub = self.create_subscription(
            String, "/cmd_pose", self._on_cmd_pose, 10)
        self._state_timer = self.create_timer(0.5, self._publish_state)

        self.get_logger().info(
            "motion v4 started in BOOT_HOLD; actor owns all SDK effects; "
            f"release={RELEASE_ID} stop_profile={_stop_profile_from_environment().value}")

    def _initial_status(self):
        return {
            "schema_version": 4,
            "release_id": RELEASE_ID,
            "state": "DISCONNECTED",
            "session": "boot_hold",
            "drive_session": "startup",
            "drive_session_phase": "boot_hold",
            "drive_session_owner": None,
            "drive_session_reason": "waiting_for_sdk_and_feedback",
            "sdk_ready": False,
            "motion_service": None,
            "velocity_authorized": False,
            "nav_scan_fresh": False,
            "nav_guard_reason": None,
            "drive_fault": None,
            "vx": 0.0,
            "vy": 0.0,
            "vyaw": 0.0,
        }

    def _enqueue(self, kind, payload=None):
        self._events.put((kind, payload))

    def _on_drive_feedback(self, message):
        self._enqueue("feedback", getattr(message, "data", ""))

    def _on_scan(self, message):
        self._enqueue("scan", message)

    def _on_cmd_vel(self, message):
        self._enqueue("velocity", (
            "manual",
            (message.linear.x, message.linear.y, message.angular.z),
        ))

    def _on_nav_cmd_vel(self, message):
        self._enqueue("velocity", (
            "nav",
            (message.linear.x, message.linear.y, message.angular.z),
        ))

    def _on_motion_session(self, message):
        self._enqueue("intent", getattr(message, "data", ""))

    def _on_cmd_pose(self, message):
        self._enqueue("pose", getattr(message, "data", ""))

    def _actor_loop(self):
        period = 1.0 / max(1.0, self._move_rate)
        next_tick = time.monotonic()
        next_sdk_attempt = 0.0
        while rclpy.ok():
            now = time.monotonic()
            if not self._sdk_initialized and now >= next_sdk_attempt:
                self._try_initialize_sdk()
                next_sdk_attempt = now + max(0.2, self._sdk_retry_sec)
            wait = max(0.0, min(0.05, next_tick - now))
            try:
                kind, payload = self._events.get(timeout=wait)
            except queue.Empty:
                kind, payload = None, None
            if kind == "shutdown":
                self._controller.shutdown()
                self._refresh_status()
                return
            if kind is not None:
                self._process_event(kind, payload)
            now = time.monotonic()
            if now >= next_tick:
                self._controller.tick()
                next_tick = now + period
            self._refresh_status()

    def _try_initialize_sdk(self):
        adapter = None
        try:
            adapter = SportGatewayClient(
                self._gateway_socket,
                timeout=self._sdk_call_timeout,
            )
            initialized = adapter.initialize()
            if (initialized.code != 0
                    or initialized.motion_service != "ai-w"):
                adapter.close()
                self.get_logger().error(
                    "Sport gateway mode check is not healthy; retrying "
                    "without velocity or posture command: "
                    f"code={initialized.code} data={initialized.raw_mode!r}")
                return
            self._controller.attach_adapter(adapter, initialized.motion_service)
            self._sdk_initialized = True
            self.get_logger().info(
                "stable Sport gateway connected; MotionSwitcher=ai-w; "
                "waiting for feedback-confirmed startup state")
        except Exception as exc:
            if adapter is not None:
                adapter.close()
            self.get_logger().warning(
                "Sport gateway connection failed; retrying without posture "
                f"command: {exc}")

    def _process_event(self, kind, payload):
        try:
            if kind == "feedback":
                self._controller.observe_feedback(payload)
            elif kind == "scan":
                if self._controller.observe_scan(payload):
                    self._scan_valid_count += 1
                else:
                    self._scan_invalid_count += 1
            elif kind == "intent":
                self._controller.handle_intent(payload)
            elif kind == "pose":
                self._handle_pose_compatibility(payload)
            elif kind == "velocity":
                owner, velocity = payload
                self._last_command_source = owner
                self._last_command = tuple(float(value) for value in velocity)
                self._controller.update_velocity(owner, velocity)
        except (MotionProtocolError, ValueError, TypeError, json.JSONDecodeError) as exc:
            if kind == "feedback":
                self._invalid_feedback_count += 1
            else:
                self._invalid_intent_count += 1
            self.get_logger().warning(f"rejected motion {kind}: {exc}")
        except Exception as exc:
            self.get_logger().error(f"motion actor event {kind} failed: {exc}")

    def _handle_pose_compatibility(self, payload):
        command = str(payload).strip().lower()
        intent = self._POSE_TO_INTENT.get(command)
        if intent is None:
            self._invalid_intent_count += 1
            self.get_logger().warning(
                f"unsupported legacy pose command {command!r}; "
                "use the versioned motion intent protocol")
            return
        envelope = MotionIntentEnvelope.parse({
            "schema_version": 1,
            "request_id": f"pose-{uuid.uuid4()}",
            "intent": intent,
            "source": "legacy_pose_panel",
        })
        self._controller.handle_intent(envelope)

    def _refresh_status(self):
        snapshot = self._controller.machine.snapshot()
        raw = self._controller.last_feedback_payload
        canonical = motion_status_dict(
            snapshot,
            release_id=RELEASE_ID,
            raw={
                "sport_mode": raw.get("sport_mode"),
                "error_code": raw.get("sport_error_code"),
                "sample_id": raw.get("sample_id"),
            },
            legacy_deprecation_count=self._controller.legacy_deprecation_count,
        )
        state = self._legacy_state(snapshot)
        drive_session = self._legacy_drive_session(snapshot.session)
        velocity = (
            self._last_command
            if snapshot.velocity_authorized else (0.0, 0.0, 0.0))
        receipt = self._controller.last_receipt
        canonical.update({
            "state": state,
            "sdk_ready": self._controller.sdk_ready,
            "vx": round(float(velocity[0]), 3),
            "vy": round(float(velocity[1]), 3),
            "vyaw": round(float(velocity[2]), 3),
            "sdk_vx": round(float(velocity[0]), 3),
            "sdk_vy": round(float(velocity[1]), 3),
            "sdk_vyaw": round(float(velocity[2]), 3),
            "motion_source": self._last_command_source,
            "nav_scan_fresh": self._controller.scan_watchdog.is_fresh(),
            "nav_guard_reason": self._controller.scan_watchdog.nav_guard_reason(),
            "scan_valid_n": self._scan_valid_count,
            "scan_invalid_n": self._scan_invalid_count,
            "invalid_feedback_n": self._invalid_feedback_count,
            "invalid_intent_n": self._invalid_intent_count,
            "battery_soc": raw.get("battery_soc"),
            "bms_status": raw.get("bms_status"),
            "sport_mode": raw.get("sport_mode"),
            "sport_progress": raw.get("sport_progress"),
            "gait_type": raw.get("gait_type"),
            "wheel_dq": raw.get("wheel_dq"),
            "roll": raw.get("roll"),
            "pitch": raw.get("pitch"),
            "motor_lost": raw.get("motor_lost"),
            "drive_fault": snapshot.fault,
            "drive_session": drive_session,
            "drive_session_owner": snapshot.owner,
            "drive_session_phase": snapshot.session.value,
            "drive_session_reason": (
                snapshot.fault or snapshot.transition_operation or "stable"),
            "wheel_activation_phase": snapshot.session.value,
            "last_sdk_code": receipt.code if receipt is not None else None,
            "last_sdk_operation": (
                receipt.operation if receipt is not None else None),
            "state_model_version": 4,
            "link_state": "online" if snapshot.telemetry_fresh else "stale",
            "motion_state": snapshot.actual_motion.value,
            "safety_state": (
                "normal" if snapshot.velocity_authorized else "inhibited"),
            "safety_reason": snapshot.fault,
            "raw_sport_mode": raw.get("sport_mode"),
            "raw_error_code": raw.get("sport_error_code"),
            "feedback_age_sec": None,
        })
        with self._status_lock:
            self._status = canonical

    @staticmethod
    def _legacy_state(snapshot):
        if snapshot.session is SessionState.PARKED:
            return "STOPPED"
        if snapshot.session in {
                SessionState.MANUAL_ACTIVE, SessionState.NAV_ACTIVE}:
            return (
                "MOVING" if snapshot.actual_motion is ActualMotionState.MOVING
                else "STOPPED")
        if snapshot.session is SessionState.ACTIVATING:
            return "STOOD"
        if snapshot.session in {SessionState.STOPPING, SessionState.PARKING}:
            return "STANDING"
        if snapshot.session is SessionState.ESTOP:
            return "EMERGENCY"
        if snapshot.session is SessionState.FAULT:
            # 应用故障(parked_state_lost/physical_mode_lost等), 非宇树底盘急停;
            # P2 自愈可恢复。前端据此与真急停区分显示, 不再误导为"狗硬件EMERGENCY"。
            return "FAULT"
        return "DISCONNECTED"

    @staticmethod
    def _legacy_drive_session(session):
        if session is SessionState.PARKED:
            return "parked"
        if session in {SessionState.MANUAL_ACTIVE, SessionState.NAV_ACTIVE}:
            return "active"
        if session is SessionState.ACTIVATING:
            return "activating"
        if session in {SessionState.STOPPING, SessionState.PARKING}:
            return "parking"
        if session is SessionState.ESTOP:
            return "estop"
        if session is SessionState.FAULT:
            return "fault"
        return "startup"

    def _publish_state(self):
        with self._status_lock:
            payload = dict(self._status)
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._state_pub.publish(message)

    def destroy_node(self):
        if not self._shutdown_enqueued:
            self._shutdown_enqueued = True
            self._enqueue('shutdown')
            if self._actor_thread.is_alive():
                self._actor_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NxMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
