"""ROS-independent velocity and execution safety gates."""

from __future__ import annotations

import math
import threading


def motion_command_timed_out(
    is_moving, source, last_cmd_time, now, manual_timeout, nav_timeout,
):
    if not is_moving or last_cmd_time <= 0.0:
        return False
    timeout = nav_timeout if source == "nav" else manual_timeout
    values = (last_cmd_time, now, timeout)
    if not all(math.isfinite(float(value)) for value in values) or timeout <= 0.0:
        return True
    age = now - last_cmd_time
    return age < 0.0 or age > timeout


def compensate_pure_turn_creep(
    velocity, *, gain, maximum, linear_epsilon, angular_threshold,
):
    """Bound creep compensation without turning zero-vx yaw into reverse."""

    vx, vy, vyaw = (float(value) for value in velocity)
    if abs(vx) > float(linear_epsilon) or abs(vyaw) < float(angular_threshold):
        return vx, vy, vyaw
    available_forward_velocity = max(0.0, vx)
    correction = min(
        float(maximum),
        float(gain) * abs(vyaw),
        available_forward_velocity,
    )
    return vx - correction, vy, vyaw


def battery_allows_drive(battery_soc, minimum_soc):
    try:
        soc = float(battery_soc)
        minimum = float(minimum_soc)
    except (TypeError, ValueError, OverflowError):
        return False
    if not math.isfinite(soc) or not math.isfinite(minimum):
        return False
    if not 0.0 <= soc <= 100.0 or not 0.0 <= minimum <= 100.0:
        return False
    return soc >= minimum


class ScanFreshnessWatchdog:
    """Fail-closed MID360 scan gate immediately before SDK Move."""

    ZERO = (0.0, 0.0, 0.0)

    def __init__(
        self,
        timeout,
        clock,
        *,
        pure_turn_clearance=0.95,
        pure_turn_linear_epsilon=0.02,
        pure_turn_angular_threshold=0.05,
        turn_flip_window=3.0,
        max_turn_flips=3,
    ):
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("scan timeout must be finite and positive")
        pure_turn_clearance = float(pure_turn_clearance)
        pure_turn_linear_epsilon = float(pure_turn_linear_epsilon)
        pure_turn_angular_threshold = float(pure_turn_angular_threshold)
        turn_flip_window = float(turn_flip_window)
        if (
            not math.isfinite(pure_turn_clearance)
            or pure_turn_clearance <= 0.0
            or not math.isfinite(pure_turn_linear_epsilon)
            or not 0.0 <= pure_turn_linear_epsilon <= 0.1
            or not math.isfinite(pure_turn_angular_threshold)
            or not 0.0 < pure_turn_angular_threshold <= 0.5
            or not math.isfinite(turn_flip_window)
            or turn_flip_window <= 0.0
            or isinstance(max_turn_flips, bool)
            or int(max_turn_flips) != max_turn_flips
            or int(max_turn_flips) < 1
        ):
            raise ValueError("invalid pure-turn safety parameters")
        self._timeout = timeout
        self._clock = clock
        self._pure_turn_clearance = pure_turn_clearance
        self._pure_turn_linear_epsilon = pure_turn_linear_epsilon
        self._pure_turn_angular_threshold = pure_turn_angular_threshold
        self._turn_flip_window = turn_flip_window
        self._max_turn_flips = int(max_turn_flips)
        self._lock = threading.Lock()
        self._last_stamp = None
        self._last_receive = None
        self._nearest_obstacle = None
        self._turn_sign = 0
        self._turn_flip_times = []
        self._nav_guard_reason = None

    @staticmethod
    def _coerce_stamp(msg):
        try:
            if msg.header.frame_id != "base_link":
                return None
            sec_value = float(msg.header.stamp.sec)
            nanosec_value = float(msg.header.stamp.nanosec)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(sec_value) or not math.isfinite(nanosec_value):
            return None
        sec = int(sec_value)
        nanosec = int(nanosec_value)
        if sec_value != sec or nanosec_value != nanosec:
            return None
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return None
        if sec == 0 and nanosec == 0:
            return None
        return sec, nanosec

    @staticmethod
    def _legal_ranges(msg):
        try:
            metadata = (
                float(msg.angle_min),
                float(msg.angle_max),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
            )
            ranges = list(msg.ranges)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        angle_min, angle_max, angle_increment, range_min, range_max = metadata
        if not all(math.isfinite(value) for value in metadata):
            return False
        if angle_increment <= 0.0 or angle_max <= angle_min:
            return False
        if range_min < 0.0 or range_max <= range_min or len(ranges) < 16:
            return False
        usable = 0
        for raw in ranges:
            try:
                value = float(raw)
            except (TypeError, ValueError, OverflowError):
                return False
            if math.isnan(value):
                continue
            if math.isinf(value):
                if value < 0.0:
                    return False
                usable += 1
                continue
            if value < range_min or value > range_max:
                return False
            usable += 1
        # Only structural sanity (enough non-NaN bins) is required. The count
        # of finite returns is deliberately NOT gated: an open environment
        # legitimately yields zero finite returns and must still be a valid,
        # fresh scan. See observe_scan.
        return usable >= max(8, len(ranges) // 4)

    @staticmethod
    def coerce_velocity(velocity):
        try:
            values = tuple(float(item) for item in velocity)
        except (TypeError, ValueError, OverflowError):
            return None
        if len(values) != 3 or not all(math.isfinite(item) for item in values):
            return None
        return values

    def observe_scan(self, msg):
        stamp = self._coerce_stamp(msg)
        if stamp is None:
            return False
        try:
            now = float(self._clock())
        except (TypeError, ValueError, OverflowError):
            return False
        if not math.isfinite(now):
            return False
        # Structural gate only: a valid LaserScan arriving on the right frame
        # marks the stream fresh regardless of how many finite returns it has.
        # An open environment legitimately yields zero finite returns;
        # staleness must mean "messages stopped", not "content empty".
        if not self._legal_ranges(msg):
            return False
        try:
            finite_ranges = [
                float(value) for value in msg.ranges
                if math.isfinite(float(value))
            ]
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        with self._lock:
            if self._last_stamp is not None and stamp <= self._last_stamp:
                return False
            self._last_stamp = stamp
            self._last_receive = now
            self._nearest_obstacle = (
                min(finite_ranges) if finite_ranges else None)
        return True

    def _fresh_locked(self, now):
        if self._last_receive is None or not math.isfinite(now):
            return False
        age = now - self._last_receive
        return 0.0 <= age <= self._timeout

    def is_fresh(self):
        try:
            now = float(self._clock())
        except (TypeError, ValueError, OverflowError):
            return False
        with self._lock:
            return self._fresh_locked(now)

    def filter_nav_velocity(self, velocity):
        values = self.coerce_velocity(velocity)
        if values is None or values == self.ZERO:
            return self.ZERO
        try:
            now = float(self._clock())
        except (TypeError, ValueError, OverflowError):
            return self.ZERO
        with self._lock:
            if not self._fresh_locked(now):
                return self.ZERO
            if self._nav_guard_reason is not None:
                return self.ZERO
            vx, _, vyaw = values
            pure_turn = (
                abs(vx) <= self._pure_turn_linear_epsilon
                and abs(vyaw) >= self._pure_turn_angular_threshold
            )
            if not pure_turn:
                self._turn_sign = 0
                self._turn_flip_times = []
                return values
            if (
                self._nearest_obstacle is not None
                and self._nearest_obstacle < self._pure_turn_clearance
            ):
                self._nav_guard_reason = "pure_turn_clearance"
                return self.ZERO
            sign = 1 if vyaw > 0.0 else -1
            if self._turn_sign and sign != self._turn_sign:
                self._turn_flip_times.append(now)
            self._turn_sign = sign
            self._turn_flip_times = [
                stamp for stamp in self._turn_flip_times
                if 0.0 <= now - stamp <= self._turn_flip_window
            ]
            if len(self._turn_flip_times) >= self._max_turn_flips:
                self._nav_guard_reason = "pure_turn_oscillation"
                return self.ZERO
            return values

    def nav_must_stop(self):
        try:
            now = float(self._clock())
        except (TypeError, ValueError, OverflowError):
            return True
        with self._lock:
            return self._nav_guard_reason is not None or not self._fresh_locked(now)

    def nav_guard_reason(self):
        with self._lock:
            return self._nav_guard_reason

    def reset_nav_guard(self):
        with self._lock:
            self._turn_sign = 0
            self._turn_flip_times = []
            self._nav_guard_reason = None


class DriveExecutionWatchdog:
    """Confirm nonzero SDK commands produce measured wheel motion."""

    ZERO = (0.0, 0.0, 0.0)
    WHEEL_INDICES = (12, 13, 14, 15)

    def __init__(self, timeout, min_wheel_speed, clock):
        timeout = float(timeout)
        min_wheel_speed = float(min_wheel_speed)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("drive timeout must be finite and positive")
        if not math.isfinite(min_wheel_speed) or min_wheel_speed <= 0.0:
            raise ValueError("minimum wheel speed must be finite and positive")
        self._timeout = timeout
        self._min_wheel_speed = min_wheel_speed
        self._clock = clock
        self._lock = threading.Lock()
        self._last_feedback = None
        self._wheel_dq = None
        self._battery_soc = None
        self._bms_status = None
        self._sport_mode = None
        self._sport_progress = None
        self._gait_type = None
        self._no_response_since = None

    def observe_low_state(self, message):
        try:
            motors = message.motor_state
            wheel_dq = tuple(
                float(motors[index].dq) for index in self.WHEEL_INDICES)
            battery_soc = int(message.bms_state.soc)
            bms_status = int(message.bms_state.status)
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
            return False
        return self.observe_feedback(wheel_dq, battery_soc, bms_status)

    def observe_feedback(
        self,
        wheel_dq,
        battery_soc,
        bms_status,
        sport_mode=None,
        sport_progress=None,
        gait_type=None,
    ):
        try:
            wheel_dq = tuple(float(value) for value in wheel_dq)
            battery_soc = int(battery_soc)
            bms_status = int(bms_status)
            sport_mode = None if sport_mode is None else int(sport_mode)
            sport_progress = (
                None if sport_progress is None else float(sport_progress))
            gait_type = None if gait_type is None else int(gait_type)
            now = float(self._clock())
        except (TypeError, ValueError, OverflowError):
            return False
        if len(wheel_dq) != 4:
            return False
        if not math.isfinite(now) or not all(
                math.isfinite(value) for value in wheel_dq):
            return False
        if not 0 <= battery_soc <= 100 or not 0 <= bms_status <= 255:
            return False
        if sport_mode is not None and not 0 <= sport_mode <= 255:
            return False
        if (
            sport_progress is not None
            and (not math.isfinite(sport_progress)
                 or not 0.0 <= sport_progress <= 1.0)
        ):
            return False
        if gait_type is not None and not 0 <= gait_type <= 255:
            return False
        with self._lock:
            self._last_feedback = now
            self._wheel_dq = wheel_dq
            self._battery_soc = battery_soc
            self._bms_status = bms_status
            self._sport_mode = sport_mode
            self._sport_progress = sport_progress
            self._gait_type = gait_type
        return True

    @staticmethod
    def _coerce_command(command):
        try:
            values = tuple(float(value) for value in command)
        except (TypeError, ValueError, OverflowError):
            return None
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            return None
        return values

    def evaluate(self, command):
        values = self._coerce_command(command)
        try:
            now = float(self._clock())
        except (TypeError, ValueError, OverflowError):
            now = float("nan")
        with self._lock:
            if values is None or not math.isfinite(now):
                reason = "wheel_feedback_invalid"
            elif values == self.ZERO:
                self._no_response_since = None
                return None
            elif self._gait_type not in (None, 0):
                # Go2-W wheel locomotion under MotionSwitcher ``ai-w`` reports
                # gait_type=0.  Trot/run/stair/adjust values belong to leg-gait
                # behaviors and must never coexist with an authorized wheel
                # velocity command.
                self._no_response_since = None
                return "unexpected_gait"
            elif (
                self._last_feedback is None
                or now - self._last_feedback > self._timeout
            ):
                reason = "wheel_feedback_stale"
            elif self._wheel_dq is None or (
                sum(abs(value) for value in self._wheel_dq) / len(self._wheel_dq)
                < self._min_wheel_speed
            ):
                reason = "wheel_no_response"
            else:
                self._no_response_since = None
                return None
            if self._no_response_since is None or now < self._no_response_since:
                self._no_response_since = now
                return None
            if now - self._no_response_since > self._timeout:
                return reason
            return None

    def reset(self):
        with self._lock:
            self._no_response_since = None

    def snapshot(self):
        with self._lock:
            return {
                "battery_soc": self._battery_soc,
                "bms_status": self._bms_status,
                "wheel_dq": self._wheel_dq,
                "sport_mode": self._sport_mode,
                "sport_progress": self._sport_progress,
                "gait_type": self._gait_type,
            }
