# FastLIO Nav2 Panel Goal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore boot-persistent FastLIO localization and Nav2 obstacle avoidance, compensate the MID360's 20-degree downward mounting angle, and let an operator click a point in Panel to drive the robot there.

**Architecture:** Keep the existing FastLIO, map-odom fuser, Nav2 and NX web processes. Repair the false TF health gate, supervise the stack from a persistent systemd unit, publish a horizontal map-frame localization pose from the fuser, use the verified horizontal `/scan` for costmaps, and add a serialized point-goal controller behind `POST /api/navigate`.

**Tech Stack:** ROS 2 Humble, FAST_LIO_ROS2, tf2, Nav2 DWB/Navfn/velocity_smoother, Python 3/rclpy, systemd, vanilla JavaScript Canvas, pytest, Node.js contract tests.

---

## File structure

- Modify `docker/bringup_slam_nav2.sh`: reliable TF gate, exact -20° default, correct FastDDS profile, restart policy for transient child units.
- Create `docker/go2w-slam-nav.service`: boot-persistent owner for the localization/navigation stack.
- Modify `docker/deploy_nav2_bprime.sh`: deploy the repaired bringup, profile and persistent unit in addition to runtime artifacts.
- Modify `docker/test_livox_deploy_contract.py`: boot/service/deploy contracts.
- Modify `docker/test_map_odom_fuser_performance_contract.py`: 20° math, pose publication and costmap contracts.
- Modify `src/go2w_bridge/go2w_bridge/map_odom_fuser.py`: conjugated map pose plus `/localization_pose`.
- Modify `src/go2w_nav/config/nav2_params_3d.yaml`: remove the invalid CustomMsg-as-PointCloud2 voxel source.
- Modify `src/go2w_nav/launch/nav2_3d.launch.py`: document and launch only the verified scan-based costmap path.
- Create `web/nx_point_nav.py`: serialized single-goal controller.
- Create `web/test_point_navigation.py`: behavior tests for submit, replacement, cancel and result mapping.
- Modify `web/nx_web_server.py`: localization-pose subscription, point-navigation lifecycle and HTTP endpoints.
- Modify `web/static/map.js`: click-to-go interaction and target marker.
- Modify `web/static/panel.html`: API call and WebSocket state rendering.
- Modify `web/test_map_contract.js`: click/drag/marker interaction contracts.
- Create `web/test_panel_navigation_contract.py`: server/panel wiring contract.
- Modify `docker/deploy_nx_web.sh`: deploy `nx_point_nav.py`.

### Task 1: Repair the TF startup gate and add boot persistence

**Files:**
- Modify: `docker/bringup_slam_nav2.sh`
- Create: `docker/go2w-slam-nav.service`
- Modify: `docker/test_livox_deploy_contract.py`

- [ ] **Step 1: Write the failing startup contracts**

Append these tests to `docker/test_livox_deploy_contract.py`:

```python
def test_bringup_tf_gate_avoids_pipefail_sigpipe_false_negative():
    script = read("docker/bringup_slam_nav2.sh")
    wait_tf = script.split("wait_tf() {", 1)[1].split("\n}", 1)[0]
    assert "set +o pipefail" in wait_tf
    assert "stdbuf -oL" in wait_tf
    assert 'grep -m1 -q "At time"' in wait_tf
    assert "for ((i=1; i<=timeout; i++))" not in wait_tf


def test_bringup_uses_exact_twenty_degree_pitch_and_real_profile_path():
    script = read("docker/bringup_slam_nav2.sh")
    assert '${BODY_TO_BASE_PITCH:--0.3490658504}' in script
    assert '$HOME/go2w_ws/fastdds_udp.xml' in script
    assert 'Restart=on-failure' in script
    assert 'RestartSec=2' in script


def test_slam_nav_service_is_boot_persistent_and_ordered():
    service = read("docker/go2w-slam-nav.service")
    assert "Requires=livox-mid360-driver.service go2w-sensor.service go2w-motion.service" in service
    assert "After=livox-mid360-driver.service go2w-sensor.service go2w-motion.service" in service
    assert "User=nx" in service
    assert "ExecStart=/bin/bash /home/nx/go2w_ws/bringup_slam_nav2.sh --no-shm" in service
    assert "Restart=on-failure" in service
    assert "WantedBy=multi-user.target" in service


def test_nav_deployer_installs_and_enables_persistent_stack():
    script = read("docker/deploy_nav2_bprime.sh")
    assert "docker/go2w-slam-nav.service" in script
    assert "docker/bringup_slam_nav2.sh" in script
    assert "docker/fastdds_udp.xml" in script
    assert "systemctl enable go2w-slam-nav.service" in script
    assert "systemctl restart go2w-slam-nav.service" in script
```

- [ ] **Step 2: Run the contracts and verify RED**

Run:

```powershell
python -m pytest docker/test_livox_deploy_contract.py -q
```

Expected: four new tests fail because the TF gate still loops with `pipefail`, the default is `-0.3037`, and the persistent service does not exist.

- [ ] **Step 3: Implement the pipeline-safe one-shot TF gate**

Replace `wait_tf()` in `docker/bringup_slam_nav2.sh` with:

```bash
wait_tf() {
  local parent=$1 child=$2 timeout_s=$3
  log "等 TF $parent → $child (最长 ${timeout_s}s)..."
  ros_env
  if (
    set +o pipefail
    timeout "$timeout_s" stdbuf -oL \
      ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null \
      | grep -m1 -q "At time"
  ); then
    ok "TF $parent → $child 可查"
    return 0
  fi
  die "TF $parent → $child ${timeout_s}s 不可查 (FastLIO/fuser 未发 TF?)"
}
```

Change the configuration defaults and child-unit properties to:

```bash
PROFILE_XML="${PROFILE_XML:-$HOME/go2w_ws/fastdds_udp.xml}"
BODY_TO_BASE_PITCH="${BODY_TO_BASE_PITCH:--0.3490658504}"
```

and add these entries to `start_transient()`'s `env_args` array:

```bash
    -p "Restart=on-failure"
    -p "RestartSec=2"
```

Before starting a replacement unit, stop and reset a same-named stale unit. Use
`systemd-run --collect` so failed transient units do not block a later bringup.
The readiness helpers must enforce one wall-clock deadline (using `SECONDS` and
the remaining timeout for each ROS CLI probe), and lifecycle matching must be
anchored to `active` so `inactive` cannot pass.

- [ ] **Step 4: Create the persistent service**

Create `docker/go2w-slam-nav.service` with:

```ini
[Unit]
Description=Go2W FastLIO localization and Nav2 navigation stack
Wants=network-online.target go2w-web.service
Requires=livox-mid360-driver.service go2w-sensor.service go2w-motion.service
After=network-online.target livox-mid360-driver.service go2w-sensor.service go2w-motion.service

[Service]
Type=oneshot
User=nx
Environment=HOME=/home/nx
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_DOMAIN_ID=0
WorkingDirectory=/home/nx/go2w_ws
ExecStart=/bin/bash /home/nx/go2w_ws/bringup_slam_nav2.sh --no-shm
ExecStopPost=+/usr/bin/systemctl stop nav2-3d.service map-odom-fuser.service fastlio.service
RemainAfterExit=yes
Restart=on-failure
RestartSec=10
TimeoutStartSec=600
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 5: Update the Nav2 deployment script**

Extend `docker/deploy_nav2_bprime.sh` so its SCP input list also contains:

```bash
  "$WIN_WS/docker/bringup_slam_nav2.sh" \
  "$WIN_WS/docker/fastdds_udp.xml" \
  "$WIN_WS/docker/go2w-slam-nav.service" \
```

Copy them on NX with:

```bash
cp /tmp/bprime/bringup_slam_nav2.sh "$HOME/go2w_ws/bringup_slam_nav2.sh"
chmod 775 "$HOME/go2w_ws/bringup_slam_nav2.sh"
cp /tmp/bprime/fastdds_udp.xml "$HOME/go2w_ws/fastdds_udp.xml"
sudo cp /tmp/bprime/go2w-slam-nav.service /etc/systemd/system/go2w-slam-nav.service
sudo systemctl daemon-reload
sudo systemctl enable go2w-slam-nav.service
sudo systemctl restart go2w-slam-nav.service
```

- [ ] **Step 6: Verify GREEN**

Run:

```powershell
python -m pytest docker/test_livox_deploy_contract.py -q
```

Expected: all tests pass.

### Task 2: Publish a horizontal map-frame localization pose

**Files:**
- Modify: `src/go2w_bridge/go2w_bridge/map_odom_fuser.py`
- Modify: `docker/test_map_odom_fuser_performance_contract.py`

- [ ] **Step 1: Replace the old pitch contract and add numerical/pose contracts**

Replace `test_bringup_passes_body_to_base_pitch_to_fuser()`'s old `-0.3037` assertion with:

```python
def test_bringup_passes_exact_twenty_degree_pitch_to_fuser():
    script = (ROOT / "docker/bringup_slam_nav2.sh").read_text(encoding="utf-8")
    assert "BODY_TO_BASE_PITCH" in script
    assert "${BODY_TO_BASE_PITCH:--0.3490658504}" in script
    assert "body_to_base_pitch:=$BODY_TO_BASE_PITCH" in script
```

Append:

```python
def _load_fuser_math():
    import math
    import numpy as np

    tree = ast.parse(FUSER.read_text(encoding="utf-8"))
    wanted = {"_rpy_to_mat", "_build_static_tf", "_conjugate_pose"}
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in wanted]
    namespace = {"math": math, "np": np}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(FUSER), "exec"), namespace)
    return namespace


def test_twenty_degree_conjugation_maps_sensor_rotation_to_level_yaw():
    import math
    import numpy as np

    ns = _load_fuser_math()
    build = ns["_build_static_tf"]
    conjugate = ns["_conjugate_pose"]
    t_body_base = build(0, 0, 0, 0, -math.radians(20), 0)
    yaw = build(0, 0, 0, 0, 0, math.radians(35))
    sensor_rotation = t_body_base @ yaw @ np.linalg.inv(t_body_base)

    leveled = conjugate(sensor_rotation, t_body_base)

    assert np.allclose(leveled, yaw, atol=1e-9)
    assert np.allclose(conjugate(np.eye(4), t_body_base), np.eye(4), atol=1e-9)


def test_fuser_publishes_map_frame_localization_pose():
    source = FUSER.read_text(encoding="utf-8")
    assert "from nav_msgs.msg import Odometry" in source
    assert "self._pose_pub = self.create_publisher(Odometry, '/localization_pose', 10)" in source
    assert "cb_time = Time.from_msg(cb.header.stamp)" in source
    assert "lookup_transform(self._odom, self._base, cb_time)" in source
    assert "self._last_cb_stamp_ns" in source
    assert "self._T_map_odom = T_map_base_scan @ np.linalg.inv(T_ob_scan)" in source
    assert "T_map_base = self._T_map_odom @ T_ob_latest" in source
    assert "msg.header.stamp = ob_latest_msg.header.stamp" in source
    assert "pose.header.stamp = ob_latest_msg.header.stamp" in source
    assert "pose.header.frame_id = self._world" in source
    assert "pose.child_frame_id = self._base" in source
    assert "self._pose_pub.publish(pose)" in source
```

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py -q
```

Expected: failures for the exact pitch, missing `_conjugate_pose`, and missing pose publisher.

- [ ] **Step 3: Add the pure conjugation helper**

Add after `_build_static_tf()`:

```python
def _conjugate_pose(T_camera_body, T_body_base):
    """Express the FastLIO body pose in the horizontal base_link basis."""
    return np.linalg.inv(T_body_base) @ T_camera_body @ T_body_base
```

- [ ] **Step 4: Publish `/localization_pose` from the same fused matrix**

Add the import and publisher:

```python
from nav_msgs.msg import Odometry

# in MapOdomFuser.__init__
self._pose_pub = self.create_publisher(Odometry, '/localization_pose', 10)
```

In `__init__`, initialize the held correction:

```python
self._last_cb_stamp_ns = None
self._T_map_odom = None
```

Change `_fuse()` to update the held correction only for a new FastLIO stamp, using wheel odometry at that exact stamp. Then predict the current map pose with latest odometry and publish both outputs at the latest odometry stamp:

```python
        try:
            cb = self._buf.lookup_transform(self._fl_world, self._fl_body, Time())
            cb_time = Time.from_msg(cb.header.stamp)
            ob_latest_msg = self._buf.lookup_transform(self._odom, self._base, Time())
        except TransformException as e:
            self.get_logger().warning(
                f"TF lookup 未就绪: {e}", throttle_duration_sec=5.0)
            return

        cb_age = self.get_clock().now() - cb_time
        if cb_age.nanoseconds > 5e9:
            self.get_logger().warning(
                f"camera_init→body 过旧 {cb_age.nanoseconds / 1e9:.2f}s, skip",
                throttle_duration_sec=5.0)
            return

        if cb_time.nanoseconds != self._last_cb_stamp_ns:
            try:
                ob_scan_msg = self._buf.lookup_transform(
                    self._odom, self._base, cb_time)
            except TransformException as e:
                self.get_logger().warning(
                    f"odom→base_link 在 FastLIO 时间不可查: {e}",
                    throttle_duration_sec=5.0)
                return
            T_cb = _tf_to_mat(cb.transform)
            T_ob_scan = _tf_to_mat(ob_scan_msg.transform)
            T_map_base_scan = _conjugate_pose(T_cb, self._T_body_base)
            self._T_map_odom = T_map_base_scan @ np.linalg.inv(T_ob_scan)
            self._last_cb_stamp_ns = cb_time.nanoseconds

        if self._T_map_odom is None:
            return

        T_ob_latest = _tf_to_mat(ob_latest_msg.transform)
        T_map_base = self._T_map_odom @ T_ob_latest

        tx, ty, tz, qx, qy, qz, qw = _mat_to_tf(self._T_map_odom)
        msg = TransformStamped()
        msg.header.stamp = ob_latest_msg.header.stamp
        msg.header.frame_id = self._world
        msg.child_frame_id = self._odom
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._broad.sendTransform(msg)

        px, py, pz, pqx, pqy, pqz, pqw = _mat_to_tf(T_map_base)
        pose = Odometry()
        pose.header.stamp = ob_latest_msg.header.stamp
        pose.header.frame_id = self._world
        pose.child_frame_id = self._base
        pose.pose.pose.position.x = px
        pose.pose.pose.position.y = py
        pose.pose.pose.position.z = pz
        pose.pose.pose.orientation.x = pqx
        pose.pose.pose.orientation.y = pqy
        pose.pose.pose.orientation.z = pqz
        pose.pose.pose.orientation.w = pqw
        self._pose_pub.publish(pose)
```

- [ ] **Step 5: Verify GREEN**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py -q
python -m py_compile src/go2w_bridge/go2w_bridge/map_odom_fuser.py
```

Expected: all fuser contracts pass and compilation exits 0.

### Task 3: Make Nav2 costmaps use only the valid scan source

**Files:**
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`
- Modify: `src/go2w_nav/launch/nav2_3d.launch.py`
- Modify: `docker/test_map_odom_fuser_performance_contract.py`

- [ ] **Step 1: Add the failing costmap contract**

Append:

```python
def test_nav_costmaps_use_valid_filtered_scan_without_custommsg_voxel_layer():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    local = params.split("local_costmap:", 1)[1].split("global_costmap:", 1)[0]
    global_map = params.split("global_costmap:", 1)[1].split("planner_server:", 1)[0]

    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in local
    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in global_map
    assert 'topic: /scan' in local
    assert 'topic: /scan' in global_map
    assert "marking: True" in local and "clearing: True" in local
    assert "marking: True" in global_map and "clearing: True" in global_map
    assert "VoxelLayer" not in local
    assert "topic: /livox/lidar" not in local
```

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py::test_nav_costmaps_use_valid_filtered_scan_without_custommsg_voxel_layer -q
```

Expected: failure because local costmap still contains VoxelLayer and `/livox/lidar` as `PointCloud2`.

- [ ] **Step 3: Remove the invalid voxel layer**

In `nav2_params_3d.yaml`, set local plugins to:

```yaml
plugins: ["obstacle_layer", "inflation_layer"]
```

Delete the entire `voxel_layer:` block. Keep the existing `/scan` `ObstacleLayer`, including both `marking: True` and `clearing: True`.

Update `nav2_3d.launch.py`'s module and p2l comments to state that the active launch consumes the filtered `/scan` from `nx_sensor_node`; leave `p2l` out of `LaunchDescription` until a real CustomMsg conversion exists.

- [ ] **Step 4: Verify GREEN**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py -q
python -m py_compile src/go2w_nav/launch/nav2_3d.launch.py
```

Expected: all contracts pass.

### Task 4: Implement a serialized point-navigation controller

**Files:**
- Create: `web/test_point_navigation.py`
- Create: `web/nx_point_nav.py`

- [ ] **Step 1: Write behavior-first tests**

Create `web/test_point_navigation.py`:

```python
import math
import threading
import time

import pytest

from nx_point_nav import PointNavigationController


def wait_for(predicate, timeout=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition not reached")


class FakeNav:
    def __init__(self, results=None):
        self.results = list(results or [{"ok": True, "status": 4}])
        self.calls = []
        self.cancel_count = 0
        self.release = threading.Event()
        self.block = False

    def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
        self.calls.append((x, y, yaw, frame_id))
        if self.block:
            self.release.wait(1.0)
        return self.results.pop(0) if self.results else {"ok": True, "status": 4}

    def cancel_current(self):
        self.cancel_count += 1
        self.release.set()
        return True


def recorder():
    events = []

    def emit(message, force=False):
        events.append(message)

    return events, emit


def test_submit_broadcasts_pending_active_and_success():
    events, emit = recorder()
    nav = FakeNav()
    ctl = PointNavigationController(None, emit, nav_factory=lambda _: nav)
    try:
        accepted = ctl.submit({"x": 2, "y": -1, "yaw": 0.25, "frame_id": "map"})
        assert accepted["ok"] is True
        wait_for(lambda: any(e["data"]["status"] == "succeeded" for e in events))
        assert nav.calls == [(2.0, -1.0, 0.25, "map")]
        assert [e["data"]["status"] for e in events] == ["pending", "active", "succeeded"]
    finally:
        ctl.stop()


def test_new_goal_cancels_old_and_only_latest_finishes():
    events, emit = recorder()
    nav = FakeNav([{"ok": False, "reason": "cancelled"}, {"ok": True, "status": 4}])
    nav.block = True
    ctl = PointNavigationController(None, emit, nav_factory=lambda _: nav)
    try:
        ctl.submit({"x": 1, "y": 0, "yaw": 0})
        wait_for(lambda: len(nav.calls) == 1)
        nav.block = False
        ctl.submit({"x": 3, "y": 1, "yaw": 0.5})
        wait_for(lambda: len(nav.calls) == 2)
        wait_for(lambda: events[-1]["data"]["status"] == "succeeded")
        assert nav.cancel_count >= 1
        assert nav.calls[-1] == (3.0, 1.0, 0.5, "map")
        succeeded = [e for e in events if e["data"]["status"] == "succeeded"]
        assert len(succeeded) == 1 and succeeded[0]["data"]["x"] == 3.0
    finally:
        ctl.stop()


def test_cancel_and_validation():
    events, emit = recorder()
    nav = FakeNav()
    nav.block = True
    ctl = PointNavigationController(None, emit, nav_factory=lambda _: nav)
    try:
        with pytest.raises(ValueError):
            ctl.submit({"x": math.inf, "y": 0})
        with pytest.raises(ValueError):
            ctl.submit({"x": 0, "y": 0, "frame_id": "odom"})
        ctl.submit({"x": 1, "y": 2})
        wait_for(lambda: any(e["data"]["status"] == "active" for e in events))
        ctl.cancel("operator_stop")
        assert events[-1]["data"]["status"] == "canceled"
        assert events[-1]["data"]["reason"] == "operator_stop"
    finally:
        ctl.stop()
```

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python -m pytest web/test_point_navigation.py -q
```

Expected: collection fails because `nx_point_nav.py` does not exist.

- [ ] **Step 3: Implement the controller**

Create `web/nx_point_nav.py`:

```python
"""Serialized Panel point goals for Nav2 NavigateToPose."""

import math
import threading

from nx_room_orchestrator import Nav2ActionClient


class PointNavigationController:
    def __init__(self, node, broadcast_fn, nav_factory=Nav2ActionClient):
        self._broadcast = broadcast_fn
        self._nav = nav_factory(node)
        self._cv = threading.Condition()
        self._generation = 0
        self._pending = None
        self._active_generation = None
        self._last_goal = None
        self._running = True
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    @staticmethod
    def _normalize(payload):
        frame_id = str(payload.get("frame_id", "map"))
        if frame_id != "map":
            raise ValueError("frame_id must be map")
        try:
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("x, y and yaw must be numbers") from exc
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("x, y and yaw must be finite")
        return {"x": x, "y": y, "yaw": yaw, "frame_id": "map"}

    def _emit(self, status, goal, **extra):
        data = dict(goal)
        data["status"] = status
        data.update(extra)
        self._broadcast({"type": "nav_goal", "data": data}, force=True)

    def submit(self, payload):
        goal = self._normalize(payload)
        with self._cv:
            had_active = self._active_generation is not None
            self._generation += 1
            generation = self._generation
            self._pending = (generation, goal)
            self._last_goal = goal
            self._cv.notify_all()
        if had_active:
            self._nav.cancel_current()
        self._emit("pending", goal, generation=generation)
        return {"ok": True, "generation": generation, "goal": goal}

    def cancel(self, reason="canceled"):
        with self._cv:
            self._generation += 1
            self._pending = None
            self._active_generation = None
            goal = dict(self._last_goal or {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map"})
            self._cv.notify_all()
        self._nav.cancel_current()
        self._emit("canceled", goal, reason=str(reason), generation=self._generation)

    def _worker(self):
        while True:
            with self._cv:
                while self._running and self._pending is None:
                    self._cv.wait()
                if not self._running:
                    return
                generation, goal = self._pending
                self._pending = None
                self._active_generation = generation
            self._emit("active", goal, generation=generation)
            result = self._nav.send_goal_and_wait(
                goal["x"], goal["y"], goal["yaw"], frame_id=goal["frame_id"]
            )
            with self._cv:
                if generation != self._generation:
                    continue
                self._active_generation = None
            if result.get("ok"):
                self._emit("succeeded", goal, generation=generation, result=result)
            elif result.get("reason") == "cancelled":
                self._emit("canceled", goal, generation=generation, result=result)
            else:
                self._emit(
                    "failed", goal, generation=generation,
                    reason=result.get("reason", "aborted"), result=result,
                )

    def stop(self):
        self.cancel("shutdown")
        with self._cv:
            self._running = False
            self._cv.notify_all()
        self._worker_thread.join(timeout=2.0)
```

- [ ] **Step 4: Verify GREEN**

Run:

```powershell
python -m pytest web/test_point_navigation.py -q
python -m py_compile web/nx_point_nav.py
```

Expected: 3 tests pass.

### Task 5: Wire map-frame pose and point goals into the NX web server

**Files:**
- Create: `web/test_panel_navigation_contract.py`
- Modify: `web/nx_web_server.py`
- Modify: `docker/deploy_nx_web.sh`

- [ ] **Step 1: Write the failing wiring contract**

Create `web/test_panel_navigation_contract.py`:

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def test_web_uses_map_frame_localization_pose():
    server = read("web/nx_web_server.py")
    assert "'/localization_pose', self._on_odom, 10" in server
    assert "'/odom', self._on_odom, 10" not in server


def test_web_exposes_point_navigation_and_cancels_on_stop():
    server = read("web/nx_web_server.py")
    assert "from nx_point_nav import PointNavigationController" in server
    assert "p.path == '/api/navigate'" in server
    assert "point_nav.submit" in server
    assert 'point_nav.cancel("operator_stop")' in server
    assert 'point_nav.cancel("emergency_stop")' in server


def test_web_deployer_copies_point_navigation_module():
    deploy = read("docker/deploy_nx_web.sh")
    assert 'web/nx_point_nav.py' in deploy
```

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python -m pytest web/test_panel_navigation_contract.py -q
```

Expected: all three tests fail.

- [ ] **Step 3: Integrate the controller and localization topic**

In `web/nx_web_server.py`, import the controller:

```python
from nx_point_nav import PointNavigationController
```

Replace the `/odom` subscription with:

```python
self.create_subscription(Odometry, '/localization_pose', self._on_odom, 10)
```

Add the global and update `main()`'s global declaration:

```python
point_nav = None

def main():
    global robot, task_mgr, node, room_orchestrator, point_nav
```

After constructing `NxWebNode`, create:

```python
point_nav = PointNavigationController(node, ws_broadcast)
```

In `do_POST`, add this route before `/api/command`:

```python
            elif p.path == '/api/navigate':
                try:
                    payload = json.loads(body or "{}")
                    if not isinstance(payload, dict):
                        raise ValueError("JSON body must be an object")
                    self._json(point_nav.submit(payload))
                except (ValueError, TypeError, json.JSONDecodeError) as exc:
                    self.send_response(400)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    data = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False).encode()
                    self.send_header('Content-Length', len(data))
                    self.end_headers()
                    self.wfile.write(data)
```

Update stop handlers:

```python
            elif p.path == '/api/stop':
                point_nav.cancel("operator_stop")
                robot.stop_move()
                self._json({"ok": True})
            elif p.path == '/api/e_stop':
                point_nav.cancel("emergency_stop")
                robot.e_stop()
                task_mgr.cancel_all()
                self._json({"ok": True})
```

In `main()`'s `finally`, before shutting down rclpy, add:

```python
        try:
            if point_nav is not None:
                point_nav.stop()
        except Exception:
            pass
```

- [ ] **Step 4: Deploy the new module with web**

Add to `docker/deploy_nx_web.sh`:

```bash
scp -q "$WS_DIR/web/nx_point_nav.py" "$NX_USER@$NX_HOST:~/go2w_ws/web/"
```

- [ ] **Step 5: Verify GREEN**

Run:

```powershell
python -m pytest web/test_panel_navigation_contract.py web/test_point_navigation.py -q
python -m py_compile web/nx_web_server.py web/nx_point_nav.py
```

Expected: all tests pass and compilation exits 0.

### Task 6: Add Panel click-to-go interaction

**Files:**
- Modify: `web/static/map.js`
- Modify: `web/static/panel.html`
- Modify: `web/test_map_contract.js`
- Modify: `web/test_panel_navigation_contract.py`

- [ ] **Step 1: Extend the JS harness and write failing interaction tests**

Change `FakeCanvas` in `web/test_map_contract.js` to store listeners:

```javascript
  constructor(ctx) {
    this.clientWidth = 800;
    this.clientHeight = 600;
    this.width = 0;
    this.height = 0;
    this._ctx = ctx;
    this.handlers = {};
  }
  getContext() { return this._ctx; }
  addEventListener(type, callback) { this.handlers[type] = callback; }
  emit(type, x, y) { this.handlers[type]({ clientX: x, clientY: y }); }
  getBoundingClientRect() { return { left: 0, top: 0 }; }
```

Change `createMap()` to accept options:

```javascript
function createMap(opts = {}) {
  const ctx = new FakeContext();
  const canvas = new FakeCanvas(ctx);
  const Go2WMap = loadMapClass();
  const map = new Go2WMap(canvas, opts);
  map._resize();
  return { map, ctx, canvas };
}
```

Append:

```javascript
function testShortClickSelectsWorldGoalButDragSelectsRegion() {
  const goals = [], regions = [];
  const { map, canvas } = createMap({
    onSelectGoal: goal => goals.push(goal),
    onSelectRegion: region => regions.push(region),
  });
  map.update({ x: 0, y: 0, trail: [[0, 0]], scan: [], map: [] });
  map._draw();

  canvas.emit('mousedown', 500, 300);
  canvas.emit('mouseup', 500, 300);
  assert.strictEqual(goals.length, 1);
  assert.strictEqual(regions.length, 0);
  assert(near(goals[0].x, map.screenToWorld(500, 300).x));

  canvas.emit('mousedown', 300, 250);
  canvas.emit('mousemove', 420, 360);
  canvas.emit('mouseup', 420, 360);
  assert.strictEqual(goals.length, 1);
  assert.strictEqual(regions.length, 1);
}


function testGoalStateRendersAndParticipatesInBounds() {
  const { map, ctx } = createMap();
  map.update({ x: 0, y: 0, trail: [[0, 0]], scan: [], map: [] });
  map.setNavGoal({ x: 22, y: -4, yaw: 0, status: 'active' });
  map._draw();
  assert(map._tf.maxX >= 22);
  const targetLabel = ctx.ops.find(op => op.type === 'fillText' && op.text === '目标');
  assert(targetLabel, 'expected active navigation goal marker');
}
```

Call both tests before the final `console.log`.

Append to `web/test_panel_navigation_contract.py`:

```python
def test_panel_posts_clicked_map_goal_and_consumes_status():
    panel = read("web/static/panel.html")
    assert "onSelectGoal" in panel
    assert "fetch('/api/navigate'" in panel
    assert "frame_id: 'map'" in panel
    assert "data.type === 'nav_goal'" in panel
    assert "map.setNavGoal" in panel
```

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
node web/test_map_contract.js
python -m pytest web/test_panel_navigation_contract.py -q
```

Expected: JS fails because goal interaction methods do not exist; Python fails because Panel has no navigation wiring.

- [ ] **Step 3: Implement map goal selection and rendering**

In `Go2WMap`'s constructor add:

```javascript
this.onSelectGoal = opts.onSelectGoal || null;
this.navGoal = null;
```

Add:

```javascript
setNavGoal(goal) {
  this.navGoal = goal ? { ...goal } : null;
  this._tf = null;
}
```

Include the goal in `_computeTransform()`:

```javascript
if (this.navGoal) {
  allX.push(Number(this.navGoal.x));
  allY.push(Number(this.navGoal.y));
}
```

Replace the small-drag branch in `endDrag()` with:

```javascript
      if (Math.abs(a.x - b.x) < 8 && Math.abs(a.y - b.y) < 8) {
        const goal = this.screenToWorld(a.x, a.y);
        this.setNavGoal({ ...goal, status: 'selected' });
        if (this.onSelectGoal) this.onSelectGoal(goal);
        this._dragStart = null;
        this._dragCur = null;
        return;
      }
```

Before drawing the robot, render the goal:

```javascript
    if (this.navGoal) {
      const gx = toX(this.navGoal.x), gy = toY(this.navGoal.y);
      const done = this.navGoal.status === 'succeeded';
      const failed = this.navGoal.status === 'failed' || this.navGoal.status === 'canceled';
      ctx.strokeStyle = done ? '#4caf50' : (failed ? '#ff5252' : '#ffd54f');
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(gx, gy, 8, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx - 11, gy); ctx.lineTo(gx + 11, gy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx, gy - 11); ctx.lineTo(gx, gy + 11); ctx.stroke();
      ctx.fillStyle = ctx.strokeStyle;
      ctx.font = '11px sans-serif';
      ctx.fillText('目标', gx + 12, gy - 8);
    }
```

- [ ] **Step 4: Wire Panel to the API and WebSocket**

In `initMap()` add this option beside `onSelectRegion`:

```javascript
    onSelectGoal: (goal) => sendNavGoal(goal),
```

Add:

```javascript
function sendNavGoal(goal) {
  if (!map) return;
  const dx = goal.x - map.slam.robotX;
  const dy = goal.y - map.slam.robotY;
  const payload = { x: goal.x, y: goal.y, yaw: Math.atan2(dy, dx), frame_id: 'map' };
  map.setNavGoal({ ...payload, status: 'pending' });
  fetch('/api/navigate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  }).then(async response => {
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || '导航目标发送失败');
  }).catch(error => {
    map.setNavGoal({ ...payload, status: 'failed', reason: error.message });
    console.error('[导航]', error);
  });
}
```

Handle WebSocket status before `status`:

```javascript
      } else if (data.type === 'nav_goal') {
        if (map && data.data) map.setNavGoal(data.data);
        console.log('[导航目标]', data.data || {});
```

Update map hints from “拖框选区” to “单击导航 · 拖框选区”.

- [ ] **Step 5: Verify GREEN**

Run:

```powershell
node web/test_map_contract.js
python -m pytest web/test_panel_navigation_contract.py web/test_point_navigation.py -q
```

Expected: JS prints `map contract tests passed`; all Python tests pass.

### Task 7: Run the complete local regression suite

**Files:**
- Verify all modified files

- [ ] **Step 1: Run focused tests**

Run:

```powershell
python -m pytest docker/test_livox_deploy_contract.py docker/test_map_odom_fuser_performance_contract.py web/test_point_navigation.py web/test_panel_navigation_contract.py -q
node web/test_map_contract.js
```

Expected: all tests pass.

- [ ] **Step 2: Run existing web and Docker regressions**

Run:

```powershell
python -m pytest web -q
python -m pytest docker -q
```

Expected: zero failures. Existing hardware-only tests may be explicitly skipped, not failed.

- [ ] **Step 3: Compile and check the diff**

Run:

```powershell
python -m py_compile src/go2w_bridge/go2w_bridge/map_odom_fuser.py src/go2w_nav/launch/nav2_3d.launch.py web/nx_point_nav.py web/nx_web_server.py
git diff --check
git status --short
```

Expected: compilation and whitespace check exit 0; status lists only intended files plus pre-existing unrelated user changes.

### Task 8: Deploy to NX and verify the real robot end to end

**Files:**
- Deploy the exact local artifacts listed in the file structure
- Runtime evidence: NX systemd, ROS graph, TF, topics, costmaps, action results and Panel

- [ ] **Step 1: Back up and deploy exact artifacts**

Run the two reviewed deployment scripts from Git Bash; `deploy_nav2_bprime.sh` creates timestamped backups before copying fuser, Nav2 config/launch, bringup/profile/service and both install/share copies, while `deploy_nx_web.sh` copies the point-navigation module, web server, `map.js` and `panel.html`:

```powershell
$gitBash = "C:\Program Files\Git\bin\bash.exe"
& $gitBash -lc "cd /c/Users/ROG/yangyuhui/DOGS/go2w_search_ws && NX_HOST=192.168.1.104 bash docker/deploy_nav2_bprime.sh"
if ($LASTEXITCODE -ne 0) { throw "Nav2 deployment failed" }
& $gitBash -lc "cd /c/Users/ROG/yangyuhui/DOGS/go2w_search_ws && NX_HOST=192.168.1.104 NX_USER=nx bash docker/deploy_nx_web.sh"
if ($LASTEXITCODE -ne 0) { throw "Web deployment failed" }
```

Expected: both scripts exit 0, print their backup timestamp, and preserve mode `775` on `bringup_slam_nav2.sh`.

- [ ] **Step 2: Install and start persistent services**

Run on NX:

```bash
sudo systemctl daemon-reload
sudo systemctl enable go2w-slam-nav.service
sudo systemctl restart go2w-slam-nav.service
sudo systemctl restart go2w-web.service
systemctl is-active livox-mid360-driver go2w-sensor go2w-motion go2w-web go2w-slam-nav fastlio map-odom-fuser nav2-3d
```

Expected: all units report `active`.

- [ ] **Step 3: Verify localization and the 20° compensation**

Run with the ROS environment sourced:

```bash
timeout 8 ros2 topic hz /livox/lidar
timeout 8 ros2 topic hz /livox/imu
timeout 8 ros2 topic hz /Odometry
timeout 8 ros2 topic hz /localization_pose
timeout 5 ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
ps -eo pid,pcpu,pmem,args --sort=-pcpu | head -20
```

Require lidar about 10 Hz, IMU about 200 Hz, odometry/localization continuous, `map -> odom -> base_link` single-chain, static roll/pitch under 1 degree, and fuser CPU under 20%.

- [ ] **Step 4: Verify Nav2 lifecycle and obstacle costmaps**

Run:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
ros2 action list | grep /navigate_to_pose
ros2 topic info /scan -v
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
```

Require every lifecycle node active, action present, `/scan` publisher/subscribers compatible, and both costmaps containing non-zero obstacle/inflation cells.

- [ ] **Step 5: Run the bounded rotation gate**

Run on NX with the ROS environment sourced:

```bash
ros2 topic echo /Odometry --once > /tmp/rotation_fastlio_before.yaml
ros2 topic echo /localization_pose --once > /tmp/rotation_map_before.yaml
timeout 5 ros2 run tf2_ros tf2_echo odom base_link > /tmp/rotation_wheel_before.txt 2>&1 || true
timeout 8 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic echo /Odometry --once > /tmp/rotation_fastlio_after.yaml
ros2 topic echo /localization_pose --once > /tmp/rotation_map_after.yaml
timeout 5 ros2 run tf2_ros tf2_echo odom base_link > /tmp/rotation_wheel_after.txt 2>&1 || true
```

Compare the captured translations and quaternions. Require approximately 45 degrees of wheel-odometry yaw, FastLIO translation drift below 1 m, map-frame roll/pitch below 2 degrees and continuous TF.

- [ ] **Step 6: Run a 1 m CLI navigation gate**

Run:

```bash
bash /home/nx/go2w_ws/diagnose_nav2_goal.sh 1 0 90
```

Require accepted/succeeded action, positive `/cmd_vel.linear.x`, approximately 1 m real `odom -> base_link` displacement and stable FastLIO.

- [ ] **Step 7: Verify obstacle avoidance**

In Panel, use the red costmap overlay to identify the nearest static obstacle cluster, then click a free target approximately 2 m beyond that cluster so the straight robot-to-goal segment crosses red cells but at least one side has a free corridor. Capture the run on NX:

```bash
timeout 120 ros2 topic echo /plan > /tmp/avoid_plan.yaml & P1=$!
timeout 120 ros2 topic echo /cmd_vel > /tmp/avoid_cmd_vel.yaml & P2=$!
timeout 120 ros2 topic echo /localization_pose > /tmp/avoid_localization.yaml & P3=$!
timeout 120 ros2 topic echo /local_costmap/costmap > /tmp/avoid_local_costmap.yaml & P4=$!
```

After Panel reports the result, run `kill "$P1" "$P2" "$P3" "$P4" 2>/dev/null || true` and inspect the captured global path against occupied cells. Require the path to bend around the cluster, physical clearance to exceed the inflated footprint, continuous localization, and `SUCCEEDED` without contact.

- [ ] **Step 8: Verify Panel click-to-go in the real browser**

Open `http://192.168.1.104:8000`, confirm the map pose matches `/localization_pose`, click a reachable point, and observe `pending -> active -> succeeded`. Verify the HTTP payload uses frame `map`, `/navigate_to_pose` receives the same coordinates, the robot moves around obstacles, and the target marker turns green at arrival.

- [ ] **Step 9: Verify reboot persistence**

After publishing zero velocity and confirming the robot is stationary, reboot and poll SSH from the PC:

```powershell
ssh nx@192.168.1.104 "source /opt/ros/humble/setup.bash; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'; sudo reboot"
Start-Sleep -Seconds 20
for ($i = 0; $i -lt 30; $i++) {
  ssh -o ConnectTimeout=3 nx@192.168.1.104 "true" 2>$null
  if ($LASTEXITCODE -eq 0) { break }
  Start-Sleep -Seconds 5
}
ssh nx@192.168.1.104 "systemctl is-active livox-mid360-driver go2w-sensor go2w-motion go2w-web go2w-slam-nav fastlio map-odom-fuser nav2-3d"
```

Repeat Steps 3 and 4, then send one short Panel goal. Require every unit active and no manual bringup command.

- [ ] **Step 10: Run fresh local verification after deployment**

Run:

```powershell
python -m pytest docker/test_livox_deploy_contract.py docker/test_map_odom_fuser_performance_contract.py web/test_point_navigation.py web/test_panel_navigation_contract.py -q
node web/test_map_contract.js
git diff --check
```

Expected: all tests pass and no whitespace errors exist.
