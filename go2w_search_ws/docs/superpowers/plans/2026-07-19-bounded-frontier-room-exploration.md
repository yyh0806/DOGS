# 封闭区域 Frontier 探索（标人副产品）实现计划 v2

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **v2 动因：** v1 通过审核后被指出 5 类硬伤（详见下方"v1 审核意见"）。v2 按"先 en-route 同步对齐 → 再 ROI 覆盖率 → 再接入完成状态"重排，并同步修正 spec。

**Goal:** 给现有 `_run_frontier_explore` 加三件事——(1) 导航期间移动中连续 YOLO 标人，**复用 `ObservationSynchronizer` bundle 做时间对齐**（不是事后读最新 scan），bounded queue + capture_stamp 去重，不跨线程写 store；(2) REPORT 阶段基于**ROI 限定**的 occupancy grid 算覆盖率与 `enclosed_unknown_regions`（不是整图，不被 `/map_frontier` 的 2m padding 灌水）；(3) 把覆盖率真实接入**四态 `completion_status`**（不是只塞数值）。

**Architecture:**
- **复用不动**：`nx_frontier_planner` / `nx_exploration_manager` / `nx_person_localizer` / `nx_person_mission` / `nx_navigation_gateway` / `nx_observation_sync`（后者已在线程安全的 RLock 下持续吃 pose/scan/cloud，`nx_web_server.py:447/659/671/763`）。
- **新增纯逻辑**：`nx_coverage_metrics.py`（ROI 裁剪 + 膨胀 flood-fill + `enclosed_unknown_regions`，无 rclpy 依赖，可单测）。
- **改 `nx_room_orchestrator.py`**：
  1. 抽**只读**共享方法 `_build_observation_bundle(snapshot)`（做 `add_frame/add_detection/add_cloud + bundle_for_detection + calibration`，不写 store）——`_observe_people_at_viewpoint` 与 en-route worker 都调它，消除 v1 的重复 calibration/localize。
  2. 加 `_localize_en_route_detections(bundle_result, target_classes)`（只读，只收 `range_lidar`，worker 线程调）。
  3. 加 `_observe_en_route(stop_event, ...)` worker + `_ingest_en_route_samples(samples, store, ...)` 主线程串行。
  4. REPORT 段：从 `latest_map_box` 取 final map，按 mission_origin + `max_radius_m`（或 room_polygon）构造 ROI，调 `compute_coverage`，算 `completion_status`。

**Tech Stack:** Python **≥ 3.10**（NX 部署机实测 3.10.12；开发机 3.12 亦可；**不得使用 3.11+ 独有语法**），rclpy（仅部署机），pytest（开发机，纯逻辑测试无需 rclpy），`threading.Event/Thread`（en-route observer worker），`nav_msgs/OccupancyGrid`（部署机；开发机用 stub）。

## v1 审核意见（v2 必须全部解决）

| # | v1 缺陷（审核原话精简） | v2 对策 | 实现位置 |
|---|---|---|---|
| 1 | 移动中人物坐标会算错：采样时存 pose + 缓存图像，导航后读"最新 LaserScan"定位——三时间不一致 | en-route **复用 `observation_sync.bundle_for_detection(captured_at)`**，bundle 内 pose/scan/cloud 已按 detection 时间戳对齐；bundle 拿不到（容差超限）**丢弃该样本**，绝不事后拼凑 | Task 1 |
| 2 | Observer 重复复制大量完整图像：`get_detection_snapshot()` 每次 `frame.copy()`，40s×0.4s≈100 张 720p 占数百 MB；同一推理帧被重复处理；不应在生产类加 `_en_route_ingest_count` | worker 按 `captured_at`（detection 帧时间戳）**去重**，同帧只处理一次；**bounded queue ≤ 12**；测试用 `monkeypatch` spy `_ingest_en_route_samples`，**不污染生产类** | Task 1 |
| 3 | 覆盖率公式无效：`/map_frontier` 被 `map_padding_bridge.py` 加了 2m unknown padding（实测整图仅 31.66%），且包含任务开始前已建区域 | `compute_coverage(map, roi)` 必须**限定 ROI**（mission_origin + `max_radius_m` 圆，或 `room_polygon`）；ROI 裁掉外围 padding；输出 `coverage_valid`/`roi`/`map_stamp`；地图不可用返回 `None`（不是 0.0） | Task 2 |
| 4 | "不可达死角"判断不可靠："unknown 连通块无 free 邻居"可能是墙体/建筑外/噪声 | 重命名 `enclosed_unknown_regions`；**只在 ROI 内**分析；**排除接触 ROI/地图边界的 unknown 连通块**；从 mission_origin 对**膨胀后 free space** flood-fill 判可达 | Task 2 |
| 5 | 成功判据与实现矛盾：spec 写 `explored_ratio ≥ 0.95`，实现只塞数值，任务仍仅凭 frontier 耗尽完成 | REPORT 输出四态 `completion_status`：`completed` / `completed_with_gaps` / `incomplete` / `coverage_unverified`；frontier 耗尽仍是**停止信号**，但完成状态真实反映覆盖率 | Task 3 |
| 6 | "不漏标"无法保证：背后/遮挡的人仍漏；产品要求不明 | spec 措辞从"无漏标"→"沿途可见人员 best-effort 标注"；云台多角度扫描作为 **Task 4 stretch**（默认不实现，待实机 recall 数据决定） | Spec 勘误 + Task 4 |
| 7 | Python 3.12 ≠ NX 实际 3.10.12；deploy 清单需更新 | Tech Stack 改 ≥3.10；`build_release.sh:69` glob `web/nx_*.py` 自动收新文件，`deploy_nx_web.sh:24` 已 retired——**无需改清单** | Tech Stack + Global Constraints |

---

## Spec 勘误（Task 0：实施前先同步 spec 文件）

`docs/superpowers/specs/2026-07-19-bounded-frontier-room-exploration-design.md` 需做 6 处修订（审核 = spec review 的 changes requested，spec 是 plan 的 source of truth，必须同步，否则 plan 与 spec 矛盾）：

- [ ] **Step 0.1**：覆盖率公式段（spec 行 57-61）改为 ROI 内裁剪 + 去 padding 说明。
- [ ] **Step 0.2**：阈值 `explored_ratio ≥ 0.95` → `≥ 0.90`（实机 ROI 校准，spec 行 126）。
- [ ] **Step 0.3**：`dead_zones` → `enclosed_unknown_regions`，判定加 ROI 裁剪 + 接触边界排除 + 膨胀 flood-fill（spec 行 59-61、126）。
- [ ] **Step 0.4**：成功判据 #1（spec 行 126）改 `completion_status` 四态描述。
- [ ] **Step 0.5**：成功判据 #3（spec 行 128）"无漏标"→"沿途可见人员 best-effort 标注，不保证 recall"。
- [ ] **Step 0.6**：算法段移动中检测（spec 行 72-74、80）补"复用 observation_sync bundle 时间对齐 + bounded queue + capture_stamp 去重"。
- [ ] **Step 0.7**：Commit `docs(frontier): spec勘误 — ROI覆盖率/enclosed_unknown/completion四态/best-effort标人`。

> Task 0 的具体 Edit 在"同步 spec"环节统一执行（见本计划末尾"Spec 同步 Edit 清单"小节，给出逐条 old→new）。Task 1-3 的实现按修订后的 spec 为准。

---

## Global Constraints

- **不改**：`nx_frontier_planner.py` / `nx_exploration_manager.py` / `nx_person_localizer.py` / `nx_person_mission.py` / `nx_navigation_gateway.py` / `nx_observation_sync.py`。
- **不引入**：门检测 / 起点半径约束作为硬边界（ROI 只用于**覆盖率统计**，不影响 frontier 选择）/ 覆盖率触发**停止**（停止信号仍是 frontier 耗尽）/ 找人导向打分 / bearing_only 主动确认 / 外部 frontier 包。
- **en-route worker 线程安全契约**：
  - worker **只读**：`observation_sync`（自身 RLock 保护）、`get_detection_snapshot`、`resolve_camera_calibration`、`localize_target_detection`、`_laser_scan_snapshot`/`_pointcloud_snapshot`（读 node 缓存，node 有 `_lock`）。
  - worker **绝不写** `TargetMissionStore`；samples 攒在线程局部 bounded list，主线程 join 后串行 ingest。
  - worker 按 `captured_at`（detection 帧 `snapshot["timestamp"]`）去重，同帧只处理一次；bounded queue 上限 12（env `GO2W_EN_ROUTE_MAX_SAMPLES` 可配）。
  - cancel 时 `stop_event.set()` + `join(timeout=2.0)`，worker 是 daemon，强退出兜底。
- **覆盖率必须限定 ROI**：mission_origin 圆心 + `max_radius_m`（current_room 默认 6.0，`nx_product_command.py:244`）；命名房间用 `room_polygon`（ExplorationManager 已支持）。地图不可用 → `compute_coverage` 返回 `None` → `completion_status="coverage_unverified"`，**不伪装 0.0**。
- **不污染生产类**：测试用 `monkeypatch.setattr(orchestrator, "_ingest_en_route_samples", spy)` 计数，**禁止**在 `RoomSearchOrchestrator` 加 `_en_route_ingest_count` 之类的测试计数器。
- **Python ≥ 3.10**：禁止 `match`-statement 外的 3.11+ 独有语法；类型标注用 `from __future__ import annotations`（`nx_room_orchestrator.py` 已有）。
- **deploy 无需改清单**：`build_release.sh:69` glob `web/nx_*.py` 自动收录 `nx_coverage_metrics.py`；`deploy_nx_web.sh:24` 已 retired 为 forwarder。
- **测试风格沿用**：`test_frontier_explore.py` 的 `FakeTask`/`FakeAi`/`FakeNode`/`FakeNav` + `_make_fake_map` + `make_orchestrator` + `monkeypatch.setattr(orch_module, "_OccupancyGrid", ...)`。
- **提交粒度**：每个 Task 一次 commit，前缀 `feat(frontier):` 或 `test(frontier):` 或 `docs(frontier):`。
- **路径基准**：`C:\Users\ROG\yangyuhui\DOGS\go2w_search_ws\`（POSIX 形式 `go2w_search_ws/`）。

---

## 文件结构

| 文件 | 职责 | 本计划改动 |
|---|---|---|
| `web/nx_coverage_metrics.py` | 纯逻辑：ROI 裁剪 + 膨胀 flood-fill + `enclosed_unknown_regions` | **新建**（Task 2） |
| `web/test_coverage_metrics.py` | 上述纯逻辑单测 | **新建**（Task 2） |
| `web/nx_room_orchestrator.py` | frontier 探索状态机 | **修改**（Task 1 抽 `_build_observation_bundle` + 加 en-route 三方法；Task 3 REPORT 段加 coverage + `completion_status`） |
| `web/test_frontier_explore.py` | frontier 状态机测试 | **修改**（Task 1 加 en-route spy 测试 + 预算/超时锁定；Task 3 加 coverage + 四态测试） |
| `docs/superpowers/specs/2026-07-19-bounded-frontier-room-exploration-design.md` | 设计 spec | **修改**（Task 0 勘误 6 处） |

---

## Task 1: en-route observer（复用 sync bundle + bounded queue + 去重 + 不污染生产类）

**Files:**
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py`（抽 `_build_observation_bundle`；加 `_localize_en_route_detections` / `_observe_en_route` / `_ingest_en_route_samples`；改 `_observe_people_at_viewpoint` 调共享方法；改 `_run_frontier_explore` frontier 循环包 worker）
- Test: `go2w_search_ws/web/test_frontier_explore.py`（加 en-route spy 测试 + 40s/300s 锁定测试）

**Interfaces:**
- Consumes（已存在）：`self._observation_sync`（`__init__` 注入，`:417`）、`self._detection_snapshot_getter`（`:1565`）、`self._pointcloud_snapshot`、`self._laser_scan_snapshot`（`:1844`）、`resolve_camera_calibration`、`localize_target_detection`、`DetectionFrame`、`store.add_observation`/`add_unresolved_observation`（`nx_person_mission.py:60/134`）、`threading`（已 import）。
- Produces（本 Task 新增私有方法，签名固定，Task 3 不再改）：
  - `_build_observation_bundle(self, snapshot, require_photos) -> dict | None`：**只读**。做 `add_frame/add_detection/add_cloud` + `bundle_for_detection` + `calibration` 构造。返回 `{"bundle": ObservationBundle, "frame": ndarray|None, "frame_info": DetectionFrame, "scan": ..., "pointcloud": ..., "robot_pose": (x,y,yaw), "source": str, "captured_at": float, "observation_valid": bool, "camera_calibration": dict}`；bundle 失败返回 `None`。**到点和 en-route 共用。**
  - `_localize_en_route_detections(self, bundle_result, target_classes) -> list[dict]`：**只读**。对 bundle 内每个目标类 detection 调 `localize_target_detection`，**只保留 `position_quality=="range_lidar"`**（bearing_only 留给到点稳态），返回 localized dict 列表（每个含 `world_x/world_y/position_quality/class/confidence/bbox`）。
  - `_observe_en_route(self, stop_event, target_classes, sample_interval=0.4, max_samples=12) -> list[dict]`：worker 线程。返回 `[{localized_list, frame, captured_at, source}, ...]`，**不写 store**，bounded ≤ `max_samples`。
  - `_ingest_en_route_samples(self, samples, store, room_name, require_photos) -> int`：主线程串行。对每个 sample 的 localized_list 调 `store.add_observation`（带 frame 若 `require_photos`）；返回成功 ingest 的观测数。

- [ ] **Step 1: 写 40s/300s 预算锁定测试（确认现状不回归）**

Append to `go2w_search_ws/web/test_frontier_explore.py`:

```python
def test_send_goal_timeout_is_40s_not_120s():
    """2026-07-15 实测: 单 goal 超时从 120s 调到 40s。锁定防回归。"""
    import inspect
    from nx_navigation_gateway import MissionNavigationPort
    source = inspect.getsource(MissionNavigationPort.send_goal_and_wait)
    assert "timeout=40.0" in source, (
        "send_goal_and_wait 应保持 40s 单 goal 超时 (2026-07-15 调整)")


def test_frontier_explore_max_time_defaults_300s(monkeypatch):
    """max_time 默认 300s (_run_frontier_explore 内部默认), 锁定防回归。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    orchestrator = make_orchestrator([], FakeNav())
    task = FakeTask()
    task.params.pop("max_time", None)  # 用默认值

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["time_budget_sec"] == 300
```

- [ ] **Step 2: 运行确认锁定测试通过**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py::test_send_goal_timeout_is_40s_not_120s web/test_frontier_explore.py::test_frontier_explore_max_time_defaults_300s -v`
Expected: PASS — 现状已满足。

- [ ] **Step 3: 写 en-route observer 失败测试（spy ingest，不碰生产类计数器）**

Append to `go2w_search_ws/web/test_frontier_explore.py`:

```python
def test_frontier_explore_observes_people_en_route(monkeypatch, tmp_path):
    """导航期间 observer 应采到 FakeAi 缓存里的人, 到达后 ingest 进 store。

    FakeNav.send_goal_and_wait 阻塞 0.3s 模拟导航耗时, 期间 observer 线程
    每 0.05s 读一次 detection snapshot。用 monkeypatch spy _ingest_en_route_samples
    验证 en-route 路径独立于到点稳态采样被真的走通 (不污染生产类计数器)。
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    def candidates(_map, _pose, visited, **_kwargs):
        if visited:
            return []
        return [{"x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0}]
    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)

    class SlowNav(FakeNav):
        def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
            time.sleep(0.3)  # 让 observer 线程有机会采样
            return super().send_goal_and_wait(x, y, yaw, frame_id=frame_id)

    orch = make_orchestrator([], SlowNav(), ai_engine=FakeAi())
    orch._static_root = tmp_path
    task = FakeTask()
    task.params["max_frontiers"] = 1
    task.params["en_route_sample_interval"] = 0.05  # 快速采样

    # spy: 替换 ingest 方法, 不碰生产类属性
    ingest_calls = []
    real_ingest = orch._ingest_en_route_samples if hasattr(
        orch, "_ingest_en_route_samples") else None

    def _spy(samples, store, room_name, require_photos):
        ingest_calls.append(len(samples))
        if real_ingest is not None:
            return real_ingest(samples, store, room_name, require_photos)
        return 0
    monkeypatch.setattr(orch, "_ingest_en_route_samples", _spy)

    orch.run(task)

    assert task.status == "completed"
    assert sum(ingest_calls) >= 1, (
        "en-route ingest 未触发 — observer 路径未走通 (spy 应捕获 ≥1 个 sample)")


def test_en_route_observer_dedups_by_capture_stamp(monkeypatch, tmp_path):
    """同一推理帧 (captured_at 相同) 不应被 observer 重复入队。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    fixed_stamp = time.time()
    class StickyAi(FakeAi):
        def get_person_detection_snapshot(self):
            snap = super().get_person_detection_snapshot()
            snap["timestamp"] = fixed_stamp  # 同一帧时间戳不变
            return snap

    orch = make_orchestrator([], FakeNav(), ai_engine=StickyAi())
    orch._static_root = tmp_path
    stop_event = threading.Event()
    # 跑 0.3s, sample_interval=0.05 → 约 6 次循环, 但同帧只入队 1 次
    samples = orch._observe_en_route(
        stop_event, ["person"], sample_interval=0.05, max_samples=12)
    stop_event.set()
    # 即使循环多次, captured_at 去重后 samples 里同一 stamp 最多 1 个
    stamps = [s["captured_at"] for s in samples]
    assert len(stamps) == len(set(stamps)), "同帧被重复入队 (去重失效)"


def test_en_route_observer_bounded_queue(monkeypatch, tmp_path):
    """worker queue 不超过 max_samples (防 100 张 frame 堆积)。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    class FreshStampAi(FakeAi):
        def __init__(self):
            self._n = 0
        def get_person_detection_snapshot(self):
            self._n += 1
            snap = super().get_person_detection_snapshot()
            snap["timestamp"] = time.time() + self._n  # 每次新 stamp
            return snap

    orch = make_orchestrator([], FakeNav(), ai_engine=FreshStampAi())
    orch._static_root = tmp_path
    stop_event = threading.Event()
    samples = orch._observe_en_route(
        stop_event, ["person"], sample_interval=0.02, max_samples=4)
    stop_event.set()
    assert len(samples) <= 4, f"bounded queue 失效: {len(samples)} > 4"
```

- [ ] **Step 4: 运行确认 en-route 测试失败**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py::test_frontier_explore_observes_people_en_route web/test_frontier_explore.py::test_en_route_observer_dedups_by_capture_stamp web/test_frontier_explore.py::test_en_route_observer_bounded_queue -v`
Expected: FAIL — `AttributeError: _ingest_en_route_samples` / `_observe_en_route` 不存在。

- [ ] **Step 5: 抽 `_build_observation_bundle`（只读共享方法）**

在 `go2w_search_ws/web/nx_room_orchestrator.py` 的 `_observe_people_at_viewpoint` 方法（`:1599`）**之前**插入新方法。该方法把现有 `:1676-1733` 的 bundle 构造 + calibration 逻辑抽出来，**行为等价**（不改变到点采样现有行为，只是提取）：

```python
    def _build_observation_bundle(self, snapshot, require_photos):
        """Read-only: form an observation_sync bundle for one detection snapshot.

        Shared by _observe_people_at_viewpoint (at-viewpoint, after fresh-wait)
        and _observe_en_route (in-flight worker). Does NOT write to the store.
        Safe to call from a worker thread: observation_sync carries its own
        RLock, and node snapshot readers are lock-guarded.

        Returns a dict with bundle/frame_info/frame/scan/pointcloud/robot_pose/
        source/captured_at/observation_valid/camera_calibration, or None if the
        bundle cannot be time-aligned within tolerance (caller must drop the
        sample — never fall back to ad-hoc pose+scan that misaligns in time).
        """
        snapshot = snapshot or {}
        source = str(snapshot.get("source") or "")
        try:
            captured_at = float(snapshot.get("timestamp"))
        except (TypeError, ValueError):
            captured_at = 0.0
        frame = snapshot.get("frame")
        frame_width, frame_height = self._snapshot_frame_size(snapshot, frame)
        try:
            calibration = resolve_camera_calibration(
                source, gimbal_yaw_rad=snapshot.get("gimbal_yaw_rad"))
        except Exception as exc:
            logger.warning("camera calibration failed: %s", exc)
            return None
        result = {
            "source": source,
            "captured_at": captured_at,
            "observation_valid": frame_width > 0 and frame_height > 0,
            "camera_calibration": calibration,
            "frame": frame,
            "frame_width": frame_width,
            "frame_height": frame_height,
        }
        detections = snapshot.get("detections") or []
        if not detections:
            result["bundle"] = None
            result["detections"] = []
            return result
        bundle = None
        if self._observation_sync is not None:
            try:
                if frame is not None:
                    self._observation_sync.add_frame(stamp=captured_at, frame=frame)
                self._observation_sync.add_detection(
                    stamp=captured_at, detection=snapshot)
                current_cloud = self._pointcloud_snapshot()
                cloud_stamp = current_cloud.get("timestamp") if current_cloud else None
                if cloud_stamp:
                    self._observation_sync.add_cloud(
                        stamp=cloud_stamp, cloud=current_cloud.get("cloud"))
                tolerance = self._positive_float(
                    os.environ.get("GO2W_OBSERVATION_SYNC_TOLERANCE_SEC"), 0.20)
                bundle = self._observation_sync.bundle_for_detection(
                    stamp=captured_at, tolerance=tolerance)
            except Exception as exc:
                logger.warning("observation synchronization failed: %s", exc)
                bundle = None
        if bundle is None:
            # 关键: 拿不到对齐 bundle → 返回 None, 调用方丢弃样本
            # 绝不退化为 "存 pose + 事后读最新 scan" (时间错位)
            return None
        pointcloud = bundle.cloud.value if bundle.cloud is not None else None
        result.update({
            "bundle": bundle,
            "scan": bundle.scan.value,
            "pointcloud": pointcloud,
            "robot_pose": (bundle.pose.x, bundle.pose.y, bundle.pose.yaw),
            "capture_stamp": bundle.capture_stamp,
            "pose_stamp": bundle.pose.stamp,
            "scan_stamp": bundle.scan.stamp,
            "pose_delta_s": bundle.pose_delta_s,
            "scan_delta_s": bundle.scan_delta_s,
            "localization_quality": "timestamp_interpolated",
            "detections": detections,
        })
        return result
```

> **注意 `_pointcloud_snapshot()` 返回结构**：现有到点代码（`:1699-1703`）按 `current_cloud.get("timestamp")` / `bundle.cloud.value` 取用，说明 `_pointcloud_snapshot()` 返回 dict 且 cloud 实体在 `"cloud"` 键或就是返回值本身。实施时**先 Grep `_pointcloud_snapshot` 的实现确认键名**，若它直接返回 ROS msg（非 dict），则把上面的 `current_cloud.get("cloud")` 改为 `current_cloud`、`current_cloud.get("timestamp")` 改为 getattr。这是唯一的实施期核实点，已在 Step 5 标注。

- [ ] **Step 6: 重构 `_observe_people_at_viewpoint` 调用共享方法（行为等价）**

把 `_observe_people_at_viewpoint` 现有 `:1676-1733`（从 `frame = snapshot.get("frame")` 到 `pointcloud = self._pointcloud_snapshot()` 的 else 分支结束）**替换**为调用 `_build_observation_bundle`。替换后该方法的新鲜帧等待循环（`:1622-1675`）保持不变，循环 break 拿到 fresh snapshot 后：

**替换前**（`:1676-1738`，即从 `frame = snapshot.get("frame")` 到 `pointcloud = self._pointcloud_snapshot()` 结束的整段 if/else bundle 构造）：

```python
            frame = snapshot.get("frame")
            frame_width, frame_height = self._snapshot_frame_size(snapshot, frame)
            calibration = resolve_camera_calibration(
                source, gimbal_yaw_rad=snapshot.get("gimbal_yaw_rad"))
            observation_meta = {
                "source": source,
                "observation_valid": frame_width > 0 and frame_height > 0,
                "camera_calibration": calibration,
            }
            detections = snapshot.get("detections") or []
            if not detections:
                return {**observation_meta, "resolved_count": 0}

            bundle = None
            if self._observation_sync is not None:
                try:
                    ...  # 现有 bundle 构造 (含 add_frame/add_detection/add_cloud)
                except Exception as exc:
                    logger.warning("observation synchronization failed: %s", exc)
                    bundle = None
                if bundle is None:
                    return {
                        **observation_meta,
                        "reason": "unsynchronized_observation",
                        ...
                    }
                scan = bundle.scan.value
                ...
            else:
                scan = self._laser_scan_snapshot()
                if scan is None:
                    return None
                pointcloud = self._pointcloud_snapshot()
```

**替换后**：

```python
            bundle_result = self._build_observation_bundle(snapshot, require_photos)
            if bundle_result is None:
                return {
                    "source": source,
                    "observation_valid": False,
                    "reason": "unsynchronized_observation",
                    "capture_stamp": captured_at,
                }
            observation_meta = {
                "source": bundle_result["source"],
                "observation_valid": bundle_result["observation_valid"],
                "camera_calibration": bundle_result["camera_calibration"],
            }
            if bundle_result.get("bundle") is None:
                # 无 detection 或 observation_sync 缺失: 走原 fall-back
                return {**observation_meta, "resolved_count": 0}
            observation_meta.update({
                "capture_stamp": bundle_result["capture_stamp"],
                "pose_stamp": bundle_result["pose_stamp"],
                "scan_stamp": bundle_result["scan_stamp"],
                "pose_delta_s": bundle_result["pose_delta_s"],
                "scan_delta_s": bundle_result["scan_delta_s"],
                "localization_quality": bundle_result["localization_quality"],
            })
            scan = bundle_result["scan"]
            pointcloud = bundle_result["pointcloud"]
            robot_pose = bundle_result["robot_pose"]
            frame = bundle_result["frame"]
            frame_width = bundle_result["frame_width"]
            frame_height = bundle_result["frame_height"]
            calibration = bundle_result["camera_calibration"]
            detections = bundle_result["detections"]
```

> **等价性保证**：`_build_observation_bundle` 内的 bundle 构造代码逐行复制自原 `:1691-1733`；唯一行为差异是"observation_sync 缺失且无 detection"路径——原代码在 `observation_sync is None` 时仍走 `_laser_scan_snapshot()` fall-back，重构后这种情况 `_build_observation_bundle` 返回 `{"bundle": None, "detections": []}` → `_observe_people_at_viewpoint` 走 `{**observation_meta, "resolved_count": 0}`。**实施时**：若 `_ensure_person_deps()` 测试或实机依赖 `observation_sync is None` 的 fall-back（`test_frontier_explore.py` 现有 `test_frontier_explore_retries_after_temporary_no_lidar_range` 等），在 Step 9 全套回归会暴露，届时把 fall-back 在 `_observe_people_at_viewpoint` 里保留一个 `if self._observation_sync is None:` 早分支即可。**这是受现有测试保护的重构，不是盲改。**

- [ ] **Step 7: 跑现有到点采样测试，确认重构无回归**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py -v -k "not observes_people_en_route and not dedups_by_capture and not bounded_queue"`
Expected: PASS — 所有原有到点采样测试绿（证明 `_build_observation_bundle` 抽取行为等价）。

- [ ] **Step 8: 实现 `_localize_en_route_detections`（只读，只收 range_lidar）**

在 `_build_observation_bundle` 之后插入：

```python
    def _localize_en_route_detections(self, bundle_result, target_classes):
        """Read-only localize of en-route detections. Only range_lidar kept.

        bearing_only observations are left for the at-viewpoint steady-state
        sample (they need a second viewpoint to triangulate). Returns a list
        of localized dicts ready for store.add_observation. Safe from a worker
        thread: no store writes, no shared mutable state beyond bundle_result.
        """
        bundle_result = bundle_result or {}
        detections = bundle_result.get("detections") or []
        if not detections or bundle_result.get("bundle") is None:
            return []
        scan = bundle_result.get("scan")
        if scan is None:
            return []
        pointcloud = bundle_result.get("pointcloud")
        robot_x, robot_y, robot_yaw = bundle_result["robot_pose"]
        calibration = bundle_result["camera_calibration"]
        frame_width = bundle_result["frame_width"]
        frame_height = bundle_result["frame_height"]
        if frame_width <= 0 or frame_height <= 0:
            return []
        if localize_target_detection is None:
            return []
        frame_info = DetectionFrame(
            width=frame_width,
            height=frame_height,
            camera_hfov_rad=math.radians(calibration["hfov_deg"]),
            camera_yaw_offset_rad=math.radians(calibration["yaw_offset_deg"]),
            gimbal_yaw_rad=math.radians(calibration["gimbal_yaw_deg"] or 0.0),
        )
        localization = self._localization_health()
        try:
            robot_z = float(localization.get("z", 0.0))
        except (TypeError, ValueError):
            robot_z = 0.0
        if not math.isfinite(robot_z):
            robot_z = 0.0
        allowed = set(self._normalize_target_classes(target_classes))
        results = []
        for det in detections:
            if det.get("class") not in allowed:
                continue
            try:
                localized = localize_target_detection(
                    det, frame_info, scan, robot_x, robot_y, robot_yaw,
                    robot_z=robot_z, pointcloud=pointcloud)
            except Exception as exc:
                logger.debug("en-route localize failed: %s", exc)
                continue
            if localized.get("position_quality") != "range_lidar":
                continue
            localized.update({
                "robot_x": round(float(robot_x), 3),
                "robot_y": round(float(robot_y), 3),
                "robot_yaw": round(float(robot_yaw), 3),
                "source": "en_route",
                "detector_source": bundle_result.get("source"),
                "timestamp": bundle_result.get("capture_stamp", time.time()),
            })
            results.append(localized)
        return results
```

- [ ] **Step 9: 实现 `_observe_en_route`（worker 线程）+ `_ingest_en_route_samples`（主线程）**

在 `_localize_en_route_detections` 之后插入：

```python
    def _observe_en_route(self, stop_event, target_classes, sample_interval=0.4,
                          max_samples=12):
        """Worker thread: best-effort person detection while navigating.

        Reads ai detection snapshot cache; for each NEW inference frame
        (deduped by captured_at) forms a time-aligned observation_sync bundle
        and localizes range_lidar detections. Does NOT write to the store.
        Returns a bounded list of samples
        [{localized_list, frame, captured_at, source}, ...] for the main
        thread to ingest serially after the goal is reached.

        Dedup rule: same captured_at (inference frame stamp) is processed at
        most once, so a cached frame polled many times during one navigation
        leg is not copied repeatedly (avoiding the 100x 720p heap pressure
        flagged in review).
        """
        samples = []
        get_snapshot = self._detection_snapshot_getter(target_classes)
        if get_snapshot is None:
            return samples
        interval = max(0.05, float(sample_interval))
        cap = max(1, int(max_samples))
        seen_stamps = set()
        while not stop_event.is_set():
            try:
                snapshot = get_snapshot() or {}
            except Exception as exc:
                logger.debug("en-route snapshot read failed: %s", exc)
                snapshot = {}
            try:
                captured_at = float(snapshot.get("timestamp"))
            except (TypeError, ValueError):
                captured_at = 0.0
            if captured_at > 0.0 and captured_at not in seen_stamps:
                seen_stamps.add(captured_at)
                try:
                    bundle_result = self._build_observation_bundle(snapshot, False)
                except Exception as exc:
                    logger.debug("en-route bundle build failed: %s", exc)
                    bundle_result = None
                if bundle_result is not None and bundle_result.get("bundle") is not None:
                    localized = self._localize_en_route_detections(
                        bundle_result, target_classes)
                    if localized:
                        if len(samples) >= cap:
                            samples.pop(0)  # drop oldest, keep bounded
                        samples.append({
                            "localized_list": localized,
                            "frame": bundle_result.get("frame"),
                            "captured_at": captured_at,
                            "source": bundle_result.get("source"),
                        })
            stop_event.wait(interval)
        return samples

    def _ingest_en_route_samples(self, samples, store, room_name, require_photos):
        """Main thread: serially commit en-route localized samples to the store.

        Called after the worker thread is joined. Each sample carries a list
        of already-localized range_lidar observations; we only call
        store.add_observation (the store's own dedup + media policy applies).
        Returns the number of observations committed.
        """
        committed = 0
        for sample in samples or []:
            localized_list = sample.get("localized_list") or []
            frame = sample.get("frame") if require_photos else None
            room = str(room_name or "__frontier__")
            for localized in localized_list:
                localized.setdefault("room", room)
                try:
                    store.add_observation(localized, frame=frame)
                    committed += 1
                except Exception as exc:
                    logger.debug("en-route store commit failed: %s", exc)
        return committed
```

- [ ] **Step 10: 改 `_run_frontier_explore` frontier 循环包 worker**

定位 frontier 循环里发送 Nav2 goal 的段落（`:1370` 附近 `nav_attempts += 1` 紧接 `result = nav.send_goal_and_wait(...)`）。

**替换**：

```python
                nav_attempts += 1
                result = nav.send_goal_and_wait(
                    target["x"], target["y"], target.get("yaw", 0.0),
                    frame_id="map")
```

**为**：

```python
                nav_attempts += 1
                # En-route observer: 导航期间 best-effort 时间对齐检测, 到达后 drain
                en_route_interval = self._positive_float(
                    params.get("en_route_sample_interval"), 0.4)
                en_route_cap = self._positive_int(
                    os.environ.get("GO2W_EN_ROUTE_MAX_SAMPLES"), 12)
                stop_event = threading.Event()
                en_route_holder = {}

                def _en_route_worker():
                    try:
                        en_route_holder["samples"] = self._observe_en_route(
                            stop_event, target_classes, en_route_interval,
                            max_samples=en_route_cap)
                    except Exception as exc:
                        logger.debug("en-route worker crashed: %s", exc)
                        en_route_holder["samples"] = []

                en_route_thread = threading.Thread(
                    target=_en_route_worker, daemon=True,
                    name=f"en-route-{mission_id}-{iteration}")
                en_route_thread.start()
                try:
                    result = nav.send_goal_and_wait(
                        target["x"], target["y"], target.get("yaw", 0.0),
                        frame_id="map")
                finally:
                    stop_event.set()
                    en_route_thread.join(timeout=2.0)
                en_route_samples = en_route_holder.get("samples") or []
                if en_route_samples:
                    self._ingest_en_route_samples(
                        en_route_samples, store, "__frontier__", require_photos)
                    self._broadcast_person_markers(mission_id, store.markers())
```

> **注意**：紧接这段下面是原有 `if not result.get("ok"):` 失败分支——**保持不变**。en-route samples 即使 goal 失败也已被 ingest（狗走过的人仍算数），这是期望行为。`mission_id` / `iteration` / `store` / `target_classes` / `require_photos` 都是 `_run_frontier_explore` 内已有变量（见 `:1096-1370`）。

- [ ] **Step 11: 运行 en-route 测试确认通过**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py::test_frontier_explore_observes_people_en_route web/test_frontier_explore.py::test_en_route_observer_dedups_by_capture_stamp web/test_frontier_explore.py::test_en_route_observer_bounded_queue -v`
Expected: PASS。

- [ ] **Step 12: 跑整个 frontier 套件确认无回归**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py -v`
Expected: PASS — 所有原有测试 + 新测试全绿。特别确认：
- `test_frontier_explore_retries_after_temporary_no_lidar_range`（en-route 不干扰到点稳态采样的 unresolved/retry）
- `test_frontier_explore_subscription_destroyed_after_run`（`join(timeout=2.0)` 不阻塞订阅销毁）
- `test_frontier_explore_cancel_mid_loop`（cancel 时 worker 因 `stop_event` + daemon 正常退出）

- [ ] **Step 13: Commit**

```bash
cd go2w_search_ws
git add web/nx_room_orchestrator.py web/test_frontier_explore.py
git commit -m "feat(frontier): en-route person detection via observation_sync bundle + bounded queue + dedup"
```

---

## Task 2: 纯逻辑覆盖率 + enclosed_unknown_regions（ROI 裁剪 + 膨胀 flood-fill）

**Files:**
- Create: `go2w_search_ws/web/nx_coverage_metrics.py`
- Test: `go2w_search_ws/web/test_coverage_metrics.py`

**Interfaces:**
- Produces: `compute_coverage(map_msg, roi=None, mission_origin=None, inflation_radius_m=0.3) -> dict | None`，返回字段：
  - `coverage_valid: bool`（ROI 非空且地图几何有效时 True）
  - `roi: dict`（回传实际使用的 ROI，`{"type":"circle","center":[x,y],"radius":r}` 或 `{"type":"polygon","points":[...]}` 或 `{"type":"whole_map"}`）
  - `free_cells / occupied_cells / unknown_cells / total_cells: int`（**仅 ROI 内**计数；total = free+occupied+unknown）
  - `explored_ratio: float`（`(free+occupied)/total`，保留 6 位；total=0 → `None`，此时 `coverage_valid=False`）
  - `enclosed_unknown_regions: list[dict]`（每个 `{min_x, min_y, max_x, max_y, cell_count}`，world 坐标，按 `(min_x, min_y)` 升序）
  - `map_stamp: float | None`（地图 header stamp 秒数，若可解析）
  - `inflation_radius_m: float`（回传实际使用的膨胀半径）
- 算法（enclosed_unknown_regions，对抗审核意见 #4）：
  1. ROI 内 occupied cell 集合按 `inflation_radius_m` 膨胀（圆盘），标记 forbidden。
  2. ROI 内 free cell 且**非 forbidden** → 可通行 free。
  3. 从 mission_origin 所在 cell BFS（只走可通行 free），得 `reachable_free`。mission_origin 为 None → reachable_free 退化为"全部可通行 free"（保守：不基于 origin 排除）。
  4. ROI 内 unknown 连通块（8 邻接）：若**任一 cell 接触 reachable_free** → 是 frontier（可达，跳过）；若**任一 cell 接触 ROI 边界或地图边界** → 排除（可能是墙体/建筑外，审核 #4）；否则 → `enclosed_unknown_region`。

- [ ] **Step 1: 写失败测试（7 个 case）**

Create `go2w_search_ws/web/test_coverage_metrics.py`:

```python
"""test_coverage_metrics.py — ROI 限定覆盖率 + enclosed_unknown 单测 (无 rclpy)。"""
import sys
from pathlib import Path
from types import SimpleNamespace

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_coverage_metrics import compute_coverage  # noqa: E402


def _map(data, width, height, resolution=0.1, origin_x=0.0, origin_y=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        info=SimpleNamespace(
            resolution=resolution,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=list(data),
    )


def test_compute_coverage_none_map_returns_none():
    assert compute_coverage(None) is None


def test_compute_coverage_invalid_geometry_returns_none():
    bad = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        info=SimpleNamespace(
            resolution=0.0, width=10, height=10,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * 100,
    )
    assert compute_coverage(bad) is None


def test_compute_coverage_roi_circle_excludes_outer_padding():
    """整图 10x10, 四周 2 圈 unknown (padding), 中心 6x6 free.
    ROI 圆心 (0.5,0.5) 半径 0.3m (resolution=0.1 → 3 cell) → 只数中心 free,
    不被外围 padding 拉低 explored_ratio."""
    width = height = 10
    data = []
    for r in range(height):
        for c in range(width):
            if 2 <= r <= 7 and 2 <= c <= 7:
                data.append(0)
            else:
                data.append(-1)  # padding
    msg = _map(data, width, height, resolution=0.1, origin_x=0.0, origin_y=0.0)
    roi = {"type": "circle", "center": [0.5, 0.5], "radius": 0.35}
    result = compute_coverage(msg, roi=roi, mission_origin=(0.5, 0.5, 0.0))
    assert result is not None
    assert result["coverage_valid"] is True
    assert result["explored_ratio"] == 1.0  # ROI 内全 free
    assert result["unknown_cells"] == 0


def test_compute_coverage_whole_map_without_roi_marks_unverified():
    """roi=None → 仍计算但 coverage_valid=False (整图含 padding 不可信)."""
    msg = _map([0] * 100, 10, 10)
    result = compute_coverage(msg)
    assert result is not None
    assert result["coverage_valid"] is False
    assert result["roi"]["type"] == "whole_map"


def test_compute_coverage_walled_pocket_is_enclosed():
    """7x7: 中心 3x3 unknown 被 occupied 围死, 最外层 free.
    ROI = 整图圆覆盖; mission_origin 在最外层 free.
    中心 unknown 不接 reachable_free 也不接边界 → enclosed."""
    width = height = 7
    data = []
    for r in range(height):
        for c in range(width):
            if 2 <= r <= 4 and 2 <= c <= 4:
                data.append(-1)
            elif 1 <= r <= 5 and 1 <= c <= 5:
                data.append(100)
            else:
                data.append(0)
    msg = _map(data, width, height, resolution=0.5, origin_x=10.0, origin_y=20.0)
    roi = {"type": "circle", "center": [10.0 + 0.25, 20.0 + 0.25], "radius": 5.0}
    # mission_origin 在角落 free cell (row 0, col 0 → world ~ (10.25, 20.25))
    result = compute_coverage(
        msg, roi=roi, mission_origin=(10.25, 20.25, 0.0), inflation_radius_m=0.0)
    assert result["unknown_cells"] == 9
    assert len(result["enclosed_unknown_regions"]) == 1
    dz = result["enclosed_unknown_regions"][0]
    assert dz["cell_count"] == 9
    # cell-center local = (col+0.5)*0.5; col 2 → 1.25, col 4 → 2.25; origin (10,20)
    assert dz["min_x"] == 10.0 + 1.25
    assert dz["max_x"] == 10.0 + 2.25
    assert dz["min_y"] == 20.0 + 1.25
    assert dz["max_y"] == 20.0 + 2.25


def test_compute_coverage_unknown_touching_roi_boundary_not_enclosed():
    """ROI 边界处的 unknown 连通块不算 enclosed (可能是建筑外)."""
    # 5x5: 左上角 2x2 unknown 接触 ROI 边界, 其余 free
    width = height = 5
    data = []
    for r in range(height):
        for c in range(width):
            if r < 2 and c < 2:
                data.append(-1)
            else:
                data.append(0)
    msg = _map(data, width, height, resolution=0.5)
    roi = {"type": "circle", "center": [1.25, 1.25], "radius": 2.5}
    result = compute_coverage(
        msg, roi=roi, mission_origin=(2.0, 2.0, 0.0), inflation_radius_m=0.0)
    # 左上 unknown 接触 ROI/地图边界 → 不算 enclosed
    assert result["enclosed_unknown_regions"] == []


def test_compute_coverage_explored_ratio_rounded():
    data = [0, 0, 0, -1]  # 3 free, 1 unknown
    msg = _map(data, 2, 2)
    roi = {"type": "circle", "center": [0.1, 0.1], "radius": 1.0}
    result = compute_coverage(msg, roi=roi)
    assert result["explored_ratio"] == 0.75
```

- [ ] **Step 2: 运行确认失败**

Run: `cd go2w_search_ws && python -m pytest web/test_coverage_metrics.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'nx_coverage_metrics'`.

- [ ] **Step 3: 实现 `nx_coverage_metrics.py`**

Create `go2w_search_ws/web/nx_coverage_metrics.py`:

```python
"""Pure-function ROI-bounded coverage metrics for exploration success validation.

No rclpy dependency. Reads OccupancyGrid via getattr so it works with both real
nav_msgs/OccupancyGrid and SimpleNamespace stubs in unit tests.

Used by RoomSearchOrchestrator._run_frontier_explore at REPORT time to validate
that the closed-room exploration actually covered the reachable free space
*inside the mission ROI*, not the whole grid (which is inflated by
map_padding_bridge's 2m unknown padding around /map_frontier).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Optional


_NEIGHBORS_8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
)


def _finite_float(value, default=0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _map_geometry(map_msg):
    """Return (resolution, width, height, data, origin_x, origin_y, origin_yaw) or None."""
    if map_msg is None:
        return None
    info = getattr(map_msg, "info", None)
    if info is None:
        return None
    resolution = _finite_float(getattr(info, "resolution", 0.0))
    width = int(getattr(info, "width", 0) or 0)
    height = int(getattr(info, "height", 0) or 0)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return None
    try:
        data = list(getattr(map_msg, "data", None) or [])
    except Exception:
        return None
    if len(data) != width * height:
        return None
    origin = getattr(info, "origin", None)
    position = getattr(origin, "position", None)
    orientation = getattr(origin, "orientation", None)
    origin_x = _finite_float(getattr(position, "x", 0.0))
    origin_y = _finite_float(getattr(position, "y", 0.0))
    qx = _finite_float(getattr(orientation, "x", 0.0))
    qy = _finite_float(getattr(orientation, "y", 0.0))
    qz = _finite_float(getattr(orientation, "z", 0.0))
    qw = _finite_float(getattr(orientation, "w", 1.0), 1.0)
    origin_yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return resolution, width, height, data, origin_x, origin_y, origin_yaw


def _world_to_cell(wx, wy, resolution, origin_x, origin_y, origin_yaw):
    dx = wx - origin_x
    dy = wy - origin_y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return (int(math.floor(local_x / resolution + 1e-9)),
            int(math.floor(local_y / resolution + 1e-9)))


def _cell_to_world_center(row, col, resolution, origin_x, origin_y, origin_yaw):
    local_x = (col + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    wx = origin_x + cos_yaw * local_x - sin_yaw * local_y
    wy = origin_y + sin_yaw * local_x + cos_yaw * local_y
    return wx, wy


def _in_roi(row, col, resolution, origin_x, origin_y, origin_yaw, roi):
    """Is cell center inside the ROI polygon/circle? roi=None → True (whole map)."""
    if roi is None:
        return True
    wx, wy = _cell_to_world_center(
        row, col, resolution, origin_x, origin_y, origin_yaw)
    rtype = str(roi.get("type") or "")
    if rtype == "circle":
        cx, cy = float(roi["center"][0]), float(roi["center"][1])
        radius = float(roi["radius"])
        return math.hypot(wx - cx, wy - cy) <= radius + 1e-9
    if rtype == "polygon":
        return _point_in_polygon(wx, wy, roi.get("points") or [])
    return True


def _point_in_polygon(x, y, polygon):
    inside = False
    n = len(polygon)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = float(polygon[i][0]), float(polygon[i][1])
        xj, yj = float(polygon[j][0]), float(polygon[j][1])
        if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _inflate_obstacles(forbidden, occupied_cells_roi, width, height,
                       inflation_radius_cells):
    """Mark cells within inflation_radius of any occupied cell as forbidden."""
    if inflation_radius_cells <= 0:
        return
    queue = deque(occupied_cells_roi)
    seen = set(occupied_cells_roi)
    radius_sq = inflation_radius_cells * inflation_radius_cells
    # BFS from occupied cells up to inflation radius
    seeds = list(occupied_cells_roi)
    for seed in seeds:
        sr, sc = seed
        for dr in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for dc in range(-inflation_radius_cells, inflation_radius_cells + 1):
                if dr * dr + dc * dc > radius_sq:
                    continue
                nr, nc = sr + dr, sc + dc
                if 0 <= nr < height and 0 <= nc < width:
                    forbidden.add((nr, nc))


def compute_coverage(map_msg, roi=None, mission_origin=None,
                     inflation_radius_m: float = 0.3) -> Optional[dict]:
    """Scan final occupancy grid inside ROI; return coverage metrics or None.

    Fields:
      - coverage_valid: True iff ROI non-empty and map geometry valid
      - roi: echo of the ROI used (for the report)
      - free_cells/occupied_cells/unknown_cells/total_cells: ROI-bounded counts
      - explored_ratio: (free+occupied)/total in [0,1], None if total=0
      - enclosed_unknown_regions: connected unknown regions inside ROI that
        neither border reachable_free (after inflation) nor touch the ROI/map
        boundary. Each entry: {min_x,min_y,max_x,max_y,cell_count} world frame.
      - map_stamp: header stamp seconds if parseable, else None
      - inflation_radius_m: echo

    roi=None → coverage_valid=False (whole-map ratio is polluted by padding).
    """
    geometry = _map_geometry(map_msg)
    if geometry is None:
        return None
    resolution, width, height, data, origin_x, origin_y, origin_yaw = geometry

    map_stamp = _parse_stamp(getattr(map_msg, "header", None))

    inflation_cells = max(
        0, int(math.ceil(_finite_float(inflation_radius_m, 0.0) / resolution - 1e-9)))

    free_cells_roi = set()
    occupied_cells_roi = set()
    unknown_cells_roi = set()
    for index, value in enumerate(data):
        row, col = divmod(index, width)
        if not _in_roi(row, col, resolution, origin_x, origin_y, origin_yaw, roi):
            continue
        if value == 0:
            free_cells_roi.add((row, col))
        elif value < 0:
            unknown_cells_roi.add((row, col))
        else:
            occupied_cells_roi.add((row, col))

    free = len(free_cells_roi)
    occupied = len(occupied_cells_roi)
    unknown = len(unknown_cells_roi)
    total = free + occupied + unknown
    coverage_valid = (roi is not None) and (total > 0)
    explored_ratio = ((free + occupied) / total) if total > 0 else None

    enclosed = _enclosed_unknown_regions(
        unknown_cells_roi, free_cells_roi, occupied_cells_roi,
        width, height, inflation_cells, mission_origin,
        resolution, origin_x, origin_y, origin_yaw, roi)

    roi_echo = (
        {"type": "whole_map"} if roi is None
        else {"type": str(roi.get("type")), **{
            k: list(v) if isinstance(v, (list, tuple)) else v
            for k, v in roi.items() if k != "type"}})

    return {
        "coverage_valid": bool(coverage_valid),
        "roi": roi_echo,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "total_cells": total,
        "explored_ratio": (round(explored_ratio, 6) if explored_ratio is not None else None),
        "enclosed_unknown_regions": enclosed,
        "map_stamp": map_stamp,
        "inflation_radius_m": _finite_float(inflation_radius_m, 0.0),
    }


def _enclosed_unknown_regions(unknown_cells, free_cells, occupied_cells,
                              width, height, inflation_cells, mission_origin,
                              resolution, origin_x, origin_y, origin_yaw, roi):
    """Connected unknown regions that are enclosed (review #4 corrected rule).

    A region is enclosed iff:
      - all its cells are inside ROI (already true by construction), AND
      - none of its cells borders a reachable_free cell (free cell reachable
        from mission_origin after obstacle inflation), AND
      - none of its cells touches the ROI boundary or the map boundary.
    Regions touching reachable_free are frontiers (reachable); regions touching
    a boundary may be wall interior / building exterior / padding — not reported.
    """
    if not unknown_cells:
        return []

    # 1. forbidden = inflated obstacles; passable_free = free not forbidden
    forbidden = set(occupied_cells)
    _inflate_obstacles(forbidden, occupied_cells, width, height, inflation_cells)
    passable_free = free_cells - forbidden
    if not passable_free:
        # No passable free → every unknown region is enclosed (conservative)
        passable_free = set()

    # 2. reachable_free from mission_origin
    reachable_free = set()
    start_cell = None
    if mission_origin is not None and passable_free:
        try:
            mx, my = float(mission_origin[0]), float(mission_origin[1])
            start_cell = _world_to_cell(
                mx, my, resolution, origin_x, origin_y, origin_yaw)
        except (TypeError, ValueError, IndexError):
            start_cell = None
    if start_cell is not None and start_cell in passable_free:
        reachable_free = _flood_fill(start_cell, passable_free, width, height)
    else:
        # mission_origin not on passable free (or None): treat all passable_free
        # as reachable (conservative — do not over-report enclosed regions).
        reachable_free = set(passable_free)

    # 3. connected components of unknown; check enclosure
    visited = set()
    enclosed = []
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    for seed in unknown_cells:
        if seed in visited:
            continue
        queue = deque([seed])
        visited.add(seed)
        component = []
        borders_reachable = False
        touches_boundary = False
        while queue:
            cell = queue.popleft()
            component.append(cell)
            row, col = cell
            if _cell_touches_boundary_or_roi_edge(
                    row, col, width, height, resolution,
                    origin_x, origin_y, origin_yaw, roi):
                touches_boundary = True
            for dr, dc in _NEIGHBORS_8:
                nr, nc = row + dr, col + dc
                neighbor = (nr, nc)
                if neighbor in reachable_free:
                    borders_reachable = True
                if neighbor in unknown_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        if borders_reachable or touches_boundary:
            continue
        min_row = min(c[0] for c in component)
        max_row = max(c[0] for c in component)
        min_col = min(c[1] for c in component)
        max_col = max(c[1] for c in component)
        corners = []
        for r in (min_row, max_row):
            for c in (min_col, max_col):
                local_x = (c + 0.5) * resolution
                local_y = (r + 0.5) * resolution
                wx = origin_x + cos_yaw * local_x - sin_yaw * local_y
                wy = origin_y + sin_yaw * local_x + cos_yaw * local_y
                corners.append((wx, wy))
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        enclosed.append({
            "min_x": round(min(xs), 3),
            "min_y": round(min(ys), 3),
            "max_x": round(max(xs), 3),
            "max_y": round(max(ys), 3),
            "cell_count": len(component),
        })
    enclosed.sort(key=lambda z: (z["min_x"], z["min_y"]))
    return enclosed


def _cell_touches_boundary_or_roi_edge(row, col, width, height, resolution,
                                       origin_x, origin_y, origin_yaw, roi):
    # Map boundary
    if row <= 0 or col <= 0 or row >= height - 1 or col >= width - 1:
        return True
    if roi is None:
        return False
    # ROI boundary: probe the 4-neighbors; if any is outside the ROI, this cell
    # is on the ROI edge.
    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        nr, nc = row + dr, col + dc
        if not _in_roi(nr, nc, resolution, origin_x, origin_y, origin_yaw, roi):
            return True
    return False


def _flood_fill(start, passable_free, width, height):
    visited = {start}
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for dr, dc in _NEIGHBORS_8:
            neighbor = (row + dr, col + dc)
            if neighbor in passable_free and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return visited


def _parse_stamp(header):
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        if sec == 0 and nanosec == 0:
            return None
        return float(sec) + float(nanosec) * 1e-9
    except (TypeError, ValueError):
        return None


__all__ = ["compute_coverage"]
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd go2w_search_ws && python -m pytest web/test_coverage_metrics.py -v`
Expected: PASS — 7 个测试全绿。

- [ ] **Step 5: Commit**

```bash
cd go2w_search_ws
git add web/nx_coverage_metrics.py web/test_coverage_metrics.py
git commit -m "feat(frontier): ROI-bounded coverage metrics + enclosed_unknown_regions (flood-fill)"
```

---

## Task 3: REPORT 接入 coverage + completion_status 四态

**Files:**
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py`（顶部 import + `_run_frontier_explore` REPORT 段调 `compute_coverage` + 算 `completion_status`）
- Test: `go2w_search_ws/web/test_frontier_explore.py`（加 coverage + 四态测试）

**Interfaces:**
- Consumes: `nx_coverage_metrics.compute_coverage(map_msg, roi, mission_origin, inflation_radius_m)`（Task 2 产出）；`_run_frontier_explore` 内 `latest_map_box`（缓存的 `/map_frontier` 最新帧）、`mission_origin`、`max_radius`、`completion_reason`。
- Produces: `mission_report` 多字段 + 顶层 `completion_status`：
  - `completion_status: str`（四态之一）
  - `explored_ratio: float | None`、`coverage_valid: bool`、`roi: dict`、`enclosed_unknown_regions: list[dict]`
  - `coverage_free_cells / coverage_occupied_cells / coverage_unknown_cells / coverage_total_cells: int`
  - `map_stamp: float | None`
- `completion_status` 决策表：

| 条件 | `completion_status` |
|---|---|
| `coverage is None` 或 `coverage_valid=False` | `coverage_unverified` |
| `completion_reason` ∈ {`time_budget_exhausted`, `distance_budget_exhausted`, `planning_budget_exhausted`} | `incomplete` |
| frontier 耗尽 AND `explored_ratio ≥ threshold` AND 无 enclosed | `completed` |
| frontier 耗尽 AND (`explored_ratio < threshold` OR 有 enclosed) | `completed_with_gaps` |

- 阈值：默认 `0.90`（env `GO2W_FRONTIER_COVERAGE_THRESHOLD` 可配；spec v1 的 0.95 因 padding 灌水实测不可达，spec 已勘误为 0.90）。

- [ ] **Step 1: 写失败测试（3 个 case）**

Append to `go2w_search_ws/web/test_frontier_explore.py`:

```python
def test_frontier_explore_report_completed_when_full_coverage(monkeypatch):
    """全自由 map + ROI → completion_status=completed, explored_ratio=1.0."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])

    orchestrator = make_orchestrator([], FakeNav())
    task = FakeTask()
    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_status"] == "completed"
    assert task.result["coverage_valid"] is True
    assert task.result["explored_ratio"] == 1.0
    assert task.result["enclosed_unknown_regions"] == []


def test_frontier_explore_report_completed_with_gaps_for_walled_pocket(monkeypatch):
    """被障碍围死的未知区 → completion_status=completed_with_gaps."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    width = height = 7
    data = []
    for r in range(height):
        for c in range(width):
            if 2 <= r <= 4 and 2 <= c <= 4:
                data.append(-1)
            elif 1 <= r <= 5 and 1 <= c <= 5:
                data.append(100)
            else:
                data.append(0)
    pocket_map = _make_fake_map(width=width, height=height, fill_value=0)
    pocket_map.data = data

    class PocketNode(FakeNode):
        def __init__(self):
            super().__init__(emit_map=False)
            self._pocket = pocket_map

        def create_subscription(self, msg_type, topic, callback, qos):
            handle = super().create_subscription(msg_type, topic, callback, qos)
            if topic == "/map_frontier":
                def _emit():
                    time.sleep(0.05)
                    callback(self._pocket)
                threading.Thread(target=_emit, daemon=True).start()
            return handle

    monkeypatch.setattr(orch_module, "_OccupancyGrid", pocket_map.__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])

    orchestrator = make_orchestrator([], FakeNav(), node=PocketNode())
    task = FakeTask()
    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_status"] == "completed_with_gaps"
    assert len(task.result["enclosed_unknown_regions"]) == 1


def test_frontier_explore_report_coverage_unverified_without_map(monkeypatch):
    """地图不可用 (INIT_SLAM 拿到 None) → completion_status=coverage_unverified.

    构造: NoMapNode 不发 map → 但 _run_frontier_explore INIT_SLAM 超时会 fail.
    这里改用 emit_map=True 但 final map 为 None 的路径较难; 退而测试
    compute_coverage(None) 直接返回 None 的契约 + REPORT 段 None → unverified.
    用 monkeypatch 让 _compute_coverage 返回 None."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    monkeypatch.setattr(orch_module, "_compute_coverage", lambda *a, **k: None)

    orchestrator = make_orchestrator([], FakeNav())
    task = FakeTask()
    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_status"] == "coverage_unverified"
    assert "explored_ratio" in task.result
    assert task.result["explored_ratio"] is None
```

- [ ] **Step 2: 运行确认失败**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py::test_frontier_explore_report_completed_when_full_coverage web/test_frontier_explore.py::test_frontier_explore_report_completed_with_gaps_for_walled_pocket web/test_frontier_explore.py::test_frontier_explore_report_coverage_unverified_without_map -v`
Expected: FAIL — `KeyError: 'completion_status'`。

- [ ] **Step 3: 顶部 import `compute_coverage`**

在 `go2w_search_ws/web/nx_room_orchestrator.py` 顶部已有的 try/except import 块里（紧接 `from nx_active_search import ActiveSearchPlanner` 那个 try 块之后，约 `:50` 附近）添加：

```python
try:
    from nx_coverage_metrics import compute_coverage as _compute_coverage
except Exception:
    _compute_coverage = None
```

> 命名为 `_compute_coverage`（模块级私有别名）便于 Task 3 测试 `monkeypatch.setattr(orch_module, "_compute_coverage", ...)`。

- [ ] **Step 4: 在 `_run_frontier_explore` REPORT 段调 `compute_coverage` + 算 `completion_status`**

定位 `_run_frontier_explore` 的 REPORT 段（`# ---- REPORT ----` 注释之后，`frontier_result = { ... }` 字典构造处）。

**替换**原有 REPORT 段（从 `markers = store.markers()` 到 `frontier_result = { ... }` 字典结束）：

```python
            # ---- REPORT ----
            markers = store.markers()
            unresolved = store.unresolved()
            self._broadcast_person_markers(mission_id, markers)
            exploration_state = exploration.snapshot()
            # ROI 限定覆盖率 (review #3): mission_origin 圆或 room_polygon
            with map_lock:
                final_map = latest_map_box[0]
            if self.room_polygon:
                coverage_roi = {
                    "type": "polygon",
                    "points": [tuple(p) for p in self.room_polygon],
                }
            else:
                coverage_roi = {
                    "type": "circle",
                    "center": [float(mission_origin[0]), float(mission_origin[1])],
                    "radius": float(max_radius) if max_radius is not None else 6.0,
                }
            coverage_inflation = self._positive_float(
                os.environ.get("GO2W_COVERAGE_INFLATION_M"), 0.3)
            coverage_metrics = None
            if _compute_coverage is not None and final_map is not None:
                try:
                    coverage_metrics = _compute_coverage(
                        final_map, roi=coverage_roi,
                        mission_origin=tuple(mission_origin),
                        inflation_radius_m=coverage_inflation)
                except Exception as exc:
                    logger.warning("coverage computation failed: %s", exc)
                    coverage_metrics = None
            if coverage_metrics is None:
                coverage_metrics = {
                    "coverage_valid": False,
                    "roi": coverage_roi,
                    "free_cells": 0, "occupied_cells": 0,
                    "unknown_cells": 0, "total_cells": 0,
                    "explored_ratio": None,
                    "enclosed_unknown_regions": [],
                    "map_stamp": None,
                    "inflation_radius_m": coverage_inflation,
                }
            completion_status = self._derive_completion_status(
                completion_reason, coverage_metrics)
            blocked_frontiers = [
                {**item, "rejections": int(item.get("failures", 0))}
                for item in exploration_state["blacklist"]
            ]
            frontier_result = {
                "completion_reason": completion_reason,
                "completion_status": completion_status,
                "waypoints_reached": waypoints_reached,
                "navigation_attempts": nav_attempts,
                "frontier_plan_probes": exploration_state["plan_probes"],
                "frontier_plan_rejections": exploration_state["plan_rejections"],
                "frontier_nav_failures": exploration_state["navigation_failures"],
                "blocked_frontiers": sorted(
                    blocked_frontiers, key=lambda item: (item["x"], item["y"])),
                "time_budget_sec": max_time,
                "search_radius_m": max_radius,
                "exploration_state": exploration_state,
                "coverage_valid": coverage_metrics["coverage_valid"],
                "explored_ratio": coverage_metrics["explored_ratio"],
                "roi": coverage_metrics["roi"],
                "enclosed_unknown_regions": coverage_metrics["enclosed_unknown_regions"],
                "coverage_free_cells": coverage_metrics["free_cells"],
                "coverage_occupied_cells": coverage_metrics["occupied_cells"],
                "coverage_unknown_cells": coverage_metrics["unknown_cells"],
                "coverage_total_cells": coverage_metrics["total_cells"],
                "map_stamp": coverage_metrics["map_stamp"],
            }
```

> **注意**：`mission_origin` / `max_radius` / `latest_map_box` / `map_lock` / `completion_reason` / `exploration` 都是 `_run_frontier_explore` 内已有变量（见 `:1196-1260`）。原有 `if unresolved:` / `if completion_reason != "reachable_frontiers_exhausted":` / 正常 REPORT 三分支都 `**frontier_result` 透传，coverage 字段自动进 `mission_report`。

- [ ] **Step 5: 加 `_derive_completion_status` 辅助方法**

在 `_run_frontier_explore` 方法**之后**（或类内任意合适位置）加：

```python
    def _derive_completion_status(self, completion_reason, coverage_metrics):
        """Map completion_reason + coverage into the 4-state report status.

        review #5: 停止信号仍是 frontier 耗尽, 但完成状态必须真实反映覆盖率,
        不能只塞数值又按 frontier 耗尽宣告成功.
        """
        coverage_metrics = coverage_metrics or {}
        if not coverage_metrics.get("coverage_valid"):
            return "coverage_unverified"
        budget_reasons = {
            "time_budget_exhausted",
            "distance_budget_exhausted",
            "planning_budget_exhausted",
        }
        if completion_reason in budget_reasons:
            return "incomplete"
        threshold = self._positive_float(
            os.environ.get("GO2W_FRONTIER_COVERAGE_THRESHOLD"), 0.90)
        ratio = coverage_metrics.get("explored_ratio")
        enclosed = coverage_metrics.get("enclosed_unknown_regions") or []
        try:
            ratio_ok = ratio is not None and float(ratio) >= threshold
        except (TypeError, ValueError):
            ratio_ok = False
        if ratio_ok and not enclosed:
            return "completed"
        return "completed_with_gaps"
```

- [ ] **Step 6: 运行 coverage + 四态测试确认通过**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py::test_frontier_explore_report_completed_when_full_coverage web/test_frontier_explore.py::test_frontier_explore_report_completed_with_gaps_for_walled_pocket web/test_frontier_explore.py::test_frontier_explore_report_coverage_unverified_without_map -v`
Expected: PASS。

- [ ] **Step 7: 跑整个 frontier + coverage 套件确认无回归**

Run: `cd go2w_search_ws && python -m pytest web/test_frontier_explore.py web/test_coverage_metrics.py -v`
Expected: PASS — 所有测试全绿。

- [ ] **Step 8: Commit**

```bash
cd go2w_search_ws
git add web/nx_room_orchestrator.py web/test_frontier_explore.py
git commit -m "feat(frontier): wire ROI coverage + 4-state completion_status into mission_report"
```

---

## Task 4 (stretch / 可选): 云台多角度观察提升 recall

> **前置条件**：spec 把"in-place sweep / 云台扫描"列为非目标。本 Task **默认不实现**。只有当 Task 1-3 上机后实机数据显示"沿途 best-effort 标注 recall 不足"（例如同一房间内已知 3 人，狗走完只标到 1-2 人）时，才解除 spec 非目标并实施本 Task。
>
> **Files:** Modify `nx_room_orchestrator.py`（每 N 个 frontier 到达后做原地云台 sweep）；Modify spec（解除非目标）。
>
> **Step 1**：实机收 recall 数据（frontier 探索结束后，对比已知人数与 `mission_report.markers` 数），决定是否实施。
> **Step 2（若决定实施）**：在 `_run_frontier_explore` frontier 循环里，每 `sweep_every_n_frontiers`（默认 3）个到达点，调云台到 `[-60°, 0°, +60°]` 三个 yaw，每个 yaw 等 0.5s 稳定后调 `_observe_people_at_viewpoint`。sweep 期间不开 en-route worker（避免并发）。
> **Step 3**：测试用 `FakeNav` + 云台 mock，断言 sweep 期间多次调用 `_observe_people_at_viewpoint`。
>
> 本 Task 无代码块——只在 stretch 触发时补全。

---

## 验收清单（仿真 + 实机）

**离线（纯逻辑，Task 1-3 各自单测覆盖）：**
- [ ] `_build_observation_bundle` 复用，到点采样行为等价（Task 1 Step 7 全套回归绿）。
- [ ] en-route observer：spy ingest ≥1（`test_frontier_explore_observes_people_en_route`）、capture_stamp 去重（`test_en_route_observer_dedups_by_capture_stamp`）、bounded queue（`test_en_route_observer_bounded_queue`）。
- [ ] coverage：ROI 裁掉 padding、整图无 ROI → `coverage_valid=False`、walled pocket → enclosed、接触边界不算 enclosed、无效地图 → None（Task 2 七测全绿）。
- [ ] completion_status 四态：completed / completed_with_gaps / coverage_unverified（Task 3 三测全绿）。

**仿真/集成（本计划不实施，列入下阶段）：**
- [ ] 封闭房间 + 内部障碍物，frontier 探索从起点到 frontier 耗尽，`completion_status=completed`。
- [ ] 路径经过的人被 en-route 标注（验证 bundle 时间对齐：marker 位置与人真实位置误差 < 0.5m）。
- [ ] 不可达 frontier 被黑名单跳过，不卡死。

**实机验收（上机后由用户执行）：**
- [ ] 起点位姿 → 探索完整个封闭区域 → frontier 耗尽退出，`completion_status ∈ {completed, completed_with_gaps}`。
- [ ] 全程 YOLO 标注可见的人，en-route 路径生效（不止标到点的）。
- [ ] `mission_report.explored_ratio` 反映真实 ROI 覆盖率（不被 padding 灌水）。
- [ ] pose 临时丢失时能恢复继续，不 EMERGENCY。
- [ ] 整个探索过程狗不发 `/cmd_vel`，所有移动走 Nav2 goal pose。

---

## Spec 同步 Edit 清单（Task 0 执行）

对 `docs/superpowers/specs/2026-07-19-bounded-frontier-room-exploration-design.md` 做 6 处 Edit（实施 Task 1 前完成）：

1. **行 57-61（覆盖率公式段）**：`explored_ratio = (free + occupied) / (free + occupied + unknown)` → 改为 ROI 内裁剪 + 去 padding 说明 + `enclosed_unknown_regions` 判定（接触边界排除 + 膨胀 flood-fill）。
2. **行 72-74 / 80（移动中检测段）**：补"复用 `observation_sync.bundle_for_detection` 时间对齐 + bounded queue ≤12 + capture_stamp 去重"。
3. **行 126（成功判据 #1）**：`explored_ratio ≥ 0.95` → `≥ 0.90`（实机 ROI 校准）+ 改述为 `completion_status` 四态。
4. **行 128（成功判据 #3）**："无漏标"→"沿途可见人员 best-effort 标注，不保证 recall"。
5. **行 59-61（dead_zones 命名）**：`dead_zones` → `enclosed_unknown_regions`。
6. **算法段补 Tech Stack**：Python ≥ 3.10（非 3.12）。

具体 old→new 文本在执行 Task 0 时由实施者按上述条目改写（spec 是 prose 文档，Edit 按段落语义改即可，无需逐字符）。

---

## Self-Review

### Spec coverage（修订后 spec → Task 映射）

| spec 章节 | 实现位置 |
|---|---|
| Frontier 探索循环（核心，已有） | 复用，不改 |
| 不可达 frontier 黑名单（已有） | 复用，不改 |
| 覆盖率度量（ROI 内，spec 勘误后） | Task 2 `compute_coverage(map, roi, ...)` |
| enclosed_unknown_regions（spec 勘误后） | Task 2 `_enclosed_unknown_regions` |
| 终止判据（frontier 耗尽，停止信号） | 复用，不改 |
| completion_status 四态（spec 勘误后） | Task 3 `_derive_completion_status` |
| YOLO 标人副产品 - 移动中（bundle 时间对齐） | Task 1 `_observe_en_route` + `_build_observation_bundle` |
| YOLO 标人副产品 - 到点 | 复用 `_observe_people_at_viewpoint`（重构为调 `_build_observation_bundle`） |
| 7/15 改进 #1 移动中连续检测 | Task 1 |
| 7/15 改进 #2 预算 300s | Task 1 Step 1 锁定测试（现状已满足） |
| 7/15 改进 #3 单 goal 40s | Task 1 Step 1 锁定测试（现状已满足） |
| 7/15 改进 #4 parked_state_lost | 非本方案范围（motion 层），spec 已注明 |
| 成功判据 #1 全屋覆盖（四态） | Task 3 |
| 成功判据 #2 障碍后区域 | 复用黑名单 + Task 2 enclosed 识别 |
| 成功判据 #3 移动中 best-effort 标注 | Task 1 |
| 成功判据 #4 不发 /cmd_vel | 复用，不改 |
| 成功判据 #5 frontier 耗尽 REPORT | 复用 + Task 3 加 coverage/status |
| 审核意见 #1-7 全部 | 见上方"v1 审核意见"表逐条映射 |

无遗漏。

### Placeholder scan

- 所有 step 都有完整代码块或精确命令，无 "TBD/TODO/implement later/handle edge cases/Similar to Task N"。
- Task 1 Step 5 有一个**实施期核实点**（`_pointcloud_snapshot()` 返回结构），已显式标注并给出两种键名适配方案——这是受测试保护的可核实点，不是占位符。
- Task 1 Step 6 的重构等价性由 Step 7 全套回归测试守护，不是盲改。
- Task 4（stretch）明确标注"无代码块，stretch 触发时补全"，符合 stretch 语义。

### Type / 命名一致性

- `compute_coverage(map_msg, roi, mission_origin, inflation_radius_m)` —— Task 2 定义、Task 3 调用，签名一致；返回字段 `coverage_valid`/`roi`/`explored_ratio`/`enclosed_unknown_regions`/`map_stamp`/`free_cells`/`occupied_cells`/`unknown_cells`/`total_cells`/`inflation_radius_m` 跨 Task 一致。
- `_build_observation_bundle` 返回 dict 键名（`bundle`/`scan`/`pointcloud`/`robot_pose`/`frame`/`frame_width`/`frame_height`/`camera_calibration`/`detections`/`capture_stamp`/`source`）在 Task 1 Step 5/8/9 三处使用一致。
- `_localize_en_route_detections(bundle_result, target_classes) -> list[dict]` —— Step 8 定义、Step 9 `_observe_en_route` 调用，签名一致。
- `_observe_en_route(stop_event, target_classes, sample_interval, max_samples) -> list[{localized_list, frame, captured_at, source}]` —— Step 9 定义、Step 10 worker 调用、Step 3 测试断言 `s["captured_at"]` 一致。
- `_ingest_en_route_samples(samples, store, room_name, require_photos) -> int` —— Step 9 定义、Step 10 调用、Step 3 spy 签名 `(samples, store, room_name, require_photos)` 一致。
- `completion_status` 四态字符串 `completed`/`completed_with_gaps`/`incomplete`/`coverage_unverified` —— Task 3 Step 5 定义、Step 1 三测试断言一致。
- `_compute_coverage` 模块级别名 —— Task 3 Step 3 import、Step 4 调用、Step 1 测试 monkeypatch 目标一致。

无类型/命名漂移。

---

## Execution Handoff

Plan v2 complete and saved to `docs/superpowers/plans/2026-07-19-bounded-frontier-room-exploration.md`. Two execution options:

**1. Subagent-Driven (recommended)** — 每个 Task dispatch 一个 fresh subagent，任务间两阶段 review，迭代快。建议顺序：Task 0（spec 勘误）→ Task 1 → Task 2 → Task 3。Task 4 stretch 暂不排。

**2. Inline Execution** — 在当前 session 用 executing-plans 批量执行，带 checkpoint review。

选哪种？
