# Product-Grade Room Person Search Design

## Goal

Build a product-grade flow for the command:

> 去搜索这个房间，把所有人标注出来

The dog must enter or resolve the target room, actively search reachable space, detect all visible people with YOLO, use LiDAR for map-coordinate localization, save photos, and mark each person on the live 2D map.

## Hard Decisions

1. Use the 2D SLAM route first: `slam_toolbox + Nav2 slim`.
2. Do not use FAST_LIO in the first product route.
3. Do not use LocateAnything.
4. Use YOLO only for person detection.
5. Use LiDAR range for person map localization; vision-only fixed-distance projection is not acceptable as a final product result.
6. Replace fixed rectangular lawnmower as the primary room-search strategy with room-bounded active search based on frontier and next-best-view scoring.
7. Keep Nav2 responsible for obstacle-aware movement; search logic chooses goals and view directions, not raw velocity control.

## Why 2D SLAM Instead Of FAST_LIO First

The immediate product problem is indoor room search and person marking on a navigable map. That needs a robust 2D occupancy map, localization, costmaps, and reachable viewpoints more than dense 3D geometry.

The current project already has a working 2D path:

- `src/go2w_nav/launch/slam.launch.py`
- `src/go2w_nav/config/slam_toolbox.yaml`
- `src/go2w_nav/launch/nav2_slim.launch.py`
- `src/go2w_nav/config/nav2_params_slim.yaml`
- `docs/slam_runbook.md`
- `docs/room_calibration.md`

The 2D route has fewer dependencies:

- Input is `/scan`, `/odom`, and TF.
- `slam_toolbox` publishes `/map` and `map -> odom`.
- Nav2 consumes `/map`, `/scan`, `/odom`, and publishes `/cmd_vel`.
- No MID360 IMU time sync, point-cloud deskewing, FAST_LIO frame bridge, or competing `map -> odom` publisher is required.

FAST_LIO remains a future precision upgrade for 3D mapping and better geometric reasoning. It should not block the first product loop.

## Mapping And Localization Algorithm

### Mapping

Use `slam_toolbox` asynchronous 2D SLAM.

Inputs:

- `/scan`: LaserScan projected from the dog LiDAR in `base_link`.
- `/odom`: wheel-odometry integration assisted by IMU yaw.
- `odom -> base_link`: published by `nx_sensor_node`.

Core algorithm:

- Laser scan matching estimates relative motion against recent scans.
- Pose graph nodes are added when motion exceeds configured thresholds.
- Loop closure searches and matches previously visited places.
- Ceres solves the pose graph.
- An occupancy grid map is produced at `0.05 m` resolution.

Configured properties:

- `use_scan_matching: true`
- `do_loop_closing: true`
- `solver_plugin: solver_plugins::CeresSolver`
- `minimum_travel_distance: 0.3`
- `minimum_travel_heading: 0.3`
- `map_update_interval: 2.0`
- `use_sensor_data_qos: false` to match the current reliable `/scan` publisher.

Output:

- `/map`: 2D OccupancyGrid.
- `map -> odom`: SLAM correction transform.
- Saved map artifacts: `.posegraph + .data`.

### Localization

Use `localization_slam_toolbox_node`, not AMCL.

The node loads the saved `.posegraph + .data`, then continuously matches live `/scan` against the map and publishes `map -> odom`.

Final TF chain:

```text
map --slam_toolbox--> odom --nx_sensor_node--> base_link
```

Only one node may publish `map -> odom`. FAST_LIO, AMCL, and slam_toolbox must not run as localization sources at the same time.

## Navigation And Obstacle Avoidance

Use Nav2 slim.

Global planning:

- Nav2 `NavfnPlanner`.
- `use_astar: true`.
- Goal frame: `map`.

Local control:

- `DWBLocalPlanner`.
- Samples candidate velocity commands.
- Scores trajectories with obstacle, path alignment, path distance, goal alignment, goal distance, oscillation, and rotate-to-goal critics.

Obstacle handling:

- Local costmap uses `/scan` obstacle layer and inflation layer.
- Global costmap uses static map, `/scan` obstacle layer, and inflation layer.
- Robot footprint is configured as a rectangle around the Go2W body.
- Search logic never drives `/cmd_vel` directly for autonomous navigation.

## Room Search Algorithm

The primary algorithm is room-bounded next-best-view active search.

Fixed lawnmower coverage remains only a fallback for simple known open rooms.

### Active Search Loop

```text
ENTER_ROOM
-> UPDATE_ROOM_MAP_AND_COSTMAP
-> EXTRACT_FRONTIERS_AND_UNOBSERVED_CELLS
-> SAMPLE_VIEWPOINTS
-> SCORE_NEXT_BEST_VIEW
-> NAVIGATE_WITH_NAV2
-> GIMBAL_SCAN
-> YOLO_PERSON_DETECT
-> LIDAR_RANGE_LOCALIZE_PERSON
-> UPDATE_COVERAGE_AND_PERSON_MARKERS
-> repeat until complete
```

### Search Space

The search is constrained to the target room from `config/rooms.yaml`.

The room definition provides:

- `nav_pose`: entry or initial search pose.
- `search_area`: room boundary approximation in `map`.
- `target_classes`: for this command, always `["person"]`.

For `room="__current__"`, resolve the room by current robot pose:

1. Read robot pose in `map`.
2. Find a `search_area` containing the pose.
3. If none contains it, choose the nearest room `nav_pose`.
4. If pose or room map is unavailable, fail the mission with a clear reason.

### Viewpoint Generation

Generate candidates from:

- Frontiers between known free space and unknown cells.
- Unobserved cells inside the target room.
- Safe points near room corners, doorways, and furniture boundaries.
- Existing coverage waypoints when the room is already well mapped.

Discard candidates that:

- Are outside the room boundary.
- Are not reachable by Nav2.
- Are too close to obstacles after inflation.
- Have poor line-of-sight coverage.

### Next-Best-View Scoring

Score candidate viewpoints with:

```text
score =
  information_gain
+ visual_coverage_gain
+ occlusion_reduction_gain
- nav_path_cost
- obstacle_risk_cost
- repeated_observation_penalty
```

Definitions:

- `information_gain`: number of unknown cells expected to become observed.
- `visual_coverage_gain`: room area visible from the candidate with the gimbal scan pattern.
- `occlusion_reduction_gain`: value for viewpoints that see behind furniture or around corners.
- `nav_path_cost`: Nav2 path length and planning cost.
- `obstacle_risk_cost`: distance to obstacles and narrow-passage penalty.
- `repeated_observation_penalty`: penalty for repeatedly scanning already-observed cells.

The selected candidate becomes one Nav2 `NavigateToPose` goal. If Nav2 rejects or aborts it, mark the candidate blocked and choose the next best candidate.

### Completion Condition

The search completes when all are true:

- No reachable high-value frontiers remain inside the room.
- Observed visual coverage exceeds the configured threshold.
- The active person marker set has been updated from all successful scan stops.

Recommended initial thresholds:

- `observed_area_ratio >= 0.90`
- `frontier_score < 0.05`
- at least one full gimbal sweep at each accepted viewpoint

## Gimbal Coordination

Navigation and viewing are separate but coordinated.

Robot path planning answers: where should the robot stand?

Gimbal planning answers: where should the camera look from that stand point?

At each selected viewpoint:

1. Stop and wait for pose stability.
2. Record robot `map` pose.
3. Sweep gimbal yaw across the room-visible range.
4. Optionally adjust pitch for near and far objects.
5. Run YOLO on each captured frame.
6. If a person is detected, center the gimbal on the bbox once and capture a higher-quality photo.
7. Use LiDAR range to localize the detection.
8. Update the observed-area mask and person markers.

Default first-pass gimbal sweep:

```text
yaw:   -90 deg, -60 deg, -30 deg, 0 deg, +30 deg, +60 deg, +90 deg
pitch: camera-level first; add low pitch only if near-field misses are observed
```

If the current gimbal hardware cannot provide angle feedback, the first implementation must keep an internal commanded-angle estimate and mark localization quality accordingly.

## YOLO-Only Person Detection

Detection uses YOLO only.

Task target class:

```json
["person"]
```

Each detection record contains:

```json
{
  "class": "person",
  "confidence": 0.87,
  "bbox": [120, 80, 260, 430],
  "frame_width": 1280,
  "frame_height": 720,
  "source": "yolo"
}
```

Model output from non-person classes is ignored for this mission.

The system may use the dog camera or the C13 visible-light stream as the YOLO source. The selected source must have known horizontal FOV and an estimated transform to `base_link`.

## Person Localization With LiDAR

Product-grade map marking requires LiDAR range.

Vision provides bearing. LiDAR provides distance. SLAM provides robot pose.

### Bearing

For each YOLO bbox:

```text
bbox_center_x = (x1 + x2) / 2
camera_angle = (bbox_center_x / frame_width - 0.5) * camera_hfov
person_bearing_base = camera_yaw_in_base + gimbal_yaw + camera_angle
person_bearing_map = robot_yaw + person_bearing_base
```

### Range

Use the live LaserScan:

1. Convert `person_bearing_base` to the LaserScan angle frame.
2. Select rays in a small window around the bearing, initially `±5 deg`.
3. Reject invalid ranges, too-near ranges, and too-far ranges.
4. Use median range for robustness.
5. If no range exists, keep a bearing-only observation and schedule another viewpoint rather than emitting a high-confidence map point.

### Map Coordinate

```text
person_x = robot_x + range * cos(person_bearing_map)
person_y = robot_y + range * sin(person_bearing_map)
```

Localization quality levels:

- `range_lidar`: LiDAR range found; usable map marker.
- `multi_view`: triangulated or confirmed from multiple viewpoints.
- `bearing_only`: no range; not final unless confirmed from another viewpoint.
- `unresolved`: detection saved as photo but not trusted as a map coordinate.

The map UI should visually distinguish `range_lidar` and `bearing_only` results.

## Person Deduplication

The mission must not create one marker per frame.

Deduplication is spatial first:

- If a new `range_lidar` person observation is within `0.7 m` of an existing marker, merge it.
- Keep the highest-confidence photo.
- Update the marker position using a weighted average biased toward LiDAR-confirmed observations.

IDs:

```text
person_001
person_002
person_003
```

The first product version does not need visual re-identification. Spatial deduplication is sufficient for room-scale search.

## Photo And Mission Artifact Storage

Every confirmed new person stores:

```text
web/static/missions/<mission_id>/
  person_001_raw.jpg
  person_001_annotated.jpg
  person_001_crop.jpg
  person_001.json
```

Metadata:

```json
{
  "id": "person_001",
  "class": "person",
  "confidence": 0.87,
  "world_x": 2.34,
  "world_y": 1.42,
  "robot_x": 1.80,
  "robot_y": 1.10,
  "robot_yaw": 0.20,
  "bbox": [120, 80, 260, 430],
  "photo_url": "/missions/<mission_id>/person_001_annotated.jpg",
  "crop_url": "/missions/<mission_id>/person_001_crop.jpg",
  "range_source": "lidar",
  "position_quality": "range_lidar",
  "timestamp": 1782979200.0
}
```

The final `mission_report` includes all person metadata.

## Map UI Design

The live map should show these layers:

1. Occupancy map or accumulated obstacle points.
2. Current LaserScan.
3. Robot pose and trail.
4. Target room boundary.
5. Observed visual coverage mask.
6. Planned candidate viewpoints.
7. Accepted next-best-view path.
8. Person markers.
9. Photo popover for each marker.

Expected map appearance:

```text
+------------------------------------------------+
| unknown                                         |
|   ###########           ###########             |
|   #         #           #         #             |
|   # 客厅    #-----door--# 卧室    #             |
|   #         #           #   P1 ▲  #             |
|   #  B2     #           #         #             |
|   ###########           ###########             |
|        |                                       |
|        | corridor                              |
|     dog▶      B1                               |
+------------------------------------------------+
```

Legend:

- `#`: wall or obstacle from LiDAR SLAM.
- `dog▶`: robot pose.
- `B1/B2`: selected or candidate next-best-view points.
- `P1`: confirmed person marker with photo.
- Unknown space remains visually distinct from free space.

## Command And Task Contract

Voice or text command:

```text
去搜索这个房间，把所有人标注出来
```

Task:

```json
{
  "type": "search_room",
  "priority": 8,
  "params": {
    "room": "__current__",
    "target_classes": ["person"],
    "require_photos": true,
    "mark_on_map": true,
    "search_strategy": "next_best_view",
    "use_lidar_person_range": true
  }
}
```

The parser should also support:

- `搜索客厅，把所有人标注出来`
- `去卧室找人`
- `搜索这个房间的人`

All map-marking person-search commands map to `target_classes: ["person"]`.

## Failure Handling

Mission failures must be explicit.

Important reasons:

- `no_room_map`: `rooms.yaml` missing or invalid.
- `no_pose`: current robot pose unavailable.
- `no_room`: named or current room cannot be resolved.
- `no_nav`: Nav2 action server unavailable.
- `nav_aborted`: Nav2 rejected or aborted the selected viewpoint.
- `no_yolo`: YOLO model unavailable.
- `no_lidar_range`: person seen but no valid LiDAR range; keep photo but do not publish final marker.
- `cancelled`: operator stop or emergency stop.

`no_lidar_range` is not a full mission failure by itself. It is an observation-quality failure. The active-search loop should try another viewpoint before giving up on the person coordinate.

## Existing Code To Reuse

Reuse:

- `web/nx_room_orchestrator.py`: room task state machine, Nav2 action wrapper, mission report foundation.
- `web/nx_web_server.py`: HTTP API, WebSocket broadcast, task manager, SLAM map broadcast.
- `web/nx_ai_node.py`: YOLO frame loop and detection cache.
- `web/static/map.js`: 2D canvas map renderer.
- `config/rooms.yaml`: room definitions and room boundary seed.
- `src/go2w_nav`: slam_toolbox and Nav2 slim launch/config.

Extend:

- `RoomSearchOrchestrator`: active-search phases, NBV selection, person mission state.
- `NxAiEngine`: YOLO-only person detection source contract, current-frame extraction for photo saving.
- `NxWebNode`: expose LaserScan snapshot for person ranging.
- `map.js` and `panel.html`: person markers, photo popover, coverage layer, mission report panel.

## Validation Plan

Offline tests:

- Room resolution from named room and `__current__`.
- Candidate viewpoint generation respects room bounds and obstacle inflation.
- Next-best-view scoring chooses high-information, reachable points.
- YOLO detections are filtered to `person`.
- Bearing plus LaserScan range produces expected map coordinates.
- Person deduplication merges nearby observations.
- Mission artifact paths and metadata are generated correctly.
- WebSocket contracts for `search_room`, `mission_report`, and `slam.data.detections`.

Mock integration tests:

- Fake Nav2 success: mission completes with person markers.
- Fake Nav2 abort: candidate is rejected and replanning occurs.
- Fake YOLO person plus fake LaserScan range: marker appears at expected map coordinate.
- Fake YOLO person with no LaserScan hit: photo saved, marker quality is `bearing_only` or unresolved.

Hardware acceptance:

- Build a 2D map of the test indoor space.
- Save and reload localization map.
- Calibrate at least one room in `rooms.yaml`.
- Say or send the target command.
- Dog resolves the room, enters it, chooses active viewpoints, avoids obstacles, sweeps the gimbal, detects all visible people, saves photos, and marks each person on the map.

## Success Criteria

The product loop is successful when:

1. The command produces a `search_room` person-search mission without manual UI selection.
2. The dog navigates through Nav2 goals without raw autonomous `/cmd_vel` control from search logic.
3. The search strategy adapts to obstacles and unknown space using next-best-view scoring.
4. YOLO is the only person detector.
5. Every final person marker has LiDAR-backed range or is clearly marked as unresolved.
6. Every confirmed person marker has a saved annotated photo and crop.
7. The map shows room boundary, explored/observed state, dog trail, viewpoints, and person markers.
8. The final mission report lists all detected people with coordinates, photos, confidence, and localization quality.

## Non-Goals For This Phase

- FAST_LIO production integration.
- 3D reconstruction.
- LocateAnything or open-vocabulary detection.
- Visual re-identification across rooms.
- Fully automatic room segmentation from raw SLAM map.
- Multi-floor search.
