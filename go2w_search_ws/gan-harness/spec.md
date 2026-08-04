# Product Specification: Go2W 阶段A — Web 通信层上移 NX

> Generated from brief: "为 Go2W 机器狗项目"阶段A：web 通信层上移 NX"产出完整实现规格"
> 主管角色: GAN Planner（产品/架构规格，不含代码实现）
> 状态: 待 Generator 实现待 Critic 审查
> 范围: 纯软件，不实车（硬件在装），但前后端 + 本机 ROS2 话题必须能通

---

## 0. 规格阅读约定

- 所有路径均为**相对仓库根** `go2w_search_ws/`（Generator 实现时拼绝对路径）。
- 每个文件给三段：**职责 / 关键签名 / 实现要点**。签名是契约，实现要点是约束。
- "前端无感切换"是最高优先级约束：**禁止改动 `panel.html` / `map.js` 的 JS 业务逻辑与 WS 消息字段名**。
- "不动 nx_motion_node 控狗逻辑"是硬红线：**禁止修改 `nx_motion_node.py` 的状态机 / SDK 调用 / `_do_*` 方法**。

---

## 1. Vision（目标态）

载荷 Orin NX 跑一个 web 服务（HTTP:8000 + WebSocket:8001），PC 浏览器**直连 `NX_IP:8000`**。该 web 服务**内嵌 rclpy 节点**，直接在本机发布 `/cmd_vel` `/cmd_pose`（被 `nx_motion_node` 消费）并订阅本机 `/dog_state` `/imu` `/scan` `/odom`（被 `nx_sensor_node` / `nx_motion_node` 发布）。**链路里不再有 docker / cmd_publisher / dog_state.json**。PC 摆脱 go2w_humble 容器，前端只改"连接的目标地址"，其余 UI、键盘控制、地图渲染、任务队列、WS 消息全部复用。

一句话验收：**PC 浏览器访问 `http://<NX_IP>:8000`，能看到 mock 狗状态推送、能发 `/api/move` 让 `ros2 topic echo /cmd_vel` 在 NX 上看到对应 Twist；关掉 docker 容器后页面照常工作。**

---

## 2. 现状链路 vs 目标链路（对比图，Generator 必读）

### 现状（过渡态，要退役）
```
浏览器(PC) → panel.py(PC,HTTP8000/WS8001)
   └─ RosRobotBridge ──subprocess──> docker exec go2w_humble
          ├─ cmd_publisher.py(stdin JSON → /cmd_vel /cmd_pose) ──热点DDS──> nx_motion_node(NX)
          └─ ros_to_json.py(订阅 NX /imu /scan /odom → dog_state.json) ──文件──> panel.py.broadcast_loop
```
痛点：panel.py 在 PC Python3.8 无 rclpy → 必须借容器；状态靠文件回传（3s 新鲜度、原子写、磁盘 IO）；移动指令 20Hz 靠 stdin 管道续命；热点 DDS 单向故障会断控。

### 目标（阶段A，本次实现）
```
浏览器(PC) ──HTTP/WS──> nx_web_node(NX,HTTP8000/WS8001, 内嵌 rclpy)
                          ├─ 发布 /cmd_vel /cmd_pose (本机)  → nx_motion_node(NX, 本机零延迟)
                          └─ 订阅 /dog_state /imu /scan /odom (本机) → 推 WS
```
收益：零跨网指令延迟；状态新鲜度从 3s → 实时；PC 无需 docker/容器/文件桥；热点断了，NX web 仍能在本机 127.0.0.1 调试。

---

## 3. 关键设计决策（已拍板，给推荐 + 理由）

### 决策 1：NX web 服务如何组织 → **推荐 (c) 独立 Python web 进程内嵌 rclpy，落地为新文件 `web/nx_web_server.py`**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 复活 `src/go2w_web` 包做 rclpy 节点 | 符合 ROS2 包规范，可 `ros2 run` | 现有 `web_bridge_node.py` 订阅的是 `/camera/image_raw` `/map` `/go2w/task_queue` 等**当前不存在的活体话题**，且 panel.py 的 HTTP API 契约（`/api/connect` `/stand` `/sit` `/e_stop` `/command` `/search` `/api/foxglove`）它**完全没有**，复活=重写；还要解决 ament 包安装 static 资源路径问题 | ❌ 不选，沉没成本高 |
| (b) 把 `web/panel.py` 改造为"NX 本机模式" | HTTP API 契约天然一致，复用 TaskManager/broadcast_loop | panel.py 强依赖 `ai/detector.py`(YOLO) `ai/vlm.py`(Qwen) `audio/capture.py`，这些**不应在 NX web 服务进程内**（NX GPU 要留给后续 FAST_LIO/YOLO 迁移，且会让 web 服务启动变慢到几十秒）；用环境变量切换会让单文件臃肿成"双模巨怪"，Critique 必挂 | ⚠️ 不选，耦合太重 |
| **(c) 新建 `web/nx_web_server.py`，内嵌 rclpy 节点 + HTTP + WS** | **完全复用 panel.py 的 HTTP API 契约和 WS 广播格式（直接照抄 `create_server`/`run_ws`/`broadcast_loop` 的消息结构）**，但**不 import ai/ audio/**，是干净的瘦 web 服务；可由 systemd 独立管理；不污染 panel.py（PC 端过渡期仍能用） | 需要新写一个文件，约 400 行 | ✅ **推荐** |

**理由总结**：(c) 是"复用契约、不复用实现"的最佳平衡——把 panel.py 里与 web 协议相关的部分（HTTP handler、WS broadcast、死推算地图、状态读取）抽到新文件，把 ai/视频/VLM 这些重依赖彻底留在 PC（阶段A 不迁移 AI）。新文件叫 `nx_web_server.py`，放在 `web/` 下（和 panel.py 同目录，static 资源共用 `web/static/`）。

> ⚠️ Generator 注意：panel.py **不删不改**（PC 过渡期 fallback），只是 NX 上不再启动它。退役在决策 3。

### 决策 2：rclpy + HTTP + WS 单进程线程模型 → **推荐"四线程 + 一个 asyncio loop"模型**

参考 panel.py 现有线程模型（main 主线程跑 HTTPServer.serve_forever、daemon 线程跑 run_ws 的 asyncio loop、daemon 线程跑 broadcast_loop），新增 rclpy 后变成：

```
主线程 (main)
  └─ HTTPServer.serve_forever()        # 同 panel.py, http.server 阻塞式
线程1 (daemon) = rclpy.spin(node)       # 新增: ROS2 回调执行线程
线程2 (daemon) = WS asyncio loop        # 同 panel.py run_ws, run_forever
线程3 (daemon) = broadcast_loop()       # 同 panel.py, 但数据源从 _read_ros2_state() 文件 → 改读 node 内存的订阅缓存
```

**关键约束**：
1. **rclpy 必须独立线程 spin**，不能和 HTTP 同线程（HTTP handler 会 publish，publish 不阻塞但 spin 是阻塞循环）。参考 `cmd_publisher.py:88-90` 的 `spin_th = threading.Thread(target=rclpy.spin, ...)`。
2. **HTTP handler 里的 publish 跨线程安全**：rclpy 的 publisher.publish() 本身线程安全（ros2 官方文档保证），HTTP handler 线程可直接调 `node.cmd_vel_pub.publish(twist)`。Generator 实现时确认 publisher 在 node 构造时创建一次，handler 只 publish。
3. **WS 广播跨线程**：复用 panel.py 的 `ws_broadcast()` + `asyncio.run_coroutine_threadsafe(_async_broadcast(msg), WS_LOOP)` 模式（panel.py:95-105），broadcast_loop 在线程3 调 ws_broadcast 把订阅缓存数据推前端。
4. **订阅缓存用 `threading.Lock` 保护**：rclpy 回调线程（线程1的 spin 内）写，broadcast_loop（线程3）读。参考 nx_sensor_node.py:82 `self._lock` 的模式。
5. **不要用 MultiThreadedExecutor**：单线程 executor 足够（话题频率都 ≤50Hz），MultiThreaded 会引入回调并发复杂性，与 HTTP/WS 线程叠加后锁设计变难。

**进程退出顺序**（SIGINT/KeyboardInterrupt）：
`rclpy.shutdown()` → `node.destroy_node()` → HTTPServer.shutdown() → WS loop.stop()。参考 nx_motion_node.py:263-272 的 finally 模式。**web 服务进程不控狗，destroy_node 不需要 Damp**（与 nx_motion_node 不同）。

### 决策 3：退役顺序 → **分 4 步退役，每步可独立验证，前 3 步都在阶段A 完成**

| 步骤 | 退役对象 | 何时退役 | 退役方式 | 验证 |
|---|---|---|---|---|
| R1 | `dog_state.json` 文件桥 | Sprint 2 完成后（NX web 能订阅 /dog_state /imu /scan /odom 后） | NX web 的 broadcast_loop 直接读订阅缓存，**不再 read 文件**；panel.py 的 `_read_ros2_state()` 保留（PC fallback）但 NX 路径不调用 | `ros2 topic echo /dog_state` 在 NX 有数据 + 浏览器看到状态 |
| R2 | `web/cmd_publisher.py` + `RosRobotBridge` 的 docker subprocess | Sprint 3 完成后（NX web 能直接 publish /cmd_vel /cmd_pose 后） | NX web 用 rclpy publisher 直接发；RosRobotBridge 类**保留代码不删**（PC fallback），但 NX 上不实例化 | NX 上 `ros2 topic echo /cmd_vel` 看到前端发的 Twist |
| R3 | PC 的 `go2w_humble` Docker 容器 | Sprint 4（端到端验证）通过后 | 改 `web/start_ros2.sh` 为 `web/start_nx_web.sh`（在 NX 上 ssh 启动 nx_web_server，PC 不再起容器）；`start_ros2.sh` 改名 `start_ros2.sh.legacy` 保留 | PC 不跑 docker ps，浏览器仍能控 |
| R4 | `web/ros_to_json.py` | R3 后 | 该文件职责完全被 NX web 订阅替代，**保留文件不删**（PC fallback + 历史参考），从 start 脚本移除引用 | grep 启动脚本无引用 |

**铁律：本阶段只"停用"不"删文件"**。所有退役对象（cmd_publisher / ros_to_json / dog_state.json 写入 / docker 容器）的源文件保留，只是不再被 NX 路径调用。这样 PC fallback 可随时回退，符合 GAN critic "可回滚"要求。

---

## 4. ROS2 话题接线表（NX web 服务视角）

| 方向 | 话题 | 消息类型 | 对端节点 | 频率 | QoS | 用途 |
|---|---|---|---|---|---|---|
| **发布** | `/cmd_vel` | `geometry_msgs/Twist` | → `nx_motion_node` 订阅 | 按需（前端按键 ~3Hz 续发，看门狗兜底） | RELIABLE, depth=10 | 速度指令 vx/vy/vyaw |
| **发布** | `/cmd_pose` | `std_msgs/String` | → `nx_motion_node` 订阅 | 按需 | RELIABLE, depth=10 | "stand"/"sit"/"estop" |
| **订阅** | `/dog_state` | `std_msgs/String`（JSON） | ← `nx_motion_node` 发布 | 2Hz（nx_motion_node `create_timer(0.5)`） | RELIABLE, depth=10 | 狗状态机：STOPPED/MOVING/STANDING/... + vx/vy/vyaw |
| **订阅** | `/imu` | `sensor_msgs/Imu` | ← `nx_sensor_node` 发布 | 50Hz | `qos_profile_sensor_data`（BEST_EFFORT） | yaw 朝向（给地图） |
| **订阅** | `/scan` | `sensor_msgs/LaserScan` | ← `nx_sensor_node` 发布 | 10Hz | `qos_profile_sensor_data` | 雷达扫描点（给地图） |
| **订阅** | `/odom` | `nav_msgs/Odometry` | ← `nx_sensor_node` 发布 | 50Hz | RELIABLE, depth=10 | 死推算 xy 位姿（阶段B 被 FAST_LIO 取代） |

**坐标系约定（关键，勿错）**：
- 前端 `vyaw` 正=左转。`/cmd_vel.angular.z` 正=左转（ROS REP-103）。**直接透传，不反转**。
- 真正的反转在 `nx_motion_node` 里做（Go2W SDK z 正=右转，nx_motion_node.py:120 已处理）。**NX web 不做任何坐标变换**。
- 这点必须和 panel.py 的 `RosRobotBridge.move()`（panel.py:377-380）一致——它也是直接透传 `{"type":"vel","vx":vx,"vy":vy,"vyaw":vyaw}`。

---

## 5. 前后端通信协议（必须与 panel.py 一致，前端无感）

### 5.1 HTTP API 表（照抄 panel.py `create_server`，路径/方法/响应字段一字不改）

| 方法 | 路径 | 参数 | 响应 | NX web 行为 |
|---|---|---|---|---|
| GET | `/` 或 `/index.html` | - | 返回 `web/static/panel.html` | 读 static 文件返回 |
| GET | `/map.js` | - | 返回 `web/static/map.js` | 读 static 文件返回 |
| GET | `/api/foxglove` | - | `{"url":"http://<IP>:8080","ws":"ws://<IP>:8765"}` | IP 取 `GO2W_PUBLIC_IP` env |
| GET | `/api/status` | - | `{"connected":bool,"imu_yaw":float,"stats":{},"tasks":{}}` | connected=`/dog_state` 3s 内有数据则 true；imu_yaw 从订阅缓存；tasks 从 TaskManager |
| POST | `/api/connect` | - | `{"ok":true,"msg":"已连接"}` | NX 模式：发 `/cmd_pose`="stand"（让 nx_motion_node 站立），connected 标位置 true |
| POST | `/api/stand` | - | `{"ok":true}` | 发 `/cmd_pose`="stand" |
| POST | `/api/sit` | - | `{"ok":true}` | 发 `/cmd_pose`="sit" |
| POST | `/api/stop` | - | `{"ok":true}` | 发零速 `/cmd_vel`（Twist 全零） |
| POST | `/api/e_stop` | - | `{"ok":true}` | 发 `/cmd_pose`="estop" + TaskManager.cancel_all() |
| POST | `/api/move` | query: `vx,vy,vyaw` | `{"ok":true}` | 发 `/cmd_vel` Twist(linear.x=vx, linear.y=vy, angular.z=vyaw) |
| POST | `/api/command` | query `text` 或 body JSON `{"text":...}` | `{"ok":true,"text":...}` | 调 TaskManager.process_command(text) |
| POST | `/api/search` | query: `width,height,spacing,origin_x,origin_y,pattern` | `{"ok":true,"msg":"搜索..."}` | 调 TaskManager.add_list([{type:search_area,...}]) |

> ⚠️ `_json()` 响应必须带 `Access-Control-Allow-Origin: *`（panel.py:714），否则浏览器跨端口报 CORS。

### 5.2 WebSocket 消息格式（照抄 panel.py `broadcast_loop`，type 字段名一字不改）

NX web 的 WS server 监听 **8001**（与 panel.py 一致）。前端 `connectWS()` 用 `${proto}://${location.hostname}:8001`（panel.html:378-380），所以**前端访问 `http://NX_IP:8000` 时，WS 会自动连 `NX_IP:8001`**——前端零改动。

NX web 的 broadcast_loop 必须发以下消息（字段名严格匹配前端解析逻辑 panel.html:382-409）：

```jsonc
// 1. 状态 (对应 panel.py:830-834, 前端 type==='status')
{"type":"status",
 "imu_yaw": 1.23,                    // 来自 /imu 订阅缓存的 yaw
 "stats": {"imu_count": N, "robot_mode": 0, "connected": true},
 "dog_state": "STOPPED",             // 来自 /dog_state 订阅缓存
 "tasks": {"active": null, "pending": [...], "completed_count": 0}}

// 2. 地图 (对应 panel.py:822-829, 前端 type==='slam')
{"type":"slam",
 "data": {
   "x": 0.5, "y": 0.2, "yaw": 1.23,  // 来自 /odom 订阅缓存 (xy) + /imu (yaw)
   "trail": [[0,0],[0.1,0.1],...],    // 本地累积 (每 0.3m 采样一个点, 上限 2000)
   "map": [],                          // 阶段A 无栅格地图
   "scan": [[x,y],...],                // 来自 /scan, 转世界坐标 (yaw 旋转), 截断 200 点
   "detections": [],                   // 阶段A 无 AI (AI 在 PC)
   "waypoints": [], "currentWP": -1,
   "slam_source": "ros2_nx"            // 关键标识: 前端地图右上角会显示 "SLAM: ros2_nx"
 }}

// 3. 任务列表变化 (TaskManager 内部 ws_broadcast, 对应 panel.py:433/450/560/586)
{"type":"tasks","data":{"active":null,"pending":[...],"completed_count":0}}

// 4. VLM 解析结果 (TaskManager.process_command, 对应 panel.py:466)
{"type":"vlm","data":{"text":"前进两米","response":"前进","tasks":[...]}}

// 5. 搜索发现 (TaskManager._execute_search, 对应 panel.py:639/646)
{"type":"search","data":{"found":["person(85%)"]}}
```

> **关键差异说明（Generator 必读）**：panel.py 的 broadcast_loop 在 ROS2 模式下还发 `{"type":"frame","data":"<base64 jpeg>"}`（视频帧，panel.py:847-849）。**阶段A 的 NX web 不发 frame**——因为视频流来自狗的 VideoClient（unitree SDK），而 NX web 决策 1 选了"不直连狗 SDK"。前端 `type==='frame'` 的处理逻辑（panel.html:384-390）会自然显示"等待视频..."，这是**预期的阶段A 行为**，不是 bug。在 spec 的 Evaluation Criteria 里会标注"视频流 N/A"。

---

## 6. 新建/修改文件清单（文件级 + 函数级，Generator 直接实现）

### 6.1 新建文件

#### `web/nx_web_server.py`（核心，约 400 行）
**职责**：NX 上跑的 web 服务进程，内嵌 rclpy 节点，提供 HTTP API + WS 广播，本机发布 /cmd_vel /cmd_pose，订阅 /dog_state /imu /scan /odom。

**关键签名**：
```python
# ---- 全局 (照抄 panel.py:92-105) ----
WS_CLIENTS: set
WS_LOOP: asyncio.AbstractEventLoop | None
def ws_broadcast(data: dict) -> None
async def _async_broadcast(msg: str) -> None

# ---- rclpy 节点 (新增) ----
class NxWebNode(rclpy.node.Node):
    def __init__(self) -> None
        # 参数: host(0.0.0.0), port(8000), ws_port(8001), state_timeout(3.0)
        # 发布器: /cmd_vel (Twist), /cmd_pose (String)
        # 订阅: /dog_state, /imu, /scan, /odom
        # 缓存 + Lock: _dog_state, _imu_yaw, _scan_ranges, _odom_x, _odom_y, _last_state_t
    def _on_dog_state(self, msg: String) -> None          # JSON 解析 → 缓存 state/vx/vy/vyaw
    def _on_imu(self, msg: Imu) -> None                   # 四元数 → yaw (复用 ros_to_json:52-55 公式)
    def _on_scan(self, msg: LaserScan) -> None            # ranges → 缓存
    def _on_odom(self, msg: Odometry) -> None             # pose.pose.position.x/y → 缓存
    def publish_cmd_vel(self, vx, vy, vyaw) -> None       # HTTP handler 调用
    def publish_cmd_pose(self, cmd: str) -> None          # 'stand'/'sit'/'estop'
    def get_status_snapshot(self) -> dict                 # 给 /api/status 用

# ---- 机器人抽象 (替代 RosRobotBridge, 公共 API 一致) ----
class NxRobotBridge:
    """与 RosRobotBridge 公共 API 完全一致 (move/stop_move/stand/sit/e_stop
    + connected/imu_yaw/stats/_lock/_vx), 让 panel.py 的 TaskManager 无感复用。"""
    def __init__(self, node: NxWebNode)
    @property
    def connected(self) -> bool                           # = node._last_state_t 距今 < 3s
    @property
    def imu_yaw(self) -> float
    @property
    def robot_state(self) -> str                          # 兼容 panel.py:833 getattr(robot,'robot_state')
    @property
    def stats(self) -> dict
    def move(self, vx, vy, vyaw) -> None                  # → node.publish_cmd_vel + 本地缓存 _vx/_vy/_vyaw
    def stop_move(self) -> None                           # → node.publish_cmd_vel(0,0,0)
    def stand(self) -> None                               # → node.publish_cmd_pose('stand')
    def sit(self) -> None                                 # → node.publish_cmd_pose('sit')
    def e_stop(self) -> None                              # → node.publish_cmd_pose('estop')

# ---- 复用 panel.py 的: plan_lawnmower / plan_spiral / _wp_to_moves ----
# (照抄 panel.py:35-86, 内联无 ROS2 依赖, 直接复制)

# ---- 复用 panel.py 的: Task / TaskManager ----
# (照抄 panel.py:411-646, 但 TaskManager.__init__ 的 detector/vlm 参数传 None)
# 注意: TaskManager._execute_search 里的 self.detector 判断要兼容 None (阶段A 不做视觉检测)

# ---- HTTP server (照抄 panel.py:654-728 的 create_server, 把 robot 指向 NxRobotBridge) ----
def create_server(host, port, static_dir, robot, task_mgr) -> HTTPServer

# ---- WS server (照抄 panel.py:731-739 的 run_ws) ----
def run_ws(host, port) -> None

# ---- 广播循环 (改造自 panel.py:800-891) ----
def broadcast_loop(robot: NxRobotBridge, node: NxWebNode, task_mgr: TaskManager) -> None
    # 数据源: robot.imu_yaw / robot.robot_state / node 缓存的 scan/odom
    # 不再 _read_ros2_state() 文件, 不再 _update_dead_reckon() 速度积分(改用 /odom 真值)
    # scan 转 world: 复用 panel.py:815-821 的 yaw 旋转公式
    # trail 累积: 复用 panel.py:772-775 的 0.1m 采样逻辑

# ---- main ----
def main() -> None
    # 1. rclpy.init()
    # 2. node = NxWebNode(); spin 线程启动
    # 3. robot = NxRobotBridge(node); task_mgr = TaskManager(robot, vlm_engine=None, detector=None)
    # 4. 启动 run_ws 线程, task_mgr.start_worker() 线程, broadcast_loop 线程
    # 5. HTTPServer.serve_forever() 阻塞主线程
    # finally: rclpy.shutdown + node.destroy_node
```

**实现要点**：
- `NxRobotBridge` 是关键抽象——它的公共 API 必须与 `RosRobotBridge`（panel.py:324-405）**字段级一致**，因为 TaskManager 依赖 `robot.move/stop_move/stand/sit/e_stop + robot._lock/_vx/_vy/_vyaw + robot.robot_state + robot.imu_yaw + robot.stats + robot.connected`。Generator 实现后用 grep 对照 panel.py RosRobotBridge 的所有属性，逐个核对。
- `/api/connect` 在 NX 模式下调 `robot.stand()`（发 /cmd_pose="stand"），让 nx_motion_node 执行站立序列。但**不要阻塞等待站立完成**（nx_motion_node 的站立是异步的，NX web 不应等）。
- broadcast_loop 频率 0.15s（与 panel.py:839/890 一致），sleep 在 try/except 内（panel.py:891）。
- yaw 计算：从 `/imu` 的四元数 `qx,qy,qz,qw` 算 yaw，公式照抄 `ros_to_json.py:52-55`：`yaw = atan2(2(wz+xy), 1-2(y²+z²))`。注意 ros_to_json 用 `q.w q.z q.x q.y`，ROS Imu 消息 `orientation` 字段顺序是 `x,y,z,w`，Generator 注意映射。

#### `web/mock_dog_state_publisher.py`（验证用，约 60 行）
**职责**：阶段A 验证用的 mock 节点，模拟 nx_motion_node + nx_sensor_node 发布 `/dog_state /imu /scan /odom`，让 NX web 在**没有真狗/没有 nx_sensor_node**时也能端到端验证。

**关键签名**：
```python
class MockDogNode(rclpy.node.Node):
    def __init__(self)
        # 发布器: /dog_state (String), /imu (Imu), /scan (LaserScan), /odom (Odometry)
        # 定时器 2Hz 发 /dog_state (STOPPED, vx=0.2 模拟移动), 50Hz /imu (yaw 缓慢旋转), 10Hz /scan (假障碍), 50Hz /odom (xy 缓慢漂移)
def main()
```
**实现要点**：yaw 用 `time.time()` 的 sin 让地图转起来；scan ranges 用 360 个值模拟一圈障碍（前 2m、两侧 1m）；odom xy 用 sin/cos 螺旋轨迹，验证 trail 渲染。

#### `web/start_nx_web.sh`（NX 启动脚本，约 30 行）
**职责**：在 NX 上启动 nx_web_server.py，参考 `web/run_panel.sh` 的 setsid 脱离会话模式 + `docker/go2w-motion.service` 的环境变量。

**关键内容**：
```bash
#!/bin/bash
# 在 NX 本机启动 web 服务 (不是 PC!)
# 用法 (NX 上): bash /home/nx/go2w_ws/start_nx_web.sh
set -e
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$LD_LIBRARY_PATH  # 若 NX web 不直连狗SDK, 可省; 保留以兼容
cd /home/nx/go2w_ws   # 或代码部署路径
pkill -f "nx_web_server.py" 2>/dev/null || true; sleep 1
setsid bash -c 'exec python3 -u web/nx_web_server.py' \
    > /tmp/nx_web.log 2>&1 < /dev/null &
echo "nx_web 启动 PID=$!"
echo "等待初始化..."
for i in $(seq 1 10); do
    sleep 1
    grep -q "Web:" /tmp/nx_web.log 2>/dev/null && { echo "就绪 (用时 ${i}s)"; break; }
done
tail -6 /tmp/nx_web.log
```

#### `docker/go2w-web.service`（systemd 服务，约 20 行）
**职责**：让 NX web 服务开机自启，参考 `docker/go2w-motion.service`。
**关键内容**：`After=go2w-motion.service`（控狗服务先起）、`ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && exec python3 -u /home/nx/go2w_ws/web/nx_web_server.py'`、`Restart=always`、`KillSignal=SIGINT`。

#### `docker/deploy_nx_web.sh`（部署脚本，约 50 行）
**职责**：把 nx_web_server.py + mock + static + service 部署到 NX，参考 `docker/deploy_nx.sh`。
**关键步骤**：scp `web/nx_web_server.py web/mock_dog_state_publisher.py web/static/` → NX；scp service 文件；ssh 安装 systemd 服务；打印"浏览器访问 http://NX_IP:8000"。

### 6.2 修改文件（仅 2 个，且都是文档/脚本，不动核心逻辑）

#### `web/start_ros2.sh` → 改名 `web/start_ros2.sh.legacy`，新建精简版 `web/start_pc_browser.sh`
**职责**：PC 端不再起 docker/panel，只提示用户开浏览器访问 NX。
**关键内容**：
```bash
#!/bin/bash
# 阶段A: PC 只开浏览器, web 服务在 NX 上
NX_HOST="${NX_HOST:-192.168.43.41}"
echo "PC 端无需启动任何服务 (docker 容器已退役)"
echo "浏览器打开: http://$NX_HOST:8000"
echo "如果打不开, 检查 NX 上 go2w-web.service: ssh nx@$NX_HOST 'systemctl status go2w-web'"
```

#### `README.md`（仅更新"快速开始"段落）
**职责**：把"PC 端每次开机"从 `bash web/start_ros2.sh` 改为 `bash web/start_pc_browser.sh`，并加一句"web 服务由 NX 的 go2w-web.service 提供"。**不改架构图**（架构图本来就是目标态）。

### 6.3 不动文件清单（Generator 勿碰，Critic 会核对）

- `web/panel.py`（PC fallback，退役不删）
- `web/cmd_publisher.py`（退役不删）
- `web/ros_to_json.py`（退役不删）
- `web/static/panel.html`（前端无感，禁止改 JS）
- `web/static/map.js`（同上）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py`（控狗逻辑红线）
- `src/go2w_bridge/go2w_bridge/nx_sensor_node.py`（NX 传感器，本阶段不动）
- `src/go2w_web/`（休眠包，决策 1 明确不复活）

---

## 7. 分 Sprint 实现顺序（每步可独立验证，前 3 步不依赖狗硬件）

### Sprint 1：NX web 骨架 + HTTP/WS 起 + 静态服务（不接 ROS2）
**目标**：浏览器访问 `NX_IP:8000` 能看到 panel.html 页面（含地图 UI），WS 能连上但无数据。
**Features**：新建 `nx_web_server.py` 的 main + create_server（只 serve static）+ run_ws；NxWebNode 类先建空壳（不订阅不发布）；NxRobotBridge 先返回假数据。
**Definition of Done**：
- `python3 web/nx_web_server.py` 在 NX 启动，日志打印 `Web: http://0.0.0.0:8000`
- PC 浏览器访问 `http://NX_IP:8000` 看到 panel.html（顶栏、地图、任务队列布局），无 JS 报错（F12 Console 干净）
- WS 连接成功（浏览器 Network 面板 8001 连接 status=101）
- **不依赖狗硬件**：nx_motion_node / nx_sensor_node 都不需要起

### Sprint 2：rclpy 订阅链路 + broadcast_loop 推真状态（无 mock 也能跑）
**目标**：NX web 订阅 /dog_state /imu /scan /odom，broadcast_loop 把订阅缓存推 WS。前端地图能看到 yaw/位姿/扫描点。
**Features**：NxWebNode 完整实现 4 个订阅器 + 缓存；NxRobotBridge 的 connected/imu_yaw/stats/robot_state 接订阅缓存；broadcast_loop 改造（去 _read_ros2_state 文件读取）。
**Definition of Done**：
- NX 上 `ros2 topic pub /dog_state std_msgs/String '{"data":"{\"state\":\"STOPPED\",\"vx\":0}"}' -r 2` 手动发，浏览器顶栏"狗"状态点变绿、显示 STOPPED
- NX 上 `ros2 topic pub /imu ...`（或起 mock_dog_state_publisher），浏览器地图狗箭头 yaw 旋转、扫描点出现
- **关键验证**：关掉 PC 的 docker 容器（`docker stop go2w_humble`），NX web 仍正常工作（证明摆脱容器）
- **不依赖狗硬件**：用 mock_dog_state_publisher 或手动 `ros2 topic pub`

### Sprint 3：rclpy 发布链路 + HTTP API 完整（指令能下发）
**目标**：前端按键发 /api/move，NX web 发布 /cmd_vel；按"站立"发 /cmd_pose。TaskManager 复用 panel.py 逻辑。
**Features**：NxWebNode 完整实现 2 个发布器；NxRobotBridge.move/stop_move/stand/sit/e_stop；create_server 所有 /api/* 接通；Task/TaskManager/planner 复制。
**Definition of Done**：
- 浏览器按 ↑（前进），NX 上 `ros2 topic echo /cmd_vel` 看到 `linear.x: 0.4`
- 浏览器点"站立"，NX 上 `ros2 topic echo /cmd_pose` 看到 `data: stand`
- 浏览器输入"前进两米"发送，任务队列出现 move 任务（TaskManager fallback parse 生效，无 VLM）
- 输入框搜区域，任务队列出现 search_area 任务（注意：search 会调 robot.move，因 nx_motion_node 没起，/cmd_vel 发了但无人消费——这是预期，阶段A 不验控狗）
- **不依赖狗硬件**：`ros2 topic echo` 验证即可

### Sprint 4：mock 验证 + 端到端 + 退役 PC 容器
**目标**：完整验证脚本 + systemd 部署 + PC 摆脱 docker。
**Features**：mock_dog_state_publisher 完整；start_nx_web.sh + go2w-web.service + deploy_nx_web.sh；README 更新。
**Definition of Done**：
- NX 上 `bash web/start_nx_web.sh` + `python3 web/mock_dog_state_publisher.py`，PC 浏览器访问 NX:8000，地图显示模拟狗走螺旋轨迹 + 雷达扫描点，发 /api/move 后 `ros2 topic echo /cmd_vel` 有响应
- `systemctl status go2w-web` active；reboot NX 后自动起
- PC 上 `docker ps` 无 go2w_humble 容器；`bash web/start_pc_browser.sh` 只提示开浏览器
- **不依赖狗硬件**：全程 mock

### Sprint 5（可选，Nice-to-have）：连通 nx_sensor_node 真传感器
**目标**：起真 nx_sensor_node（不连狗 SDK 也能发，因为 SDK 初始化失败会 graceful 退化——见 nx_sensor_node.py:65-68）。
**Features**：无新代码，只是验证 nx_web + nx_sensor_node 同进程组共存。
**Definition of Done**：NX 上同时起 go2w-motion + nx_sensor_node + go2w-web 三个服务，浏览器看到 imu_count 增长（即使狗没连，nx_sensor_node 的 DDS 订阅会失败但不崩）。
**依赖硬件**：否（SDK 不可用时 graceful 退化）。

---

## 8. 前后端连通验证方法（不依赖狗硬件，Generator 必须实现并跑通）

### 8.1 一键验证脚本 `web/verify_nx_web.sh`（新建，约 40 行）
**职责**：NX 上启动 nx_web + mock，跑一组 curl + python websocket 客户端断言，输出 PASS/FAIL。

**验证项**（每项独立 PASS/FAIL）：
1. `curl http://localhost:8000/` → 200 + 含 `<title>Go2W 搜索控制台</title>`
2. `curl http://localhost:8000/map.js` → 200 + 含 `class Go2WMap`
3. `curl http://localhost:8000/api/status` → JSON 含 `connected` 字段
4. `curl -X POST http://localhost:8000/api/move?vx=0.3&vy=0&vyaw=0` → `{"ok":true}`，且 `ros2 topic echo /cmd_vel --once` 的 linear.x≈0.3
5. `curl -X POST http://localhost:8000/api/stand` → `{"ok":true}`，且 `ros2 topic echo /cmd_pose --once` 的 data=stand
6. python websockets 客户端连 ws://localhost:8001，5 秒内收到 type=slam 消息，slam_source=ros2_nx
7. python websockets 客户端 5 秒内收到 type=status 消息，含 dog_state 字段
8. 起 mock_dog_state_publisher 后，dog_state 从 UNKNOWN → STOPPED（证明订阅链路通）

**通过标准**：8/8 PASS。Critic 验收时跑这个脚本看输出。

### 8.2 跨机验证（PC 浏览器 → NX）
- PC 浏览器开 `http://NX_IP:8000`（非 localhost），页面正常加载
- F12 Network 面板：WS 连接是 `ws://NX_IP:8001`，status=101
- F12 Console 无报错
- 按前进键，NX 上 `ros2 topic echo /cmd_vel` 看到 0.4

---

## 9. 技术栈

- **NX 端**（nx_web_server.py）：Python3.10（Humble 默认）、rclpy（Humble）、`http.server`（标准库）、`websockets`（pip，与 panel.py 一致）、`cv2`/`numpy`（仅 scan 转坐标用，可降级为纯 math）
- **ROS2 消息**：`geometry_msgs/Twist`、`std_msgs/String`、`sensor_msgs/Imu`、`sensor_msgs/LaserScan`、`nav_msgs/Odometry`（全部标准消息，无自定义 msg 依赖）
- **前端**：零改动，纯 HTML/JS（panel.html + map.js）
- **部署**：systemd（go2w-web.service）、scp（deploy_nx_web.sh）
- **无新依赖**：NX 已装 rclpy/websockets（panel.py 在 PC 用过 websockets，NX 同环境）

---

## 10. 边界情况与状态处理（Critic 必查）

| 场景 | 期望行为 | 实现位置 |
|---|---|---|
| `/dog_state` 3s 内无数据（nx_motion_node 挂了） | `connected=false`，顶栏"狗"点变灰；地图仍推 odom/imu（来自 nx_sensor_node） | NxRobotBridge.connected 判 timeout |
| `/imu` 无数据（nx_sensor_node 没起） | imu_yaw=0，地图狗箭头不转；slam_source 仍标 ros2_nx | broadcast_loop 兜底 0.0 |
| `/scan` 无数据 | scan=[] 空数组，地图无扫描点 | broadcast_loop 兜底 [] |
| HTTP handler publish 失败（rclpy 未初始化） | logger.warning，响应仍 `{"ok":true}`（不阻塞前端） | try/except |
| WS 无客户端 | ws_broadcast 直接 return（panel.py:96 的 `if WS_LOOP and WS_CLIENTS`） | 照抄 |
| 浏览器跨端口（8000 → 8001） | 前端用 `location.hostname` 拼 WS URL，天然跨端口；CORS 头已加 | _json() 带 ACAO:* |
| SIGINT 退出 | rclpy.shutdown + destroy_node，不发 Damp（web 不控狗） | main finally |
| mock 节点先于 web 启动 | rclpy 订阅是 RELIABLE+Volatile（默认），会丢启动前的消息——无所谓，mock 持续发 | QoS 默认即可 |

---

## 11. Anti-AI-slop / 反模式清单（Generator 自查）

- ❌ 不要给 nx_web_server.py 加"健康检查/重连/心跳"复杂逻辑——panel.py 没有，照抄即可
- ❌ 不要用 FastAPI/Flask/aiohttp 替换 http.server——panel.py 用 http.server，保持一致
- ❌ 不要把 rclpy spin 和 HTTP 放同线程——会 publish 时阻塞
- ❌ 不要在 NxRobotBridge 里做坐标反转——nx_motion_node 已处理
- ❌ 不要给前端加"连接中..."loading 动画——前端无感切换，不改 panel.html
- ❌ 不要在 broadcast_loop 里 `_read_ros2_state()` 读文件——那是退役的 PC 模式
- ❌ 不要 import ai/detector 或 ai/vlm——NX web 不跑 AI（阶段A）
- ❌ 不要 MultiThreadedExecutor——单线程 + 锁足够
- ❌ 不要给 mock_dog_state_publisher 加复杂物理模拟——简单 sin/cos 螺旋即可
- ❌ 不要删 panel.py / cmd_publisher.py / ros_to_json.py——退役不删

---

## 12. Evaluation Criteria（见 gan-harness/eval-rubric.md，权重已定）

详见独立的 `gan-harness/eval-rubric.md`，Critic 直接消费。核心四维：
- 契约一致性（0.3）：HTTP API + WS 消息字段与 panel.py 逐字对齐
- 链路正确性（0.3）：订阅/发布话题、坐标系、线程模型无错
- 可验证性（0.2）：verify_nx_web.sh 8/8 PASS，不依赖狗硬件
- 退役干净度（0.2）：docker/文件桥停用但源文件保留，可回滚
