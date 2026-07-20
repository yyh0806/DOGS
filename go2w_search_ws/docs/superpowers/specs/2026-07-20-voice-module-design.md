# 语音模块：运动指令 + 房间搜索目标扩展

> 日期: 2026-07-20
> 状态: 设计已定稿，待实现
> Tech Stack: Python ≥ 3.10（PC 端 Vosk 离线 STT；NX 端 nav2 + YOLO-World）
> 入口: PC 终端 CLI `tools/voice_console.py`（麦克风接 PC）

## 目标

PC 麦克风采音 → Vosk 中文 STT → POST `/api/command` → NX 侧 NLU 解析 → 执行 → WS 反馈 → PC TTS 播报。覆盖七条语音指令：

1. 前进（带距离参数，如"前进两米"）
2. 后退（带距离参数）
3. 左转（带角度参数，如"左转 45 度"）
4. 右转（带角度参数）
5. 搜索房间并标注所有人
6. 搜索房间并标注所有桌子椅子
7. 搜索房间并标注所有物体

## 现状盘点（复用，不重写）

| 组件 | 位置 | 现状 |
|---|---|---|
| PC 语音 CLI | `tools/voice_console.py` | 成熟：Vosk STT + sounddevice + pyttsx3 TTS + WS 反馈 + 15s dedupe。**但 `validate_search_command` 只放行 search_room**，运动指令被拒 |
| 文本/语音 NLU | `web/nx_product_command.py` | `parse_product_command` 规则解析。**仅 search_room（人/桌子）**，运动指令返回 None |
| 命令入口 | `POST /api/command` → `task_mgr.submit_command` | 失败走异步 VLM 兜底。运动指令无执行通道 |
| nav2 点导航 | `web/nx_point_nav.py` `PointNavigationController.submit(x,y,yaw)` | 生产级封装：完整状态机、健康检查、超时、quarantine。发 map frame 绝对位姿，串行化（新 goal 取消旧 goal） |
| 速度控制 | `POST /api/move?vx=&vy=&vyaw=` + `NxRobotBridge.move(manual=True)` | 速度空间持续控制，安全上限 vx≤0.4 / vy≤0.3 / vyaw≤0.5；手动模式透传，自带前方 /scan 障碍 guard |
| 搜索任务 schema | `web/nx_mission_schema.py` `SearchMissionRequest` | `target_classes: tuple` **天然支持多类**；`_ALIASES` 别名表 |
| 检测器 | `ai/detector.py` `Detector` | **生产部署 YOLO-World（yolov8x-worldv2.pt）**，开放词汇。`is_world=True` → `set_classes(任意类)`；`default_classes=["person"]`；`_current_classes` 缓存避免重复 set |
| 检测器词汇表注入 | `web/nx_ai_node.py` `set_detection_targets` | `None`=全类/默认，`list`=只检这些类。任务级原子替换 + `finally` 恢复 |
| 前端快捷按钮 | `web/static/panel.html` | `quickCmd('前进两米'/'后退一米'/'左转90度'/'右转90度')` 4 个（**当前死按钮**，NLU 不识别运动）；`searchRoom()` 只搜人 |

**关键事实**：生产 systemd `go2w-web.service:43` 明确 `Environment=GO2W_YOLO_MODEL=/home/nx/go2w_ws/models/yolov8x-worldv2.pt`，部署 preflight 强制要求该模型存在。所以"指定哪类标哪类"走 YOLO-World 开放词汇，不受 COCO 80 类限制。

## 范围

**In（本次做）**
- NLU 扩展：运动指令解析（含中文数字）+ 椅子/桌椅组合/任意物品词/所有物体
- 运动执行：前进/后退走 nav2（绕行避障），左/右转走 cmd_vel+odom 闭环（精确原地转）
- `voice_console` 放行运动 + 搜索指令（保留预校验 + dedupe）
- 前端快捷按钮：加无参数"前进/后退/左转/右转"4 个 + 搜索扩展"搜人/搜桌椅/搜物体"3 个
- 多类目标标注：颜色 + 类名标签区分

**Out（不做）**
- 浏览器 Web Speech API 入口（维持 CLI，记忆里明确"绕开安全上下文限制"）
- 唤醒词 / push-to-talk（维持 Vosk 静音自动结束）
- 切换 YOLO-World ↔ 标准 YOLO（不为"所有物体"切模型）
- 连续/持续运动控制（"说前进一直走到说停"——一次性行程更安全）

## 关键决策

1. **运动指令接进 `/api/command` 的 NLU**，与 search_room 同层。语音/文本/快捷按钮三入口统一，`quickCmd('前进两米')` 死按钮自动复活，复用 arbiter 安全门与审计日志。
2. **前进/后退走 nav2，左/右转走 cmd_vel+odom**。nav2 给前进避障（绕行而非硬撞）；转向是原地旋转无撞墙风险，cmd_vel+odom 更快更精确。
3. **nav2 遇障 = 绕行或 abort，不是原地暂停**。用户明确要绕行语义。"前进两米中间有墙" → nav2 绕路到前方 2m 点；目标不可达 → abort → TTS 播报"前方不可达"。
4. **距离上限 20m，角度无上限**。仅用于拦截明显口误（Vosk 小模型数字识别会翻车，"前进一百米"截断到 20m）；原地转多少圈无所谓。
5. **默认值：前进/后退 1m，左/右转 90°**。
6. **目标 = YOLO-World 开放词汇**。NLU 抽取命令里的物品词 → 中→英词典映射 → `set_classes`。"所有物体"展开为预设室内清单（YOLO-World 必须有词汇表，不能真空检所有）。
7. **detector 默认 = 只标人**（`default_classes=["person"]`，与产品语义天然吻合）。注意：这是**检测器层面**的默认。**NLU 层面本次七条命令都带显式目标词**（人/桌椅/物体/具体物品）；"搜索这个房间"不带目标的命令**不在本次范围**，保持现状行为（NLU 不匹配 → 走 VLM 兜底或被拒）。

## 设计 Section 1：运动指令

### 1.1 NLU 解析（`nx_product_command.py` 扩展）

新增运动意图识别，与现有 search_room 解析并列。先于 search_room 匹配（避免"搜索"里的"前/后"等词误触发）。

**触发词**：
- 前进：`前进 / 向前 / 往前 / 直走`
- 后退：`后退 / 向后 / 往后 / 倒退`
- 左转：`左转 / 向左转 / 左转弯`
- 右转：`右转 / 向右转 / 右转弯`

**中文数字解析**（`_parse_distance` / `_parse_angle`）：
- 单字数字：`一二三四五六七八九` → 1-9，`十` → 10，`零` → 0
- `两` → 2；`半` → 0.5（仅"半米"合法，"半度"非法）
- 阿拉伯数字：`2 / 45 / 1.5`
- 复合：`十二 / 二十 / 一百`（基础复合，覆盖常见说法）
- 单位：距离 `米 / 公尺`，角度 `度 / °`，`圈`（1 圈=360°，半圈=180°）

**解析结果**：
- "前进" → forward, 1.0m（默认）
- "前进两米" → forward, 2.0m
- "前进半米" → forward, 0.5m
- "左转" → left, 90°（默认）
- "左转四十五度" → left, 45°
- "右转半圈" → right, 180°

**距离上限**：解析后 `min(distance, 20.0)`，截断时在 task response 标记 `clamped: true` 供 TTS 提示。角度不截断。

### 1.2 任务 schema

```python
{
  "type": "move",
  "priority": 5,
  "params": {
    "mode": "linear" | "angular",
    "direction": "forward" | "backward" | "left" | "right",
    "distance_m": 1.0,      # mode=linear 时存在
    "angle_deg": 90.0,      # mode=angular 时存在
    "clamped": False        # 距离被 20m 截断时 True
  }
}
```

`priority=5` 低于 search_room（8），高于手动控制（手动按钮直接发 `/api/move`，不经任务队列）。

### 1.3 执行（task_mgr 新增 move 任务执行器）

**前进/后退（linear，走 nav2）**：
```
1. 读当前 map pose (x, y, yaw)  ← NxWebNode.get_map_localization_snapshot
2. 算目标点:
   forward:  x' = x + distance·cos(yaw), y' = y + distance·sin(yaw), yaw' = yaw
   backward: x' = x − distance·cos(yaw), y' = y − distance·sin(yaw), yaw' = yaw
3. point_nav_controller.submit(x', y', yaw')
4. 监听状态机 → succeeded / aborted / timed_out
```
nav2 负责路径规划与避障（绕行）。`submit()` 串行化保证"连续说两次前进"时第二次取消第一次，不堆积。

**左/右转（angular，走 cmd_vel+odom 闭环）**：
```
1. 读当前 map yaw 作为基准 yaw_base
2. 算目标 yaw: left → yaw_base + angle；right → yaw_base − angle
3. 循环发 /cmd_vel(0, 0, ±vyaw=0.5) 直到 odom yaw 与目标 yaw 偏差 < 3°
4. 兜底硬超时 = (angle / 0.5) · 2 + 1s，到时强停（odom 丢失保护）
5. 发 /cmd_vel(0,0,0) 停
```
转向无撞墙风险，不走 nav2（避免 nav2 控制器对"同位置不同朝向"目标的行为不确定）。

### 1.4 反馈（WS 广播 + TTS）

| 结果 | WS `move_result` phase | TTS |
|---|---|---|
| 前进/后退成功 | `succeeded` | "已前进两米" / "已后退一米" |
| 转向成功 | `succeeded` | "已左转 90 度" |
| nav2 不可达/abort | `aborted` | "前方不可达，已取消" |
| nav2 超时 | `timed_out` | "导航超时，已停止" |
| 距离截断 | （含 clamped 标记） | "距离过长，已限到 20 米" |

## 设计 Section 2：房间搜索目标扩展

### 2.1 NLU 目标词抽取

`parse_product_command` 现有逻辑（current_room/named_room × 人/桌子）扩展为：
1. 先匹配运动指令（Section 1）
2. 再匹配搜索指令：动词（搜索/标注/标记/找）+ 房间（当前/这个/命名房间）+ 目标词
3. 目标词抽取顺序：`所有物体` → 展开清单；`桌椅` → 组合；具体物品词 → 词典映射；`人/所有人` → person

### 2.2 中→英物品词典（两处对称扩展）

**`nx_mission_schema.py` `_ALIASES`**（schema 层，归一化）+ **`nx_product_command.py` NLU 词表**（解析层）：

```
人/人员/所有人 → person
桌子/餐桌 → dining table
椅子/座椅/凳子 → chair
沙发 → couch
床 → bed
电视 → tv
冰箱 → refrigerator
微波炉 → microwave
烤箱 → oven
笔记本 → laptop
杯子 → cup
瓶子 → bottle
书 → book
钟 → clock
花瓶 → vase
绿植/盆栽 → potted plant
背包 → backpack
碗 → bowl
键盘 → keyboard
```

词典是本次支持的物品范围（~20 类）。用户说词典内的词 → 映射成英文类名（如"沙发"→`couch`）；词典内**任意**物品词都能单独或组合指定（"搜索房间里的沙发""搜索桌椅和背包"）。词典没有的中文词 NLU 不识别（命令被拒）。**要支持新物品，词典 + schema 两处各加一行**——YOLO-World 开放词汇，新词立即可检，无需重训练。

### 2.3 多类目标

- "搜索房间里所有桌子椅子" → `target_classes=["dining table", "chair"]`（schema 已支持 tuple）
- "搜索房间里所有椅子" → `["chair"]`
- 任意组合：NLU 把命令里出现的所有物品词都收进 target_classes

### 2.4 "所有物体"预设清单

NLU 识别"所有物体/全部物体/所有东西" → 展开为：

```
person, chair, couch, dining table, bed, tv, laptop, refrigerator,
microwave, oven, book, clock, vase, potted plant, backpack, bottle,
cup, bowl
```

聚焦室内家具电器大件，不标食物/动物/室外类。清单作为常量定义在 `nx_product_command.py`，可配置。

### 2.5 标注（颜色 + 类名标签）

`ai/detector.py:annotate` 现状是所有框统一绿色 + `class conf` 标签。改为：
- **每类一个固定颜色**（按类名 hash → HSV → RGB，保证同类同色、不同类异色）
- 标签显示**中文类名**（英文→中文反向词典，如 `dining table→桌`、`chair→椅`）+ 置信度
- 地图 marker 与画面框共用同一套类→色映射，用户一眼区分椅 vs 桌 vs 人

英文→中文反向映射复用 2.2 词典的逆。

## 设计 Section 3：PC 语音入口 + 快捷按钮 + 反馈

### 3.1 语音入口：PC CLI（`tools/voice_console.py`）

维持现有架构，**只改 `validate_search_command` 的放行范围**：

```python
# 旧: 只放行 search_room
def validate_search_command(text) -> dict: ...

# 新: 放行 search_room + move（所有 NLU 能解析的）
def validate_voice_command(text) -> dict:
    # 1. 先试 parse_product_command（现在能解析运动 + 搜索）
    # 2. 解析成功 → ok=True（含 task fingerprint 供 dedupe）
    # 3. 解析失败 → ok=False, reason=unsupported_voice_command
```

dedupe（15s）保留：相同 task fingerprint 在 15s 内不重复下发，防 STT 抖动重复触发。运动指令的 fingerprint 包含 direction + distance/angle，所以"前进两米"和"前进一米"是不同 fingerprint，不互相压制。

### 3.2 前端快捷按钮（`panel.html`）

**运动**（现有 4 个带参数按钮保留 + 新增 4 个无参数按钮）：
```html
<!-- 现有（保留） -->
<span class="quick-btn" onclick="quickCmd('前进两米')">前进两米</span>
<span class="quick-btn" onclick="quickCmd('后退一米')">后退一米</span>
<span class="quick-btn" onclick="quickCmd('左转90度')">左转90度</span>
<span class="quick-btn" onclick="quickCmd('右转90度')">右转90度</span>
<!-- 新增（用默认 1m/90°） -->
<span class="quick-btn" onclick="quickCmd('前进')">前进</span>
<span class="quick-btn" onclick="quickCmd('后退')">后退</span>
<span class="quick-btn" onclick="quickCmd('左转')">左转</span>
<span class="quick-btn" onclick="quickCmd('右转')">右转</span>
```

**搜索**（现有"搜索房间"按钮只搜人 → 扩展为 3 个）：
```html
<span class="quick-btn" onclick="searchRoom(['person'])">搜人</span>
<span class="quick-btn" onclick="searchRoom(['dining table','chair'])">搜桌椅</span>
<span class="quick-btn" onclick="searchRoom('all_objects')">搜物体</span>
```

`searchRoom()` 改为接受 target_classes 参数（`'all_objects'` 在前端或 NLU 层展开为清单）。直接调 `/api/search_room`，绕过 NLU（比 quickCmd 更可靠，与现有 searchRoom 一致）。

### 3.3 TTS 反馈（`voice_console._on_ws_message` 扩展）

现有处理 `mission_report` / `search_room` 两个 type。新增 `move_result`：

```python
elif mtype == "move_result":
    phase = payload.get("phase")
    direction = payload.get("direction")
    amount = payload.get("distance_m") or payload.get("angle_deg")
    if phase == "succeeded":
        speak(f"已{_dir_cn(direction)}{amount}{_unit_cn(direction)}")
    elif phase == "aborted":
        speak("前方不可达，已取消")
    elif phase == "timed_out":
        speak("导航超时，已停止")
```

搜索类反馈（已有）保留：`任务完成，在客厅找到 3 人` / 多类时 `找到 2 桌 4 椅`。

## 端到端流程

**运动指令**：
```
PC 麦克风 → Vosk STT → "前进两米"
  → voice_console.validate_voice_command → ok (move task fingerprint)
  → POST /api/command {text:"前进两米"}
  → task_mgr.submit_command → parse_product_command
  → {type:move, params:{linear, forward, 2.0m}}
  → move 执行器: 读 map pose → 算目标 → point_nav.submit(x', y', yaw)
  → nav2 规划+绕行避障+到点
  → WS 广播 move_result{succeeded}
  → PC TTS "已前进两米"
```

**搜索指令（以"搜物体"为例）**：
```
PC 麦克风 → Vosk → "搜索房间里所有物体"
  → validate_voice_command → ok (search_room fingerprint)
  → POST /api/command
  → parse_product_command → "所有物体"展开 → target_classes=18类清单
  → SearchMissionRequest(current_room, 18类, frontier_explore)
  → RoomSearchOrchestrator._run_frontier_explore
    （每帧）set_detection_targets(18类) → YOLO-World 检测 → 多色标注 + map marker
  → 任务完成 → mission_report{detections}
  → PC TTS "任务完成，找到 X 个物体"
```

## 错误处理

| 场景 | 处理 |
|---|---|
| Vosk 误识别成运动指令 | dedupe 15s + 距离上限 20m 双保险；客户端预校验拒绝非 NLU 文本 |
| nav2 不可达（目标在墙后） | nav2 abort → move_result{aborted} → TTS"前方不可达" |
| odom 丢失（转向时） | 硬超时强停，绝不失控；move_result{timed_out} |
| nav2 服务未就绪 | PointNavigationController 已有 `server_unavailable` 状态 + 重试 |
| YOLO-World set_classes 失败 | detector.detect 异常 → 返回 []（已有，视频流不断） |
| 麦克风无设备/权限 | voice_console 已有 PortAudioError 处理 |
| 控制冲突（运动 vs 搜索） | arbiter 仲裁：move priority=5，search_room priority=8，手动 `/api/move` 最高（操作员全权） |

## 测试策略

**NLU 单测**（`test_product_command.py` 扩展）：
- 运动指令：默认值、中文数字（两/半/十/二十/一百）、阿拉伯数字、单位、距离截断
- 搜索目标：椅子/桌椅组合/任意物品词/所有物体展开
- 边界：运动与搜索不互扰（"搜索"里的"前"不触发前进）；空文本/否定词

**执行器单测**（新 `test_move_executor.py`）：
- linear：mock map pose → 验证 submit 的目标点坐标正确
- angular：mock odom yaw 序列 → 验证停止时机与硬超时
- nav2 abort/timed_out 透传到 move_result

**标注单测**（`test_detector_annotate.py` 扩展）：
- 同类同色、异类异色；中文标签正确

**voice_console 单测**（`test_voice_console.py` 扩展）：
- validate 放行 move + search_room；dedupe fingerprint 含距离/角度

**端到端**（mock nav2 + mock 麦克风文本）：
- 文本模式 `--text "前进两米"` 走完整链路，验证 task 产出 +（mock）执行结果

## 受影响文件

**NX 侧（部署到 ~/go2w_ws/web/）**：
- `web/nx_product_command.py` — 加运动解析 + 数字解析 + 物品词典 + "所有物体"清单
- `web/nx_mission_schema.py` — `_ALIASES` 扩展物品别名
- `web/nx_web_server.py` 或新 `web/nx_move_executor.py` — move 任务执行器（linear→point_nav，angular→cmd_vel+odom）
- `web/task_mgr`（`nx_web_server.py` 内）— 注册 move 任务类型分发
- `ai/detector.py:annotate` — 多色 + 中文标签

**PC 侧**：
- `tools/voice_console.py` — `validate_search_command` → `validate_voice_command` 放行运动；`_on_ws_message` 加 `move_result`

**前端**：
- `web/static/panel.html` — 运动快捷按钮加无参数版；搜索按钮扩展三按钮；`searchRoom(target_classes)` 参数化

## 非目标（YAGNI）

- 浏览器麦克风入口（CLI 已满足）
- 唤醒词 / 持续监听模式
- 持续运动控制（"说到停"）
- 声纹/多人识别
- 运动指令的"走多久/多快"参数（速度固定用 panel 档位）
- 标准 YOLO COCO-80 全开（不为"所有物体"切模型）
