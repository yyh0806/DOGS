# NX 重新可用时 — 快速部署清单

## 2026-07-14 原子发布流程（当前唯一生产流程）

下方旧的 `scp`/直接覆盖脚本保留作历史记录，不再用于生产部署。当前发布包包含完整 motion、web、nav、sensor payload；`subsystem` 只决定本次允许重启哪些服务。

### 1. 狗未上电时，在 PC 完成离线门禁与构建

```powershell
cd C:\Users\ROG\yangyuhui\DOGS\go2w_search_ws
python tools\verify_release.py
& 'C:\Program Files\Git\bin\bash.exe' docker/build_release.sh all
```

门禁必须显示 `architecture: PASS`、全部 Python/Node 测试通过和 `offline release gate: PASS`。构建产物位于 `dist/go2w-<release_id>-all.tar.gz`；`release_id` 同时包含 Git 基线和完整 payload 哈希。

控制 Token 不得提交到 Git。首次 `web`/`all` 部署可在部署命令中显式传
`--generate-control-token-file control-token.txt`：部署器会先严格验证发布包，随后在连接 NX 前创建 256-bit URL-safe Token；内容不会输出，文件不会覆盖，POSIX 权限固定为 `0600`。保留该文件供浏览器和 PC 语音控制使用。已有本地文件时改用 `--control-token-file control-token.txt`；若 NX 已有 `/etc/go2w/control.env`，也可以不传 Token 参数并继续使用原凭据。手工或旧 Token 必须是至少 32 个 URL-safe 字符，否则 Web、PC 语音和 NX preflight 都会 fail-closed，需显式轮换。

### 2. 上电后分阶段部署

前提：狗放稳、场地安全、急停可用，确认 NX SSH 地址。生产全量部署会重启 motion，所以必须显式授权：

```powershell
$env:NX_HOST = '<NX_IP>'
$env:NX_USER = 'nx'
# 若代理/TUN 抢走局域网 SSH 路由，绑定电脑实际 WLAN 地址：
# $env:NX_BIND_ADDRESS = '<PC_WLAN_IP>'
# 仅当自动探测不到 MID360 USB 网卡时设置，例如：
# $env:LIVOX_INTERFACE = 'enx207bd2edf780'
& 'C:\Program Files\Git\bin\bash.exe' docker/deploy_release.sh `
  dist/go2w-<release_id>-all.tar.gz `
  --allow-motion-restart --generate-control-token-file control-token.txt
```

上面是“首次部署且本地还没有 Token”的命令。再次部署时必须把最后一个参数改为 `--control-token-file control-token.txt`；生成参数拒绝覆盖已有文件。

这一条命令会按固定顺序完成：

1. 在 PC 严格验证归档类型、路径、文件清单、SHA-256、payload digest 和内容寻址 release ID；验证失败时不会创建 Token，也不会连接 NX。若显式要求首次生成 Token，只在此门禁通过后安全创建。
2. 上传前执行只读 NX preflight：检查 `nx` 用户、免密 sudo、磁盘、ROS 2 Humble、`unitree_sdk2py`、Nav2/Livox workspace、AI 依赖、YOLO-World 模型；通过实际路由自动识别狗网卡和 MID360 网卡，雷达网卡也可用 `LIVOX_INTERFACE` 显式覆盖。该阶段不会停止服务或发运动指令。
3. 在 NX staging 目录再次验证同一归档并执行离线编译；源码验证后先落到最终内容寻址 release 目录，再在最终路径运行 `colcon --symlink-install`，避免 ROS overlay 指向已经删除的 staging 路径。构建成功后才原子切换 `/home/nx/go2w/current`。
4. 安装主服务以及 costmap、MID360 网络/驱动/watchdog units，把探测到的网卡、NX 地址、面板来源和模型路径写入 `/etc/go2w/hardware.env`。Nav/all 模式会在切换 release 后显式按“雷达网络 → 驱动 → 数据 watchdog → Nav2”重启，避免旧进程继续引用旧 release；同时禁用完整 `go2w-sensor` 自启，Nav 内部只运行 `/wheel_odom` 受限实例。该实例显式继承同一个 `DOG_INTERFACE` 和 `GO2W_RELEASE_ID`，不会退回旧硬编码网卡，也不会产生重复 `/odom`/TF。
5. 全量部署依次运行三项只读验收：release/safe-park；TF/action、局部/全局 costmap 和前端 costmap bridge；开放词汇感知新鲜度。任一项不通过即回滚。
6. 失败时先停止本次 release 的完整依赖进程，再恢复旧 release、release/hardware/control 环境、systemd units 与 enable 状态；首次安装失败会清除新状态。没有 `--allow-motion-restart` 时，motion/all 发布会拒绝执行。

如果只想先更新不碰狗运动，可分别构建和部署 `web`、`nav` 或 `sensor` 子系统包；其中 `nav` 发布不得覆盖或重启 motion。

### 3. 部署后只读验收

```bash
ssh nx@<NX_IP> 'readlink -f /home/nx/go2w/current; cat /etc/go2w/release.env'
ssh nx@<NX_IP> 'systemctl --no-pager --full status go2w-motion go2w-web go2w-slam-nav costmap-bridge livox-mid360-driver livox-mid360-watchdog'
ssh nx@<NX_IP> 'systemctl is-enabled go2w-motion go2w-web go2w-slam-nav go2w-sensor'
ssh nx@<NX_IP> 'journalctl -u go2w-motion -n 120 --no-pager'
ssh nx@<NX_IP> 'sudo cat /etc/go2w/hardware.env'
ssh nx@<NX_IP> 'cat /home/nx/go2w/validation/<release_id>-deploy.json /home/nx/go2w/validation/<release_id>-nav2-preflight.json /home/nx/go2w/validation/<release_id>-perception-preflight.json'
```

三份 JSON 都是部署器原子写入的只读验收凭证，`ok` 必须全部为 `true`。它们分别证明发布一致且安全停车、Nav2 单链 TF/action/局部与全局 costmap/前端障碍桥可用、YOLO-World 已加载且真实相机帧新鲜；不等同于实车运动验收。Nav/all 部署后 `go2w-sensor` 应显示 `disabled/inactive`，这是预期状态。浏览器打开 `http://<NX_IP>:8000`，首次控制时输入同一 Token。状态中 `release_id` 与 `motion_release_id` 必须一致，motion status schema 必须为 `4`。不一致时禁止开始手动或 Nav2 会话。

### 4. 上狗分阶段验收（不可跳过）

1. 只观察启动：期望 `BOOT_HOLD -> PARKED`，轮速为零；不得出现启动即 `BalanceStand`、`Damp` 或 `RecoveryStand`。
2. 手动会话：点击站立/启动后才允许 `BalanceStand`；反馈确认 `MANUAL_ACTIVE` 后，用极短低速指令验证前后/转向，然后停车回到 `PARKED`。
3. Nav2 空场：先做 0.3 m 目标，确认 `NAV_ACTIVE`、目标成功、终点零速和停车。
4. Nav2 障碍：运行 `python3 /home/nx/go2w/current/payload/tools/nav2_benchmark.py --record --duration 60`，再由操作员点击安全目标，检查规划时延、首次速度时延、最小净空和终止停车。
5. 房间探索：小区域、单一 `person`，确认前沿预算、不可达 blacklist、取消和任务报告。
6. 感知标注：确认照片时间、插值位姿、scan/cloud 时间差均在阈值内；同一人不会重复标注。
7. 泛类目标：把 `target_classes` 改为 `["dining table"]`（中文“桌子”会规范化为此类别），重复第 5–6 步。

“搜索这个房间”可直接使用受半径/时间约束的前沿探索；“去客厅搜索”等命名房间必须先按 `docs/room_calibration.md` 实测 `map -> base_link` 坐标并设置 `calibrated: true`，不得用占位坐标绕过门禁。

任何阶段出现红灯、姿态异常、定位跳变、扫描过期、release 不一致或非零轮速无法收敛，立即操作员急停并停止后续阶段；不要自动调用恢复站立。

> NX 当前不在线时,在 PC 上跑测试 + 改代码;NX 回来时按本页一次性同步。
> 目标:**`scp` 代码 → 重启 service → 浏览器打开** 全流程 < 2 分钟。

---

## 以下内容为旧版排障记录，不是当前部署流程

旧的逐文件 `scp`、`deploy_nx*.sh` 操作和 `~/go2w_ws` 布局仅用于阅读历史故障背景。下一次 NX 上线请只执行本页顶部的 `verify_release.py -> build_release.sh all -> deploy_release.sh` 原子流程。

## 0. 前提自检(NX 必须满足)

| 检查 | 命令(PC 上) | 期望 |
|---|---|---|
| NX SSH 可达 | `ssh nx@<NX_IP> echo ok` | `ok` |
| 狗主控连了 NX 的 USB 网卡 | `ssh nx@<NX_IP> ip -br addr` | 一个网卡带 `192.168.123.x` |
| C13 云台通电 | `ping 192.168.144.108` | 通(rtsp://192.168.144.108:554/555) |
| MID360 雷达通电 | `ping 192.168.1.160` | 通(/livox/lidar 10Hz) |
| go2w-motion.service active | `ssh nx@<NX_IP> systemctl is-active go2w-motion` | `active` |

> ⚠️ NX 的 IP 是 DHCP 动态的(实测会变)。**ping/ARP 都可能假阳性**,必须用 `ssh` 验真。
> Mihomo / Tailscale 会干扰路由,排障时先关。

---

## 1. 找 NX 的当前 IP

```bash
# 方法 A:扫手机热点网段(43.x)
arp-scan --interface=<WiFi 网卡> 192.168.43.0/24

# 方法 B:NX 上次在线时记下 MAC,直接定位
arp -a | grep -i "<NX_MAC>"
```

---

## 2. 一键部署(在 PC 仓库根目录)

```bash
export NX_HOST=<第 1 步找到的 IP>
export NX_USER=nx

# 2a. Web 层 (HTTP:8000 + WS:8001 + 阶段E 房间搜索编排)
bash go2w_search_ws/docker/deploy_nx_web.sh

# 2b. AI 层 (nx_ai_node + ai/ 推理包; /api/locate 走这)
bash go2w_search_ws/docker/deploy_nx_ai.sh

# 2c. (可选) 若只改了 motion 节点(rclpy GC 修复等)
bash go2w_search_ws/docker/deploy_nx.sh
```

`deploy_nx_web.sh stop` 可单独停服务(不 disable,重启 NX 仍自启)。

---

## 3. 部署后冒烟测试(NX 上 SSH)

```bash
ssh nx@$NX_HOST
cd ~/go2w_ws

# 3a. 全套产品/人员/契约测试 (PC 也能跑, 但 NX 上跑才验真部署完整性)
bash web/verify_product_room_person_search.sh

# 3b. web 基础连通
bash web/verify_nx_web.sh

# 3c. AI 推理 (YOLO/VLM/Locate)
bash web/verify_nx_ai.sh
```

任一 FAIL → 看 `journalctl -u go2w-web -f` 排障。

---

## 4. PC 浏览器访问

```
http://<NX_IP>:8000        ← HTTP (面板)
ws://<NX_IP>:8001          ← WebSocket (前端自动连)
```

---

## 5. 语音搜索人员(核心目标场景)

> 目标指令:**"去搜索这个房间，把所有人标注出来"** → 狗自动进房间 → YOLO 检测人 → lidar 定位 → 拍照 + 地图标注。
> 语音采集 + STT 在 PC,任务发布给 NX,NX 控狗执行;NX 状态经 WS 推回 PC,TTS 播报。

### 5.0 推荐:PC 端 voice_console.py(绕开 HTTPS 限制)

`tools/voice_console.py` — PC 本地 Vosk 离线 STT,识别文本 POST 到 NX `/api/command`,
同时连 NX WebSocket 收任务反馈,pyttsx3 离线 TTS 播报("开始探索房间" / "已到达客厅" / "找到 N 人")。
**不受浏览器 Web Speech API 的 HTTPS 限制**(`http://IP` 也能用),是 NX 直连场景的首选入口。

```bash
# 一次性准备 (PC 上)
python -m pip install -r requirements-voice.txt
# 下 Vosk 中文模型: https://alphacephei.com/vosk/models → vosk-model-small-cn-0.22
# 解压到 go2w_search_ws/models/vosk-model-small-cn-0.22；也可用 VOSK_MODEL_PATH 覆盖

# 不发任务的本机自检: 模型可加载 + 找得到输入设备
python -c "from vosk import Model; from tools.voice_console import default_model_path; p=default_model_path(); print(p); Model(p); print('model_loaded')"
python -c "import sounddevice as sd; print(sd.query_devices())"

# 跑 (NX 在线时；使用部署时同一个控制 Token)
cd go2w_search_ws && python tools/voice_console.py --nx <NX_IP> --token-file control-token.txt
# 说"去搜索这个房间，把所有人标注出来" → 自动发送 → 听 TTS 反馈
```

自动发送必须提供 `--token-file` 或 `GO2W_CONTROL_TOKEN`；缺失时 PC 端直接失败，不会发送未认证请求。可选:`--no-auto-send`(识别不自动发,防 STT 误识别让狗乱跑，此模式不需 Token)/ `--no-tts`(只看文字)。
依赖缺失时**优雅降级**(pyttsx3 没装→只打印;NX 离线→继续监听;WS 断→2s 重连),STT 主功能不崩。

### 5.1 PC 端 NLU 验证(不需 NX)
```bash
# 验证各自然说法被正确解析成 search_room task (16 个用例)
python go2w_search_ws/web/verify_voice_search.py

# 或 pytest 套件 (含 resolve_current_room 边界用例)
python -m pytest go2w_search_ws/web/test_voice_search_contract.py -v
```
**接入 NX 前先跑这个** — 确认指令解析链路健康,任何 STT 误识别 / NLU 回归在此暴露。

### 5.2 前端语音入口
- **🎤 按钮**(指令输入框旁):浏览器 Web Speech API 中文 STT → 识别文本填入输入框 → **回车确认**发送(不自动发,防 STT 误识别让狗乱跑)
- **🎤 房间搜人快捷按钮**:一键触发"去搜索这个房间，把所有人标注出来"(无需麦克风,测端到端最快)
- ⚠️ **浏览器限制**:Web Speech API 需 **Chrome/Edge + 联网**;`http://IP:8000` 可能被 Chrome 拒绝(非安全上下文)。失败时用文本输入或"房间搜人"快捷按钮;正式部署建议给 NX 配 HTTPS 反代。

### 5.3 端到端验证(NX + 狗接入时)
1. 浏览器打开 `http://<NX_IP>:8000`(或用 PC voice_console.py),触发"去搜索这个房间，把所有人标注出来"
2. “搜索这个房间”固定走有界 `frontier_explore`，不读取占位房间坐标，默认受 6 m 半径、180 s、前沿次数和可达性预算约束。命名房间（例如“去客厅搜索”）才走 `next_best_view`，并要求 `config/rooms.yaml` 中该房间明确 `calibrated: true`；未标定时会在发送导航目标前以 `room_uncalibrated` 拒绝。
3. 地图上应同时显示房间边界、候选/已访问视点、覆盖率和 **person marker**。人员标记必须带照片缩略图(`require_photos=True`)，坐标由对应相机帧的方位角 + MID360 距离投影到 map；同一人多次观测做置信度加权融合与去重。
4. 默认覆盖阈值为 90%、视觉覆盖半径 2.5m、最多 12 个视点。覆盖只计算 C13 水平视锥内的网格，无有效新鲜相机帧不增加覆盖；覆盖未达阈值时任务必须以 `coverage_incomplete` 失败而不是误报“已搜完”。暂时没有雷达距离的人员会保存未定位照片并在后续视点重试，耗尽视点仍未定位则以 `no_lidar_range` 失败。
5. 检测必须来自新鲜的源帧(默认最大 2s)；照片落盘失败不得提交地图 marker。
6. `/dog_state` 的 `cmd_vel_n` 应随狗移动递增(否则 subscription 又被 GC)
7. 失败排查:`journalctl -u go2w-web -f | grep -E "search_room|mission_report|person_marker|localize"`

### 5.4 C13 与 MID360 方位标定(部署前必做)

人框中心角度只有在 C13 水平视场角和相对 base_link 的 yaw 外参正确时才能命中同一方向的 LiDAR 射线。不要把默认 `70°/0°` 当作实测值。按当前检测源 `c13_vis` 用 systemd drop-in 保存标定结果:

```ini
# sudo systemctl edit go2w-web
[Service]
Environment=GO2W_CAMERA_HFOV_C13_VIS_DEG=<实测水平视场角>
Environment=GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG=<C13光轴相对base_link的左正角度>
```

执行 `sudo systemctl daemon-reload && sudo systemctl restart go2w-web`，把人放在已知方位/距离，确认地图 marker 与 LiDAR 中该目标位置重合。其他检测源使用同样命名规则，例如 `source=dog` 对应 `GO2W_CAMERA_HFOV_DOG_DEG` 和 `GO2W_CAMERA_YAW_OFFSET_DOG_DEG`。通用回退变量是 `GO2W_CAMERA_HFOV_DEG`、`GO2W_CAMERA_YAW_OFFSET_DEG`。

### 5.5 NLU 覆盖的说法(部分)
- **当前房间**:"搜索这个房间标注所有人" / "找这个房间里的人" / "标记本房间人员" / "圈出当前房间的全部人员"
- **命名房间**:"去客厅搜索所有人" / "搜索实验室里的人" / "去办公室找所有人标注出来"
- **否定句(正确拒绝)**:"别搜索" / "不要找人" / "不用标记"
- 完整清单见 `web/test_voice_search_contract.py` 和 `web/verify_voice_search.py`

## 6. 常见排障

| 症状 | 先查 |
|---|---|
| 前端打不开 | `journalctl -u go2w-web -f`,看 Python traceback |
| 视频不流 | C13 通电?`GO2W_AI_VIDEO_ENABLE=0` 默认走 C13 不走狗原生相机 |
| 找不到物体(`/api/locate`) | 每次冷启动 ~17s(CLI fork + 6.2GB gguf 加载),正常;常驻 server 已删(占 VRAM 拖垮 fps) |
| 雷达没数据 | `systemctl status livox-mid360-driver`;雷达网卡必须 `/32` 不能 `/24`(详见 memory) |
| 控不动狗 | `/dog_state` 里 `cmd_vel_n` 应随键盘递增;卡在 0 = rclpy subscription 又被 GC 了(已修,见 `nx_motion_node.py` 注释) |
| 急停后"持续扫描"不停 | 已修(`panel.html` 的 `stopMove`/`eStop`/`onclose` 都 `clearTimeout`);清浏览器缓存 |

---

## 7. 当前已知限制

- **`/api/locate` 冷启动 ~17s/帧**:常驻 `LocateAnythingServer` 已删除(6.2GB gguf 持续占 VRAM 把 C13 fps 拖垮)。前端"持续扫描"按钮仍可用,代价是每帧 17s。
- **`_FPS=12`**:gimbal 默认帧率目标,NX Orin 实测能否达标待确认(记忆:VideoClient SDK 上限 5.9fps,WebRTC 才是高帧率路径,待接入)。
- **NX 中心化架构**:所有重活在 NX,PC 仅 UI;`deploy_nx_ai.sh` + `deploy_nx_web.sh` 是必须的两个脚本(分阶段调试时可只跑其一,但功能会缺)。

---

## 8. 文件清单一览(部署后 NX 应有)

```
~/go2w_ws/
├── web/
│   ├── nx_web_server.py            # 主入口 (HTTP:8000 + WS:8001)
│   ├── nx_gimbal_node.py           # C13 云台 RTSP
│   ├── nx_lidar_node.py            # MID360 雷达
│   ├── nx_slam_map.py              # 障碍网格
│   ├── nx_ai_node.py               # ⚠️ 来自 deploy_nx_ai.sh
│   ├── nx_room_orchestrator.py     # 阶段E 房间搜索编排
│   ├── nx_product_command.py       # 产品命令解析
│   ├── nx_active_search.py         # 主动搜索
│   ├── nx_person_mission.py        # 人员任务
│   ├── nx_person_localizer.py      # 人员定位
│   ├── mock_dog_state_publisher.py # 调试用
│   ├── verify_*.sh                 # 冒烟测试
│   └── static/{panel.html, map.js}
└── ai/                             # ⚠️ 来自 deploy_nx_ai.sh
    ├── locate_anything.py
    ├── detector.py / vlm.py / tracker.py
    └── config.py
```

`/etc/systemd/system/`:
- `go2w-web.service` (After=go2w-motion)
- `go2w-motion.service`
- `livox-mid360-{net,driver}.service`
