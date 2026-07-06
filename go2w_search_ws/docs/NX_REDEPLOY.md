# NX 重新可用时 — 快速部署清单

> NX 当前不在线时,在 PC 上跑测试 + 改代码;NX 回来时按本页一次性同步。
> 目标:**`scp` 代码 → 重启 service → 浏览器打开** 全流程 < 2 分钟。

---

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
pip install vosk sounddevice pyttsx3            # requests + websocket-client 已装
# 下 Vosk 中文模型 (~50MB): https://alphacephei.com/vosk/models → vosk-model-small-cn-0.22
# 解压到 go2w_search_ws/tools/vosk-model-small-cn-0.22 或设 VOSK_MODEL_PATH

# 跑 (NX 在线时)
cd go2w_search_ws && python tools/voice_console.py --nx <NX_IP>
# 说"去搜索这个房间，把所有人标注出来" → 自动发送 → 听 TTS 反馈
```

可选:`--no-auto-send`(识别不自动发,防 STT 误识别让狗乱跑)/ `--no-tts`(只看文字)。
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
2. **状态机取决于有没有预建房间图**:
   - **有图**(next_best_view 路径):`SELECT_ROOM → NAVIGATE → ARRIVED → SEARCH → DETECT → REPORT`,TTS 播"已到达客厅"
   - **无图**(frontier_explore 路径,默认/未知地图):`INIT_SLAM → FRONTIER_DETECT → NAVIGATING → DETECT → REPORT`,TTS 播"开始探索房间"
   - 无图自动降级由 `nx_web_server._resolve_product_current_room` 处理(commit e32ab10)
3. 地图上应出现 **person marker**(带照片缩略图,`require_photos=True`),坐标由 lidar+相机几何定位
4. `/dog_state` 的 `cmd_vel_n` 应随狗移动递增(否则 subscription 又被 GC)
5. 失败排查:`journalctl -u go2w-web -f | grep -E "search_room|mission_report|person_marker|localize"`

### 5.4 NLU 覆盖的说法(部分)
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
