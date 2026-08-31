# 外场(室外)测试就绪度清单 — 2026-08 审计合并版

> 来源:三路并行审计(网络与远程操作链路 / 无人值守鲁棒性 / 室外感知接入)+ 集成自查。
> 每条均附 文件:行号 证据;关键条目已由集成官抽查复核。
> 结论:**当前状态下外场测试不可安全开展**,存在 2 个阻塞项与 8 个高严重度项。

---

## 一、阻塞(不修则外场核心场景直接死)

### BL-1 GPS 链路(PX4/RTK 已接硬件,软件接线 2026-08-28 起补齐中)
- 硬件现状: RTK+GPS → PX4 → NX(USB)。全仓原本 0 处 mavros/PX4 接线;
  web 订阅硬编码 /gps/fix(nx_web_server.py:3950),而 mavros 默认发布
  /mavros/global_position/global —— **话题不匹配,上电后 /gps/fix 仍为空,gps_route 报 gps_no_fix**。
- 已做(2026-08-28):
  ① web/nx_web_server.py 订阅话题改为 GO2W_GPS_FIX_TOPIC 可配(默认 /gps/fix 不变);
  ② docker/mavros-px4.service.template —— mavros 常驻服务模板(含 global→/gps/fix remap、
    StartLimitIntervalSec=0、fastdds profile),设备名现场确认后手工安装;
  ③ docker/udev/99-go2w-serial.rules.template —— PX4(ttyPX4)/UWB(ttyUWB)串口固定名模板。
- 仍缺(外场前必须回答):
  a. **mavros 由谁拉起**: 手工一条命令或装 template 服务,二选一,写进开机流程;
  b. **RTK 差分从哪来**: 若 NTRIP(需 NX 有网跑 str2str 或 PX4 走数传电台)——NTRIP 走热点
     则 BL-2 的 NTP 顺便解决;若纯电台差分且无网,BL-2 仍在;
  c. **质量门不区分 RTK 等级**: GpsFixGate 只拒 fix_status<0(nx_gps_nav.py:359);
     RTK fixed→float→3D 降级不触发任何门禁,航点精度从 cm 级掉到米级,gate 照放行 —
     外场需人工盯 /mavros/global_position/global 的 status.status(RTK fixed 通常=2);
  d. 北向标定简化机会: mavros /mavros/global_position/compass_hdg 可给出航向角,
     可交叉验证 derive_heading_from_track 的人工流程(磁偏角偏差需现场核对一次)。

### BL-2 NTP 门禁在无外网热点下死锁全栈 — 且有超时矛盾 bug
- 证据: docker/livox-mid360-driver.service:22 与 docker/go2w-fastlio.service:20 各死等
  NTPSynchronized=yes 最长 300s,失败 exit 1;仓库内无任何离线时间源配置;
  连带 bug: livox 驱动 TimeoutStartSec=120(:27) < 门禁 300s,NTP 在 120~300s 间同步时首轮必被杀。
  go2w-slam-nav 依赖 livox 驱动(bringup_slam_nav2.sh:565-567 is-active || die)→ 导航栈永不就绪。
- 后果: 手机热点无蜂窝数据/信号差 → NX 重启后 SLAM/Nav2 100% 起不来,且症状埋在 driver "activating" 里。
- 修法: ① 部署离线时间源: gpsd+chrony 把 GPS 作为 SHM refclock(与 BL-1 同一硬件),或随队 4G 路由;
  ② 门禁放宽为"NTP 同步或系统年份≥2026"兜底防死锁;
  ③ livox 驱动 TimeoutStartSec 提到 ≥360。

---

## 二、高(外场大概率翻车或安全风险)

### HI-1 鉴权短路 + 全网卡监听 + CSRF — 热点内任何人/任意网页可控狗
- 证据: web/nx_control_auth.py:52 无条件放行(Bearer 校验成死代码);nx_web_server.py:599 绑 0.0.0.0;
  /api/move 是纯 query-string POST(nx_web_server.py:3102-3114,上限 vx=0.4)→ 浏览器"简单请求"无预检,
  操作员 PC 上任意外部网页可盲发 CSRF 控狗/急停;POST 处理器不校验 Origin。
- 修法: 删短路行恢复 fail-closed(54-70 行逻辑完好);panel.html:326 恢复 Bearer 头;
  或至少 do_POST 加 Origin 强校验 + GO2W_HOST 只绑热点网卡。

### HI-2 UWB/GPS 串口设备名无固定机制 — 双串口必冲突,且手工配置会被发布抹掉
- 证据: web/nx_uwb_bridge.py:45 默认 /dev/ttyUSB0;go2w-web.service:78-84 钉了 MODE/PROTOCOL/BAUD 但没钉 PORT;
  /etc/go2w/hardware.env 每次 deploy_release.sh:319-322 重生成覆盖手工追加项;全仓无 udev 规则。
- 修法: udev by-vid/pid(UWB CH343 常见 1a86:7523 → ttyUWB;GPS → ttyGPS);
  GO2W_UWB_PORT=/dev/ttyUWB 写进 go2w-web.service(随发布走,不被 hardware.env 抹)。

### HI-3 pyserial 无装机/发布保障 — UWB 链路静默不可用
- 证据: requirements.txt:31-33 列了,但 setup_jetson.sh:206-216 与 deploy 链都不装;
  uwb_serial_bridge.py:176-179 懒加载失败 → 源拒绝(仅 journal 一行 warning)。
- 修法: setup_jetson.sh 加 pip3 install "pyserial>=3.5";deploy 预检 python3 -c "import serial";
  外场 checklist 加验证命令。

### HI-4 自主任务(GPS 航线/UWB 跟随)无操作员在线门控 — PC 断 WiFi 狗继续跑
- 证据: nx_web_server.py:3956-3968(GPS tick)与 :2690-2694(UWB tick)无条件运行,不查 WS_CLIENTS;
  死亡看门狗只在命令流停止时清零(motion_controller.py:161-181),自主 producer 本地持续供流永不触发。
- 修法: tick 循环加"WS 客户端连续 8s 为空且任务 active → controller.stop(operator_link_lost)";
  外场随身带 Unitree 遥控器作独立急停通道并写入 runbook。

### HI-5 C13 双流 30fps base64 挤占唯一 WS — 热点带宽下遥测/视频同死
- 证据: go2w-web.service:90 C13_FPS=30;nx_gimbal_node.py:271-286 可见光+红外 base64 同推 WS:8001
  (约 12-22Mbps);panel.html:2136 单 WS 连接;6s 无消息即断线重连(:1773-1777)拥塞时反复震荡。
- 修法: 外场档 C13_FPS=12 + _JPEG_Q 降 30;红外流加开关;中期视频独立 WS 端口故障隔离。

### HI-6 transient 导航链运行期无人看护 — 雷达 watchdog 重启驱动后 FastLIO 变僵尸
- 证据: go2w-slam-nav.service Type=oneshot+RemainAfterExit,成功后不监控任何 transient;
  livox_stream_watchdog.py:115-141 重启 livox 驱动后不复核 /Odometry、不重启 fastlio
  (bringup_slam_nav2.sh:438-440 注释自认该坑);nav_health_supervisor.py 只读无动作。
- 修法: supervisor 升级为带动作:/Odometry stamp-age>2s 且 livox active → restart fastlio + slam-nav;
  start_transient 追加 -p StartLimitIntervalSec=0。

### HI-7 狗主控断电重启后 sport-gateway lease 重取无保障 — 运动控制变僵尸
- 证据: nx_sport_gateway.py:44-49 WaitLeaseApplied 仅启动时一次;sport_gateway_server.py:397-407
  失联时仅 503 拒请求,不退出进程(退出才会触发 Restart=always 重取 lease);
  web 前端 gateway 灯只看 systemd active(nx_web_server.py:1180),狗重启后"绿灯但全废"。
- 修法: 主循环加 zero-unhealthy>60s 且狗可 ping → sys.exit(1) 交 systemd 重启;
  外场前做一次狗断电重启演练。

### HI-8 RestartSec=0.2/0.5 无 StartLimit — 快速崩溃 5 次即永久 failed
- 证据: go2w-sport-gateway.service:26(RestartSec=0.2)、go2w-safety-observer.service:18(0.5)、
  livox-mid360-watchdog.service:16(2s 临界)均无 StartLimitIntervalSec;systemd 默认 5 次/10s。
  gateway failed → Requires 它的 go2w-motion 不再拉起。
- 修法: 上述 service [Unit] 段统一加 StartLimitIntervalSec=0(或 300/Burst 20)。

---

## 三、中(特定场景翻车)

| # | 问题 | 证据 | 修法 |
|---|------|------|------|
| MD-1 | DDS UDP 无 loopback 白名单,热点换 IP 后静默断流无自愈 | docker/fastdds_udp.xml:21-35;NX_REDEPLOY.md:95 自认 IP 会变 | udp_only descriptor 加 WhiteList 127.0.0.1;runbook 写"热点重连后 restart go2w-web" |
| MD-2 | NX 动态 IP 发现仅 Linux arp-scan+安卓 43 网段文档;Windows PC/iPhone 172.20.10/无 DHCP 无预案 | NX_REDEPLOY.md:100-108 | NX 对固定热点 SSID 配静态 IP(nmcli);补 tools/find_nx.ps1;文档补两节 |
| MD-3 | 语音输入在 http://热点IP 非安全上下文被浏览器拒 | panel.html:1028-1029 | 外场降级只用文本框;或 NX 加 TLS 反代 |
| MD-4 | GPS 参数无 env 注入:arrive_timeout=300s 偏保守、max_age=2.0s 对 1Hz 接收机余量~1 帧 | nx_gps_nav.py:426-427,335;nx_web_server.py:3943 构造全默认 | 构造处读 GO2W_GPS_ARRIVE_TIMEOUT / GO2W_GPS_MAX_AGE(1Hz 机建议 2.5~3.0) |
| MD-5 | global costmap 50×50m 滚窗 vs 航点最远 2km:窗外 planner 必拒;scan range_max=8m 无 outdoor 档 | nav2_params_3d.yaml:251-253;mid360_nav_bridge.cpp:117 | 外场 SOP:航点间距≤20m;正式:nav2_params_outdoor.yaml + launch 切换 |
| MD-6 | FAST-LIO 无漂移抑制(map→odom 恒等),北向标定存内存重启即丢 | map_odom_fuser.py:425-437;nx_gps_nav.py:462-470 | heading 标定值落盘随服务恢复;tick 中 GPS track vs map 航迹差监控漂移 |
| MD-7 | UWB 串口运行中断线后桥不重开,热插拔后跟随永久失效(先开后插可自愈,运行中断不行) | uwb_serial_bridge.py:151-152,383-388 | _pump_once 捕 SerialException → close+置空触发重开(open 幂等已支持) |
| MD-8 | GPS 航线无面板 UI,受理/标定只能 PC curl | panel.html 全文 0 处 gps | panel 加最小 UI;或 tools/gps_field.py SSH 一条龙(采 A/B 点→derive→calibrate→submit) |
| MD-9 | slam-nav TimeoutStartSec=600 < bringup 各 gate 预算和 ~975s,慢收敛被掐进重启循环 | go2w-slam-nav.service:27;bringup_slam_nav2.sh 各 gate 累加 | 提到 1200 或拆两段 |
| MD-10 | bringup --watch-imu 调用未定义函数 topic_hz → 一启用每 ~40s 重启一次雷达 | bringup_slam_nav2.sh:433-439(无定义,返回 127 空) | 删除该分支(livox-mid360-watchdog 已常驻覆盖) |
| MD-11 | 电池<20% = FAULT+原地停,无返航无预警;<20% 开机 → slam-nav 无限重启循环 | nx_web_server.py:686;motion_machine.py:474-485;bringup:306-307 | SOC≤25% 强制告警;≤20% 且 GPS 可用走返航航线;阈值入 hardware.env |
| MD-12 | NX 散热/功耗零管理,室外暴晒降频打穿 0.35s FastLIO 延迟门 | 全仓无 nvpmodel/jetson_clocks;温度遥测 0 消费者 | 部署固定 nvpmodel;nx_sensor 顺带消费 temperature_ntc,>75°C 告警暂停任务 |
| MD-13 | go2w-fastlio.service 文档写常驻但 deploy 从不安装 → 按文档排障会起双 FastLIO | deploy_release.sh:361 managed_units 无它;ARCHITECTURE.md:107 仍写常驻链 | 删该 service 或纳入 managed_units;同步修文档 |

---

## 四、低(现场注意即可)

- PANEL_ORIGINS/GO2W_PUBLIC_IP 烙死部署日 IP(go2w-web.service:34,40;deploy_release.sh:318)→ 换网后 foxglove 死链;换网后 sed 更新 hardware.env + restart go2w-web。
- websockets 未钉版本(requirements.txt:9)+ WS 无 Origin 校验(nx_web_server.py:3497-3509)。
- UWB 波特率 docstring(115200)与代码默认(921600)不一致(nx_uwb_bridge.py:10 vs :49)。
- 磁盘:missions 目录无上限(nx_person_mission.py:634-641)、journald 未配上限 → 配 SystemMaxUse=500M + mission 只留最近 20 个;出发前清 releases 旧版本。
- /mnt/c 路径残留于 sim 工具(NX 不受影响,runbook 标注勿在 NX 上用)。
- fastdds_udp.xml:11 注释路径过期。
- 掉电重启后 frontier 探索进度丢失(slam_toolbox 无序列化)→ 写进 runbook 预期。
- 硬件遗留:TROUBLESHOOTING.md 问题7 MID360 USB 版供电不足未闭环(需带供电 Hub/换线);
  PX4/GPS 接线、云台相机在环境清单中仍是"待查"。

---

## 五、已核实就绪(无需改)

- UWB fail-closed 双保险(服务钉 serial + 适配层拒 mock),忘插串口错误可见。
- 手动遥控断链兜底:WS 断开清续发 → keyup manual_stop → 服务端 0.5s 死亡看门狗清零。
- WS 重连退避健康(1-10s 封顶+抖动);latest-value 合流防积压;雷达 WS 已限幅 5Hz/600 点。
- MID360 -20° 下倾外参三处一致(bringup:47/615/634),BODY_TO_BASE_PITCH 可 env 覆盖。
- GPS fail-closed 七条停车路径完整(nx_gps_nav.py §3)。
- 安全事件日志轮转(4MB×4)、FastLIO PCD 落盘已关、release 原子发布。

---

## 六、外场前最低动作集(建议顺序)

1. BL-2 离线时间源(gpsd+chrony 或 4G 路由)+ 门禁兜底 + TimeoutStartSec 修正。
2. BL-1 GPS 驱动接线(nmea service 或进程内串口线程)+ HI-2 udev 固定双串口。
3. HI-1 恢复鉴权(删 1 行短路 + panel 发 Bearer)。
4. HI-4 自主任务加操作员离线门控;HI-5 C13_FPS 降 12。
5. HI-8 全部 Restart 服务加 StartLimitIntervalSec=0;HI-6 supervisor 加 fastlio 重启动作;MD-10 删 watch_imu。
6. HI-3 pyserial 装机;MD-9 TimeoutStartSec 1200。
7. 演练:狗主控断电重启(HI-7)、热点闪断重连(MD-1)、低电穿越 20%(MD-11)。
8. 出发 checklist:timedatectl NTPSynchronized=yes / df -h / lsusb 确认雷达与双串口 / 携带 Unitree 遥控器 / 热点静态租约。

---

## 七、PX4/RTK 上电验证清单(NX 上按序跑, 2026-08-28)

```bash
ls /dev/ttyACM*                                     # PX4 在不在(mavros fcu_url 按此改)
ros2 topic list | grep mavros                       # mavros 起没起
ros2 topic hz /mavros/global_position/global        # GPS 数据率(常见 1-10Hz)
ros2 topic echo /mavros/global_position/global --once | grep -A2 status   # RTK 等级
# 接通(二选一):
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:115200   -r /mavros/global_position/global:=/gps/fix       # 手工 remap 到 /gps/fix
# 或: docker/mavros-px4.service.template 装成 systemd 服务
# 然后验证 web 链路(应从 gps_no_fix 变为 heading_not_calibrated — 这说明 fix 已进来):
curl -s http://localhost:8000/api/gps/route | python3 -m json.tool | head
ros2 topic echo /mavros/global_position/compass_hdg --once   # 北向标定参考角
```

注意: web 服务在读到 GO2W_GPS_FIX_TOPIC 前提下默认订 /gps/fix,remap 方案无需重启 web;
若用 env 方案(GO2W_GPS_FIX_TOPIC=/mavros/global_position/global)需写进
go2w-web.service 并 systemctl restart go2w-web。
