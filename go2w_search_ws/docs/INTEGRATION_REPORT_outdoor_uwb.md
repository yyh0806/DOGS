# 集成报告:室外 GPS 导航 + UWB 钥匙扣跟随

> 2026-08-24,feature/outdoor-follow-integration(基线 706bda1)。
> 三线并行开发(eA GPS / eB UWB 协议桥 / eC UWB 跟随)+ 集成官收尾(本报告作者)。

## 1. 合并冲突点清单

| 冲突点 | 解法 |
|---|---|
| 无 git 冲突 | eA 零修改既有文件(纯新增 4 文件);eB 纯新增 6 文件;eC 只改 nx_web_server.py/panel.html + 新增。三线物理上不重叠,ort 策略自动合并 |
| 接口冲突(逻辑层) | eC 期望 `nx_uwb_bridge.get_follow_fix_source()`,eB 交付 `go2w_bridge.uwb_serial_bridge.UwbSerialBridge.latest()` → 集成官新写 `web/nx_uwb_bridge.py` 适配层(fix 合同转换 + 单例生命周期 + fail-closed mock 拦截) |
| GPS 集成面 | eA 交付核心+壳但未接 web(工单由集成官补)→ 集成官在 nx_web_server.py 补:main 装配(point_port=point_nav / park_hook=arbiter.stop_all / state_callback→WS)、/gps/fix 订阅、5Hz tick 线程、POST /api/gps/route + /api/gps/route_cancel + /api/gps/calibrate、GET /api/gps/route、/api/status 汇聚、/api/stop 与 /api/e_stop 联动 cancel、退出清理 |

## 2. 集成期发现并修复的问题(3 个)

1. **eB 夹具污染**(执行期发现):eB 测试代码加载后回写 `web/tests/missions/large-room-16/person_001.json`(135→225 行)。已要求还原;合并后 diff 验证干净。**集成期勘误:集成官本人跑全量测试也触发了同样污染——该回写 bug 是 master 既有测试的副作用(某测试加载夹具后 json.dump 回写源文件),非 eB 代码所致。每次跑完测试需 `git checkout` 还原;根治列入遗留(建议改用 tmp_path 或只读加载)。**
2. **uint32 时间戳回绕 bug**(集成期实证):`system_time_ms` 是 uint32 墙钟毫秒,`&0xFFFFFFFF` 截断后每 49.7 天回绕,适配层用它算 age 得到 17 亿秒 → 恒 stale。修复:`uwb_serial_bridge._record_latest` 记录 `received_monotonic`(到达时刻),age 以它为准;system_time_ms 不再用于新鲜度。
3. **隐式 mock 数据危险**:eB 桥串口失败自动降级 mock(输出"3m 正前方"假测距),若直接喂生产跟随=盲走真狗。修复:`nx_uwb_bridge.get_follow_fix_source()` 在 source=="mock" 且 GO2W_UWB_MODE≠mock 时拒绝接入(fail-closed);并新增回归测试 `test_load_uwb_follow_source_returns_none_without_bridge`(语义升级)+ `test_load_uwb_follow_source_explicit_mock_allowed`。

## 3. 测试对比表

全量:`python -m pytest web/tests src/go2w_bridge/test -q`

| 阶段 | 结果 |
|---|---|
| 基线(706bda1) | 918 passed / 10 failed(已知基线,与两功能无关;其中 sport_gateway 1 条为 flaky) |
| eA 独立 | +74(test_gps_nav) |
| eB 独立 | +50(test_nooploop_uwb_protocol + test_uwb_serial_bridge) |
| eC 独立 | +49(test_uwb_follow 35 + test_uwb_follow_web 14) |
| 集成后 | **1093 passed / 9 failed** —— 9 条全部为已知基线(sport_gateway flaky 本次通过),**零新增失败** |

已知基线清单(允许失败):test_ai_video_contract 2 条 / test_frontier_explore 1 条 / test_gimbal_latency_contract 1 条 / test_lidar_latency_contract 3 条 / test_panel_navigation_contract 2 条(map_frame + deployer)/ test_sport_gateway_server 1 条(flaky)。
集成新增回归:UWB web 测试语义升级 2 例(见 §2.3)。既有断言零弱化(git diff 706bda1..HEAD -- web/tests 核对:除 test_uwb_follow_web.py 上述语义升级外,既有测试文件零改动)。

## 4. 安全条款核对(fail-closed 专项)

- GPS:eA 七条停车路径(docs/GPS_NAV.md §3 成文)——GPS 降级撤目标+park / 北向未标定拒绝受理 / 航点超时或 Nav2 失败整线中止+park / 所有权被抢中止(停车义务随所有权) / 航点>2km 拒绝 / 提交异常中止+park / 正常完成 park。web 侧 /api/stop、/api/e_stop、退出清理均联动 cancel。
- UWB 跟随:eC 五条(nx_uwb_follow.py 模块头成文)——避障闸门第一优先(fail-closed+迟滞+净空降速)/ 只进不退 / 野值剔除 / 超时自停 / fault 锁存;所有权走 arbiter manual 通道(零速 handoff);/api/stop、/api/e_stop、退出清理先停跟随再停车。
- UWB 桥:质量门(距离窗/有限性/新鲜度 0.5s);隐式 mock 拒绝(§2.3)。
- 速度通道:跟随速度经 robot.move(manual=True) → /cmd_vel(manual 通道,受 arbiter manual idle 租约与 motion 看门狗管辖);GPS 航点经 PointNavigationController → Nav2(/cmd_vel_nav 通道)。两条通道都收敛在既有 motion 安全出口之后,未新增旁路。

## 5. 遗留风险(实机标定清单)

| # | 项 | 影响 | 缓解 |
|---|---|---|---|
| 1 | GPS 北向标定为人工流程(docs/GPS_NAV.md §4,每场地一次) | 标错→航线整体旋转 | derive_heading_from_track 辅助 + 受理门禁(未标定拒绝) |
| 2 | AOA 角度零偏未标定(GO2W_UWB_ANGLE_OFFSET_DEG) | 方位偏差→跟随转向偏 | 距离控制律为主;实机走 8 字标定(docs/UWB_FOLLOW.md) |
| 3 | LinkTrack AOA 帧字段偏移以官方手册为准实现,待硬件实测确认 | 字段错位→测距错误 | build→feed 往返测试 + 实机对照上位机数据 |
| 4 | UWB 串口波特率默认 115200(出厂),若固件改过需 GO2W_UWB_BAUD | 无数据→降级→源拒绝(fail-closed 不盲走) | 上位机确认 |
| 5 | 室外 costmap 尺度未实测(本次未交付 outdoor yaml——eA 方案不动 Nav2 参数,点导航链路室内外共用) | 大场地 global costmap 50×50m 滚窗可能不够长航线 | 航点分段(≤2km 门禁)+ 实测后按需加 outdoor profile |
| 6 | arrive_timeout=300s 对长腿航线偏保守 | 长航线可能超时中止 | 实车调参(controller 构造注入) |
| 7 | GPS 航线与 UWB 跟随互斥性:两者都经 arbiter(point owner vs manual owner),同时启动时 arbiter 会拒绝其一 | 无安全隐患(串行化),但用户需先停一个 | 文档注明;后续可在 API 层加互斥提示 |

## 6. 交付物清单

- 功能A:`web/nx_gps_nav.py`(803 行纯核心)/ `web/nx_gps_nav_ros.py`(薄壳)/ `web/tests/test_gps_nav.py`(74)/ `docs/GPS_NAV.md`
- 功能B-1:`src/go2w_bridge/go2w_bridge/nooploop_uwb_protocol.py`(623)/ `uwb_serial_bridge.py`(298+修复)/ 测试 50 / `docs/UWB_BRIDGE_B1_ACCEPTANCE.md`
- 功能B-2:`web/nx_uwb_follow.py`(718)/ 测试 49 / `docs/UWB_FOLLOW.md` / 前端跟随按钮
- 集成:`web/nx_uwb_bridge.py`(适配层)/ nx_web_server.py GPS+UWB API 面与生命周期 / README + ARCHITECTURE 同步 / 本报告
