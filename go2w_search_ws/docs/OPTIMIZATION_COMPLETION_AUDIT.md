# Go2W 优化路线完成度审计

审计时间：2026-07-15。本文把“代码存在”与“目标已经被证据证明”分开记录。NX 和机器狗当前不在线，所以实机结果不会用离线测试代替。

## 当前结论

Tasks 1–13 的离线实现已完成。统一门禁结果为 `architecture: PASS`、`696 passed`、两套 Node 合同测试通过、`offline release gate: PASS`。当前唯一发布物是：

```text
dist/go2w-f10b14504edb-dirty-b2b7d5faa28e-all.tar.gz
release_id: f10b14504edb-dirty-b2b7d5faa28e
payload_digest: b2b7d5faa28e4a8c89c0f633e1ab0474f4aaa229c619b189ef606274d289198a
SHA-256: DBA16D7E90ED765C39DB8FE10A354DF3BC4EE2621AEB480EF4F39A9BAD2D2A62
files: 83
```

## 要求—证据矩阵

| 优化要求 | 当前权威证据 | 结论 |
|---|---|---|
| SDK 对齐、安全启动、单一运动所有者 | `motion_machine.py`、`unitree_sport_adapter.py`、运动类型/协议/控制器测试；架构门禁要求恰好一个 machine 和 adapter，禁止自动 `Damp/RecoveryStand/StandDown` | 离线完成；启动、进程重启和停车仍需同一 release 的实狗日志 |
| 手动控制与 Nav2 控制隔离 | `motion_controller.py`、intent/status schema、owner/遥测/scan 门禁；Nav 自主倒车在 Nav2 与 SDK 两层封锁 | 离线完成；最小速度位移和最终 `PARKED` 待实狗 |
| 地图点选、规划、避障 | 唯一 `NavigationGateway`/`NavigateToPose` 端口，动态安全 BT、双 costmap、MID360 bridge、`nav2_preflight.py`、`nav2_benchmark.py`；Nav 内部 `/wheel_odom` 显式继承部署探测的 `DOG_INTERFACE` 和 release 指纹 | 离线完成；0.3 m 和箱体净空待现场测量 |
| 未知楼层/当前房间探索 | 持久化 frontier planner/manager、revision、visited/blacklist、半径/时间/次数预算和取消测试 | 离线完成；复杂场地覆盖率待实测 |
| person/table 感知、定位、照片和去重 | 统一目标类别、YOLO-World preflight、时间同步 detection/pose/range、稳定 ID、证据照片与 marker 测试 | 离线完成；C13 外参与真实 person/table 标注待实测 |
| 语音发送同一搜索任务 | `voice_console.py`、`voice_command.py` 和 VLM/HTTP 都进入 `SearchMissionRequest`；Bearer Token fail-closed | 离线完成；语音作为实测最后一步 |
| 原子发布与回滚 | 严格归档校验的 required set 与全部 83 个 payload 文件完全一致；最终 release 前缀 colcon、原子 `current`、环境/unit/enable 回滚、MID360→Nav2 顺序、三份只读部署凭证 | 离线完成；NX 上三份 JSON 必须全部 `ok:true` |
| 首次部署无需临时拼接凭据命令 | `--generate-control-token-file` 在本地归档验证后独占创建 256-bit Token，权限 `0600`，不输出、不覆盖；Web、PC 语音和 NX preflight 同时拒绝少于 32 字符的手工/旧 Token；相关行为和架构回归已进入门禁 | 完成 |

## 下一次连接 NX 的唯一部署入口

首次且本地没有 Token：

```powershell
$env:NX_HOST = '<NX_IP>'
$env:NX_USER = 'nx'
# 代理/TUN 抢走局域网 SSH 时设置为电脑的 WLAN IPv4：
# $env:NX_BIND_ADDRESS = '<PC_WLAN_IP>'
& 'C:\Program Files\Git\bin\bash.exe' docker/deploy_release.sh `
  dist/go2w-f10b14504edb-dirty-b2b7d5faa28e-all.tar.gz `
  --allow-motion-restart `
  --generate-control-token-file control-token.txt
```

本地已有 Token 时，把最后一个参数改成：

```text
--control-token-file control-token.txt
```

该命令只有在操作员确认狗放稳、场地安全、急停可用后才能执行。它会先做只读 NX preflight；任一依赖、网卡、模型、SDK、磁盘或控制凭据条件不满足都会在上传/切换前失败。

## 仍必须由同一 release 现场证明的项目

1. `BOOT_HOLD -> PARKED`，轮速持续为零，启动和重启没有隐式 `BalanceStand/Damp/RecoveryStand`。
2. 极短低速手动前进/转向、停车并反馈 `PARKED`。
3. 0.3 m 空场点选 Nav2，记录接受、首速度、实际位移、终态。
4. 可移动箱体同时进入 scan、局部/全局 costmap，规划绕行且无接触。
5. 有界当前房间探索，先 `person`、再 `dining table`，保留同步照片、地图坐标、稳定去重 ID 和任务报告。
6. 最后测试中文语音，证明产生的 canonical mission 与 HTTP/文本路径一致。

命名房间还需要现场采集 `map -> base_link` 坐标并设置 `calibrated: true`；当前房间的有界 frontier 搜索不依赖这些占位坐标。
