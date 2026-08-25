# 功能 B-1 验收报告:Nooploop UWB 串口协议桥

> 分支:`feature/uwb-bridge`(worktree `DOGS-wt-uwb-bridge`,基线 706bda1)
> 执行者:eB | 日期:2026-08

## 1. 交付物

| 文件 | 说明 |
|---|---|
| `go2w_search_ws/src/go2w_bridge/go2w_bridge/nooploop_uwb_protocol.py` | **纯逻辑协议解析器**(零 ROS / 零 pyserial 依赖)+ 帧编码器 |
| `go2w_search_ws/src/go2w_bridge/go2w_bridge/uwb_serial_bridge.py` | 串口桥:pyserial 懒加载读线程 + **mock 注入路径** + 回调/快照 API |
| `go2w_search_ws/src/go2w_bridge/test/test_nooploop_uwb_protocol.py` | 解析器字节流回归测试(31 项) |
| `go2w_search_ws/src/go2w_bridge/test/test_uwb_serial_bridge.py` | 桥/降级/注入测试(19 项) |
| `go2w_search_ws/requirements.txt` | 新增 `pyserial>=3.5`(缺失时自动降级,不阻塞) |

## 2. 协议依据(逐字节核对官方源码)

帧布局取自 Nooploop 官方 C 库 `nooploop-dev/nlink_unpack`(克隆核对,非凭记忆):

- **0x55 0x01** LinkTrack P TagFrame0:定长 128B(位置/速度/8 基站测距/IMU/姿态四元数/电压)
- **0x55 0x07** LinkTrack AOA NodeFrame0:帧长 u16 LE @偏移 2;每节点 11B(role,id,dist int24,angle int16,fp/rx RSSI)——**钥匙扣跟随主数据源**
- **0x55 0x02** LinkTrack P NodeFrame0:TLV 透传容器
- 校验和 = 除末字节外累加取低 8 位(`NLINK_VerifyCheckSum`)
- 换算:int24 小端符号扩展;距离/位置 /1000、速度 /10000、角度 /100、电压 /1000、RSSI /-2(官方 `MULTIPLY_*`)

## 3. 工单硬性要求达成

| 要求 | 达成 | 证据 |
|---|---|---|
| 解析器纯逻辑、无 ROS 依赖 | ✅ | 模块级 import 仅 `dataclasses/struct/typing`;AST 静态断言 + 干净子进程导入断言(无 rclpy/serial) |
| 串口不可用走 mock 注入 | ✅ | 无端口 / pyserial 缺失 / 打开失败 三分支自动降级,`status().fallback_reason` 记录原因;mock 与真机共用同一 `NLinkStreamParser`(协议同构,非旁路假对象);`inject_bytes()/inject_tag()` 测试注入入口 |
| 解析器字节流测试全绿 | ✅ | `test_nooploop_uwb_protocol.py` 31 项全过:逐字节喂入、任意切分点、噪声重同步、校验和损坏恢复、超长/超短长度字段、结构矛盾帧、统计与复位 |
| 提交完成 | ✅ | 见本分支提交(仅新增文件 + requirements 一行,无存量文件改动) |

## 4. 测试结果

- 新增测试:**50 项全绿**(0.21s,无 ROS/串口/硬件)。
- 全量回归(`python -m pytest web/tests src/go2w_bridge/test -q`):966-968 passed;
  失败 = 9 个 web 基线失败 + sport_gateway_server 0~3 个**时序 flake**。
- flake 取证:基线(无本次代码)重跑 3 次,sport_gateway 分别挂 1/2/3 个(含工单列出的
  `test_bad_frame_does_not_kill_gateway_or_block_reconnect` 与同文件 `test_client_disconnect...`),
  单独运行恒绿 → 与本功能无关的既有 flake,不计入新增失败。

## 5. 已知限制 / 移交 eC(B-2 跟随逻辑)事项

1. 未在真机验证(本机无 UWB 硬件);首次接真机需用 `status()["fallback_reason"]` 确认走的是 serial 源。
2. TagFrame 的 `distances_m[i]==0.0` 表示对应基站无有效测量,有效性解释留给上层。
3. 0x55 0x03~0x06/0x08/0x09 等 NodeFrame1-6 未实现(按未知帧重同步跳过);跟随功能用不到,如需再扩。
4. eC 消费建议:`UwbSerialBridge(on_tag=...)` 或 `latest()` 快照;mock 轨迹用 `MockUwbSource(scenario=t->(d,θ))`。
5. ROS 节点封装由 eC/eI 负责(本模块刻意不 import rclpy,工单要求)。
