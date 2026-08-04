# wheel odom 轮速标定 Runbook (H2)

> `nx_sensor_node.py` 的 `/odom` xy 用 `motor_state[12-15]` 4 轮 `dq` 平均 × `wheel_radius` 积分（代码 `nx_sensor_node.py:186-197`）。
> 审查 H2 指出: 4 轮 `dq` **符号约定未验证**(镜像安装时左右轮符号可能相反 → `sum/4≈0` → xy 恒 0, 建图失败)。
> **阶段A 红线**: 不改 `nx_sensor_node.py` 代码, 本文档只指导**实车标定参数 + 诊断符号**。

## 前置
- `go2w-sensor.service` 跑着, `/imu` `/odom` 有数据
- 狗能 `BalanceStand` + `Move` (实车验证 2026-06-29)
- NX 能订阅 `rt/lowstate` (unitree_sdk2py 装好)

## 步骤 1: 读 4 轮 dq 确认符号(关键)

写一个**独立诊断脚本**(不入库, 不碰红线文件), 订阅 `rt/lowstate` 打印 4 轮 `dq`:

```python
# tools/check_wheel_dq.py  (临时诊断脚本, 不 commit)
from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
import time
ChannelFactory().Init(0, 'enxc8a362616c4c')   # 连狗 USB 网卡
def cb(msg):
    ms = msg.motor_state
    if len(ms) >= 16:
        print(f"轮 dq[12-15]: {[round(float(ms[i].dq), 2) for i in (12, 13, 14, 15)]}")
ChannelSubscriber('rt/lowstate', LowState_).Init(cb, 1)
time.sleep(30)
```

让狗**匀速直行** (`Move(0.2, 0, 0)` 约 2 秒, 操作员护着), 观察 4 轮 `dq`:

| 观察 | 结论 | 动作 |
|------|------|------|
| 4 轮 `dq` **同号**(全正或全负, 数值接近) | ✅ `sum/4` 有效 | wheel odom 正常, 进步骤 2 |
| 左右轮 `dq` **符号相反**(如 [+2,+2,-2,-2]) | ❌ 镜像安装 | 需在 nx_sensor 加 `sign[i]` 数组(阶段A 红线, **需专门破线授权**, 见下) |
| 4 轮 `dq` 全 ≈ 0(狗没动) | 狗未进入 locomotion | 先确认 BalanceStand 后能 Move(见 TECH_DECISIONS 第一节) |

## 步骤 2: 标定 wheel_radius

默认 `wheel_radius = 0.065` m (`nx_sensor_node.py:64`, Go2W 轮径)。标定二选一:

1. **直接量**: 卡尺测轮胎外径 / 2(最准)
2. **闭环标定**: 让狗走已知距离 `L`(米, 卷尺量), 读 `/odom` 累计位移 `D`:
   - 实际轮径 = `0.065 × L / D`
   - 用 env `wheel_radius` 或 launch 参数覆盖(deploy 时传, 不改代码)

## 步骤 3: 验证 odom 精度

| 动作 | 期望 |
|------|------|
| 推狗直走 5 米 | `/odom.pose.position.x` ≈ 5 (±0.3, 室内硬地) |
| 原地转 360° | `/odom.yaw`(IMU) ≈ ±2π, xy 不漂(只转不走) |
| 走方形(3m×3m) 回原点 | xy 误差 < 0.5 m(室内硬地, 无打滑) |

## 已知局限

- **轮足切换/打滑**: wheel odom 会漂。FAST_LIO(MID360 自带 IMU)是更精确备选(见 `TECH_DECISIONS.md` 二节)。
- **阶段A 红线**: `nx_sensor_node.py` 不可改。符号修正若必需, 须先在 gan-harness 申请破线(更新 `eval-rubric-slam.md` 检查项 4), 不能擅改。
- **dq 正方向**: 本文档假设 SDK `motor_state[i].dq` 正 = 狗前进方向轮子转动。步骤 1 实测确认。

## 相关

- 代码: `src/go2w_bridge/go2w_bridge/nx_sensor_node.py:186-197` (wheel odom 积分)
- 决策: `docs/TECH_DECISIONS.md` 二节(FAST_LIO + MID360), `docs/PROJECT_STRUCTURE.md` 五节(建图阻塞已解除)
- 长期记忆: `mid360-livox-online`(雷达点亮, FAST_LIO 备选解锁)
