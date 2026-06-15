# Go2W 控制测试方案 v2

## 状态机 (v2 - 含机器人反馈)

```
DISCONNECTED
     │ connect() + stand()
     ▼
 STANDING ──RiseSit+BalanceStand──→ STOPPED ←──stop_move()── MOVING
                                        │                      │
                                        │ sit()                │ move()
                                        ▼                      │
                                     SITTING ──Sit()──→ SEATED
                                        │
                                        │ estop() (任意状态)
                                        ▼
                                    EMERGENCY (Damp)
```

| 状态 | 控制线程行为 | 机器人反馈 (SportModeState.mode) |
|------|-------------|------|
| STOPPED | BalanceStand + Move(0,0,0) @ 2Hz | mode=0 (NORMAL) |
| MOVING | BalanceStand + Move(vx,vy,vyaw) @ 20Hz | mode=1 (GAIT) |
| SITTING | Move(0,0,0)→StopMove→Sit | mode=2, progress 0→100 |
| SEATED | 不发送任何命令 | mode=2 |
| STANDING | RiseSit→BalanceStand→Move(0,0,0) | mode=3, progress 0→100 |
| EMERGENCY | Damp | mode=4 |

### 关键改进 (v1 → v2)

1. **订阅 `rt/sportmodestate`** — 获取机器人真实 mode/velocity/progress，不再盲猜状态
2. **STOPPED 显式 Move(0,0,0)** — 每 0.5s 发送一次，防止机器人速度指令超时后漂移
3. **MOVING→STOPPED 显式停止** — 控制线程检测到 STOPPED 后发送 Move(0,0,0)，不再仅靠"不发 Move"
4. **Sit/RiseSit 替代 StandDown/StandUp** — 使用专用坐下/起立 API
5. **StopMove 作为坐下前置** — 先取消速度再执行 Sit
6. **stats API 返回 robot_mode** — 前端可实时看到机器人状态

---

## 测试 1: 启动静止验证

| 项目 | 内容 |
|------|------|
| 前提 | panel.py 刚启动，站立序列完成 |
| 操作 | 不做任何操作，观察 30 秒 |
| 期望 | 机器人保持静止，不前进/后退/旋转 |
| 状态 | STOPPED |
| 日志验证 | `CTRL 启动` → `STANDING: RiseSit → BalanceStand` → `→ STOPPED` |
| 反馈验证 | `stats.robot_mode` = 0 (NORMAL), `stats.robot_velocity` ≈ [0,0,0] |
| 传感器 | IMU yaw 变化 < ±0.03 rad，视频画面静止 |

---

## 测试 2: 键盘 ↑ 前进

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED，狗静止 |
| 操作 | 按住 ↑ 键 2 秒 → 松开 |
| 期望 | 狗以 ~0.5 m/s 前进，松开后立即停止，不滑动 |
| 状态转换 | STOPPED → MOVING(vx=0.5) → STOPPED |
| 反馈验证 | MOVING 时 robot_mode=1, 停止后 robot_mode=0 |
| 日志 | `API: move` → `API: stop → STOPPED` |

---

## 测试 3: 键盘 ↓ 后退

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED，狗静止 |
| 操作 | 按住 ↓ 键 2 秒 → 松开 |
| 期望 | 狗以 -0.5 m/s 后退，松开后立即停止，不继续后滑 |
| 状态转换 | STOPPED → MOVING(vx=-0.5) → STOPPED |
| 注意 | 这是之前最容易出 bug 的操作，重点验证 |
| 反馈验证 | 停止后 robot_velocity ≈ [0,0,0] |

---

## 测试 4: 键盘 ← 左转

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 按住 ← 键 2 秒 → 松开 |
| 期望 | 狗逆时针旋转，松开后停 |
| 传感器 | IMU yaw 单调增加 |

---

## 测试 5: 键盘 → 右转

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 按住 → 键 2 秒 → 松开 |
| 期望 | 狗顺时针旋转，松开后停 |
| 传感器 | IMU yaw 单调减小 |

---

## 测试 6: 坐下按钮

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED，狗站立 |
| 操作 | 点击"坐下" |
| 期望 | 狗坐下（Sit，非完全趴下） |
| 状态转换 | STOPPED → SITTING → SEATED |
| 日志 | `API: sit 入队` → `SITTING: Move(0,0,0) → StopMove → Sit` → `→ SEATED` |
| 反馈验证 | robot_mode=2, robot_progress 从 0→100 |
| 后续验证 | 按下任意方向键 → **不应响应**（SEATED 忽略 move） |
| 后续验证 | 30 秒内不自行站起 |

---

## 测试 7: 坐下 → 站立 → 前进

| 项目 | 内容 |
|------|------|
| 前提 | SEATED |
| 操作 | 1) 点"站立"等 7 秒 2) 按 ↑ |
| 期望 | 狗站起后静止 → 按键后前进 |
| 状态转换 | SEATED → STANDING → STOPPED → MOVING |
| 日志 | `API: stand 入队` → `STANDING: RiseSit → BalanceStand` → `→ STOPPED` |

---

## 测试 8: 急停按钮

| 项目 | 内容 |
|------|------|
| 前提 | MOVING，狗正在前进 |
| 操作 | 点击"急停" |
| 期望 | 狗立即趴下（Damp），不管之前什么状态 |
| 状态转换 | MOVING → EMERGENCY |
| 后续 | 按方向键 → 不响应（需先点"站立"） |

---

## 测试 9: 快速交替按键

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 快速交替按 ↑/↓，每次按 0.2 秒，重复 10 次 |
| 期望 | 每次按键狗有对应方向动作，松开即停，不出现滑动 |
| 重点 | 验证状态机竞态保护 |

---

## 测试 10: 长时间静止

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 静置 2 分钟 |
| 期望 | 狗始终保持静止，不漂移 |
| 验证 | stats.robot_velocity 始终 ≈ [0,0,0] |

---

## 测试 11: 看门狗自动停止

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 快速点按 ↑（只按一下不持续）|
| 期望 | 狗可能短暂动一下，0.3 秒后自动停止 |
| 日志 | `看门狗: 0.3s 无指令, 自动停止` |

---

## 测试 12: 浏览器关闭重连

| 项目 | 内容 |
|------|------|
| 前提 | 正常使用中 |
| 操作 | 关浏览器 → 等 10 秒 → 重新打开 |
| 期望 | 视频恢复，狗保持静止，按钮可正常控制 |

---

## 测试 13: 进程重启

| 项目 | 内容 |
|------|------|
| 前提 | 正常使用中 |
| 操作 | `pkill -9 -f panel.py` → 等待 → 重新启动 |
| 期望 | 狗短暂执行站立序列 → 静止在 STOPPED 状态 |
| 重点 | 确保旧进程 DDS 速度指令不会残留 |

---

## 测试 14: 鼠标点击虚拟方向键

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 鼠标按住 ↑ 按钮 → 松开 |
| 期望 | 同键盘 ↑，前进后停止 |

---

## 测试 15: 文字指令 (VLM)

| 项目 | 内容 |
|------|------|
| 前提 | STOPPED |
| 操作 | 输入"前进两米" → 发送 |
| 期望 | VLM 解析 → move 任务 → 前进 → stop |

---

## 测试 16: robot_mode 反馈验证 (新增)

| 项目 | 内容 |
|------|------|
| 前提 | 正常使用中 |
| 操作 | 在不同状态间切换，监控 `/api/status` 返回的 `stats.robot_mode` |
| 期望 | STOPPED→0, MOVING→1, SEATED→2, EMERGENCY→4 |
| 用途 | 确保机器人反馈与我们状态机一致 |

---

## 测试 17: 前进中突然坐下 (新增)

| 项目 | 内容 |
|------|------|
| 前提 | MOVING，狗正在前进 |
| 操作 | 点击"坐下" |
| 期望 | 狗先停止前进，然后坐下 |
| 状态转换 | MOVING → SITTING → SEATED |
| 日志 | `API: sit 入队` → `SITTING: Move(0,0,0) → StopMove → Sit` |