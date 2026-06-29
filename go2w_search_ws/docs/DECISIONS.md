# 项目决策记录

> 记录关键架构/部署决策及背景，避免遗忘"为什么这么做"。

---

## 决策 1: 暂不换 NX，保留当前载荷 NX (2026-06-26)

**背景**: 当前载荷 NX (192.168.43.41) 出厂跑无人艇(USV)项目，已清理（禁用 usv_project + perception service），部署了我们的 nx_motion_node。曾考虑换一台干净的 NX。

**决策**: **暂不换**。当前 NX 环境(ROS2 Humble + unitree SDK + CycloneDDS)已就绪且我们的程序跑通了站立/姿态控制，换新 NX 收益不大、有迁移成本。

**已做的准备**: 写了 `docker/deploy_nx.sh` 一键部署脚本，**如果将来要换 NX**，前提是新 NX 出厂装好环境，一条命令即可部署我们的程序（自动探测网卡名）。

**留档**: 部署能力已具备，随时可换，不是阻塞项。

---

## 决策 2: 移动控制走 NX 的 /cmd_vel（不走 PC 直连）(2026-06-25)

**背景**: 加了 NX 载荷后，控狗职责从"PC 直连狗 SDK"迁移到"NX 经 ROS2 话题"。

**决策**: 手动控制指令走 `前端 → panel.py → cmd_publisher → /cmd_vel → NX nx_motion_node → SDK`。

**理由**:
- NX 网线直连狗主控，不依赖手机热点，控制最可靠
- lease 持续在 NX 上，压制狗主控残留乱跑程序
- 看门狗在 NX 上，即使 PC↔NX 热点断了，NX 也会自动停狗

---

## 决策 3: nx_motion_node 做成 systemd 服务 (2026-06-26)

**背景**: 狗乱跑根因是 motion 进程意外退出 → lease 释放 → 残留程序接管。

**决策**: motion 节点做成 `go2w-motion.service`（enabled + Restart=always）。

**效果**: 进程崩溃 2 秒内 systemd 自动重启并重新夺 lease，乱跑最多持续 2 秒。实测 kill -9 后自动恢复。
