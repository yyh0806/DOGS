# DOGS 编码规范（Code Standards）

> 本规范约束本仓库的文件命名、目录组织与代码风格，提升可维护性。
> 创建于 2026-08 优化分支（`codex/optimize-project-structure`）。

---

## 1. 命名约定

### 1.1 Python 模块（`web/` 目录）

| 前缀 | 含义 | 示例 |
|------|------|------|
| `nx_` | NX 机载运行的核心节点/模块 | `nx_web_server.py`、`nx_motion_node.py` |
| `test_` | pytest 单测（与模块同目录） | `test_navigation_gateway.py` |
| `mock_` | 无硬件时的仿真/测试替身 | `mock_dog_state_publisher.py` |
| `verify_` | 端到端验证脚本 | `verify_nx_web.sh` |
| `diag_` | 故障诊断工具（`tools/`） | `diag_nav2_plan.py` |

### 1.2 ROS2 包（`src/`）

- 包名用 `go2w_` 前缀：`go2w_bridge`、`go2w_nav`、`go2w_sim`。
- 节点名用 `nx_` 前缀：`nx_sensor_node`、`nx_motion_node`。

### 1.3 systemd 服务（`docker/`）

- `go2w-` 前缀：`go2w-web.service`、`go2w-motion.service`。
- `livox-` 前缀（雷达相关）：`livox-mid360-driver.service`。
- 一次性/常驻分开：`Type=oneshot` 用于网络配置类（`livox-mid360-net.service`）。

### 1.4 话题命名

- 雷达：`/livox/lidar`（原始）、`/scan_mid360`（2D 扫描，Nav2 用）、`/mid360/points_nav`（导航点云）。
- 里程计：`/Odometry`（FastLIO 输出）、`/odom`（fuser 输出）、`/wheel_odom`（轮式，诊断用）。
- 控制：`/cmd_vel`（统一速度入口）、`/cmd_vel_nav`（Nav2 输出，诊断用）。

## 2. 目录组织

```
web/
  nx_*.py          # 核心模块（保持扁平，模块间同目录 import）
  test_*.py        # 单测（与模块同目录，pytest 自动发现）
  static/          # 前端静态资源（panel.html / map.js / *.png）
  verify_*.sh      # 验证脚本
docker/
  *.service        # systemd 单元
  deploy_*.sh      # 部署脚本
  test_*.py        # 部署契约测试
tools/
  diag_*.py|sh     # 诊断
  verify_*.sh      # 端到端验证
docs/
  ARCHITECTURE.md  # 架构（本仓库权威入口）
  *Runbook.md      # 运维手册
  TROUBLESHOOTING.md  # 排障（按问题编号）
```

## 3. 代码风格

- **Python**：PEP8；类型提示用于公共 API；`logger` 用 `logging.getLogger("go2w.<module>")`。
- **日志**：关键状态转换必须打日志；`INFO` 记录节点就绪/切换，`WARNING` 记录降级，异常只 `debug` 不抛（长驻节点）。
- **错误处理**：长驻节点（systemd 服务）任何异常不得导致进程退出；入口 `try/except` 包裹并记录。
- **配置**：环境变量优先（`Environment=` + `EnvironmentFile=`），默认值用 `os.environ.get(key, default)`。
- **文档**：中文注释解释"为什么"（背景坑），英文术语保留原文。

## 4. 部署与测试要求

1. 新增 `nx_*.py` 模块必须带同目录 `test_*.py`（契约测试）。
2. systemd 服务改动必须更新 `docker/test_*_contract.py`（部署契约，`build_release.sh` 会跑）。
3. 前端 WS 消息类型改动必须同步 `static/panel.html` 处理分支和 `web/test_panel_nav_state.js`。
4. 提交前跑：`git diff --check`、`node --check <js>`、`python3 -m pytest <改动目录> -q`。

## 5. 废弃流程

- 被取代的脚本：优先删除并更新 `docs/PROJECT_STRUCTURE.md` 引用；确有保留价值的历史版本移入 `docs/archive/`。
- 历史快照：`nx_deploy_snapshot/` 这类与主代码重复的目录一律归档到 `docs/archive/`，不再参与运行与构建。
