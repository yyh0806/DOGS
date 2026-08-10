"""充电后一键验证 9 项搜索改动协同 (2026-08-05).

背景: 狗低电 FAULT blocker 解除后 (充电>40% + 物理重启恢复 sport_mode),
用此脚本一键触发 current_room 搜索 + 监控 coverage/waypoint 轨迹 + 报告完成状态.

9 项改动 (已部署 NX + commit 在 codex/search-fluency-continuous):
  ① DETECT skip (en_route 有样本跳 at_viewpoint fresh-wait)
  ② stale/unsync continue (循环内 + initial viewpoint)
  ③ motion_trapped 清 lockout (_motion_trap + _last_selection_reason)
  ④ _derive coverage 例外 (motion_trapped + coverage>=阈值 → completed)
  ⑤ Spin recovery (BT XML + params, motion_trapped 7→0)
  ⑥ env 注入 (PREFETCH/YAW_OPTIONAL/VLM_DISABLED/DETECTION_WAIT_SEC/RELEASE_ID)
  ⑦ backtrack penalty 量级修正 (angle→等效turn时间统一heading量级)
  ⑧ scan_start_infeasible 容忍 (默认1, NX env=3 给Spin脱困时间)
  ⑨ initial viewpoint unsync 容错

用法:
  python tools/search_validate_after_charge.py

期望 (良性起点): coverage 持续涨到 >=0.9 → completed; waypoint 序列持续前进
  (backtrack penalty 防往返); 偶发 motion_trapped 后 continue 不终结.
"""
import json
import math
import os
import sys
import time

import paramiko

# 内部开发工具: NX 局域网 192.168.1.200.
# 凭证通过环境变量注入, 无默认值 (禁止硬编码凭据入库).
# 生产用 SSH key + known_hosts (见 deploy_release.sh).
NX_HOST = os.environ.get("NX_HOST", "")
NX_USER = os.environ.get("NX_USER", "")
NX_PASS = os.environ.get("NX_PASS", "")
NX_KEY_PATH = os.environ.get("NX_KEY_PATH", "")
# SKIP_HOST_KEY=1 允许跳过 known_hosts 校验 (仅开发网段, 默认拒绝).
_SKIP_HOST_KEY = os.environ.get("SKIP_HOST_KEY", "") == "1"
MONITOR_SEC = int(os.environ.get("MONITOR_SEC", "600"))
STEP_SEC = int(os.environ.get("STEP_SEC", "30"))


def main():
    if not NX_HOST:
        print("FATAL: NX_HOST 未设置 (export NX_HOST=192.168.1.200)")
        return 1
    if not NX_USER:
        print("FATAL: NX_USER 未设置 (export NX_USER=nx)")
        return 1
    if not NX_PASS and not NX_KEY_PATH:
        print("FATAL: NX_PASS 或 NX_KEY_PATH 未设置 (export NX_PASS=... 或 NX_KEY_PATH=~/.ssh/id_rsa)")
        return 1

    ssh = paramiko.SSHClient()
    if _SKIP_HOST_KEY:
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    else:
        ssh.load_system_host_keys()
        ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
    if NX_KEY_PATH:
        key = paramiko.RSAKey.from_private_key_file(os.path.expanduser(NX_KEY_PATH))
        ssh.connect(NX_HOST, username=NX_USER, pkey=key, timeout=15)
    else:
        ssh.connect(NX_HOST, username=NX_USER, password=NX_PASS, timeout=15)

    def status():
        _, s, _ = ssh.exec_command(
            "curl -s http://127.0.0.1:8000/api/status --max-time 6")
        try:
            return json.loads(s.read().decode())
        except Exception as exc:
            print(f"status parse/SSH 失败: {type(exc).__name__}: {exc}")
            return None

    d = status()
    if not d:
        print("FAIL: web 不可达 (go2w-web down?)")
        return 1
    nav = d["navigation"]
    print(f"dog={d['dog_state']} sport={nav['sport_mode']} "
          f"activatable={nav['activatable']} battery={nav['battery_soc']} "
          f"reason={nav['reason']}")
    if not nav["activatable"]:
        print("BLOCKED: 狗未恢复 (activatable=False). 请充电/物理重启狗后重跑.")
        return 1

    trigger = (
        'curl -s -X POST http://127.0.0.1:8000/api/search_room '
        '-H "Content-Type: application/json" '
        '-H "Origin: http://127.0.0.1:8000" '
        "-d '{\"room\":\"__current__\",\"target_classes\":[\"person\"]}' "
        "--max-time 10"
    )
    _, s, _ = ssh.exec_command(trigger)
    resp = s.read().decode()
    if '"ok": true' not in resp:
        print(f"trigger FAIL: {resp[:300]}")
        return 1
    print(f"搜索已触发, 监控 {MONITOR_SEC}s ...")

    p0 = d["localization"]
    steps = MONITOR_SEC // STEP_SEC
    for i in range(steps):
        time.sleep(STEP_SEC)
        d = status()
        if not d:
            continue
        loc = d["localization"]
        t = d.get("tasks", {}).get("active")
        dist = math.hypot(loc["x"] - p0["x"], loc["y"] - p0["y"])
        active = "active" if t else "none"
        print(f"T{(i + 1) * STEP_SEC:3d}s: ({loc['x']:6.2f},{loc['y']:6.2f}) "
              f"{d['dog_state']:8s} task={active:5s} disp={dist:5.1f}m")
        if not t:
            print("搜索结束")
            break

    _, s, _ = ssh.exec_command(
        "sudo journalctl -u go2w-web --since '12 min ago' --no-pager 2>/dev/null "
        "| grep -oE \"completion_(reason|status)': '[^']*'|"
        "coverage_ratio': [0-9.]+|waypoints_reached': [0-9]+|"
        "targets_found': [0-9]+|elapsed_s': [0-9.]+\" | sort -u")
    print("\n=== mission result ===")
    print(s.read().decode()[:1500])
    ssh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
