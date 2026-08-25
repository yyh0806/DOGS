# 审核记录:R9 集成分支 fail-closed 专项(r1 职责, 主会话执行)

verdict: **approve**(2026-08-24, 对 c3addcb)

## 逐项核对
1. fail-closed 全路径:
   - UWB 断流: fix age>0.5s/无效 → 全停(test_fix_staleness_and_bounds_rejected)✓
   - scan 断流: directional_clearance age>0.5s → None → 闸门挡 → 平移零
     (test_gate_fail_closed_when_probe_raises_or_missing;motion 侧 manual
     指令超时 0.5s 双保险)✓
   - GPS 七条停车路径成文且各自有测试(GPS_NAV.md §3 / test_gps_nav 74 项)✓
   - 隐式 mock 拒绝 + uint32 回绕修复各带回归测试 ✓
2. 速度通道: UWB 跟随 = robot.move(manual=True)→/cmd_vel(manual 通道,
   arbiter run_manual_action 所有权 + 0.5s 指令超时);GPS = PointNav→
   Nav2→/cmd_vel_nav(nav 通道 + scan watchdog + forward clamp)。无旁路 ✓
3. 测试真实性: 既有测试零弱化(web/tests 仅 3 个新文件);基线 9 失败
   逐条属已知清单;新增 175 项全绿 ✓
4. 既有契约: map_odom_fuser/bringup_slam_nav2.sh/nav2_params_3d/
   nx_navigation_arbiter/nx_motion_node/motion_controller/motion_safety
   全部零 diff ✓
5. 参数双通道: GO2W_UWB_{PORT,BAUD,TAG_ID,MODE,ANGLE_OFFSET_DEG}+
   GO2W_UWB_FOLLOW_DISABLE 经 nx_uwb_bridge 环境映射 ✓
6. 文档: README 两行 / ARCHITECTURE 两条链+WS 表 / 集成报告 ✓

## 非阻塞备注
- manual 通道理论允许倒车, eC 控制器层 vx>=0 由测试锁住(架构可接受)
- 遗留风险 7 项见集成报告 §5(全部为实机标定类, 无代码缺陷)
