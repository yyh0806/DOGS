#!/usr/bin/env python3
"""UWB 桥装配层 (功能 B-1 ↔ B-2 接口适配)。

职责(单一): 把 go2w_bridge.uwb_serial_bridge.UwbSerialBridge (eB 交付,
拉模式 latest() 快照) 适配成 nx_uwb_follow 期望的测距源合同 (eC 交付,
get_fix() -> fix dict, 字段合同见 nx_uwb_follow.py 模块头)。

环境变量(全部可选, 缺省即无硬件安全降级):
    GO2W_UWB_PORT             串口设备 (默认 /dev/ttyUSB0; 置空串强制 mock)
    GO2W_UWB_BAUD             波特率 (默认 115200 = LinkTrack 出厂值)
    GO2W_UWB_TAG_ID           跟随的钥匙扣标签 id (默认 1)
    GO2W_UWB_MODE             auto | serial | mock (默认 auto)
    GO2W_UWB_ANGLE_OFFSET_DEG AOA 方位角零偏标定 (默认 0; 正值=逆时针补偿)

fail-closed 说明:
    - 桥未启动/无快照/数值非有限/快照过老 => get_fix() 返回 ok=False 或
      None, 由 UwbFollowController 按"无有效测距"路径全停 (宁停不盲走)。
    - 角度零偏未标定时方位可能整体偏转, 但跟随控制律以距离为主、方位仅
      决定转向; 标定流程见 docs/UWB_FOLLOW.md (实机走 8 字记录偏差)。
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time

logger = logging.getLogger("go2w.nx_uwb_bridge")

# 快照信任窗: 超过此年龄的测距视为失效 (与 UwbFollowController 的
# fix 最大年龄同量级, 这里再设一道防线: 源头不给陈旧数据)。
_SNAPSHOT_MAX_AGE_SEC = 0.5

_singleton_lock = threading.Lock()
_singleton_bridge = None
_singleton_source = None


def _bridge_config_from_env():
    """环境变量 → UwbBridgeConfig (eB 约定: 环境映射由上层负责)。"""
    from go2w_bridge.uwb_serial_bridge import UwbBridgeConfig

    port = os.environ.get("GO2W_UWB_PORT", "/dev/ttyUSB0").strip() or None
    try:
        baud = int(os.environ.get("GO2W_UWB_BAUD", "115200"))
    except ValueError:
        baud = 115200
    try:
        tag_id = int(os.environ.get("GO2W_UWB_TAG_ID", "1"))
    except ValueError:
        tag_id = 1
    mode = os.environ.get("GO2W_UWB_MODE", "auto").strip().lower()
    if mode not in ("auto", "serial", "mock"):
        mode = "auto"
    return UwbBridgeConfig(
        port=port, baudrate=baud, mode=mode, followed_tag_id=tag_id,
    )


class _FollowFixSource:
    """把桥快照适配成 nx_uwb_follow 测距源合同 (get_fix)。"""

    def __init__(self, bridge, angle_offset_deg: float) -> None:
        self._bridge = bridge
        self._angle_offset_rad = math.radians(angle_offset_deg)

    def get_fix(self):
        snapshot = self._bridge.latest()
        if not isinstance(snapshot, dict):
            return None
        distance = snapshot.get("distance_m")
        if not isinstance(distance, (int, float)) or not math.isfinite(distance):
            return {"ok": False, "reason": "invalid_distance"}
        if distance <= 0.0:
            return {"ok": False, "reason": "non_positive_distance"}
        age = self._snapshot_age_sec(snapshot)
        if age is None or age < 0.0 or age > _SNAPSHOT_MAX_AGE_SEC:
            return {"ok": False, "reason": "stale_snapshot",
                    "age_sec": age}
        angle_deg = snapshot.get("angle_deg")
        if isinstance(angle_deg, (int, float)) and math.isfinite(angle_deg):
            # LinkTrack AOA: 0°=模块正前, 逆时针为正 (待实机确认符号);
            # azimuth_rad 合同: 机器人系 +左 (REP-103)。零偏用于标定修正。
            azimuth = math.radians(angle_deg) + self._angle_offset_rad
        else:
            azimuth = None  # 一维测距模块: 只跟不转 (控制器支持)
        return {
            "ok": True,
            "range_m": float(distance),
            "azimuth_rad": azimuth,
            "age_sec": age,
            "anchor_id": f"uwb-tag-{snapshot.get('id', '?')}",
        }

    @staticmethod
    def _snapshot_age_sec(snapshot):
        # 首选 received_monotonic (桥收包时刻, 无回绕问题); 旧快照无此
        # 字段时按"年龄未知"处理 (fail-closed), 不退回 uint32 回绕的
        # system_time_ms (见 uwb_serial_bridge._record_latest 注释)。
        received = snapshot.get("received_monotonic")
        if not isinstance(received, (int, float)) or not math.isfinite(received):
            return None
        return max(0.0, time.monotonic() - float(received))

    def status(self):
        """透传桥状态 (诊断/验收用, 不进跟随控制路径)。"""
        try:
            return self._bridge.status()
        except Exception:
            return {"source": None, "error": "bridge_status_unavailable"}


def get_follow_fix_source():
    """单例工厂: 启动 UWB 串口桥并返回测距源 (eC 的 _load_uwb_follow_source 合同)。

    任何初始化失败都返回 None —— 上层 /api/uwb_follow/start 会拒绝
    (uwb_source_unavailable), 状态查询/调参仍可用, 进程不崩。
    """
    global _singleton_bridge, _singleton_source
    with _singleton_lock:
        if _singleton_source is not None:
            return _singleton_source
        try:
            from go2w_bridge.uwb_serial_bridge import UwbSerialBridge

            config = _bridge_config_from_env()
            bridge = UwbSerialBridge(config=config)
            source_name = bridge.start()  # 串口不可用自动降级 mock (eB 保证)
            # fail-closed: 串口失败自动降级的 mock 会持续输出"3m 正前方"
            # 假测距, 喂给生产跟随等于盲走真狗。mock 只允许作为显式意图
            # (GO2W_UWB_MODE=mock, 演示/联调用), 其余一律视为源不可用。
            if source_name == "mock" and config.mode != "mock":
                logger.warning(
                    "UWB 串口不可用 (降级原因: %s) 且未显式要求 mock, "
                    "跟随源不接入 (fail-closed)", bridge.status().get(
                        "fallback_reason"))
                bridge.stop()
                return None
            if source_name != "serial":
                # 双保险: service 层已钉 GO2W_UWB_MODE=serial; 即使环境被
                # 误改, 非 serial 源也一律拒绝 (真机不碰任何 mock 数据)。
                logger.warning(
                    "UWB 源非串口 (%s), 真机模式拒绝接入", source_name)
                bridge.stop()
                return None
            try:
                offset = float(os.environ.get("GO2W_UWB_ANGLE_OFFSET_DEG", "0.0"))
            except ValueError:
                offset = 0.0
            _singleton_bridge = bridge
            _singleton_source = _FollowFixSource(bridge, offset)
            logger.info(
                "UWB 跟随源就绪: source=%s port=%s baud=%s tag=%s angle_offset=%sdeg",
                source_name, config.port, config.baudrate,
                config.followed_tag_id, offset,
            )
            return _singleton_source
        except Exception as exc:
            logger.warning("UWB 桥初始化失败, 跟随源不可用: %s", exc)
            return None


def shutdown_bridge():
    """进程退出时停桥 (幂等; web 主进程清理钩子调用)。"""
    global _singleton_bridge, _singleton_source
    with _singleton_lock:
        bridge, _singleton_bridge = _singleton_bridge, None
        _singleton_source = None
    if bridge is not None:
        try:
            bridge.stop()
        except Exception:
            logger.debug("UWB bridge stop failed", exc_info=True)


def reset_singleton_for_tests():
    """测试辅助: 清单例并停桥 (生产代码不得调用)。"""
    shutdown_bridge()
