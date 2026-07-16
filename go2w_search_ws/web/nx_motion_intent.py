"""Thin web adapter for the canonical bridge motion protocol."""

from __future__ import annotations

from pathlib import Path
import sys
from uuid import uuid4

try:
    from go2w_bridge.motion_protocol import MotionIntentEnvelope
except ImportError:
    bridge_root = Path(__file__).resolve().parents[1] / "src" / "go2w_bridge"
    if bridge_root.is_dir() and str(bridge_root) not in sys.path:
        sys.path.insert(0, str(bridge_root))
    try:
        from go2w_bridge.motion_protocol import MotionIntentEnvelope
    except ImportError:  # Direct-file NX deployment.
        from motion_protocol import MotionIntentEnvelope


def build_motion_intent(intent: object, *, source: object, request_id=None) -> str:
    envelope = MotionIntentEnvelope.parse({
        "schema_version": 1,
        "request_id": str(request_id or uuid4()),
        "intent": str(intent),
        "source": str(source),
    })
    return envelope.to_json()


__all__ = ["build_motion_intent"]
