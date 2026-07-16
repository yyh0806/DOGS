"""Release fingerprint contract shared by motion, web, and deployment."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
BRIDGE_ROOT = ROOT / "src" / "go2w_bridge"
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))


def test_release_id_uses_one_bounded_environment_value():
    from go2w_bridge.build_info import release_id

    assert release_id({"GO2W_RELEASE_ID": "abc123"}) == "abc123"
    assert release_id({"GO2W_RELEASE_ID": "  "}) == "development"
    assert len(release_id({"GO2W_RELEASE_ID": "x" * 100})) == 64


def test_motion_and_web_import_the_shared_release_id():
    motion = (BRIDGE_ROOT / "go2w_bridge" / "nx_motion_node.py").read_text(
        encoding="utf-8")
    web = (ROOT / "web" / "nx_web_server.py").read_text(encoding="utf-8")

    assert "from .build_info import release_id" in motion
    assert "from go2w_bridge.build_info import release_id" in web
    assert '"release_id"' in motion
    assert '"release_id"' in web


def test_deploy_scripts_export_one_release_id():
    for relative in ("docker/deploy_nx.sh", "docker/deploy_nx_web.sh"):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "GO2W_RELEASE_ID" in text, relative


def test_legacy_motion_deploy_copies_the_complete_v4_runtime():
    deploy = (ROOT / "docker" / "deploy_nx.sh").read_text(encoding="utf-8")
    for filename in (
        "build_info.py",
        "motion_types.py",
        "motion_machine.py",
        "motion_protocol.py",
        "motion_safety.py",
        "motion_controller.py",
        "unitree_sport_adapter.py",
        "nx_motion_node.py",
        "nx_sensor_node.py",
    ):
        assert filename in deploy, filename


def test_legacy_web_deploy_copies_auth_and_canonical_motion_protocol_runtime():
    deploy = (ROOT / "docker" / "deploy_nx_web.sh").read_text(encoding="utf-8")
    for filename in (
        "nx_control_auth.py",
        "nx_motion_intent.py",
        "nx_navigation_gateway.py",
        "motion_types.py",
        "motion_machine.py",
        "motion_protocol.py",
    ):
        assert filename in deploy, filename
