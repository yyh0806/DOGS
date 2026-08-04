import pytest
from pathlib import Path

from wait_lifecycle_active import normalize_nodes, states_are_active


def test_normalize_nodes_accepts_slashes_and_removes_duplicates():
    assert normalize_nodes([
        "/controller_server", "bt_navigator", "controller_server"
    ]) == ("controller_server", "bt_navigator")


def test_normalize_nodes_rejects_empty_names():
    with pytest.raises(ValueError, match="non-empty"):
        normalize_nodes(["/"])


def test_states_are_active_requires_state_id_and_label():
    nodes = ("controller_server", "bt_navigator")
    assert states_are_active(nodes, {
        "controller_server": {"id": 3, "label": "active"},
        "bt_navigator": {"id": 3, "label": "active"},
    }) is True
    assert states_are_active(nodes, {
        "controller_server": {"id": 3, "label": "active"},
        "bt_navigator": {"id": 2, "label": "inactive"},
    }) is False
    assert states_are_active(nodes, {
        "controller_server": {"id": 3, "label": "active"},
    }) is False


def test_probe_uses_unique_ros_node_name():
    source = Path(__file__).with_name(
        "wait_lifecycle_active.py").read_text(encoding="utf-8")

    assert 'Node(f"go2w_lifecycle_readiness_probe_{os.getpid()}")' in source
