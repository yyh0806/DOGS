from pathlib import Path
import ast


ROOT = Path(__file__).resolve().parents[1]


def test_costmap_bridge_is_a_persistent_slam_nav_companion():
    unit = (ROOT / "docker/costmap-bridge.service").read_text(encoding="utf-8")
    slam_unit = (ROOT / "docker/go2w-slam-nav.service").read_text(encoding="utf-8")

    assert "PartOf=go2w-slam-nav.service" in unit
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in unit
    assert (
        "FASTRTPS_DEFAULT_PROFILES_FILE="
        "/home/nx/go2w/current/payload/docker/fastdds_udp.xml"
    ) in unit
    assert "ExecStartPre=/usr/bin/rm -f /tmp/costmap_lite.json" in unit
    assert "/tmp/map_frontier_walls.json" in unit
    assert "WorkingDirectory=/home/nx/go2w/current/payload" in unit
    assert "source /home/nx/go2w/current/payload/install/setup.bash" in unit
    assert (
        "python3 -u /home/nx/go2w/current/payload/web/costmap_bridge.py"
        in unit
    )
    assert "Restart=always" in unit
    assert "Wants=costmap-bridge.service" in slam_unit


def test_frontend_labels_mapping_and_live_nav_obstacles():
    panel = (ROOT / "web/static/panel.html").read_text(encoding="utf-8")
    assert "MID360 建图" in panel
    assert "Nav2 实时障碍" in panel


def _load_downsample_helper():
    source = (ROOT / "web/costmap_bridge.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_downsample_values"
    )
    namespace = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), "<helper>", "exec"), namespace)
    return namespace["_downsample_values"]


def test_costmap_bridge_uses_global_map_and_preserves_small_obstacles():
    source = (ROOT / "web/costmap_bridge.py").read_text(encoding="utf-8")
    assert "'/global_costmap/costmap'" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "'/map_frontier'" in source
    assert "'/tmp/map_frontier_walls.json'" in source
    assert "self._frontier_pub" not in source

    downsample = _load_downsample_helper()
    values = [0] * 36
    values[1 * 6 + 1] = 100
    values[4 * 6 + 5] = 80
    assert downsample(values, 6, 6, 3) == [100, 0, 0, 80]

    unknown = [-1] * 9
    assert downsample(unknown, 3, 3, 3) == [-1]


def test_frontier_wall_extraction_keeps_a_bounded_persistent_wall_layer():
    source = (ROOT / "web/costmap_bridge.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_extract_occupied_points"
    )
    namespace = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 "<helper>", "exec"), namespace)
    extract = namespace["_extract_occupied_points"]

    values = [0] * 100
    for col in range(10):
        values[5 * 10 + col] = 100
    points = extract(
        values, 10, 10, resolution=0.1, origin_x=-0.5,
        origin_y=-0.5, occupied_threshold=65, max_points=4)

    assert 1 <= len(points) <= 4
    assert all(abs(point[1] - 0.05) < 1e-6 for point in points)
