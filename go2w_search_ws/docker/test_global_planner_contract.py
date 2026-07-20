"""Production contracts for the NX global planner configuration."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PARAMS = ROOT / "src/go2w_nav/config/nav2_params_3d.yaml"
PACKAGE_XML = ROOT / "src/go2w_nav/package.xml"


def test_dynamic_replanning_uses_smac_2d_instead_of_navfn_path_extraction():
    """Avoid NavFn's intermittent legal-potential/empty-path failure on Humble."""
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    planner = params["planner_server"]["ros__parameters"]
    grid = planner["GridBased"]

    assert grid["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert grid["allow_unknown"] is True
    assert grid["downsample_costmap"] is False
    assert grid["downsampling_factor"] == 1
    assert grid["max_iterations"] >= 1_000_000
    assert grid["max_on_approach_iterations"] >= 1_000
    assert grid["max_planning_time"] >= 1.0
    assert grid["cost_travel_multiplier"] >= 1.0
    # A tolerated endpoint can make ComputePath report success even when the
    # requested pose itself is inside an inscribed/lethal costmap cell.  Room
    # exploration needs the requested cell to be the reachable endpoint.
    assert grid["tolerance"] <= 0.05
    assert "use_astar" not in grid


def test_go2w_nav_declares_the_selected_smac_runtime_dependency():
    package_xml = PACKAGE_XML.read_text(encoding="utf-8")

    assert "<exec_depend>nav2_smac_planner</exec_depend>" in package_xml
