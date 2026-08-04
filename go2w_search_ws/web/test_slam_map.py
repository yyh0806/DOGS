from nx_slam_map import ObstacleGridAccumulator


def test_obstacle_grid_quantizes_and_deduplicates_scan_points():
    grid = ObstacleGridAccumulator(resolution=0.1, max_points=10)

    points = grid.update([[1.04, 2.04], [1.05, 2.05], [3.0, -1.0]])

    assert [1.0, 2.0] in points
    assert [3.0, -1.0] in points
    assert len(points) == 2


def test_obstacle_grid_keeps_most_recent_cells_when_limited():
    grid = ObstacleGridAccumulator(resolution=1.0, max_points=2)

    points = grid.update([[0, 0], [1, 0], [2, 0]])

    assert points == [[1.0, 0.0], [2.0, 0.0]]
