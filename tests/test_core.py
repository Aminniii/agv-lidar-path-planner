import math
import numpy as np
from agv_planner.core import (astar, search_path, optimized_search, inflate_free_grid,
                              clear_segment, compact_path, mega_bridge_mix,
                              mecanum_wheel_rates, mecanum_body_velocity)

def test_planners_find_safe_route():
    grid = np.ones((20, 20), dtype=bool)
    grid[1:19, 10] = False
    for algorithm in ("A*", "Dijkstra", "Theta*"):
        path = search_path(grid, (2, 10), (17, 10), algorithm)
        assert path is not None
        assert path[0] == (2, 10) and path[-1] == (17, 10)
        assert all(clear_segment(grid, a, b) for a, b in zip(path, path[1:]))

def test_blocked_route():
    grid = np.ones((12, 12), dtype=bool)
    grid[:, 6] = False
    assert astar(grid, (2, 5), (9, 5)) is None

def test_inflation():
    grid = np.ones((11, 11), dtype=bool)
    grid[5, 5] = False
    result = inflate_free_grid(grid, 2)
    assert not result[5, 7] and result[5, 8]

def test_corner_cutting():
    grid = np.ones((4, 4), dtype=bool)
    grid[0, 1] = False
    grid[1, 0] = False
    assert astar(grid, (0, 0), (1, 1)) is None

def test_wheel_roundtrip():
    expected = (0.25, -0.1, 0.3)
    wheels = mecanum_wheel_rates(*expected, 0.04, 0.235)
    actual = mecanum_body_velocity(wheels, 0.04, 0.235)
    assert np.allclose(expected, actual)

def test_mega_command_limit():
    commands = mega_bridge_mix(10, 10, 10)
    assert max(abs(x) for x in commands) <= 2000

def test_clear_segment_blocked():
    grid = np.ones((8, 8), dtype=bool)
    grid[4, 4] = False
    assert not clear_segment(grid, (1, 1), (6, 6))
