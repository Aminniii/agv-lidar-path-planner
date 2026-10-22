"""Additional deterministic path-planning boundary regressions (offline only)."""
from unittest.mock import patch

import numpy as np

from agv_planner.planning.algorithms import compact_path, optimized_search


def test_clearance_search_disallows_diagonal_corner_cutting():
    grid = np.ones((4, 4), dtype=bool)
    grid[0, 1] = False
    distance = np.ones((4, 4), dtype=float)
    route = optimized_search(grid, (0, 0), (3, 3), 'A*', distance, 2.0)
    assert route is not None
    assert route[0] == (0, 0) and route[-1] == (3, 3)
    assert route[1] != (1, 1)  # diagonal through blocked corner forbidden


def test_compaction_does_not_jump_over_blocked_segment():
    grid = np.ones((6, 6), dtype=bool)
    path = [(0, 0), (1, 0), (2, 0), (3, 0)]
    with patch('agv_planner.planning.algorithms.clear_segment', side_effect=lambda g, a, b: abs(a[0]-b[0]) <= 1):
        result = compact_path(path, grid, max_segment_px=50)
    assert result == path
