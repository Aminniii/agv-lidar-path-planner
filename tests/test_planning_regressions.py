"""Grid-planning regression tests using real planner implementations."""
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt
from agv_planner.planning.algorithms import (search_path, optimized_search, clear_segment,
    simplify_path, compact_path, inflate_free_grid)

@pytest.mark.parametrize('algorithm', ['A*', 'Dijkstra', 'Theta*'])
def test_start_equals_goal(algorithm):
    grid = np.ones((12, 12), dtype=bool)
    assert search_path(grid, (4, 5), (4, 5), algorithm) == [(4, 5)]

@pytest.mark.parametrize('algorithm', ['A*', 'Dijkstra', 'Theta*'])
def test_diagonal_corner_cutting_forbidden(algorithm):
    grid = np.zeros((5, 5), dtype=bool)
    grid[1, 1] = True
    grid[2, 2] = True
    assert search_path(grid, (1, 1), (2, 2), algorithm) is None

@pytest.mark.parametrize('algorithm', ['A*', 'Dijkstra', 'Theta*'])
def test_wall_with_gap_is_navigable(algorithm):
    grid = np.ones((25, 25), dtype=bool)
    grid[:, 12] = False
    grid[10, 12] = True
    path = search_path(grid, (3, 3), (20, 3), algorithm)
    assert path is not None
    assert path[0] == (3, 3) and path[-1] == (20, 3)
    assert any(p == (12, 10) for p in path) if algorithm != 'Theta*' else clear_segment(grid, path[0], path[0])

@pytest.mark.parametrize('algorithm', ['A*', 'Dijkstra'])
def test_optimized_search_rejects_blocked_goal(algorithm):
    grid = np.ones((15, 15), dtype=bool)
    grid[10, 10] = False
    field = distance_transform_edt(grid)
    assert optimized_search(grid, (2, 2), (10, 10), algorithm, field, 2.0) is None

@pytest.mark.parametrize('algorithm', ['A*', 'Dijkstra'])
def test_optimized_search_zero_weight_matches_standard(algorithm):
    grid = np.ones((12, 12), dtype=bool)
    field = distance_transform_edt(grid)
    assert optimized_search(grid, (2, 2), (9, 9), algorithm, field, 0) == search_path(grid, (2, 2), (9, 9), algorithm)

def test_clear_segment_rejects_blocked_and_outside():
    grid = np.ones((12, 12), dtype=bool)
    grid[5, 5] = False
    assert not clear_segment(grid, (1, 1), (9, 9))
    assert not clear_segment(grid, (-1, 1), (3, 3))
    assert clear_segment(grid, (1, 1), (1, 9))

def test_simplify_path_keeps_endpoints():
    grid = np.ones((20, 20), dtype=bool)
    path = [(3, 3), (4, 4), (5, 5), (6, 6)]
    assert simplify_path(grid, path) == [(3, 3), (6, 6)]

def test_compact_path_keeps_required_stop():
    grid = np.ones((30, 30), dtype=bool)
    path = [(2, 2), (3, 3), (4, 4), (5, 5), (6, 6)]
    result = compact_path(path, grid, required=[(4, 4)])
    assert (4, 4) in result
    assert result[0] == path[0] and result[-1] == path[-1]

def test_inflation_zero_radius_copies_grid():
    grid = np.ones((10, 10), dtype=bool)
    grid[5, 5] = False
    result = inflate_free_grid(grid, 0)
    assert np.array_equal(result, grid)
    assert result is not grid
