"""Boundary and failure-mode regressions for offline collision safety."""
import math

import numpy as np
import pytest

from agv_planner.planning.proximity import (
    footprint_is_free,
    nearest_obstacle_distance,
    obstacle_distance_field,
    swept_footprint_is_free,
)
from agv_planner.planning.safety import (
    UnsafePathError,
    footprint_radius,
    validate_footprint_clearance,
    validate_route,
)


@pytest.mark.parametrize('dimensions', [(0, 1), (1, 0), (-1, 1), (1, -1)])
def test_radius_rejects_nonpositive_dimensions(dimensions):
    with pytest.raises(ValueError, match='positive'):
        footprint_radius(*dimensions)


def test_radius_uses_bounding_circle():
    assert footprint_radius(3, 4) == 2.5


@pytest.mark.parametrize('clearance', [0, 0.5, 2.49, -1])
def test_clearance_rejects_insufficient_radius(clearance):
    with pytest.raises(UnsafePathError, match='Clearance'):
        validate_footprint_clearance(clearance, 3, 4)


@pytest.mark.parametrize('clearance', [2.5, 3.0])
def test_clearance_accepts_boundary_and_above(clearance):
    assert validate_footprint_clearance(clearance, 3, 4) is True


@pytest.mark.parametrize('path', [None, [], [(1, 1)]])
def test_route_requires_two_points(path):
    with pytest.raises(UnsafePathError, match='at least two'):
        validate_route(path, np.ones((20, 20), dtype=bool))


@pytest.mark.parametrize('grid', [np.array([]), np.ones((4,), dtype=bool), np.ones((2, 2, 2), dtype=bool)])
def test_route_rejects_invalid_grid(grid):
    with pytest.raises(UnsafePathError, match='Invalid occupancy'):
        validate_route([(1, 1), (2, 2)], grid)


def test_route_rejects_changed_start():
    with pytest.raises(UnsafePathError, match='start changed'):
        validate_route([(1, 1), (3, 3)], np.ones((20, 20), dtype=bool), start=(2, 1))


def test_route_rejects_changed_goal():
    with pytest.raises(UnsafePathError, match='goal changed'):
        validate_route([(1, 1), (3, 3)], np.ones((20, 20), dtype=bool), goal=(4, 3))


def test_route_rejects_missing_mandatory_stop():
    with pytest.raises(UnsafePathError, match='Mandatory stop'):
        validate_route([(1, 1), (3, 3)], np.ones((20, 20), dtype=bool), required=[(2, 2)])


def test_route_accepts_required_stop_and_matching_endpoints():
    grid = np.ones((20, 20), dtype=bool)
    assert validate_route([(2, 2), (5, 5), (8, 8)], grid, start=(2, 2), goal=(8, 8), required=[(5, 5)])


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_route_rejects_nonfinite_coordinates(bad):
    with pytest.raises(UnsafePathError, match='Nonfinite'):
        validate_route([(2, 2), (bad, 4)], np.ones((20, 20), dtype=bool))


def test_route_rejects_collision_midsegment():
    grid = np.ones((20, 20), dtype=bool)
    grid[5, 5] = False
    with pytest.raises(UnsafePathError, match='Collision'):
        validate_route([(2, 5), (9, 5)], grid)


@pytest.mark.parametrize('resolution', [0, -0.1, math.nan, math.inf])
def test_distance_field_rejects_invalid_resolution(resolution):
    with pytest.raises(ValueError, match='Resolution'):
        obstacle_distance_field(np.ones((10, 10), dtype=bool), resolution)


@pytest.mark.parametrize('position', [(-1, 3), (3, -1), (10, 3), (3, 10)])
def test_nearest_obstacle_outside_map_returns_zero(position):
    assert nearest_obstacle_distance(np.ones((10, 10)), *position) == 0.0


def test_nearest_obstacle_clamps_negative_distance():
    field = np.ones((10, 10))
    field[4, 5] = -2
    assert nearest_obstacle_distance(field, 5, 4) == 0.0


@pytest.mark.parametrize('dimensions', [(0.0, 0.3, 0.05), (0.3, 0.0, 0.05), (0.3, 0.3, 0.0), (-0.3, 0.3, 0.05)])
def test_footprint_rejects_invalid_dimensions(dimensions):
    with pytest.raises(ValueError, match='positive'):
        footprint_is_free(np.ones((40, 40), dtype=bool), 20, 20, 0, *dimensions)


def test_swept_footprint_accepts_stationary_safe_pose():
    grid = np.ones((100, 100), dtype=bool)
    assert swept_footprint_is_free(grid, (50, 50, 0), (50, 50, 0), .3, .3, .05)


def test_swept_footprint_detects_intermediate_translation_collision():
    grid = np.ones((100, 100), dtype=bool)
    grid[50, 50] = False
    assert not swept_footprint_is_free(grid, (30, 50, 0), (70, 50, 0), .3, .3, .05)


def test_swept_footprint_detects_intermediate_rotation_collision():
    grid = np.ones((100, 100), dtype=bool)
    grid[46, 54] = False
    assert not swept_footprint_is_free(grid, (50, 50, 0), (50, 50, math.pi / 2), .5, .12, .05)
