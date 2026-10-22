import numpy as np
import pytest

from agv_planner.planning.proximity import footprint_is_free, swept_footprint_is_free
from agv_planner.planning.safety import UnsafePathError, validate_route
from agv_planner.ros.support import load_ros_dependencies


def test_simulator_stops_before_wall():
    free = np.ones((100, 100), dtype=bool)
    free[:, 55] = False
    assert not swept_footprint_is_free(free, (45, 50, 0), (52, 50, 0), .34, .30, .05)


def test_unknown_is_blocked_for_footprint():
    free = np.ones((60, 60), dtype=bool)
    free[30, 30] = False
    assert not footprint_is_free(free, 30, 30, 0, .34, .30, .05)


def test_route_outside_map_is_rejected():
    free = np.ones((40, 40), dtype=bool)
    with pytest.raises(UnsafePathError):
        validate_route([(10, 10), (45, 10)], free)


def test_ros_optional_boundary_returns_result():
    available, error, modules = load_ros_dependencies()
    assert isinstance(available, bool)
    if not available:
        assert isinstance(error, str)
        assert modules is None
