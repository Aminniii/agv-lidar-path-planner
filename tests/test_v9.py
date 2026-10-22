import math

import numpy as np
import pytest

from agv_planner.config.settings import load_settings
from agv_planner.planning.algorithms import (
    adaptive_reduce_path,
    inflate_free_grid,
    mecanum_body_velocity,
    mecanum_wheel_rates,
    optimized_search,
    search_path,
    smooth_corners,
)
from agv_planner.planning.safety import (
    UnsafePathError,
    footprint_radius,
    validate_footprint_clearance,
    validate_route,
)


def test_default_config():
    s = load_settings()
    assert s.wheel_radius_m == 0.04
    assert s.track_width_m == 0.29

def test_config_override(tmp_path):
    p = tmp_path/'robot.yaml'
    p.write_text('robot:\n  wheel_radius_m: 0.05\n')
    assert load_settings(p).wheel_radius_m == 0.05

def test_footprint_validation():
    assert math.isclose(footprint_radius(.34,.30),math.hypot(.34,.30)/2)
    with pytest.raises(UnsafePathError): validate_footprint_clearance(.20,.34,.30)
    assert validate_footprint_clearance(.25,.34,.30)

def test_missing_stop():
    grid=np.ones((30,30),dtype=bool)
    with pytest.raises(UnsafePathError): validate_route([(4,4),(20,20)],grid,required=[(10,15)])

def test_blocked_segment():
    grid=np.ones((30,30),dtype=bool); grid[12,12]=False
    with pytest.raises(UnsafePathError): validate_route([(4,4),(20,20)],grid)

def test_preserved_endpoints():
    grid=np.ones((30,30),dtype=bool)
    with pytest.raises(UnsafePathError): validate_route([(4,4),(20,20)],grid,start=(5,4))

@pytest.mark.parametrize('algorithm',['A*','Dijkstra','Theta*'])
def test_path_algorithms_validated(algorithm):
    grid=np.ones((50,50),dtype=bool);grid[5:45,24]=False;grid[23:27,24]=True
    path=search_path(grid,(4,20),(46,20),algorithm)
    assert validate_route(path,grid,start=(4,20),goal=(46,20))

def test_smoothed_route_validation():
    grid=np.ones((70,70),dtype=bool);grid[20:50,30:40]=False
    path=search_path(grid,(10,35),(60,35),'A*')
    smoothed=smooth_corners(path,grid,3)
    reduced=adaptive_reduce_path(smoothed,grid,max_spacing_px=10)
    assert validate_route(reduced,grid,start=(10,35),goal=(60,35))

def test_unknown_blocked():
    grid=np.ones((30,30),dtype=bool);grid[:,15]=False
    assert search_path(grid,(3,15),(25,15),'A*') is None

def test_inflation_footprint():
    grid=np.ones((40,40),dtype=bool);grid[20,20]=False
    safe=inflate_free_grid(grid,5)
    assert not safe[20,25] and safe[20,26]

def test_mecanum_roundtrip():
    v=(.2,.1,-.5)
    assert np.allclose(mecanum_body_velocity(mecanum_wheel_rates(*v,.04,.235),.04,.235),v)
