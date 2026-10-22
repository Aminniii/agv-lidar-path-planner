import numpy as np

from agv_planner.planning.proximity import (
    footprint_is_free,
    nearest_obstacle_distance,
    obstacle_distance_field,
    swept_footprint_is_free,
)


def test_empty_room():
    g=np.ones((80,80),dtype=bool)
    assert footprint_is_free(g,40,40,0,.34,.30,.05)

def test_wall_collision():
    g=np.ones((80,80),dtype=bool);g[:,45]=False
    assert not footprint_is_free(g,43,40,0,.34,.30,.05)

def test_rotation_sweep():
    g=np.ones((80,80),dtype=bool);g[42,44]=False
    assert not swept_footprint_is_free(g,(40,40,0),(40,40,1.57),.5,.15,.05)

def test_map_edge():
    assert not footprint_is_free(np.ones((20,20),dtype=bool),1,1,0,.4,.3,.05)

def test_distance():
    g=np.ones((30,30),dtype=bool);g[15,15]=False
    f=obstacle_distance_field(g,.1)
    assert nearest_obstacle_distance(f,15,15)==0
    assert nearest_obstacle_distance(f,16,15) == .1
