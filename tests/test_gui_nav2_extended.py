"""Headless checks for ROS execution guards and route segmentation."""
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from agv_planner.gui.app import PathPlannerGUI
from agv_planner.planning.safety import UnsafePathError


def gui(**kwargs):
    data=dict(path=[(0,0),(3,0),(6,0),(9,0),(12,0)],
              start=(0,0),goal=(12,0),stops=[],visit_order=[],
              inflated_grid=np.ones((16,16),dtype=bool),
              clearance_var=Mock(),robot_pose=None,last_tf_time=None,
              dynamic_obstacles=[],resolution=.1,
              validate_current_route=Mock(return_value=True),
              ros_node=None)
    data.update(kwargs)
    return SimpleNamespace(**data)

@pytest.mark.parametrize('kwargs', [dict(path=None),dict(inflated_grid=None)])
def test_export_without_route_is_rejected(kwargs):
    with pytest.raises(UnsafePathError):
        PathPlannerGUI.validate_current_route(gui(**kwargs))

def test_valid_offline_route_passes():
    assert PathPlannerGUI.validate_current_route(gui()) is True

def test_robot_export_requires_footprint_clearance():
    x=gui()
    x.clearance_var.get.return_value=0.01
    with pytest.raises((ValueError, UnsafePathError)):
        PathPlannerGUI.validate_current_route(x,for_robot=True)

def test_route_rejects_blocked_cell():
    grid=np.ones((16,16),dtype=bool)
    grid[0,2]=False
    with pytest.raises(UnsafePathError):
        PathPlannerGUI.validate_current_route(gui(inflated_grid=grid))

@pytest.mark.parametrize('pose,age,fresh', [(None,None,False),((1,2,0),None,False),((1,2,0),-1000000,False)])
def test_nav2_pose_missing_or_stale(pose,age,fresh):
    assert PathPlannerGUI._nav2_pose_fresh(gui(robot_pose=pose,last_tf_time=age)) is fresh

def test_nav2_pose_fresh():
    assert PathPlannerGUI._nav2_pose_fresh(gui(robot_pose=(1,2,0),last_tf_time=time.monotonic()))

def test_segments_without_stops():
    path=SimpleNamespace(header=object(),poses=list(range(5)))
    with patch("agv_planner.gui.app.RosPath", SimpleNamespace, create=True):
        result=PathPlannerGUI._build_nav2_segments(gui(),path)
    assert len(result)==1 and result[0].poses==list(range(5))

def test_segments_with_one_stop():
    path=SimpleNamespace(header=object(),poses=list(range(5)))
    with patch("agv_planner.gui.app.RosPath", SimpleNamespace, create=True):
        result=PathPlannerGUI._build_nav2_segments(gui(visit_order=[(6,0)]),path)
    assert [x.poses for x in result]==[[0,1,2],[2,3,4]]

def test_segments_with_two_stops():
    path=SimpleNamespace(header=object(),poses=list(range(5)))
    with patch("agv_planner.gui.app.RosPath", SimpleNamespace, create=True):
        result=PathPlannerGUI._build_nav2_segments(gui(visit_order=[(3,0),(9,0)]),path)
    assert [x.poses for x in result]==[[0,1],[1,2,3],[3,4]]

def test_missing_stop_rejected():
    path=SimpleNamespace(header=object(),poses=list(range(5)))
    with pytest.raises(ValueError,match='missing'):
        PathPlannerGUI._build_nav2_segments(gui(visit_order=[(9,9)]),path)
