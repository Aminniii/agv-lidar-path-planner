"""ROS Path construction regressions, using inert message doubles only."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from agv_planner.gui.app import PathPlannerGUI


def gui(**kw):
    base=dict(path=[(0,0),(2,0),(4,0)], resolution=0.1, dynamic_obstacles=[],
              ros_node=None, robot_pose=(0.,0.,math.pi/2),
              _nav2_pose_fresh=Mock(return_value=False),
              pixel_to_world=Mock(side_effect=lambda x,y:(x*.1,y*.1)),
              validate_current_route=Mock(), visit_order=[],
              nav2_path_publisher=Mock(),nav2_status=Mock(),
              nav2_segments=[],nav2_pending=False,nav2_goal_handle=None,
              stop_simulation=Mock(),root=Mock())
    base.update(kw)
    return NS(**base)


def fake_path():
    return NS(header=NS(frame_id='',stamp=None),poses=[])


def fake_pose():
    return NS(header=None,pose=NS(position=NS(x=None,y=None),orientation=NS(z=None,w=None)))

@pytest.mark.parametrize('path,resolution,obstacles,ros,expected',[
    ([],.1,[],True,'Plan a path'),
    ([(0,0)],None,[],True,'Plan a path'),
    ([(0,0)],.1,[(1,1)],True,'Virtual obstacles'),
    ([(0,0)],.1,[],False,'ROS 2 is unavailable'),
])
def test_build_path_rejects_unusable_input(path,resolution,obstacles,ros,expected):
    g=gui(path=path,resolution=resolution,dynamic_obstacles=obstacles)
    with patch('agv_planner.gui.app.ROS_AVAILABLE',ros):
        with pytest.raises((ValueError,RuntimeError),match=expected):
            PathPlannerGUI.build_nav2_path(g)
    g.validate_current_route.assert_called_once_with(for_robot=True)

@pytest.mark.parametrize('fresh,yaw',[(False,0.),(True,math.pi/2)])
def test_build_path_uses_constant_heading_and_world_coordinates(fresh,yaw):
    g=gui(_nav2_pose_fresh=Mock(return_value=fresh))
    with patch('agv_planner.gui.app.ROS_AVAILABLE',True),\
         patch('agv_planner.gui.app.RosPath',side_effect=fake_path,create=True),\
         patch('agv_planner.gui.app.PoseStamped',side_effect=fake_pose,create=True):
        result=PathPlannerGUI.build_nav2_path(g)
    assert result.header.frame_id=='map'
    assert len(result.poses)==3
    assert [(p.pose.position.x,p.pose.position.y) for p in result.poses]==[(0.,0.),(.2,0.),(.4,0.)]
    assert all(p.header is result.header for p in result.poses)
    assert all(p.pose.orientation.z==pytest.approx(math.sin(yaw/2)) for p in result.poses)
    assert all(p.pose.orientation.w==pytest.approx(math.cos(yaw/2)) for p in result.poses)


def test_build_path_stamps_header_when_ros_node_available():
    stamp=object()
    node=Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value=stamp
    g=gui(ros_node=node)
    with patch('agv_planner.gui.app.ROS_AVAILABLE',True),\
         patch('agv_planner.gui.app.RosPath',side_effect=fake_path,create=True),\
         patch('agv_planner.gui.app.PoseStamped',side_effect=fake_pose,create=True):
        result=PathPlannerGUI.build_nav2_path(g)
    assert result.header.stamp is stamp


def test_segments_split_at_mandatory_stops_and_share_boundary():
    g=gui(visit_order=[(2,0)])
    full=fake_path()
    full.poses=[object() for _ in range(3)]
    with patch('agv_planner.gui.app.RosPath',side_effect=fake_path,create=True):
        result=PathPlannerGUI._build_nav2_segments(g,full)
    assert len(result)==2
    assert result[0].poses==full.poses[:2]
    assert result[1].poses==full.poses[1:]
    assert all(segment.header is full.header for segment in result)


def test_segments_reject_missing_required_stop():
    g=gui(visit_order=[(100,100)])
    full=fake_path()
    full.poses=[object() for _ in range(3)]
    with pytest.raises(ValueError,match='required stop is missing'):
        PathPlannerGUI._build_nav2_segments(g,full)


def test_close_window_destroys_ros_node_without_ros_shutdown_when_offline():
    node=Mock()
    g=gui(ros_node=node)
    with patch('agv_planner.gui.app.ROS_AVAILABLE',False):
        PathPlannerGUI.close_window(g)
    g.stop_simulation.assert_called_once_with(redraw=False)
    node.destroy_node.assert_called_once()
    g.root.destroy.assert_called_once()


def test_close_window_shutdown_errors_are_swallowed():
    g=gui(ros_node=Mock())
    with patch('agv_planner.gui.app.ROS_AVAILABLE',True),\
         patch('agv_planner.gui.app.rclpy') as ros:
        ros.ok.return_value=True
        ros.shutdown.side_effect=RuntimeError('shutdown failed')
        PathPlannerGUI.close_window(g)
    g.root.destroy.assert_called_once()


def test_close_window_rejects_active_route():
    g=gui(nav2_pending=True)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.close_window(g)
    warning.assert_called_once()
    g.stop_simulation.assert_not_called()
    g.root.destroy.assert_not_called()
