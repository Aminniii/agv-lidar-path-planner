"""Offline regression checks for ROS dependency isolation and live-start safety."""
import builtins
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from agv_planner.gui.app import PathPlannerGUI
from agv_planner.ros.support import load_ros_dependencies


def fake_gui(**changes):
    values = dict(auto_start=True, robot_pose=(0.5, 0.5, 0.0),
                  inflated_grid=np.ones((12, 12), dtype=bool),
                  world_to_pixel=Mock(return_value=(5, 5)),
                  status_var=Mock(), start=(1, 1), path=[(1, 1)],
                  visit_order=[(1, 1)])
    values.update(changes)
    return SimpleNamespace(**values)


@pytest.mark.parametrize('auto,pose,grid', [
    (False, (0.5, 0.5, 0), np.ones((12, 12), dtype=bool)),
    (True, None, np.ones((12, 12), dtype=bool)),
    (True, (0.5, 0.5, 0), None),
])
def test_live_start_requires_all_prerequisites(auto, pose, grid):
    g = fake_gui(auto_start=auto, robot_pose=pose, inflated_grid=grid)
    assert PathPlannerGUI.sync_start_to_robot(g) is False
    g.world_to_pixel.assert_not_called()


def test_live_start_rejects_pose_outside_map():
    g = fake_gui(world_to_pixel=Mock(return_value=None))
    assert PathPlannerGUI.sync_start_to_robot(g) is False
    assert 'outside saved map' in g.status_var.set.call_args.args[0]
    assert g.start == (1, 1)


def test_live_start_rejects_blocked_location():
    grid = np.ones((12, 12), dtype=bool)
    grid[5, 5] = False
    g = fake_gui(inflated_grid=grid)
    assert PathPlannerGUI.sync_start_to_robot(g) is False
    assert 'blocked' in g.status_var.set.call_args.args[0]
    assert g.start == (1, 1)


def test_live_start_updates_and_invalidates_old_route():
    g = fake_gui()
    assert PathPlannerGUI.sync_start_to_robot(g) is True
    assert g.start == (5, 5)
    assert g.path is None
    assert g.visit_order == []


def test_live_start_same_cell_keeps_route():
    old_path = [(5, 5), (6, 6)]
    g = fake_gui(start=(5, 5), path=old_path)
    assert PathPlannerGUI.sync_start_to_robot(g) is False
    assert g.path is old_path


def test_ros_missing_dependency_returns_safe_offline_result():
    original_import = builtins.__import__
    def blocked_import(name, *args, **kwargs):
        if name == 'rclpy' or name.startswith('rclpy.'):
            raise ImportError('ROS deliberately unavailable')
        return original_import(name, *args, **kwargs)
    with patch('builtins.__import__', side_effect=blocked_import):
        available, reason, dependencies = load_ros_dependencies()
    assert available is False
    assert 'ROS deliberately unavailable' in reason
    assert dependencies is None


def test_ros_dependency_loader_success_with_stub_modules():
    """Exercise optional import boundary without importing ROS or commanding motors."""
    modules = {}
    def package(name):
        m = ModuleType(name)
        m.__path__ = []
        modules[name] = m
        return m
    def module(name, **attributes):
        m = ModuleType(name)
        m.__dict__.update(attributes)
        modules[name] = m
        return m
    package('rclpy')
    module('rclpy.node', Node=type('Node', (), {}))
    module('rclpy.action', ActionClient=type('ActionClient', (), {}))
    package('nav2_msgs')
    module('nav2_msgs.action', FollowPath=type('FollowPath', (), {}))
    package('action_msgs')
    module('action_msgs.msg', GoalStatus=type('GoalStatus', (), {}))
    module('tf2_ros', Buffer=type('Buffer', (), {}), TransformListener=type('TransformListener', (), {}), TransformException=type('TransformException', (Exception,), {}))
    package('nav_msgs')
    module('nav_msgs.msg', Path=type('Path', (), {}))
    package('geometry_msgs')
    module('geometry_msgs.msg', PoseStamped=type('PoseStamped', (), {}))
    module('rclpy.qos', QoSProfile=type('QoSProfile', (), {}), DurabilityPolicy=type('DurabilityPolicy', (), {}), ReliabilityPolicy=type('ReliabilityPolicy', (), {}))
    with patch.dict(sys.modules, modules):
        available, reason, dependencies = load_ros_dependencies()
    assert available is True
    assert reason is None
    assert len(dependencies) == 13
    assert dependencies[0] is modules['rclpy']

@pytest.mark.parametrize('selection', ['', None])
def test_choose_map_cancel_is_noop(selection):
    g = SimpleNamespace(load_map=Mock())
    with patch('agv_planner.gui.app.filedialog.askopenfilename', return_value=selection):
        PathPlannerGUI.choose_map(g)
    g.load_map.assert_not_called()


def test_choose_map_selected_yaml_is_passed_to_loader():
    from pathlib import Path
    g = SimpleNamespace(load_map=Mock())
    with patch('agv_planner.gui.app.filedialog.askopenfilename', return_value='/tmp/map.yaml'):
        PathPlannerGUI.choose_map(g)
    g.load_map.assert_called_once_with(Path('/tmp/map.yaml'))


def test_choose_map_reports_loader_failure_without_crashing():
    g = SimpleNamespace(load_map=Mock(side_effect=ValueError('invalid map metadata')))
    with patch('agv_planner.gui.app.filedialog.askopenfilename', return_value='/tmp/bad.yaml'), \
         patch('agv_planner.gui.app.messagebox.showerror') as popup:
        PathPlannerGUI.choose_map(g)
    popup.assert_called_once_with('Could not load map', 'invalid map metadata')


def poll_gui(**changes):
    values = dict(ros_node=None, root=Mock(), last_tf_time=None,
                  local_status=Mock(), pose_var=Mock(), poll_robot_pose=Mock())
    values.update(changes)
    return SimpleNamespace(**values)


def test_offline_pose_poll_reschedules_without_touching_ros():
    g = poll_gui()
    PathPlannerGUI.poll_robot_pose(g)
    g.root.after.assert_called_once_with(200, g.poll_robot_pose)
    g.local_status.set.assert_not_called()


def test_pose_poll_handles_ros_transport_exception():
    g = poll_gui(ros_node=object())
    with patch('agv_planner.gui.app.rclpy', SimpleNamespace(spin_once=Mock(side_effect=RuntimeError('transport unavailable')))):
        PathPlannerGUI.poll_robot_pose(g)
    g.local_status.set.assert_called_with('Localization: ROS error')
    assert 'transport unavailable' in g.pose_var.set.call_args.args[0]
    g.root.after.assert_called_once()


def test_pose_poll_marks_old_localization_stale():
    g = poll_gui(last_tf_time=1.0)
    with patch('agv_planner.gui.app.time.monotonic', return_value=1000.0):
        PathPlannerGUI.poll_robot_pose(g)
    assert 'STALE' in g.local_status.set.call_args.args[0]
    g.root.after.assert_called_once()
