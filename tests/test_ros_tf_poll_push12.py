"""Offline TF polling regressions; never starts ROS, Tk, or motors."""
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from agv_planner.gui.app import PathPlannerGUI


def gui(**overrides):
    translation = SimpleNamespace(x=1.0, y=2.0)
    rotation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    tf = SimpleNamespace(transform=SimpleNamespace(translation=translation, rotation=rotation))
    base = dict(ros_node=Mock(), tf_buffer=Mock(lookup_transform=Mock(return_value=tf)),
                robot_pose=None, last_tf_time=None, trail=[], local_status=Mock(),
                pose_var=Mock(), root=Mock(), redraw=Mock(), sync_start_to_robot=Mock(), poll_robot_pose=Mock())
    base.update(overrides)
    return SimpleNamespace(**base)


def poll(g, now=100.0):
    with patch('agv_planner.gui.app.rclpy', create=True) as ros, \
         patch('agv_planner.gui.app.time.monotonic', return_value=now):
        ros.time.Time.return_value = object()
        PathPlannerGUI.poll_robot_pose(g)
        return ros


def test_offline_poll_only_reschedules():
    g = gui(ros_node=None)
    ros = poll(g)
    ros.spin_once.assert_not_called()
    g.tf_buffer.lookup_transform.assert_not_called()
    g.root.after.assert_called_once_with(200, g.poll_robot_pose)


def test_tf_first_pose_updates_live_display_and_trail():
    g = gui()
    poll(g)
    assert g.robot_pose == (1.0, 2.0, 0.0)
    assert g.trail == [(1.0, 2.0)]
    assert g.last_tf_time == 100.0
    g.sync_start_to_robot.assert_called_once()
    g.redraw.assert_called_once()
    assert 'TF available' in g.local_status.set.call_args.args[0]


def test_tf_unchanged_pose_does_not_redraw():
    g = gui(robot_pose=(1.0, 2.0, 0.0), trail=[(1.0, 2.0)])
    poll(g)
    g.redraw.assert_not_called()
    g.sync_start_to_robot.assert_not_called()
    assert len(g.trail) == 1


@pytest.mark.parametrize('old_pose', [(0.0, 2.0, 0.0), (1.0, 2.0, 0.3)])
def test_tf_significant_translation_or_rotation_triggers_redraw(old_pose):
    g = gui(robot_pose=old_pose)
    poll(g)
    g.redraw.assert_called_once()


def test_tf_short_motion_does_not_extend_trail():
    g = gui(robot_pose=(1.0, 2.0, 0.0), trail=[(0.99, 2.0)])
    poll(g)
    assert len(g.trail) == 1


def test_tf_missing_sets_waiting_status():
    import agv_planner.gui.app as app
    g = gui()
    g.tf_buffer.lookup_transform.side_effect = app.TransformException('missing')
    poll(g)
    assert 'unavailable' in g.local_status.set.call_args.args[0]
    assert g.robot_pose is None


def test_spin_failure_does_not_crash_gui():
    g = gui()
    with patch('agv_planner.gui.app.rclpy', create=True) as ros, \
         patch('agv_planner.gui.app.time.monotonic', return_value=100.0):
        ros.spin_once.side_effect = RuntimeError('ROS disconnected')
        PathPlannerGUI.poll_robot_pose(g)
    assert 'ROS error' in g.local_status.set.call_args.args[0]
    assert 'ROS disconnected' in g.pose_var.set.call_args.args[0]


@pytest.mark.parametrize('elapsed,expected_stale', [(0.0, False), (100000.0, True)])
def test_tf_freshness_status(elapsed, expected_stale):
    g = gui(ros_node=None, last_tf_time=100.0-elapsed)
    poll(g)
    assert any('STALE' in c.args[0] for c in g.local_status.set.call_args_list) == expected_stale


def test_every_poll_schedules_next_iteration_even_after_error():
    g = gui()
    g.tf_buffer.lookup_transform.side_effect = RuntimeError('unexpected')
    poll(g)
    g.root.after.assert_called_once()
    assert g.root.after.call_args.args[0] == 200
