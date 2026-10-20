"""Offline, fail-closed tests for arming and dispatching Nav2 paths.

These tests never instantiate ROS, connect to hardware, or publish commands.
"""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from agv_planner.gui.app import PathPlannerGUI


def fake(**overrides):
    values = dict(
        nav2_pending=False, nav2_goal_handle=None, nav2_segments=[],
        nav2_execution_enabled=Mock(get=Mock(return_value=True)),
        auto_start=True, robot_pose=(0.0, 0.0, 0.0),
        nav2_client=Mock(server_is_ready=Mock(return_value=True)),
        _nav2_pose_fresh=Mock(return_value=True),
        build_nav2_path=Mock(), _build_nav2_segments=Mock(),
        nav2_run_id=4, nav2_cancel_requested=True,
        nav2_segment_index=8, _send_nav2_leg=Mock(),
    )
    values.update(overrides)
    g = SimpleNamespace(**values)
    path = SimpleNamespace(poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))])
    g.build_nav2_path.return_value = path
    g._build_nav2_segments.return_value = [path]
    return g


@pytest.mark.parametrize('overrides,expected', [
    ({'nav2_pending': True}, 'already active'),
    ({'nav2_goal_handle': object()}, 'already active'),
    ({'nav2_segments': [object()]}, 'already active'),
    ({'nav2_execution_enabled': Mock(get=Mock(return_value=False))}, 'ARM REAL ROBOT'),
    ({'auto_start': False}, 'AUTO START'),
    ({'_nav2_pose_fresh': Mock(return_value=False)}, 'missing or stale'),
    ({'nav2_client': None}, 'not available'),
    ({'nav2_client': Mock(server_is_ready=Mock(return_value=False))}, 'not available'),
])
def test_dispatch_guards_fail_closed(overrides, expected):
    g = fake(**overrides)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.send_nav2_path(g)
    warning.assert_called_once()
    assert expected in warning.call_args.args[1]
    g.build_nav2_path.assert_not_called()
    g._send_nav2_leg.assert_not_called()
    assert g.nav2_run_id == 4


@pytest.mark.parametrize('stage', ['build', 'segment'])
def test_dispatch_rejects_invalid_route(stage):
    g = fake()
    getattr(g, 'build_nav2_path' if stage == 'build' else '_build_nav2_segments').side_effect = ValueError('unsafe route')
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.send_nav2_path(g)
    assert 'unsafe route' in warning.call_args.args[1]
    g._send_nav2_leg.assert_not_called()


@pytest.mark.parametrize('start_x', [-0.151, 0.151, 0.5])
def test_dispatch_rejects_start_too_far_from_live_pose(start_x):
    g = fake()
    g.build_nav2_path.return_value.poses[0].pose.position.x = start_x
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.send_nav2_path(g)
    assert 'over 15 cm' in warning.call_args.args[1]
    g._send_nav2_leg.assert_not_called()


@pytest.mark.parametrize('confirmed', [False, True])
def test_dispatch_requires_confirmation_and_only_then_starts(confirmed):
    g = fake()
    with patch('agv_planner.gui.app.messagebox.askyesno', return_value=confirmed) as confirm:
        PathPlannerGUI.send_nav2_path(g)
    confirm.assert_called_once()
    assert 'REAL ROBOT MOVEMENT' in confirm.call_args.args[0]
    if confirmed:
        assert g.nav2_run_id == 5
        assert g.nav2_cancel_requested is False
        assert g.nav2_segment_index == 0
        assert len(g.nav2_segments) == 1
        g._send_nav2_leg.assert_called_once_with(5)
    else:
        assert g.nav2_run_id == 4
        assert g.nav2_segments == []
        g._send_nav2_leg.assert_not_called()


def test_preview_without_publisher_warns_and_never_builds_path():
    g = fake(nav2_path_publisher=None)
    g.nav2_status = Mock()
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.publish_nav2_preview(g)
    assert 'unavailable' in warning.call_args.args[1]
    g.build_nav2_path.assert_not_called()


def test_preview_publish_failure_warns_without_propagating():
    g = fake(nav2_path_publisher=Mock(publish=Mock(side_effect=RuntimeError('transport unavailable'))))
    g.nav2_status = Mock()
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.publish_nav2_preview(g)
    assert 'transport unavailable' in warning.call_args.args[1]


def test_preview_success_is_labeled_no_motion():
    g = fake(nav2_path_publisher=Mock(), nav2_status=Mock())
    PathPlannerGUI.publish_nav2_preview(g)
    g.nav2_path_publisher.publish.assert_called_once_with(g.build_nav2_path.return_value)
    assert 'NO MOTION' in g.nav2_status.set.call_args.args[0]
