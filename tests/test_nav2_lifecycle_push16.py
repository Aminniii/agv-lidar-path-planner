"""Headless Nav2 action lifecycle regressions: no ROS transport or motors used."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from agv_planner.gui.app import PathPlannerGUI


def fake_gui():
    g = NS(nav2_run_id=9, nav2_pending=True, nav2_cancel_requested=False,
           nav2_goal_handle=None, nav2_segment_index=0,
           nav2_segments=[NS(poses=[NS(pose=NS(position=NS(x=1.0,y=2.0)))])],
           robot_pose=(1.0,2.0,0.0), nav2_status=Mock(), root=Mock(),
           nav2_dwell_job=None, nav2_execution_enabled=Mock(),
           _nav2_pose_fresh=Mock(return_value=True),
           _finish_nav2_route=Mock(), _send_nav2_leg=Mock(), _nav2_dwell_complete=Mock())
    return g


@pytest.mark.parametrize('accepted', [False, True])
def test_goal_response_acceptance(accepted):
    g=fake_gui()
    handle=Mock(accepted=accepted)
    PathPlannerGUI._nav2_goal_response(g, Mock(result=Mock(return_value=handle)),9)
    if accepted:
        assert g.nav2_goal_handle is handle
        handle.get_result_async.return_value.add_done_callback.assert_called_once()
    else:
        assert 'rejected' in g._finish_nav2_route.call_args.args[0]


def test_goal_response_late_and_exception():
    g=fake_gui()
    f=Mock()
    PathPlannerGUI._nav2_goal_response(g,f,8)
    f.result.assert_not_called()
    f.result.side_effect=RuntimeError('action disconnected')
    PathPlannerGUI._nav2_goal_response(g,f,9)
    assert 'goal error' in g._finish_nav2_route.call_args.args[0]


def test_goal_response_cancellation_race():
    g=fake_gui()
    g.nav2_cancel_requested=True
    handle=Mock(accepted=True)
    PathPlannerGUI._nav2_goal_response(g,Mock(result=Mock(return_value=handle)),9)
    handle.cancel_goal_async.assert_called_once()


@pytest.mark.parametrize('scenario,expected', [
    ('cancel','cancelled'), ('failed','failed'), ('stale','stale'),
    ('far','target not reached'), ('complete','completed successfully'),
    ('exception','result error')])
def test_result_safety_gate(scenario,expected):
    g=fake_gui()
    g.nav2_goal_handle=Mock()
    result=NS(status=4)
    f=Mock(result=Mock(return_value=result))
    if scenario=='cancel': g.nav2_cancel_requested=True
    if scenario=='failed': result.status=6
    if scenario=='stale': g._nav2_pose_fresh.return_value=False
    if scenario=='far': g.robot_pose=(100.,100.,0.)
    if scenario=='exception': f.result.side_effect=RuntimeError('broken')
    with patch('agv_planner.gui.app.GoalStatus',NS(STATUS_SUCCEEDED=4),create=True):
        PathPlannerGUI._nav2_result(g,f,9)
    assert expected in g._finish_nav2_route.call_args.args[0]


def test_result_next_leg_waits_at_stop():
    g=fake_gui()
    g.nav2_segments.append(NS(poses=[NS(pose=NS(position=NS(x=2.,y=2.)))]))
    with patch('agv_planner.gui.app.GoalStatus',NS(STATUS_SUCCEEDED=4),create=True):
        PathPlannerGUI._nav2_result(g,Mock(result=Mock(return_value=NS(status=4))),9)
    assert g.nav2_segment_index==1
    g.root.after.assert_called_once()


def test_result_ignores_old_run():
    g=fake_gui()
    f=Mock()
    PathPlannerGUI._nav2_result(g,f,7)
    f.result.assert_not_called()


def test_cancel_nav2_no_active_route():
    g=fake_gui(); g.nav2_segments=[]; g.nav2_pending=False
    PathPlannerGUI.cancel_nav2_path(g)
    assert 'no active route' in g.nav2_status.set.call_args.args[0]


@pytest.mark.parametrize('mode', ['dwell','active','pending'])
def test_cancel_nav2_modes(mode):
    g=fake_gui()
    if mode=='dwell': g.nav2_dwell_job='job'
    if mode=='active': g.nav2_goal_handle=Mock()
    PathPlannerGUI.cancel_nav2_path(g)
    assert g.nav2_cancel_requested
    if mode=='dwell':
        g.root.after_cancel.assert_called_once_with('job')
        assert 'during dwell' in g._finish_nav2_route.call_args.args[0]
    elif mode=='active': g.nav2_goal_handle.cancel_goal_async.assert_called_once()
    else: assert 'queued' in g.nav2_status.set.call_args.args[0]


def test_dwell_only_dispatches_current_uncancelled_run():
    g=fake_gui()
    PathPlannerGUI._nav2_dwell_complete(g,8)
    g._send_nav2_leg.assert_not_called()
    g.nav2_cancel_requested=True
    PathPlannerGUI._nav2_dwell_complete(g,9)
    g._send_nav2_leg.assert_not_called()
    g.nav2_cancel_requested=False
    PathPlannerGUI._nav2_dwell_complete(g,9)
    g._send_nav2_leg.assert_called_once_with(9)


def test_finish_nav2_route_clears_all_execution_state():
    g=fake_gui(); g.nav2_dwell_job='timer'; g.nav2_goal_handle=Mock()
    PathPlannerGUI._finish_nav2_route(g,'done')
    assert not g.nav2_segments and not g.nav2_pending and not g.nav2_cancel_requested
    assert g.nav2_goal_handle is None and g.nav2_dwell_job is None
    g.root.after_cancel.assert_called_once_with('timer')
    g.nav2_execution_enabled.set.assert_called_once_with(False)
