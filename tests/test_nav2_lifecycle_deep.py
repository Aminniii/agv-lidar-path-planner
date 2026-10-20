"""Headless Nav2 lifecycle regression checks; no ROS or motor connections."""
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import pytest
from agv_planner.gui.app import PathPlannerGUI


def gui(**changes):
    state = dict(nav2_run_id=7, nav2_cancel_requested=False,
                 nav2_pending=False, nav2_goal_handle=None,
                 nav2_segments=[Mock()], nav2_segment_index=0,
                 nav2_dwell_job=None, nav2_client=Mock(),
                 nav2_status=Mock(), nav2_execution_enabled=Mock(),
                 robot_pose=(0., 0., 0.), root=Mock(),
                 _nav2_pose_fresh=Mock(return_value=True),
                 _finish_nav2_route=Mock(), _send_nav2_leg=Mock(), _nav2_dwell_complete=Mock(), _nav2_goal_response=Mock(), _nav2_result=Mock())
    state.update(changes)
    return S(**state)

@pytest.mark.parametrize('run_id,cancelled', [(6,False),(8,False),(7,True)])
def test_send_leg_does_not_dispatch_outdated_or_cancelled(run_id,cancelled):
    g=gui(nav2_cancel_requested=cancelled)
    PathPlannerGUI._send_nav2_leg(g,run_id)
    g.nav2_client.send_goal_async.assert_not_called()

@pytest.mark.parametrize('fresh,ready,expected', [
    (False,True,'localization stale'), (True,False,'server unavailable')])
def test_send_leg_safety_gates(fresh,ready,expected):
    g=gui()
    g._nav2_pose_fresh.return_value=fresh
    g.nav2_client.server_is_ready.return_value=ready
    PathPlannerGUI._send_nav2_leg(g,7)
    assert expected in g._finish_nav2_route.call_args.args[0]

@pytest.mark.parametrize('offset', [0.251, 1.0, 10.0])
def test_send_leg_rejects_distant_start(offset):
    leg=Mock(); leg.poses=[S(pose=S(position=S(x=offset,y=0.))) ]
    g=gui(nav2_segments=[leg])
    PathPlannerGUI._send_nav2_leg(g,7)
    assert 'too far' in g._finish_nav2_route.call_args.args[0]
    g.nav2_client.send_goal_async.assert_not_called()

@pytest.mark.parametrize('run_id', [6,8])
def test_goal_response_ignores_stale_callback(run_id):
    g=gui()
    PathPlannerGUI._nav2_goal_response(g,Mock(),run_id)
    assert g.nav2_goal_handle is None

@pytest.mark.parametrize('handle', [None, S(accepted=False)])
def test_goal_response_rejected(handle):
    g=gui()
    f=Mock();f.result.return_value=handle
    PathPlannerGUI._nav2_goal_response(g,f,7)
    assert 'rejected' in g._finish_nav2_route.call_args.args[0]
    assert not g.nav2_pending

@pytest.mark.parametrize('cancel', [False,True])
def test_goal_response_accepts_and_registers_result(cancel):
    handle=Mock(accepted=True)
    g=gui(nav2_cancel_requested=cancel)
    f=Mock(); f.result.return_value=handle
    PathPlannerGUI._nav2_goal_response(g,f,7)
    assert g.nav2_goal_handle is handle
    handle.get_result_async.return_value.add_done_callback.assert_called_once()
    if cancel:
        handle.cancel_goal_async.assert_called_once()
    else:
        handle.cancel_goal_async.assert_not_called()

@pytest.mark.parametrize('message', ['transport down','server died'])
def test_goal_response_exception_fails_closed(message):
    g=gui();f=Mock();f.result.side_effect=RuntimeError(message)
    PathPlannerGUI._nav2_goal_response(g,f,7)
    assert message in g._finish_nav2_route.call_args.args[0]

@pytest.mark.parametrize('run_id', [6,8])
def test_result_ignores_stale_callback(run_id):
    g=gui(nav2_pending=True)
    PathPlannerGUI._nav2_result(g,Mock(),run_id)
    assert g.nav2_pending

@pytest.mark.parametrize('cancel', [False,True])
def test_result_cancelled_or_failed(cancel):
    g=gui(nav2_cancel_requested=cancel)
    f=Mock();f.result.return_value=S(status=4)
    with patch('agv_planner.gui.app.GoalStatus',S(STATUS_SUCCEEDED=0),create=True):
        PathPlannerGUI._nav2_result(g,f,7)
    assert ('cancelled' if cancel else 'failed') in g._finish_nav2_route.call_args.args[0]

@pytest.mark.parametrize('fresh', [False,True])
def test_result_checks_localization_and_endpoint(fresh):
    g=gui();g._nav2_pose_fresh.return_value=fresh
    leg=Mock();leg.poses=[S(pose=S(position=S(x=0.,y=0.))), S(pose=S(position=S(x=.5,y=0.)))]
    g.nav2_segments=[leg]
    f=Mock();f.result.return_value=S(status=0)
    with patch('agv_planner.gui.app.GoalStatus',S(STATUS_SUCCEEDED=0),create=True):
        PathPlannerGUI._nav2_result(g,f,7)
    assert ('target not reached' if fresh else 'localization stale') in g._finish_nav2_route.call_args.args[0]

@pytest.mark.parametrize('has_next', [False,True])
def test_result_success_finishes_or_schedules_dwell(has_next):
    leg=Mock();leg.poses=[S(pose=S(position=S(x=0.,y=0.)))]
    g=gui(nav2_segments=[leg,leg] if has_next else [leg])
    f=Mock();f.result.return_value=S(status=0)
    with patch('agv_planner.gui.app.GoalStatus',S(STATUS_SUCCEEDED=0),create=True):
        PathPlannerGUI._nav2_result(g,f,7)
    assert g.nav2_segment_index==1
    if has_next:
        g.root.after.assert_called_once()
    else:
        assert 'completed' in g._finish_nav2_route.call_args.args[0]

@pytest.mark.parametrize('active', [False,True])
def test_cancel_path_idle_or_pending(active):
    g=gui(nav2_segments=[Mock()] if active else [],nav2_pending=active)
    PathPlannerGUI.cancel_nav2_path(g)
    assert g.nav2_cancel_requested == active
    assert ('queued' if active else 'no active') in g.nav2_status.set.call_args.args[0]

@pytest.mark.parametrize('with_handle', [False,True])
def test_cancel_dwell_or_active_goal(with_handle):
    handle=Mock() if with_handle else None
    g=gui(nav2_goal_handle=handle,nav2_dwell_job=None if with_handle else 'job')
    PathPlannerGUI.cancel_nav2_path(g)
    if with_handle:
        handle.cancel_goal_async.assert_called_once()
    else:
        g.root.after_cancel.assert_called_once_with('job')
        assert 'during dwell' in g._finish_nav2_route.call_args.args[0]

@pytest.mark.parametrize('run_id,cancelled',[(7,False),(6,False),(7,True)])
def test_dwell_callback_only_continues_current_route(run_id,cancelled):
    g=gui(nav2_dwell_job='job',nav2_cancel_requested=cancelled)
    PathPlannerGUI._nav2_dwell_complete(g,run_id)
    assert g.nav2_dwell_job is None
    assert g._send_nav2_leg.called == (run_id==7 and not cancelled)

@pytest.mark.parametrize('dwell', [None,'job'])
def test_finish_nav2_resets_every_execution_flag(dwell):
    g=gui(nav2_dwell_job=dwell,nav2_pending=True,nav2_cancel_requested=True,
          nav2_goal_handle=Mock(),nav2_segment_index=2)
    PathPlannerGUI._finish_nav2_route(g,'stopped safely')
    assert g.nav2_segments==[] and g.nav2_segment_index==0
    assert not g.nav2_pending and not g.nav2_cancel_requested
    assert g.nav2_goal_handle is None and g.nav2_dwell_job is None
    g.nav2_execution_enabled.set.assert_called_once_with(False)
    g.nav2_status.set.assert_called_once_with('stopped safely')
    if dwell: g.root.after_cancel.assert_called_once_with(dwell)
