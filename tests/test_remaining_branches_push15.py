"""Coverage of entrypoints and failure boundaries without moving a real robot."""
import runpy
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
import pytest
from agv_planner.gui.app import PathPlannerGUI
from agv_planner.planning.algorithms import simplify_path, search_path, optimized_search, smooth_corners, adaptive_reduce_path
from agv_planner.config.settings import load_settings


def test_entrypoint_module_invokes_main():
    with patch('agv_planner.planner.main') as main:
        runpy.run_module('agv_planner.__main__', run_name='__main__')
    main.assert_called_once_with()


def test_legacy_export_modules_importable():
    import agv_planner.planner as planner
    import agv_planner.maps.occupancy as occupancy
    assert callable(planner.main)
    assert callable(occupancy.inflate_free_grid)


def test_gui_main_builds_window_and_enters_loop():
    from agv_planner.gui import app
    root=Mock()
    with patch.object(app.tk,'Tk',return_value=root), patch.object(app,'PathPlannerGUI') as gui:
        app.main()
    gui.assert_called_once_with(root)
    root.mainloop.assert_called_once_with()


def test_settings_rejects_nonpositive_value(tmp_path):
    cfg=tmp_path/'bad.yaml'
    cfg.write_text('robot:\n  wheel_radius_m: 0\n')
    with pytest.raises(ValueError,match='positive'):
        load_settings(cfg)


def test_simplify_path_short_and_blocked():
    grid=np.ones((7,7),dtype=bool)
    assert simplify_path(grid,[(0,0),(1,0)])==[(0,0),(1,0)]
    grid[1,2]=False
    route=[(0,0),(1,0),(1,1),(1,2),(1,3),(2,3),(3,3)]
    assert simplify_path(grid,route)[0]==route[0]


def test_non_astar_rejects_blocked_endpoints():
    grid=np.ones((3,3),dtype=bool)
    grid[0,0]=False
    for algorithm in ('Dijkstra','Theta*'):
        assert search_path(grid,(0,0),(2,2),algorithm=algorithm) is None


def test_optimized_search_returns_none_for_unreachable():
    grid=np.zeros((5,5),dtype=bool)
    grid[0,0]=grid[4,4]=True
    assert optimized_search(grid,(0,0),(4,4),"A*",np.ones_like(grid,dtype=float),1.0) is None


def test_smoothing_preserves_required_corner_and_repeated_point():
    grid=np.ones((20,20),dtype=bool)
    path=[(1,1),(5,1),(5,5)]
    assert (5,1) in smooth_corners(path,grid,2,required=[(5,1)])
    assert (5,1) in smooth_corners([(1,1),(5,1),(5,1),(5,5)],grid,2)


def test_adaptive_reduce_handles_identical_endpoints():
    gridpath=[(0,0),(2,3),(0,0)]
    result=adaptive_reduce_path(gridpath,np.ones((5,5),dtype=bool),tolerance_px=0.1,max_spacing_px=100)
    assert result[0]==(0,0) and result[-1]==(0,0)


def test_nav2_send_goal_success_and_transport_failure():
    path=NS(poses=[NS(pose=NS(position=NS(x=0.,y=0.)))])
    future=Mock()
    client=Mock()
    client.server_is_ready.return_value=True
    client.send_goal_async.return_value=future
    g=NS(nav2_run_id=4,nav2_cancel_requested=False,_nav2_pose_fresh=Mock(return_value=True),
         nav2_client=client,nav2_segments=[path],nav2_segment_index=0,robot_pose=(0.,0.,0.),
         nav2_pending=False,nav2_status=Mock(),root=Mock(),_nav2_goal_response=Mock(),
         _finish_nav2_route=Mock())
    with patch('agv_planner.gui.app.FollowPath',NS(Goal=NS),create=True):
        PathPlannerGUI._send_nav2_leg(g,4)
        assert g.nav2_pending is True
        assert future.add_done_callback.called
        client.send_goal_async.side_effect=RuntimeError('transport down')
        PathPlannerGUI._send_nav2_leg(g,4)
    assert 'send failed' in g._finish_nav2_route.call_args.args[0]


def test_stop_simulation_handles_cancel_error():
    g=NS(sim_after_id='old',root=Mock(),sim_state='running',sim_pose=(1,2),sim_route=[1],
         sim_index=1,sim_velocity=1.,wheel_rates=[1]*4,mega_commands=(1,1,1,1),
         mega_diagnostics=Mock(),wheel_display=Mock(),wheel_indicators={},proximity_display=Mock(),sim_telemetry=Mock(),sim_status=Mock(),redraw=Mock())
    import tkinter as tk
    g.root.after_cancel.side_effect=tk.TclError('gone')
    PathPlannerGUI.stop_simulation(g,redraw=False)
    assert g.sim_state=='stopped' and g.sim_after_id is None


def test_export_nav2_without_path_warns():
    g=NS(path=None,resolution=None)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warn:
        PathPlannerGUI.export_nav2_path(g)
    warn.assert_called_once()


def test_export_path_without_path_warns():
    g=NS(path=None,resolution=None,validate_current_route=Mock())
    with patch('agv_planner.gui.app.messagebox.showwarning') as warn:
        PathPlannerGUI.export_path(g)
    warn.assert_called_once()
