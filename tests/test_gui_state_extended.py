"""Headless GUI-state regressions: no Tk window or ROS hardware required."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import pytest
from agv_planner.gui.app import PathPlannerGUI


def state(**values):
    base = dict(original_map=object(), stops=[], path=[(1, 1), (2, 2)],
                visit_order=[(2, 2)], mode='start', auto_start=True,
                auto_button=Mock(), status_var=Mock(), redraw=Mock(),
                sim_state='stopped', sim_status=Mock(),
                sim_last_time=0.0, root=Mock(), sim_after_id=None,
                sim_pose=None, sim_route=[], sim_index=0, sim_velocity=0,
                wheel_rates=[0.0]*4, mega_commands=(0,)*4,
                mega_diagnostics=Mock(), wheel_display=Mock(),
                wheel_indicators={n:Mock() for n in ('FL','FR','RL','RR')},
                proximity_display=Mock(), sim_dwell_remaining=0.0,
                sim_telemetry=Mock(), trail=[(1,1)],
                free_grid=np.ones((10,10),dtype=bool), resolution=0.1,
                static_inflated_grid=None, inflated_grid=None,
                clearance_var=Mock(), stop_simulation=Mock(),
                refresh_dynamic_grid=Mock(), start=(1,1), goal=(8,8))
    base.update(values)
    return SimpleNamespace(**base)

@pytest.mark.parametrize('method,expected', [('set_start_mode','start'),('set_stop_mode','stop'),('set_goal_mode','goal')])
def test_mode_selection(method,expected):
    gui=state()
    getattr(PathPlannerGUI,method)(gui)
    assert gui.mode==expected

@pytest.mark.parametrize('method', ['set_start_mode','set_stop_mode','set_goal_mode'])
def test_modes_do_nothing_without_map(method):
    gui=state(original_map=None)
    getattr(PathPlannerGUI,method)(gui)
    assert gui.mode=='start'
    gui.status_var.set.assert_not_called()

def test_stop_limit_prevents_extra_stops():
    gui=state(stops=[(i,i) for i in range(5)])
    with patch('agv_planner.gui.app.messagebox.showinfo') as popup:
        PathPlannerGUI.set_stop_mode(gui)
    popup.assert_called_once()
    assert gui.mode=='start'

def test_undo_stop_invalidates_route():
    gui=state(stops=[(3,3)])
    PathPlannerGUI.undo_stop(gui)
    assert gui.stops==[] and gui.path is None and gui.visit_order==[]
    gui.redraw.assert_called_once()

def test_undo_empty_stop_is_noop():
    gui=state(stops=[])
    PathPlannerGUI.undo_stop(gui)
    gui.redraw.assert_not_called()

def test_clear_trail():
    gui=state()
    PathPlannerGUI.clear_trail(gui)
    assert gui.trail==[]
    gui.redraw.assert_called_once()

def test_clear_resets_navigation_state():
    gui=state(stops=[(3,3)], original_map=object())
    PathPlannerGUI.clear(gui)
    assert gui.start is None and gui.goal is None and gui.stops==[]
    assert gui.path is None and gui.visit_order==[] and gui.mode=='start'
    gui.stop_simulation.assert_called_once_with(redraw=False)

def test_pause_running_simulation():
    gui=state(sim_state='running')
    PathPlannerGUI.pause_simulation(gui)
    assert gui.sim_state=='paused'

def test_resume_paused_simulation():
    gui=state(sim_state='paused')
    PathPlannerGUI.pause_simulation(gui)
    assert gui.sim_state=='running'
    assert gui.sim_last_time>0

def test_pause_stopped_is_noop():
    gui=state(sim_state='stopped')
    PathPlannerGUI.pause_simulation(gui)
    assert gui.sim_state=='stopped'
    gui.sim_status.set.assert_not_called()

def test_stop_simulation_clears_motor_commands():
    gui=state(sim_after_id='scheduled',sim_state='running',sim_pose=(1,1,0),wheel_rates=[2]*4)
    PathPlannerGUI.stop_simulation(gui,redraw=False)
    assert gui.sim_state=='stopped' and gui.sim_pose is None
    assert gui.wheel_rates==[0]*4 and gui.mega_commands==(0,)*4
    gui.root.after_cancel.assert_called_once_with('scheduled')
    gui.redraw.assert_not_called()

def test_stop_simulation_redraws_by_default():
    gui=state()
    PathPlannerGUI.stop_simulation(gui)
    gui.redraw.assert_called_once()

@pytest.mark.parametrize('value', ['nan','-0.1','1.6','bad'])
def test_invalid_clearance_rejected(value):
    gui=state()
    gui.clearance_var.get.return_value=value
    with patch('agv_planner.gui.app.messagebox.showerror') as popup:
        PathPlannerGUI.apply_clearance(gui)
    popup.assert_called_once()
    gui.stop_simulation.assert_not_called()

@pytest.mark.parametrize('value', ['0','0.25','1.5'])
def test_valid_clearance_resets_route(value):
    gui=state()
    gui.clearance_var.get.return_value=value
    PathPlannerGUI.apply_clearance(gui)
    assert gui.path is None and gui.visit_order==[]
    assert gui.static_inflated_grid is not None
    gui.refresh_dynamic_grid.assert_called_once()

def test_clearance_no_map_noop():
    gui=state(free_grid=None)
    PathPlannerGUI.apply_clearance(gui)
    gui.stop_simulation.assert_not_called()

def test_route_stats_missing_path():
    gui=state(path=None,distance_field=None,route_stats=Mock())
    PathPlannerGUI.update_route_stats(gui)
    gui.route_stats.set.assert_called_once_with('Route: not planned')

def test_simulation_requires_route():
    gui=state(path=None)
    with patch('agv_planner.gui.app.messagebox.showwarning') as popup:
        PathPlannerGUI.start_simulation(gui)
    popup.assert_called_once()
