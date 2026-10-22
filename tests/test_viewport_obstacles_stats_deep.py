"""Headless regressions for viewport, virtual obstacles and route statistics."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import pytest

from agv_planner.gui.app import PathPlannerGUI


def gui(**overrides):
    canvas=Mock()
    canvas.winfo_width.return_value=800
    canvas.winfo_height.return_value=600
    canvas.canvasx.side_effect=lambda v:v
    canvas.canvasy.side_effect=lambda v:v
    obj=dict(canvas=canvas,root=Mock(),original_map=object(),map_width=100,map_height=80,
             map_offset_x=0,map_offset_y=0,scale=1.0,redraw=Mock(),
             _fit_on_resize=True,_fit_resize_job=None,fullscreen=False,
             mouse_position=Mock(),pixel_to_world=lambda x,y:(x*.1,y*.1),
             static_inflated_grid=np.ones((20,20),bool),dynamic_obstacles=[],
             clearance_var=Mock(),resolution=.1,inflated_grid=np.ones((20,20),bool),
             distance_field=np.full((20,20),4.0),sim_state='stopped',sim_pose=None,
             sim_index=0,sim_route=[],visit_order=[],stop_simulation=Mock(),
             path=[(1,1),(4,5)],stops=[],start=(1,1),auto_start=True,
             auto_button=Mock(),status_var=Mock(),route_stats=Mock(),plan_path=Mock(),
             sim_speed_var=Mock(),free_grid=np.ones((20,20),bool))
    obj['clearance_var'].get.return_value='0'
    obj['sim_speed_var'].get.return_value='0.5'
    obj.update(overrides)
    return NS(**obj)

@pytest.mark.parametrize('x,y,expected',[(0,0,'pixel (0, 0)'),(99,79,'pixel (99, 79)'),(-1,4,'outside map'),(100,2,'outside map'),(2,80,'outside map')])
def test_cursor_inside_and_outside(x,y,expected):
    g=gui()
    PathPlannerGUI.update_mouse_coordinates(g,NS(x=x,y=y))
    assert expected in g.mouse_position.set.call_args.args[0]

def test_cursor_no_map_does_not_update():
    g=gui(original_map=None)
    PathPlannerGUI.update_mouse_coordinates(g,NS(x=4,y=5))
    g.mouse_position.set.assert_not_called()

@pytest.mark.parametrize('factor,expected',[(100,12.0),(.001,.2),(1.0,1.0),(1.25,1.25)])
def test_zoom_clamps_and_noops(factor,expected):
    g=gui()
    PathPlannerGUI.zoom_at(g,NS(x=100,y=100),factor)
    assert g.scale==pytest.approx(expected)
    assert g.redraw.call_count==(0 if factor==1.0 else 1)

def test_zoom_without_map_does_nothing():
    g=gui(original_map=None)
    PathPlannerGUI.zoom_at(g,NS(x=2,y=2),2)
    assert g.scale==1
    g.redraw.assert_not_called()

@pytest.mark.parametrize('method,expected',[('toggle_fullscreen',True),('leave_fullscreen',False)])
def test_fullscreen_commands(method,expected):
    g=gui(fullscreen=False if method=='toggle_fullscreen' else True)
    getattr(PathPlannerGUI,method)(g)
    assert g.fullscreen is expected
    g.root.attributes.assert_called_with('-fullscreen',expected)

def test_fit_map_scales_to_canvas():
    g=gui()
    PathPlannerGUI.fit_map(g)
    assert g.scale==pytest.approx(min(12,782/100,582/80))
    g.canvas.xview_moveto.assert_called_with(0)
    g.canvas.yview_moveto.assert_called_with(0)

def test_resize_cancels_old_job_and_schedules_redraw():
    g=gui(_fit_on_resize=False,_fit_resize_job='old')
    PathPlannerGUI._on_canvas_resize(g,NS())
    g.root.after_cancel.assert_called_once_with('old')
    assert g.root.after.call_args.args[0]==150
    assert g.root.after.call_args.args[1]==g.redraw

def test_dynamic_obstacle_blocks_cells_and_keeps_original():
    original=np.ones((20,20),bool)
    g=gui(static_inflated_grid=original,dynamic_obstacles=[(10,10,.2)])
    PathPlannerGUI.refresh_dynamic_grid(g)
    assert not g.inflated_grid[10,10]
    assert g.inflated_grid[0,0]
    assert original.all()
    assert g.distance_field.shape==(20,20)

def test_clearance_expands_dynamic_obstacle():
    g=gui(dynamic_obstacles=[(10,10,.1)])
    g.clearance_var.get.return_value='.2'
    PathPlannerGUI.refresh_dynamic_grid(g)
    assert not g.inflated_grid[10,13]
    assert g.inflated_grid[10,15]

def test_refresh_no_static_grid_is_noop():
    g=gui(static_inflated_grid=None)
    PathPlannerGUI.refresh_dynamic_grid(g)
    assert g.inflated_grid is not None

def test_undo_obstacle_rebuilds_and_invalidates():
    g=gui(dynamic_obstacles=[(5,5,.1)])
    g.refresh_dynamic_grid=Mock()
    g.handle_obstacle_change=Mock()
    PathPlannerGUI.undo_obstacle(g)
    assert g.dynamic_obstacles==[]
    g.refresh_dynamic_grid.assert_called_once()
    g.handle_obstacle_change.assert_called_once()

def test_undo_empty_obstacles_noop():
    g=gui(dynamic_obstacles=[])
    g.refresh_dynamic_grid=Mock()
    PathPlannerGUI.undo_obstacle(g)
    g.refresh_dynamic_grid.assert_not_called()

def test_clear_obstacles_always_revalidates():
    g=gui(dynamic_obstacles=[(2,2,.2)])
    g.refresh_dynamic_grid=Mock()
    g.handle_obstacle_change=Mock()
    PathPlannerGUI.clear_obstacles(g)
    assert g.dynamic_obstacles==[]
    g.handle_obstacle_change.assert_called_once()

def test_obstacle_change_stops_and_invalidates_idle_route():
    g=gui()
    PathPlannerGUI.handle_obstacle_change(g)
    g.stop_simulation.assert_called_once_with(redraw=False)
    assert g.path is None and g.visit_order==[]
    assert 'updated' in g.status_var.set.call_args.args[0]

def test_obstacle_change_replans_from_valid_virtual_position():
    g=gui(sim_state='running',sim_pose=(7.2,8.1,0),sim_route=[(1,1),(7,8),(10,10)],
          sim_index=0,visit_order=[(7,8),(10,10)],stops=[(7,8),(10,10)])
    PathPlannerGUI.handle_obstacle_change(g)
    assert g.start==(7,8)
    assert g.stops==[(7,8),(10,10)]
    assert g.auto_start is False
    g.plan_path.assert_called_once()

def test_obstacle_change_inside_blocked_cell_refuses_replan():
    grid=np.ones((20,20),bool);grid[8,7]=False
    g=gui(sim_state='running',sim_pose=(7,8,0),inflated_grid=grid)
    PathPlannerGUI.handle_obstacle_change(g)
    g.plan_path.assert_not_called()
    assert 'overlaps' in g.status_var.set.call_args.args[0]

def test_route_stats_without_path():
    g=gui(path=None)
    PathPlannerGUI.update_route_stats(g)
    g.route_stats.set.assert_called_once_with('Route: not planned')

def test_route_stats_length_and_dwell():
    g=gui(path=[(1,1),(4,5)],stops=[(4,5)])
    PathPlannerGUI.update_route_stats(g)
    result=g.route_stats.set.call_args.args[0]
    assert '0.50 m' in result
    assert '0.30 m' in result
    assert '3.0 s' in result

def test_route_stats_bad_speed_falls_back():
    g=gui(path=[(1,1),(4,5)])
    g.sim_speed_var.get.return_value='invalid'
    PathPlannerGUI.update_route_stats(g)
    assert '1.7 s' in g.route_stats.set.call_args.args[0]
