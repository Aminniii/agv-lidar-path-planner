"""Integration-style headless route, drawing, viewport and obstacle regressions."""
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import math
import numpy as np
import pytest
from PIL import Image
from agv_planner.gui.app import PathPlannerGUI

class V:
    def __init__(self,v): self.v=v
    def get(self): return self.v
    def set(self,v): self.v=v

def gui(**kw):
    grid=np.ones((50,50),dtype=bool)
    d=dict(original_map=Image.new('RGB',(50,50),'white'),canvas=Mock(),
           map_width=50,map_height=50,scale=1.,resolution=.1,
           map_offset_x=0,map_offset_y=0,origin_x=0.,origin_y=0.,origin_yaw=0.,
           dynamic_obstacles=[],path=None,start=(3,3),goal=(40,40),stops=[],visit_order=[],
           trail=[],show_trail=V(False),show_footprint=V(False),show_detection=V(False),
           robot_pose=None,sim_pose=None,detection_radius_var=V('.3'),
           inflated_grid=grid.copy(),static_inflated_grid=grid.copy(),free_grid=grid.copy(),
           distance_field=np.ones((50,50)),clearance_var=V('0'),
           algorithm_var=V('A*'),route_mode=V('Shortest'),smooth_enabled=V(False),
           smooth_radius_var=V('0.1'),status_var=V(''),mouse_position=V(''),
           root=Mock(),auto_start=False,nav2_segments=[],nav2_pending=False,nav2_goal_handle=None,
           stop_simulation=Mock(),sync_start_to_robot=Mock(),update_route_stats=Mock(),
           redraw=Mock(),pixel_to_world=lambda x,y:(x*.1,y*.1),
           world_to_pixel=lambda x,y:(int(x/.1),int(y/.1)),
           _fit_on_resize=True,_fit_resize_job=None,fullscreen=False,
           handle_obstacle_change=Mock(),refresh_dynamic_grid=Mock())
    d.update(kw)
    return S(**d)

@pytest.mark.parametrize('reason,change',[
    ('No map',{'original_map':None}),('No start',{'start':None}),('No goal',{'goal':None})])
def test_plan_requires_inputs(reason,change):
    g=gui(**change)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.plan_path(g)
    assert warning.call_args.args[0]==reason
    assert not g.status_var.get()

@pytest.mark.parametrize('which', ['start','goal'])
def test_plan_rejects_blocked_endpoint(which):
    g=gui(); g.inflated_grid[getattr(g,which)[1],getattr(g,which)[0]]=False
    PathPlannerGUI.plan_path(g)
    assert 'blocked' in g.status_var.get()
    g.redraw.assert_called_once()

def test_plan_rejects_active_nav2():
    g=gui(nav2_pending=True)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.plan_path(g)
    assert 'Cancel' in warning.call_args.args[1]
    g.stop_simulation.assert_not_called()

@pytest.mark.parametrize('algorithm,mode', [('A*','Shortest'),('Theta*','Shortest'),('A*','Balanced'),('A*','Safest')])
def test_plan_real_search_without_stops(algorithm,mode):
    g=gui(algorithm_var=V(algorithm),route_mode=V(mode))
    PathPlannerGUI.plan_path(g)
    assert g.path and g.path[0]==g.start and g.path[-1]==g.goal
    assert 'PATH FOUND' in g.status_var.get()
    g.update_route_stats.assert_called_once()

@pytest.mark.parametrize('stops', [[(15,20)],[(10,30),(30,10)]])
def test_plan_preserves_real_required_stops(stops):
    g=gui(stops=stops)
    PathPlannerGUI.plan_path(g)
    assert g.path and all(p in g.path for p in stops)
    assert set(g.visit_order)==set(stops)

def test_plan_bad_smoothing_radius_falls_back():
    g=gui(smooth_enabled=V(True),smooth_radius_var=V('bad'))
    PathPlannerGUI.plan_path(g)
    assert g.path is not None

def test_plan_disconnected_route():
    g=gui();g.inflated_grid[25,:]=False
    PathPlannerGUI.plan_path(g)
    assert g.path is None and 'NO PATH' in g.status_var.get()

@pytest.mark.parametrize('size',[(50,50),(150,100)])
def test_redraw_map_and_viewport_centering(size):
    g=gui(path=[(3,3),(40,40)],scale=1)
    g.canvas.winfo_width.return_value=size[0]
    g.canvas.winfo_height.return_value=size[1]
    with patch('agv_planner.gui.app.ImageTk.PhotoImage',return_value=object()):
        PathPlannerGUI.redraw(g)
    assert g.map_offset_x==max(0,(size[0]-50)//2)
    assert g.map_offset_y==max(0,(size[1]-50)//2)
    g.canvas.create_image.assert_called_once()
    g.canvas.configure.assert_called_once()

def test_redraw_empty_map_clears_canvas():
    g=gui(original_map=None)
    PathPlannerGUI.redraw(g)
    g.canvas.delete.assert_called_once_with('all')

@pytest.mark.parametrize('extras',[
    {'dynamic_obstacles':[(12,13,.2)]},
    {'stops':[(10,10)],'visit_order':[(10,10)]},
    {'trail':[(.1,.1),(.5,.5)],'show_trail':V(True)},
    {'robot_pose':(2.,2.,0.),'show_footprint':V(True)},
    {'sim_pose':(20.,20.,0.),'show_detection':V(True)},
    {'sim_pose':(20.,20.,1.),'detection_radius_var':V('bad'),'show_detection':V(True)},
    {'start':None,'goal':None},
])
def test_redraw_optional_layers(extras):
    g=gui(**extras);g.canvas.winfo_width.return_value=100;g.canvas.winfo_height.return_value=100
    with patch('agv_planner.gui.app.ImageTk.PhotoImage',return_value=object()):
        PathPlannerGUI.redraw(g)
    assert g.tk_image is not None
    g.canvas.create_image.assert_called_once()

@pytest.mark.parametrize('factor,expected',[(2,2),(.5,.5),(100,12),(.001,.2)])
def test_zoom_limits_and_redraw(factor,expected):
    g=gui();g.canvas.canvasx.side_effect=lambda x:x;g.canvas.canvasy.side_effect=lambda y:y
    g.canvas.winfo_width.return_value=100;g.canvas.winfo_height.return_value=100
    PathPlannerGUI.zoom_at(g,S(x=25,y=25),factor)
    assert g.scale==expected
    assert g._fit_on_resize is False
    g.redraw.assert_called_once()

@pytest.mark.parametrize('method', ['zoom_in','zoom_out'])
def test_zoom_buttons(method):
    g=gui(zoom_at=Mock())
    g.canvas.winfo_width.return_value=100;g.canvas.winfo_height.return_value=100
    PathPlannerGUI.zoom_center(g,1.25 if method=='zoom_in' else .8)
    g.zoom_at.assert_called_once()

@pytest.mark.parametrize('delta,factor',[(120,1.25),(-120,.8)])
def test_wheel_zoom(delta,factor):
    g=gui(zoom_at=Mock())
    PathPlannerGUI.mouse_wheel(g,S(delta=delta))
    assert g.zoom_at.call_args.args[1]==factor

def test_cursor_inside_outside():
    g=gui();g.canvas.canvasx.side_effect=lambda x:x;g.canvas.canvasy.side_effect=lambda y:y
    PathPlannerGUI.update_mouse_coordinates(g,S(x=10,y=20))
    assert 'pixel (10, 20)' in g.mouse_position.get()
    PathPlannerGUI.update_mouse_coordinates(g,S(x=500,y=500))
    assert 'outside' in g.mouse_position.get()

@pytest.mark.parametrize('viewport,expected',[((100,100),1.64),((500,200),3.64)])
def test_fit_map(viewport,expected):
    g=gui();g.canvas.winfo_width.return_value=viewport[0];g.canvas.winfo_height.return_value=viewport[1]
    PathPlannerGUI.fit_map(g)
    assert g.scale==pytest.approx(expected)
    g.redraw.assert_called_once()

def test_pan_and_fullscreen():
    g=gui()
    PathPlannerGUI.begin_pan(g,S(x=10,y=20))
    assert g._drag_origin==(10,20)
    PathPlannerGUI.drag_pan(g,S(x=12,y=25))
    PathPlannerGUI.end_pan(g,S(x=12,y=25))
    assert g._drag_origin is None
    PathPlannerGUI.toggle_fullscreen(g)
    assert g.fullscreen
    PathPlannerGUI.leave_fullscreen(g)
    assert not g.fullscreen

def test_dynamic_obstacle_inflation_real_grid():
    g=gui(dynamic_obstacles=[(25,25,.2)],clearance_var=V('.1'))
    PathPlannerGUI.refresh_dynamic_grid(g)
    assert not g.inflated_grid[25,25]
    assert not g.inflated_grid[25,27]
    assert g.inflated_grid[0,0]
    assert np.isfinite(g.distance_field).all()

def test_dynamic_obstacle_no_static_map():
    g=gui(static_inflated_grid=None)
    PathPlannerGUI.refresh_dynamic_grid(g)
    assert g.inflated_grid is not None
