"""Headless GUI window-control regressions; no Tk window or robot required."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import pytest
from agv_planner.gui.app import PathPlannerGUI


def ui(**kwargs):
    defaults = dict(root=Mock(), canvas=Mock(), original_map=object(),
                    status_var=Mock(), redraw=Mock(), fit_map=Mock(),
                    zoom_at=Mock(), zoom_center=Mock(), fullscreen=False,
                    mode='start', _fit_resize_job=None, _fit_on_resize=True)
    defaults.update(kwargs)
    return SimpleNamespace(**defaults)


@pytest.mark.parametrize('method,factor', [('zoom_in',1.25),('zoom_out',0.8)])
def test_zoom_buttons_delegate_to_center(method,factor):
    g=ui()
    getattr(PathPlannerGUI,method)(g)
    g.zoom_center.assert_called_once_with(factor)


@pytest.mark.parametrize('delta,factor', [(120,1.25),(-120,0.8),(0,0.8)])
def test_mouse_wheel_direction(delta,factor):
    g=ui()
    event=SimpleNamespace(delta=delta)
    PathPlannerGUI.mouse_wheel(g,event)
    g.zoom_at.assert_called_once_with(event,factor)


def test_zoom_center_uses_canvas_midpoint():
    g=ui()
    g.canvas.winfo_width.return_value=801
    g.canvas.winfo_height.return_value=601
    PathPlannerGUI.zoom_center(g,1.5)
    event,factor=g.zoom_at.call_args.args
    assert (event.x,event.y,factor)==(400,300,1.5)


@pytest.mark.parametrize('initial', [False,True])
def test_toggle_fullscreen(initial):
    g=ui(fullscreen=initial)
    PathPlannerGUI.toggle_fullscreen(g)
    assert g.fullscreen is not initial
    g.root.attributes.assert_called_once_with('-fullscreen',not initial)


def test_leave_fullscreen():
    g=ui(fullscreen=True)
    PathPlannerGUI.leave_fullscreen(g)
    assert g.fullscreen is False
    g.root.attributes.assert_called_once_with('-fullscreen',False)


def test_obstacle_mode_without_map_is_noop():
    g=ui(original_map=None)
    PathPlannerGUI.set_obstacle_mode(g)
    assert g.mode=='start'
    g.status_var.set.assert_not_called()


def test_obstacle_mode_with_map_updates_status():
    g=ui()
    PathPlannerGUI.set_obstacle_mode(g)
    assert g.mode=='obstacle'
    assert 'VIRTUAL' in g.status_var.set.call_args.args[0]


def test_resize_without_map_is_noop():
    g=ui(original_map=None)
    PathPlannerGUI._on_canvas_resize(g,object())
    g.root.after.assert_not_called()


@pytest.mark.parametrize('fit', [False,True])
def test_resize_cancels_prior_job_and_reschedules(fit):
    g=ui(_fit_resize_job='previous',_fit_on_resize=fit)
    PathPlannerGUI._on_canvas_resize(g,object())
    g.root.after_cancel.assert_called_once_with('previous')
    g.root.after.assert_called_once_with(150,g.fit_map if fit else g.redraw)


def test_resize_without_prior_job_does_not_cancel():
    g=ui(_fit_resize_job=None)
    PathPlannerGUI._on_canvas_resize(g,object())
    g.root.after_cancel.assert_not_called()


def test_fit_map_without_map_does_not_redraw():
    g=ui(original_map=None)
    PathPlannerGUI.fit_map(g)
    g.redraw.assert_not_called()


def test_fit_map_reschedules_when_canvas_too_small():
    g=ui(map_width=100,map_height=100)
    g.canvas.winfo_width.return_value=10
    g.canvas.winfo_height.return_value=10
    PathPlannerGUI.fit_map(g)
    g.root.after.assert_called_once_with(100,g.fit_map)
    g.redraw.assert_not_called()


def test_fit_map_scales_to_canvas_and_resets_scroll():
    g=ui(map_width=400,map_height=200,scale=1)
    g.canvas.winfo_width.return_value=818
    g.canvas.winfo_height.return_value=418
    PathPlannerGUI.fit_map(g)
    assert g.scale==2.0
    assert g._fit_on_resize is True
    g.redraw.assert_called_once()
    g.canvas.xview_moveto.assert_called_once_with(0)
    g.canvas.yview_moveto.assert_called_once_with(0)


def test_fit_map_limits_scale_to_twelve():
    g=ui(map_width=10,map_height=10)
    g.canvas.winfo_width.return_value=1000
    g.canvas.winfo_height.return_value=1000
    PathPlannerGUI.fit_map(g)
    assert g.scale==12.0


def test_close_window_refuses_active_route():
    g=ui(nav2_segments=['leg'],nav2_pending=False,nav2_goal_handle=None)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.close_window(g)
    warning.assert_called_once()
    g.root.destroy.assert_not_called()


def test_close_window_offline_cleans_up():
    g=ui(nav2_segments=[],nav2_pending=False,nav2_goal_handle=None,
         stop_simulation=Mock(),ros_node=None)
    with patch('agv_planner.gui.app.ROS_AVAILABLE',False):
        PathPlannerGUI.close_window(g)
    g.stop_simulation.assert_called_once_with(redraw=False)
    g.root.destroy.assert_called_once()
