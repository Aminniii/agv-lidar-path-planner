"""Real Tk widget integration checks; isolated Xvfb display, never sends robot commands."""
import os
import shutil
import subprocess
import time
import tkinter as tk
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
from PIL import Image

from agv_planner.gui import app


@pytest.fixture(scope='module')
def display():
    if not shutil.which('Xvfb'):
        pytest.skip('Xvfb is required for real Tk integration checks')
    process = None
    original = os.environ.get('DISPLAY')
    for n in range(87, 108):
        p = subprocess.Popen(['Xvfb', f':{n}', '-screen', '0', '1440x900x24', '-nolisten', 'tcp'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(.12)
        if p.poll() is None:
            process = p
            os.environ['DISPLAY'] = f':{n}'
            break
    if process is None:
        pytest.skip('Could not start an isolated Xvfb server')
    yield
    if original is None:
        os.environ.pop('DISPLAY', None)
    else:
        os.environ['DISPLAY'] = original
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()


@pytest.fixture
def gui(display):
    root = tk.Tk()
    with patch.object(app, 'ROS_AVAILABLE', False), patch.object(app, 'DEFAULT_MAP_YAML', Path('/nonexistent/agv_integration_map.yaml')):
        g = app.PathPlannerGUI(root)
    yield g
    try:
        root.destroy()
    except tk.TclError:
        pass


def attach_map(g):
    g.original_map = Image.new('RGB', (40, 30), 'white')
    g.map_width, g.map_height = 40, 30
    g.resolution = .1
    g.origin_x = g.origin_y = g.origin_yaw = 0.
    g.free_grid = np.ones((30, 40), dtype=bool)
    g.inflated_grid = g.free_grid.copy()
    g.static_inflated_grid = g.free_grid.copy()
    g.distance_field = np.ones((30,40))
    return g


@pytest.mark.parametrize('attribute,expected', [
    ('mode','start'),('auto_start',False),('sim_state','stopped'),('fullscreen',False),
    ('nav2_pending',False),('nav2_cancel_requested',False),('sim_index',0),
    ('sim_distance',0.0),('sim_elapsed',0.0),('sim_velocity',0.0),
    ('map_width',0),('map_height',0),('path',None),('start',None),('goal',None),
    ('robot_pose',None),('ros_node',None),('nav2_goal_handle',None),
    ('_fit_on_resize',True),('sim_after_id',None),('nav2_dwell_job',None),
])
def test_real_gui_initial_state(gui, attribute, expected):
    assert getattr(gui, attribute) == expected


@pytest.mark.parametrize('variable,expected', [
    ('algorithm_var','A*'),('route_mode','Shortest'),('sim_speed_var',.30),
    ('sim_accel_var',.35),('sim_yaw_rate_var',90.),('smooth_enabled',True),
    ('show_trail',True),('show_footprint',True),('show_detection',True),
    ('mega_mode',True),('nav2_execution_enabled',False),
])
def test_real_tk_variables(gui, variable, expected):
    assert getattr(gui, variable).get() == expected


def test_offline_labels_and_ros_disarmed(gui):
    assert 'OFFLINE' in gui.nav2_status.get()
    assert 'offline' in gui.local_status.get().lower()
    assert not gui.nav2_execution_enabled.get()


def test_real_canvas_exists(gui):
    assert isinstance(gui.canvas, tk.Canvas)
    assert gui.canvas.winfo_exists()


def test_real_root_window_configured(gui):
    assert gui.root.title().startswith('AGV LiDAR Path Planner')
    assert gui.root.minsize() == (900, 580)


def test_real_map_mode_guard(gui):
    gui.set_stop_mode(); gui.set_goal_mode(); gui.set_start_mode()
    assert gui.mode == 'start'


def test_real_mode_switches_with_map(gui):
    attach_map(gui)
    gui.set_stop_mode(); assert gui.mode == 'stop'
    gui.set_goal_mode(); assert gui.mode == 'goal'
    gui.set_start_mode(); assert gui.mode == 'start'


def test_real_auto_start_toggle(gui):
    with patch.object(gui, 'redraw'), patch.object(gui, 'sync_start_to_robot'):
        gui.toggle_auto_start()
        assert gui.auto_start and gui.mode == 'goal'
        assert 'ON' in gui.auto_button.cget('text')
        gui.toggle_auto_start()
        assert not gui.auto_start
        assert 'OFF' in gui.auto_button.cget('text')


def test_real_clear_state(gui):
    gui.start=(1,2); gui.goal=(3,4); gui.stops=[(2,3)]; gui.path=[(1,2),(3,4)]
    with patch.object(gui,'redraw'):
        gui.clear()
    assert gui.start is gui.goal is gui.path is None
    assert gui.stops == [] and gui.mode == 'start'


def test_real_stop_undo(gui):
    gui.stops=[(1,1),(2,2)]; gui.path=[(1,1),(2,2)]
    with patch.object(gui,'redraw'):
        gui.undo_stop()
    assert gui.stops == [(1,1)] and gui.path is None


def test_real_coordinate_conversion(gui):
    attach_map(gui)
    for p in [(0,0),(5,7),(39,29)]:
        assert gui.world_to_pixel(*gui.pixel_to_world(*p)) == p


def test_real_close_window(gui):
    with patch.object(gui,'stop_simulation') as stop:
        gui.close_window()
    stop.assert_called_once_with(redraw=False)
    with pytest.raises(tk.TclError):
        gui.root.winfo_exists()


def test_real_close_blocked_during_nav2(gui):
    gui.nav2_pending=True
    with patch.object(app.messagebox,'showwarning') as warning:
        gui.close_window()
    warning.assert_called_once()
    assert gui.root.winfo_exists()


def test_real_offline_pose_poll_reschedules(gui):
    with patch.object(gui.root,'after') as after:
        gui.poll_robot_pose()
    after.assert_called_once_with(200,gui.poll_robot_pose)


def test_real_map_render(gui):
    attach_map(gui)
    gui.start=(2,3);gui.goal=(35,25)
    gui.path=[(2,3),(35,25)]
    gui.redraw()
    assert gui.canvas.find_all()


def test_real_map_fit(gui):
    attach_map(gui)
    gui.fit_map()
    assert gui.scale > 0


def test_real_trail_clear(gui):
    gui.trail.append((1,2))
    with patch.object(gui,'redraw'):
        gui.clear_trail()
    assert not gui.trail
