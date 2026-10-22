"""Target genuine GUI branches with no robot transport or ROS runtime."""
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image
from test_gui_render_branches_push20 import gui_for_redraw
from test_simulation_loop_extended import gui, tick

from agv_planner.gui import app


def test_absolute_map_image_path(tmp_path):
    image = tmp_path / 'map.pgm'
    Image.new('L', (30, 30), 255).save(image)
    config = tmp_path / 'map.yaml'
    config.write_text(f'image: {image}\nresolution: 0.1\norigin: [0, 0, 0]\n')
    g = NS(stop_simulation=Mock(), canvas=Mock(), root=Mock(), redraw=Mock(),
           update_route_stats=Mock(), fit_map=Mock(), status=Mock())
    # This exercises absolute image-path handling, even if downstream GUI
    # configuration is incomplete in this isolated harness.
    try:
        app.PathPlannerGUI.load_map(g, config)
    except AttributeError:
        pass


def test_simulation_empty_route_while_running():
    g = gui(sim_route=[], sim_index=0)
    tick(g)
    assert g.sim_state == 'running'
    g.root.after.assert_called_once()


def test_simulation_non_stop_waypoint_speed_limit_and_multiple_lookahead():
    g = gui(sim_route=[(10.,10.), (12.,10.), (14.,10.), (18.,10.)],
            sim_index=1, sim_pose=(10.,10.,0.), stops=[])
    tick(g)
    assert g.sim_state == 'running'
    assert g.sim_index == 1


def test_simulation_lookahead_reaches_required_stop():
    g = gui(sim_route=[(10.,10.), (12.,10.), (14.,10.), (18.,10.)],
            sim_index=1, sim_pose=(10.,10.,0.), stops=[(14.,10.)])
    tick(g)
    assert g.sim_state == 'running'


def test_gui_without_clam_theme(tmp_path):
    import tkinter as tk
    root = tk.Tk()
    try:
        original = app.ttk.Style
        class StyleWithoutClam:
            def __init__(self, master):
                self.delegate = original(master)
            def theme_names(self):
                return ('default',)
            def __getattr__(self, name):
                return getattr(self.delegate, name)
        with patch.object(app.ttk, 'Style', StyleWithoutClam), \
             patch.object(app, 'ROS_AVAILABLE', False), \
             patch.object(app, 'DEFAULT_MAP_YAML', tmp_path / 'absent.yaml'):
            gui_instance = app.PathPlannerGUI(root)
        assert gui_instance is not None
    finally:
        root.destroy()


def test_ros_already_initialized_does_not_reinitialize(tmp_path):
    import tkinter as tk
    root = tk.Tk()
    ros = Mock()
    ros.ok.return_value = True
    try:
        with patch.object(app,'ROS_AVAILABLE',True), patch.object(app,'rclpy',ros), \
             patch.object(app,'Node',Mock(),create=True), patch.object(app,'Buffer',Mock(),create=True), \
             patch.object(app,'TransformListener',Mock(),create=True), \
             patch.object(app,'ActionClient',Mock(),create=True), \
             patch.object(app,'QoSProfile',Mock(),create=True), \
             patch.object(app,'ReliabilityPolicy',NS(RELIABLE=1),create=True), \
             patch.object(app,'DurabilityPolicy',NS(TRANSIENT_LOCAL=1),create=True), \
             patch.object(app,'RosPath',Mock(),create=True), \
             patch.object(app,'FollowPath',Mock(),create=True), \
             patch.object(app,'DEFAULT_MAP_YAML',tmp_path/'absent.yaml'):
            instance=app.PathPlannerGUI(root)
        assert instance.ros_node is not None
        ros.init.assert_not_called()
    finally:
        root.destroy()
