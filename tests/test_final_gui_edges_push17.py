"""Additional isolated boundary tests; never connects to a robot."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
import pytest
from agv_planner.gui.app import PathPlannerGUI


def test_route_stats_samples_off_map_coordinates_and_bad_speed():
    g = NS(path=[(-3, -3), (2, 2)], distance_field=np.ones((5, 5)),
           map_width=5, map_height=5, resolution=.1,
           sim_speed_var=Mock(), stops=[], route_stats=Mock())
    g.sim_speed_var.get.side_effect=ValueError('bad speed')
    PathPlannerGUI.update_route_stats(g)
    assert 'Route:' in g.route_stats.set.call_args.args[0]


def test_route_stats_entirely_outside_map():
    g = NS(path=[(-5, -5), (-4, -4)], distance_field=np.ones((5, 5)),
           map_width=5, map_height=5, resolution=.1,
           sim_speed_var=Mock(), stops=[], route_stats=Mock())
    g.sim_speed_var.get.return_value=.2
    PathPlannerGUI.update_route_stats(g)
    g.route_stats.set.assert_called_once()


def test_nav2_segment_rejects_missing_required_stop():
    g = NS(path=[(0,0),(2,0),(4,0)], visit_order=[(3,3)])
    with pytest.raises(ValueError, match='required stop'):
        PathPlannerGUI._build_nav2_segments(g, NS(header=object(),poses=[1,2,3]))


def test_nav2_segment_rejects_zero_length_final_segment():
    # The last route waypoint cannot also be an intermediate stop.
    g = NS(path=[(0,0),(1,0),(2,0)], visit_order=[(1,0),(1,0)])
    with pytest.raises(ValueError, match='required stop'):
        PathPlannerGUI._build_nav2_segments(g, NS(header=object(),poses=[1,2,3]))


def test_update_wheel_bars_ignores_invalid_rate_in_real_gui():
    import tkinter as tk
    from pathlib import Path
    from agv_planner.gui import app
    root = tk.Tk()
    try:
        with patch.object(app,'ROS_AVAILABLE',False), patch.object(app,'DEFAULT_MAP_YAML',Path('/nonexistent/push17.yaml')):
            g = app.PathPlannerGUI(root)
        g.wheel_rates = ['invalid',0,0,0]
        g.wheel_display.set('trigger wheel update')
        g.wheel_rates = []
        g.wheel_display.set('trigger again')
    finally:
        root.destroy()

@pytest.mark.parametrize('raises', [False, True])
def test_startup_auto_map_load_success_and_warning(tmp_path, raises):
    import tkinter as tk
    from agv_planner.gui import app
    root = tk.Tk()
    map_file = tmp_path/'exists.yaml'
    map_file.write_text('image: example.pgm')
    try:
        with patch.object(app,'ROS_AVAILABLE',False), patch.object(app,'DEFAULT_MAP_YAML',map_file), \
             patch.object(app.PathPlannerGUI,'load_map',autospec=True) as load, \
             patch.object(app.messagebox,'showwarning') as warn:
            if raises:
                load.side_effect=ValueError('bad map')
            app.PathPlannerGUI(root)
        load.assert_called_once()
        assert warn.called is raises
    finally:
        root.destroy()


def test_startup_with_mock_ros_components():
    import tkinter as tk
    from agv_planner.gui import app
    from pathlib import Path
    root=tk.Tk()
    ros=Mock()
    ros.ok.return_value=False
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
             patch.object(app,'DEFAULT_MAP_YAML',Path('/nonexistent/push17.yaml')):
            g=app.PathPlannerGUI(root)
        ros.init.assert_called_once()
        assert g.ros_node is not None
    finally:
        root.destroy()
