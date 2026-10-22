"""Cover remaining real branches without using physical robot hardware."""
import runpy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
from test_gui_render_branches_push20 import gui_for_redraw

from agv_planner.gui import app


def test_gui_module_ros_available_import_branch():
    # Run module as a non-main script so it cannot open a Tk window.
    stub = tuple(Mock(name=f"ros_dependency_{i}") for i in range(13))
    with patch("agv_planner.ros_support.load_ros_dependencies", return_value=(True, None, stub)):
        result = runpy.run_path(str(Path(app.__file__)), run_name="agv_planner_gui_coverage_probe")
    assert result["ROS_AVAILABLE"] is True
    assert result["rclpy"] is stub[0]


def test_nav2_segment_rejects_incomplete_ros_path():
    gui = NS(path=[(0, 0), (1, 0)], visit_order=[])
    ros_path = NS(header=Mock(), poses=[Mock()])
    with pytest.raises(ValueError, match="incomplete ROS path"):
        app.PathPlannerGUI._build_nav2_segments(gui, ros_path)


def test_redraw_with_no_map_resolution_skips_scale_bar():
    gui = gui_for_redraw(resolution=0)
    with patch.object(app.ImageTk, "PhotoImage", return_value=object()):
        app.PathPlannerGUI.redraw(gui)
    gui.canvas.delete.assert_called_once_with("all")
