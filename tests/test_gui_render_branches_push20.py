"""Exercise unusual GUI rendering branches without ROS or physical motors."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from PIL import Image
import pytest
from agv_planner.gui import app


def gui_for_redraw(**changes):
    g = dict(original_map=Image.new("RGB", (40, 40), "white"), canvas=Mock(),
             dynamic_obstacles=[], path=[], start=None, goal=None, stops=[], visit_order=[],
             show_trail=Mock(get=Mock(return_value=False)), trail=[],
             show_footprint=Mock(get=Mock(return_value=False)), robot_pose=None,
             sim_pose=None, show_detection=Mock(get=Mock(return_value=False)),
             resolution=.1, map_width=40, map_height=40, scale=1., origin_yaw=0.,
             world_to_pixel=Mock(return_value=None), detection_radius_var=Mock(get=Mock(return_value=.2)))
    g.update(changes)
    g["canvas"].winfo_width.return_value = 300
    g["canvas"].winfo_height.return_value = 250
    return NS(**g)


@pytest.mark.parametrize("trail,footprint,robot,world_values", [
    ([(0.,0.), (1.,1.)], False, None, [None, None]),
    ([(0.,0.), (1.,1.)], False, None, [(1,1),(2,2)]),
    ([], True, (1.,1.,0.), [None, None, None, None, None]),
    ([], True, (1.,1.,0.), [(1,1),(2,2),(3,3),(4,4),None]),
    ([], True, (1.,1.,0.), [(1,1),(2,2),(3,3),(4,4),(5,5)]),
])
def test_redraw_branch(trail,footprint,robot,world_values):
    g = gui_for_redraw(trail=trail, robot_pose=robot,
                       show_trail=Mock(get=Mock(return_value=bool(trail))),
                       show_footprint=Mock(get=Mock(return_value=footprint)),
                       world_to_pixel=Mock(side_effect=world_values))
    with patch.object(app.ImageTk, "PhotoImage", return_value=object()):
        app.PathPlannerGUI.redraw(g)
    g.canvas.create_image.assert_called_once()
