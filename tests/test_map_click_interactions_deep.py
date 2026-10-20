"""Map click behavior without a display or robot hardware."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from agv_planner.gui.app import PathPlannerGUI
from agv_planner.gui import app


def make_gui(mode='start', free=True):
    g = SimpleNamespace(
        original_map=object(), canvas=SimpleNamespace(canvasx=lambda x: x, canvasy=lambda y: y),
        map_offset_x=0, map_offset_y=0, scale=1, map_width=12, map_height=10,
        inflated_grid=np.full((10, 12), free, dtype=bool), mode=mode,
        status_var=Mock(), obstacle_radius_var=Mock(), dynamic_obstacles=[],
        refresh_dynamic_grid=Mock(), handle_obstacle_change=Mock(),
        start=None, goal=None, stops=[], path=[(1, 1)], visit_order=[(1, 1)],
        pixel_to_world=lambda x,y: (x*.1,y*.1), auto_start=False,
        sync_start_to_robot=Mock(), plan_path=Mock(), redraw=Mock(),
    )
    g.obstacle_radius_var.get.return_value='0.2'
    return g


def click(g, x=3, y=4):
    PathPlannerGUI.map_clicked(g, SimpleNamespace(x=x, y=y))


@pytest.mark.parametrize('xy', [(-1, 2), (12, 2), (2, -1), (2, 10)])
def test_click_outside_map_does_nothing(xy):
    g=make_gui()
    click(g,*xy)
    assert g.start is None
    g.redraw.assert_not_called()


def test_click_without_map_does_nothing():
    g=make_gui(); g.original_map=None
    click(g)
    g.redraw.assert_not_called()


@pytest.mark.parametrize('mode', ['start','goal','stop'])
def test_blocked_cell_rejected(mode):
    g=make_gui(mode, free=False)
    click(g)
    assert g.start is None and g.goal is None and not g.stops
    assert 'unknown' in g.status_var.set.call_args.args[0]


def test_start_selection_invalidates_route_and_changes_mode():
    g=make_gui()
    click(g)
    assert g.start==(3,4) and g.mode=='goal'
    assert g.path is None and g.visit_order==[]
    g.redraw.assert_called_once()


def test_goal_selection_invalidates_route():
    g=make_gui('goal')
    click(g)
    assert g.goal==(3,4) and g.path is None
    g.redraw.assert_called_once()


def test_auto_start_replans_from_live_robot():
    g=make_gui('goal'); g.auto_start=True
    click(g)
    g.sync_start_to_robot.assert_called_once()
    g.plan_path.assert_called_once()
    g.redraw.assert_not_called()


def test_stop_addition_clears_route():
    g=make_gui('stop')
    click(g)
    assert g.stops==[(3,4)] and g.path is None and g.visit_order==[]
    g.redraw.assert_called_once()


@pytest.mark.parametrize('kind', ['start','goal','duplicate'])
def test_stop_cannot_duplicate_special_points(kind):
    g=make_gui('stop')
    if kind=='start': g.start=(3,4)
    elif kind=='goal': g.goal=(3,4)
    else: g.stops=[(3,4)]
    click(g)
    assert len(g.stops)==(1 if kind=='duplicate' else 0)
    g.redraw.assert_not_called()


def test_stop_limit_shows_message():
    g=make_gui('stop'); g.stops=[(i,0) for i in range(5)]
    with patch.object(app.messagebox,'showinfo') as info:
        click(g)
    info.assert_called_once()
    assert len(g.stops)==5


@pytest.mark.parametrize('radius', ['0.05','0.2','1.0'])
def test_dynamic_obstacle_valid_radius(radius):
    g=make_gui('obstacle'); g.obstacle_radius_var.get.return_value=radius
    click(g)
    assert g.dynamic_obstacles==[(3,4,float(radius))]
    g.refresh_dynamic_grid.assert_called_once()
    g.handle_obstacle_change.assert_called_once()


@pytest.mark.parametrize('radius', ['bad','0','0.049','1.01','nan','inf'])
def test_dynamic_obstacle_invalid_radius(radius):
    g=make_gui('obstacle'); g.obstacle_radius_var.get.return_value=radius
    with patch.object(app.messagebox,'showerror') as error:
        click(g)
    error.assert_called_once()
    assert not g.dynamic_obstacles


def test_canvas_offset_and_scale_select_expected_cell():
    g=make_gui(); g.map_offset_x=10; g.map_offset_y=20; g.scale=2
    click(g, 16, 28)
    assert g.start == (3, 4)


def test_obstacle_mode_accepts_occupied_cell():
    g=make_gui('obstacle', free=False)
    click(g)
    assert g.dynamic_obstacles == [(3,4,0.2)]
