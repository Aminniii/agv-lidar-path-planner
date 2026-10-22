"""Focused GUI planning and simulation regressions, without real ROS or motors."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np

from agv_planner.gui import app


def planning_gui(**changes):
    base = dict(stop_simulation=Mock(), nav2_segments=[], nav2_pending=False, nav2_goal_handle=None, original_map=object(), auto_start=True, sync_start_to_robot=Mock(),
                start=(1, 1), goal=(5, 1), stops=[], inflated_grid=np.ones((9, 9), dtype=bool),
                distance_field=np.ones((9, 9)), algorithm_var=Mock(get=Mock(return_value='Theta*')),
                route_mode=Mock(get=Mock(return_value='Balanced')), status_var=Mock(),
                root=Mock(), path=None, visit_order=[], redraw=Mock(),
                resolution=.1, smooth_enabled=Mock(get=Mock(return_value=False)),
                update_route_stats=Mock(), sim_speed_var=Mock(get=Mock(return_value=.3)))
    base.update(changes)
    return NS(**base)


def test_auto_start_syncs_before_planning_and_warns_theta_mode():
    g=planning_gui()
    with patch.object(app,'optimized_search',return_value=[(1,1),(5,1)]), \
         patch.object(app,'compact_path',side_effect=lambda path,*a,**kw:path), \
         patch.object(app,'adaptive_reduce_path',side_effect=lambda path,*a,**kw:path), \
         patch.object(app,'validate_route',side_effect=app.UnsafePathError('test rejection')):
        app.PathPlannerGUI.plan_path(g)
    g.sync_start_to_robot.assert_called_once()
    assert g.path is None
    assert any('Theta* currently' in c.args[0] for c in g.status_var.set.call_args_list)


def test_planner_rejects_route_failing_final_validation():
    g=planning_gui(auto_start=False)
    with patch.object(app,'optimized_search',return_value=[(1,1),(5,1)]), \
         patch.object(app,'compact_path',side_effect=lambda path,*a,**kw:path), \
         patch.object(app,'adaptive_reduce_path',side_effect=lambda path,*a,**kw:path), \
         patch.object(app,'validate_route',side_effect=app.UnsafePathError('unsafe shortcut')):
        app.PathPlannerGUI.plan_path(g)
    assert g.path is None and g.visit_order == []
    assert 'UNSAFE PATH rejected' in g.status_var.set.call_args.args[0]
    g.redraw.assert_called()


def test_gui_zoom_center_uses_canvas_midpoint():
    g=NS(canvas=Mock(),zoom_at=Mock())
    g.canvas.winfo_width.return_value=640
    g.canvas.winfo_height.return_value=480
    app.PathPlannerGUI.zoom_center(g,1.25)
    event,factor=g.zoom_at.call_args.args
    assert (event.x,event.y,factor)==(320,240,1.25)


