"""Real YAML serialization and export guards, without GUI dialogs."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import yaml

from agv_planner.gui.app import PathPlannerGUI
from agv_planner.planning.safety import UnsafePathError


def gui():
    return SimpleNamespace(path=[(1,1),(2,2),(3,3)],resolution=.1,
        visit_order=[(2,2)], dynamic_obstacles=set(), robot_pose=None,
        _nav2_pose_fresh=Mock(return_value=False),
        validate_current_route=Mock(),pixel_to_world=lambda x,y:(x*.1,y*.1),
        status_var=Mock(),nav2_status=Mock())

@pytest.mark.parametrize('nav2',[False,True])
def test_export_writes_parseable_yaml(tmp_path,nav2):
    g=gui(); dest=tmp_path/'export.yaml'
    with patch('agv_planner.gui.app.filedialog.asksaveasfilename',return_value=str(dest)):
        if nav2: PathPlannerGUI.export_nav2_path(g)
        else: PathPlannerGUI.export_path(g)
    result=yaml.safe_load(dest.read_text())
    assert result['frame_id']=='map'
    if nav2:
        assert result['execution_enabled'] is False
        assert result['message_type']=='nav_msgs/msg/Path'
        assert len(result['poses'])==3 and len(result['stops'])==1
        assert result['poses'][0]['qw']==1
    else:
        assert len(result['waypoints'])==3
        assert result['waypoints'][1]['stop_id']==1
        assert result['waypoints'][1]['dwell_seconds']==2

@pytest.mark.parametrize('nav2',[False,True])
def test_cancelled_export_does_not_write(tmp_path,nav2):
    g=gui()
    with patch('agv_planner.gui.app.filedialog.asksaveasfilename',return_value=''):
        if nav2: PathPlannerGUI.export_nav2_path(g)
        else: PathPlannerGUI.export_path(g)
    assert list(tmp_path.iterdir())==[]

@pytest.mark.parametrize('nav2',[False,True])
def test_unsafe_route_warns_and_does_not_export(nav2):
    g=gui(); g.validate_current_route.side_effect=UnsafePathError('blocked')
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning,\
         patch('agv_planner.gui.app.filedialog.asksaveasfilename') as dialog:
        if nav2: PathPlannerGUI.export_nav2_path(g)
        else: PathPlannerGUI.export_path(g)
    warning.assert_called_once(); dialog.assert_not_called()

def test_nav2_rejects_virtual_obstacles():
    g=gui(); g.dynamic_obstacles={(2,2)}
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning,\
         patch('agv_planner.gui.app.filedialog.asksaveasfilename') as dialog:
        PathPlannerGUI.export_nav2_path(g)
    warning.assert_called_once(); dialog.assert_not_called()

@pytest.mark.parametrize('nav2',[False,True])
def test_export_reports_write_failure(tmp_path,nav2):
    g=gui()
    with patch('agv_planner.gui.app.filedialog.asksaveasfilename',return_value=str(tmp_path/'nonexistent'/'out.yaml')),\
         patch('agv_planner.gui.app.messagebox.showerror') as error:
        if nav2: PathPlannerGUI.export_nav2_path(g)
        else: PathPlannerGUI.export_path(g)
    error.assert_called_once()

def test_nav2_export_uses_fresh_yaw(tmp_path):
    g=gui(); g.robot_pose=(0,0,3.141592653589793); g._nav2_pose_fresh.return_value=True
    with patch('agv_planner.gui.app.filedialog.asksaveasfilename',return_value=str(tmp_path/'out.yaml')):
        PathPlannerGUI.export_nav2_path(g)
    data=yaml.safe_load((tmp_path/'out.yaml').read_text())
    assert data['poses'][0]['qz']==1
    assert abs(data['poses'][0]['qw'])<1e-7
