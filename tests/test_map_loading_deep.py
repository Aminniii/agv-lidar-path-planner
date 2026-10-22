"""ROS map loading integration regressions without creating a Tk window."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
import yaml
from PIL import Image

from agv_planner.gui.app import PathPlannerGUI


def gui(clearance=0, auto_start=False):
    c=Mock(); c.get.return_value=clearance
    return SimpleNamespace(stop_simulation=Mock(), trail=[(1,2)], robot_pose=(1,2,3),
        last_tf_time=3, clearance_var=c, dynamic_obstacles={(2,2)},
        refresh_dynamic_grid=Mock(), auto_start=auto_start, sync_start_to_robot=Mock(),
        root=Mock(), fit_map=Mock(), info_var=Mock(), status_var=Mock(), redraw=Mock(),
        start=(1,1), goal=(2,2), stops=[(3,3)], path=[(1,1)], visit_order=[(3,3)])


def map_files(tmp_path, pixels=None, **metadata):
    if pixels is None: pixels=np.array([[255,0,127],[255,255,0],[0,255,255]],dtype=np.uint8)
    Image.fromarray(pixels).save(tmp_path/'map.pgm')
    data=dict(image='map.pgm',resolution=.1,origin=[-2,3,.25])
    data.update(metadata)
    file=tmp_path/'map.yaml'; file.write_text(yaml.safe_dump(data))
    return file

@pytest.mark.parametrize('clearance',[0,.1,.2])
def test_load_map_metadata_and_reset(tmp_path,clearance):
    g=gui(clearance)
    PathPlannerGUI.load_map(g,map_files(tmp_path))
    assert (g.map_width,g.map_height)==(3,3)
    assert g.resolution==.1 and g.origin_x==-2 and g.origin_y==3
    assert g.origin_yaw==.25
    assert g.trail==[] and g.robot_pose is None and g.last_tf_time is None
    assert g.start is None and g.goal is None and g.stops==[]
    assert g.path is None and g.visit_order==[]
    assert not g.dynamic_obstacles
    assert g._fit_on_resize is True
    g.root.after_idle.assert_called_once()
    g.redraw.assert_called_once()
    assert g.static_inflated_grid.shape==(3,3)

@pytest.mark.parametrize('auto,mode',[(False,'start'),(True,'goal')])
def test_map_loading_selects_mode(tmp_path,auto,mode):
    g=gui(auto_start=auto)
    PathPlannerGUI.load_map(g,map_files(tmp_path))
    assert g.mode==mode
    g.sync_start_to_robot.assert_called_once()

@pytest.mark.parametrize('key',['image','resolution','origin'])
def test_missing_required_metadata(tmp_path,key):
    p=map_files(tmp_path)
    data=yaml.safe_load(p.read_text()); del data[key]; p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match=key): PathPlannerGUI.load_map(gui(),p)

@pytest.mark.parametrize('origin',[[],[1],[1,2]])
def test_short_origin_rejected(tmp_path,origin):
    with pytest.raises(ValueError,match='origin'): PathPlannerGUI.load_map(gui(),map_files(tmp_path,origin=origin))

def test_missing_yaml_rejected(tmp_path):
    with pytest.raises(FileNotFoundError,match='YAML'): PathPlannerGUI.load_map(gui(),tmp_path/'absent.yaml')

def test_missing_image_rejected(tmp_path):
    with pytest.raises(FileNotFoundError,match='image'): PathPlannerGUI.load_map(gui(),map_files(tmp_path,image='absent.pgm'))

@pytest.mark.parametrize('negate,expected',[(0,[[True,False,False],[True,True,False],[False,True,True]]),
                                           (1,[[False,True,False],[False,False,True],[True,False,False]])])
def test_trinary_occupancy_and_negate(tmp_path,negate,expected):
    g=gui(); PathPlannerGUI.load_map(g,map_files(tmp_path,negate=negate))
    assert g.free_grid.tolist()==expected
    assert g.raw_distance_m.shape==(3,3)

def test_custom_thresholds(tmp_path):
    g=gui(); PathPlannerGUI.load_map(g,map_files(tmp_path,free_thresh=.55,occupied_thresh=.9))
    assert g.free_grid[0,2] # gray becomes traversable at 0.55 threshold

def test_relative_image_resolves_from_yaml_folder(tmp_path):
    nested=tmp_path/'nested'; nested.mkdir()
    p=map_files(nested)
    g=gui(); PathPlannerGUI.load_map(g,p)
    assert g.image_path==nested/'map.pgm'

@pytest.mark.parametrize('pixels',[
    np.full((6,8),255,dtype=np.uint8),
    np.zeros((6,8),dtype=np.uint8),
    np.tile(np.array([0,255],dtype=np.uint8),(6,4)),
])
def test_map_shapes(tmp_path,pixels):
    g=gui(); PathPlannerGUI.load_map(g,map_files(tmp_path,pixels=pixels))
    assert g.free_grid.shape==pixels.shape
    assert (g.map_width,g.map_height)==(8,6)

@pytest.mark.parametrize('selection',['',None])
def test_choose_map_cancel(selection):
    g=gui()
    with patch('agv_planner.gui.app.filedialog.askopenfilename',return_value=selection):
        PathPlannerGUI.choose_map(g)
    assert g.path==[(1,1)]

def test_choose_map_success(tmp_path):
    g=gui(); g.load_map=Mock()
    with patch('agv_planner.gui.app.filedialog.askopenfilename',return_value=str(tmp_path/'m.yaml')):
        PathPlannerGUI.choose_map(g)
    g.load_map.assert_called_once_with(tmp_path/'m.yaml')

def test_choose_map_shows_load_errors(tmp_path):
    g=gui(); g.load_map=Mock(side_effect=ValueError('broken'))
    with patch('agv_planner.gui.app.filedialog.askopenfilename',return_value=str(tmp_path/'m.yaml')),\
         patch('agv_planner.gui.app.messagebox.showerror') as error:
        PathPlannerGUI.choose_map(g)
    assert 'broken' in error.call_args.args[1]
