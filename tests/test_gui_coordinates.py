"""Headless tests for coordinate conversions in the existing GUI class."""
import math
from types import SimpleNamespace

import pytest

from agv_planner.gui.app import PathPlannerGUI


@pytest.mark.parametrize('yaw', [0.0, math.pi/2, -math.pi/3, math.pi])
@pytest.mark.parametrize('pixel', [(0, 0), (4, 8), (39, 29), (19, 15)])
def test_map_pixel_world_roundtrip(yaw, pixel):
    gui = object.__new__(PathPlannerGUI)
    gui.resolution = 0.05
    gui.origin_x, gui.origin_y, gui.origin_yaw = -2.3, 4.7, yaw
    gui.map_width, gui.map_height = 40, 30
    world = gui.pixel_to_world(*pixel)
    assert gui.world_to_pixel(*world) == pixel


def test_world_to_pixel_without_map_resolution():
    gui = object.__new__(PathPlannerGUI)
    gui.resolution = None
    assert gui.world_to_pixel(0, 0) is None


def test_world_to_pixel_rejects_outside_map():
    gui = object.__new__(PathPlannerGUI)
    gui.resolution = 0.1
    gui.origin_x = gui.origin_y = gui.origin_yaw = 0.0
    gui.map_width = gui.map_height = 10
    assert gui.world_to_pixel(-0.2, 0.0) is None
    assert gui.world_to_pixel(1.5, 0.0) is None
    assert gui.world_to_pixel(0.0, -0.2) is None
    assert gui.world_to_pixel(0.0, 1.5) is None
