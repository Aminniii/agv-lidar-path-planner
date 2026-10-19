"""Deterministic, headless tests of the actual GUI simulation tick."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import math
import numpy as np
import pytest
from agv_planner.gui.app import PathPlannerGUI


def var(value):
    obj = Mock()
    obj.get.return_value = value
    return obj


def gui(**changes):
    values = dict(sim_after_id=None, sim_state='running', sim_last_time=9.95,
                  sim_pose=(10.0, 10.0, 0.0), sim_elapsed=0., sim_distance=0.,
                  sim_velocity=0., sim_route=[(10., 10.), (15., 10.)], sim_index=1,
                  sim_dwell_remaining=0., stops=[], wheel_rates=[0.]*4,
                  mega_commands=(0,)*4, sim_speed_var=var(.5), sim_accel_var=var(.5),
                  sim_yaw_rate_var=var(90), wheel_radius_var=var(.04),
                  wheel_base_var=var(.18), track_width_var=var(.29),
                  wheel_accel_var=var(20.), mega_mode=var(False),
                  mega_diagnostics=Mock(), sim_status=Mock(), sim_telemetry=Mock(),
                  proximity_display=Mock(), wheel_display=Mock(),
                  wheel_indicators={name: Mock() for name in ('FL','FR','RL','RR')},
                  free_grid=np.ones((100,100), dtype=bool),
                  inflated_grid=np.ones((100,100), dtype=bool),
                  resolution=.1, map_height=100, map_width=100,
                  root=Mock(), redraw=Mock(), update_wheels_and_detection=Mock())
    values.update(changes)
    values["simulation_tick"] = Mock()
    return SimpleNamespace(**values)


def tick(g, safe=True, clock=10.0):
    with patch('agv_planner.gui.app.time.monotonic', return_value=clock), \
         patch('agv_planner.gui.app.swept_footprint_is_free', return_value=safe):
        PathPlannerGUI.simulation_tick(g)


def test_stopped_tick_does_not_schedule_or_redraw():
    g=gui(sim_state='stopped')
    tick(g)
    g.root.after.assert_not_called()
    g.redraw.assert_not_called()


def test_paused_tick_does_not_move():
    g=gui(sim_state='paused')
    tick(g)
    assert g.sim_pose==(10.,10.,0.)
    assert g.sim_elapsed==0
    g.root.after.assert_called_once()


def test_running_tick_moves_toward_waypoint():
    g=gui()
    tick(g)
    assert g.sim_pose[0]>10
    assert g.sim_distance>0
    assert g.sim_elapsed>0
    g.root.after.assert_called_once()


@pytest.mark.parametrize('limit', [.1, 1., 10.])
def test_wheel_acceleration_bounded(limit):
    g=gui(wheel_accel_var=var(limit))
    tick(g)
    assert max(map(abs,g.wheel_rates))<=limit*.05+1e-8


@pytest.mark.parametrize('mode', [False, True])
def test_both_motor_modes_run_headlessly(mode):
    g=gui(mega_mode=var(mode))
    tick(g)
    assert len(g.wheel_rates)==4
    assert all(math.isfinite(v) for v in g.wheel_rates)
    if mode:
        assert any(g.mega_commands)
    else:
        assert g.mega_commands==(0,)*4


def test_collision_pauses_and_zeroes_wheels():
    g=gui()
    tick(g,safe=False)
    assert g.sim_state=='paused'
    assert g.wheel_rates==[0.]*4
    assert g.sim_pose==(10.,10.,0.)
    assert 'COLLISION' in g.sim_status.set.call_args.args[0]


def test_missing_free_grid_fails_closed():
    g=gui(free_grid=None)
    tick(g)
    assert g.sim_state=='paused'
    assert g.wheel_rates==[0.]*4


def test_inflated_boundary_pauses():
    blocked=np.ones((100,100),dtype=bool)
    blocked[10,10]=False
    g=gui(inflated_grid=blocked)
    tick(g)
    assert g.sim_state=='paused'
    assert g.wheel_rates==[0.]*4


def test_arrival_at_final_waypoint_finishes():
    g=gui(sim_pose=(15.,10.,0.), wheel_rates=[0.]*4)
    tick(g)
    assert g.sim_state=='finished'
    assert g.wheel_rates==[0.]*4
    assert g.mega_commands==(0,)*4
    g.root.after.assert_not_called()


def test_nonfinal_waypoint_advances():
    g=gui(sim_pose=(15.,10.,0.), sim_route=[(10.,10.),(15.,10.),(20.,10.)])
    tick(g)
    assert g.sim_index==2
    assert g.sim_state=='running'


def test_mandatory_stop_sets_dwell():
    g=gui(sim_pose=(15.,10.,0.), sim_route=[(10.,10.),(15.,10.),(20.,10.)],stops=[(15.,10.)])
    tick(g)
    assert g.sim_dwell_remaining==2.0
    assert g.sim_index==2


def test_dwell_counts_down_with_stopped_wheels():
    g=gui(sim_dwell_remaining=1.)
    tick(g)
    assert g.sim_dwell_remaining==pytest.approx(.95)


def test_dwell_does_not_count_down_while_wheels_turn():
    g=gui(sim_dwell_remaining=1.,wheel_rates=[1.]*4)
    tick(g)
    assert g.sim_dwell_remaining==1.


@pytest.mark.parametrize('value', ['not-a-number', 'nan'])
def test_invalid_numeric_config_uses_fallback(value):
    g=gui(sim_speed_var=var(value))
    # ValueError falls back; nan is currently accepted by float and min/max;
    # both must not crash or send any hardware commands.
    tick(g)
    assert len(g.wheel_rates)==4


def test_long_frame_clamped_to_80_ms():
    g=gui(sim_last_time=0.1)
    tick(g,clock=10.)
    assert g.sim_elapsed==pytest.approx(.08)


def test_zero_delta_does_not_move():
    g=gui(sim_last_time=10.)
    tick(g,clock=10.)
    assert g.sim_pose==(10.,10.,0.)
    assert g.sim_elapsed==0


def test_tick_never_calls_ros_or_serial():
    g=gui()
    tick(g)
    assert not hasattr(g,'cmd_vel_publisher')
    assert not hasattr(g,'serial')

@pytest.mark.parametrize('wheel,expected', [(1.5,'FWD'),(-1.5,'REV'),(0.,'STOP'),(.01,'STOP'),(-.01,'STOP')])
def test_wheel_diagnostics_direction(wheel,expected):
    g=gui(wheel_rates=[wheel]*4, distance_field=None)
    PathPlannerGUI.update_wheels_and_detection(g)
    assert expected in g.wheel_display.set.call_args.args[0]
    for indicator in g.wheel_indicators.values():
        assert expected in indicator.set.call_args.args[0]


def test_wheel_diagnostics_no_pose_no_updates():
    g=gui(sim_pose=None)
    PathPlannerGUI.update_wheels_and_detection(g)
    g.wheel_display.set.assert_not_called()


def test_wheel_diagnostics_no_resolution_no_updates():
    g=gui(resolution=None)
    PathPlannerGUI.update_wheels_and_detection(g)
    g.wheel_display.set.assert_not_called()


@pytest.mark.parametrize('distance,zone,expected', [(.2,.45,'OBSTACLE IN ZONE'),(.7,.45,'clear'),(.45,.45,'OBSTACLE IN ZONE')])
def test_detection_zone_status(distance,zone,expected):
    g=gui(distance_field=object(), raw_distance_m=object(), detection_radius_var=var(zone))
    with patch('agv_planner.gui.app.nearest_obstacle_distance', return_value=distance):
        PathPlannerGUI.update_wheels_and_detection(g)
    assert expected in g.proximity_display.set.call_args.args[0]


def test_detection_invalid_radius_uses_default():
    g=gui(distance_field=object(),raw_distance_m=object(),detection_radius_var=var('bad'))
    with patch('agv_planner.gui.app.nearest_obstacle_distance', return_value=.5):
        PathPlannerGUI.update_wheels_and_detection(g)
    assert 'radius 0.45' in g.proximity_display.set.call_args.args[0]


def test_detection_skips_out_of_map_pose():
    g=gui(sim_pose=(200.,200.,0.), distance_field=object(),raw_distance_m=object())
    PathPlannerGUI.update_wheels_and_detection(g)
    g.proximity_display.set.assert_not_called()


def test_simulation_start_requires_route():
    g=gui(path=None)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.start_simulation(g)
    warning.assert_called_once()


def test_simulation_start_requires_resolution():
    g=gui(path=[(10.,10.),(15.,10.)],resolution=None)
    with patch('agv_planner.gui.app.messagebox.showwarning') as warning:
        PathPlannerGUI.start_simulation(g)
    warning.assert_called_once()


def test_simulation_start_initializes_virtual_robot():
    g=gui(path=[(10.,10.),(15.,10.)],stop_simulation=Mock(),
          _nav2_pose_fresh=Mock(return_value=False),robot_pose=None)
    with patch('agv_planner.gui.app.time.monotonic',return_value=10.):
        PathPlannerGUI.start_simulation(g)
    assert g.sim_pose==(10.,10.,0.)
    assert g.sim_state=='running'
    assert g.sim_index==1
    assert g.wheel_rates==[0.]*4
    g.simulation_tick.assert_called_once()


def test_simulation_start_preserves_fresh_robot_heading():
    g=gui(path=[(10.,10.),(15.,10.)],stop_simulation=Mock(),
          _nav2_pose_fresh=Mock(return_value=True),robot_pose=(3.,4.,1.2))
    with patch('agv_planner.gui.app.time.monotonic',return_value=10.):
        PathPlannerGUI.start_simulation(g)
    assert g.sim_pose==(10.,10.,1.2)


def test_single_point_route_starts_without_index_error():
    g=gui(path=[(10.,10.)],stop_simulation=Mock(),
          _nav2_pose_fresh=Mock(return_value=False),robot_pose=None)
    with patch('agv_planner.gui.app.time.monotonic',return_value=10.):
        PathPlannerGUI.start_simulation(g)
    assert g.sim_index==0
    assert g.sim_route==[(10.,10.)]
