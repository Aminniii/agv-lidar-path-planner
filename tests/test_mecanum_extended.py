"""Deterministic offline tests of the existing mecanum and Mega command math."""
import math

import numpy as np
import pytest

from agv_planner.simulation.kinematics import (
    angle_difference,
    approach,
    mecanum_body_velocity,
    mecanum_wheel_rates,
    mega_bridge_mix,
    virtual_mega_wheel_targets,
)

RADIUS = 0.04
LEVER = 0.235

@pytest.mark.parametrize('velocity', [
    (0., 0., 0.), (0.3, 0., 0.), (-0.2, 0., 0.),
    (0., 0.2, 0.), (0., -0.2, 0.),
    (0., 0., 0.6), (0., 0., -0.6),
    (0.25, -0.12, 0.4), (-0.1, 0.2, -0.3),
])
def test_wheel_inverse_roundtrip(velocity):
    wheel_rates = mecanum_wheel_rates(*velocity, RADIUS, LEVER)
    assert np.allclose(mecanum_body_velocity(wheel_rates, RADIUS, LEVER), velocity, atol=1e-12)

@pytest.mark.parametrize('velocity,expected', [
    ((0., 0., 0.), (0, 0, 0, 0)),
    ((0.1, 0., 0.), (350, 350, 350, 350)),
    ((0., 0.1, 0.), (-350, 350, 350, -350)),
    ((0., 0., 1.), (-700, 700, -700, 700)),
])
def test_mega_bridge_direction_signs(velocity, expected):
    assert mega_bridge_mix(*velocity) == expected

@pytest.mark.parametrize('velocity', [(2., 0., 0.), (0., 2., 0.), (0., 0., 4.), (3., -2., 5.)])
def test_mega_bridge_saturates_proportionally(velocity):
    commands = mega_bridge_mix(*velocity)
    assert max(map(abs, commands)) == 2000
    assert all(isinstance(command, int) for command in commands)

@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_invalid_command_is_zero(bad):
    assert mega_bridge_mix(bad, 0., 0.) == (0, 0, 0, 0)
    assert mega_bridge_mix(0., bad, 0.) == (0, 0, 0, 0)
    assert mega_bridge_mix(0., 0., bad) == (0, 0, 0, 0)

def test_virtual_motor_targets_nominal_scaling():
    commands = (350, -350, 700, -700)
    assert np.allclose(virtual_mega_wheel_targets(commands, RADIUS), (2.5, -2.5, 5., -5.))

def test_angle_difference_wraparound():
    assert angle_difference(-math.pi + 0.1, math.pi - 0.1) == pytest.approx(0.2)
    assert angle_difference(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)
    assert angle_difference(0., 0.) == pytest.approx(0.)

@pytest.mark.parametrize('current,target,limit,expected', [
    (0., 10., 2., 2.), (10., 0., 3., 7.),
    (1., 1.5, 2., 1.5), (-2., -2., 1., -2.),
])
def test_approach_rate_limit(current, target, limit, expected):
    assert approach(current, target, limit) == pytest.approx(expected)
