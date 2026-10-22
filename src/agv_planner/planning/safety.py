"""Conservative occupancy-grid route verification (unknown cells are blocked)."""
import math

import numpy as np

from .algorithms import clear_segment


class UnsafePathError(ValueError):
    pass

def footprint_radius(length_m, width_m):
    if length_m <= 0 or width_m <= 0:
        raise ValueError('Footprint dimensions must be positive')
    return math.hypot(length_m, width_m) / 2

def validate_route(path, grid, *, start=None, goal=None, required=()):
    if not path or len(path) < 2:
        raise UnsafePathError('Route must have at least two points')
    if grid.ndim != 2 or grid.size == 0:
        raise UnsafePathError('Invalid occupancy grid')
    if start is not None and math.dist(path[0],start) > 1e-6:
        raise UnsafePathError('Route start changed')
    if goal is not None and math.dist(path[-1],goal) > 1e-6:
        raise UnsafePathError('Route goal changed')
    for stop in required:
        if not any(math.dist(stop,point) < 1e-6 for point in path):
            raise UnsafePathError(f'Mandatory stop missing: {stop}')
    if not all(all(math.isfinite(v) for v in point) for point in path):
        raise UnsafePathError('Nonfinite coordinate')
    for a,b in zip(path,path[1:]):
        if not clear_segment(grid,a,b):
            raise UnsafePathError(f'Collision or map boundary between {a} and {b}')
    return True

def validate_footprint_clearance(clearance_m, length_m, width_m):
    minimum = footprint_radius(length_m,width_m)
    if clearance_m < minimum:
        raise UnsafePathError(f'Clearance {clearance_m:.3f} m is less than footprint bounding radius {minimum:.3f} m')
    return True
