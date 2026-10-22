"""Conservative continuous rectangular-footprint collision checks for offline simulation."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def obstacle_distance_field(free_grid, resolution):
    if resolution <= 0 or not math.isfinite(resolution):
        raise ValueError("Resolution must be positive")
    free = np.asarray(free_grid, dtype=bool)
    padded = np.pad(free, 1, constant_values=False)
    return distance_transform_edt(padded)[1:-1, 1:-1] * resolution

def nearest_obstacle_distance(field, x, y):
    ix, iy = round(x), round(y)
    if iy < 0 or ix < 0 or iy >= field.shape[0] or ix >= field.shape[1]:
        return 0.0
    return max(0.0, float(field[iy, ix]))

def footprint_is_free(free_grid, x, y, yaw, length_m, width_m, resolution):
    """Check all grid-cell centers within an oriented rectangular footprint.

    Cells touching its perimeter are included conservatively; unknown cells blocked.
    """
    if min(length_m, width_m, resolution) <= 0:
        raise ValueError("Robot dimensions and map resolution must be positive")
    grid = np.asarray(free_grid, dtype=bool)
    half_l, half_w = length_m / (2*resolution), width_m / (2*resolution)
    extent = math.hypot(half_l, half_w) + 1.0
    xmin, xmax = math.floor(x-extent), math.ceil(x+extent)
    ymin, ymax = math.floor(y-extent), math.ceil(y+extent)
    if xmin < 0 or ymin < 0 or xmax >= grid.shape[1] or ymax >= grid.shape[0]:
        return False
    yy, xx = np.mgrid[ymin:ymax+1, xmin:xmax+1]
    dx, dy = xx-x, -(yy-y)
    c, s = math.cos(yaw), math.sin(yaw)
    forward = c*dx+s*dy
    lateral = -s*dx+c*dy
    # Half-cell inflation avoids treating a barely touching obstacle as clear.
    inside = (np.abs(forward) <= half_l + .71) & (np.abs(lateral) <= half_w + .71)
    return bool(np.all(grid[ymin:ymax+1,xmin:xmax+1][inside]))

def swept_footprint_is_free(grid, start, end, length_m, width_m, resolution):
    """Check interpolated poses, including intermediate rotation and translation."""
    x0,y0,a0=start; x1,y1,a1=end
    da=math.atan2(math.sin(a1-a0),math.cos(a1-a0))
    distance=math.hypot(x1-x0,y1-y0)
    steps=max(1,math.ceil(max(distance/.5,abs(da)/math.radians(3))))
    for i in range(steps+1):
        t=i/steps
        if not footprint_is_free(grid,x0+t*(x1-x0),y0+t*(y1-y0),a0+t*da,length_m,width_m,resolution):
            return False
    return True
