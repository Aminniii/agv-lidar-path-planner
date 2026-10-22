"""ROS-independent occupancy-grid planning and mecanum math."""
import heapq
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

MEGA_TRANSLATION_GAIN = 3500.0
MEGA_ROTATION_GAIN = 700.0
MEGA_COMMAND_LIMIT = 2000.0

def heuristic(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1]
    )


def astar(grid, start, goal):

    height, width = grid.shape

    neighbours = [
        (-1,  0, 1.0),
        ( 1,  0, 1.0),
        ( 0, -1, 1.0),
        ( 0,  1, 1.0),

        (-1, -1, math.sqrt(2)),
        (-1,  1, math.sqrt(2)),
        ( 1, -1, math.sqrt(2)),
        ( 1,  1, math.sqrt(2)),
    ]

    open_heap = []

    heapq.heappush(
        open_heap,
        (
            heuristic(start, goal),
            0.0,
            start
        )
    )

    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:

        _, current_g, current = heapq.heappop(
            open_heap
        )

        if current in closed:
            continue

        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()

            return path

        closed.add(current)

        cx, cy = current

        for dx, dy, cost in neighbours:

            nx = cx + dx
            ny = cy + dy

            if not (
                0 <= nx < width
                and
                0 <= ny < height
            ):
                continue

            if not grid[ny, nx]:
                continue

            # Prevent diagonal corner cutting.
            if dx != 0 and dy != 0:

                if not grid[cy, nx]:
                    continue

                if not grid[ny, cx]:
                    continue

            neighbour = (nx, ny)

            tentative_g = current_g + cost

            if tentative_g < g_score.get(
                neighbour,
                float("inf")
            ):

                came_from[neighbour] = current
                g_score[neighbour] = tentative_g

                f_score = (
                    tentative_g
                    + heuristic(neighbour, goal)
                )

                heapq.heappush(
                    open_heap,
                    (
                        f_score,
                        tentative_g,
                        neighbour
                    )
                )

    return None


def clear_segment(grid, a, b):
    """Conservative segment test on the already-inflated free-space grid.

    Sample at <= 0.25 pixel increments, checking the nearby 2x2 cells.
    This intentionally rejects shortcuts that graze unknown/blocked cells.
    """
    height, width = grid.shape
    dx, dy = b[0] - a[0], b[1] - a[1]
    steps = max(1, math.ceil(math.hypot(dx, dy) * 4))
    for i in range(steps + 1):
        x = a[0] + dx * i / steps
        y = a[1] + dy * i / steps
        # Map cells are centred at integer pixel coordinates.
        # Check cells on either side of boundaries as a conservative guard.
        x0, y0 = math.floor(x), math.floor(y)
        x1, y1 = math.ceil(x), math.ceil(y)
        for xx in (x0, x1):
            for yy in (y0, y1):
                if not (0 <= xx < width and 0 <= yy < height and grid[yy, xx]):
                    return False
    return True


def simplify_path(grid, path):
    """Greedy farthest-visible waypoint selection; keep start and goal."""
    if len(path) <= 2:
        return path[:]
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        # Walk along the original safe A* route; never skip around a blockage.
        next_idx = anchor + 1
        for candidate in range(anchor + 2, len(path)):
            if clear_segment(grid, path[anchor], path[candidate]):
                next_idx = candidate
            else:
                break
        result.append(path[next_idx])
        anchor = next_idx
    return result


def inflate_free_grid(free_grid, radius_pixels):

    if radius_pixels <= 0:
        return free_grid.copy()

    # distance_transform_edt calculates the distance of every
    # free pixel to its nearest obstacle.
    #
    # This is dramatically faster than looping around every
    # obstacle pixel, especially for 0.01 m/pixel maps.

    distance_to_obstacle = distance_transform_edt(free_grid)

    inflated_free_grid = (
        distance_to_obstacle > radius_pixels
    )

    return inflated_free_grid


def search_path(grid, start, goal, algorithm="A*"):
    """Grid A*, Dijkstra, or any-angle Theta* (on inflated free-space)."""
    if algorithm == "A*":
        return astar(grid, start, goal)
    h, w = grid.shape
    if not (0 <= start[0] < w and 0 <= start[1] < h and grid[start[1], start[0]]
            and 0 <= goal[0] < w and 0 <= goal[1] < h and grid[goal[1], goal[0]]):
        return None
    offsets = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1)
               for dy in (-1, 0, 1) if dx or dy]
    scores = {start: 0.0}
    parents = {start: start}
    heap = [(0.0 if algorithm == "Dijkstra" else heuristic(start, goal), start)]
    closed = set()
    while heap:
        _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal:
            result = [cur]
            while cur != start:
                cur = parents[cur]
                result.append(cur)
            return result[::-1]
        closed.add(cur)
        x, y = cur
        for dx, dy, cost in offsets:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h and grid[ny, nx]):
                continue
            if dx and dy and (not grid[y, nx] or not grid[ny, x]):
                continue
            nxt = (nx, ny)
            if nxt in closed:
                continue
            parent = cur
            tentative = scores[cur] + cost
            if algorithm == "Theta*" and parents[cur] != cur:
                candidate = parents[cur]
                if clear_segment(grid, candidate, nxt):
                    candidate_cost = scores[candidate] + heuristic(candidate, nxt)
                    if candidate_cost < tentative:
                        parent, tentative = candidate, candidate_cost
            if tentative < scores.get(nxt, float("inf")):
                scores[nxt] = tentative
                parents[nxt] = parent
                heapq.heappush(heap, (tentative + (0 if algorithm == "Dijkstra" else heuristic(nxt, goal)), nxt))
    return None


def optimized_search(grid, start, goal, algorithm, distance_field, safety_weight):
    """Clearance-penalized A*/Dijkstra; Theta* uses its original any-angle search."""
    if safety_weight <= 0 or algorithm == "Theta*":
        return search_path(grid, start, goal, algorithm)
    h, w = grid.shape
    if not (0 <= start[0] < w and 0 <= start[1] < h and grid[start[1], start[0]]
            and 0 <= goal[0] < w and 0 <= goal[1] < h and grid[goal[1], goal[0]]):
        return None
    costs = {start: 0.0}
    parents = {}
    queue = [(heuristic(start, goal), 0.0, start)]
    visited = set()
    directions = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1)
                  for dy in (-1, 0, 1) if dx or dy]
    while queue:
        _, cost, cur = heapq.heappop(queue)
        if cur in visited:
            continue
        if cur == goal:
            path = [cur]
            while cur in parents:
                cur = parents[cur]
                path.append(cur)
            return path[::-1]
        visited.add(cur)
        x, y = cur
        for dx, dy, step in directions:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h and grid[ny, nx]):
                continue
            if dx and dy and (not grid[y, nx] or not grid[ny, x]):
                continue
            nxt = (nx, ny)
            # Penalize routes close to obstacles, without treating them as blocked.
            clearance_px = max(0.01, distance_field[ny, nx])
            penalty = safety_weight * step / (1.0 + clearance_px / 10.0)
            new_cost = cost + step + penalty
            if new_cost < costs.get(nxt, float('inf')):
                costs[nxt] = new_cost
                parents[nxt] = cur
                heuristic_cost = 0 if algorithm == "Dijkstra" else heuristic(nxt, goal)
                heapq.heappush(queue, (new_cost + heuristic_cost, new_cost, nxt))
    return None


def compact_path(path, free_grid, required=(), max_segment_px=35):
    """Bounded collision-safe simplification, preserving all mandatory stops.

    Unlike the old greedy simplifier, a candidate is accepted only if its
    entire straight line is free; small chunks limit path deviation.
    """
    if len(path) < 3:
        return list(path)
    required = set(required)
    out = [path[0]]
    i = 0
    while i < len(path)-1:
        furthest = i+1
        j = i+1
        while j < len(path):
            if j > i+1 and path[j-1] in required:
                break
            if heuristic(path[i], path[j]) > max_segment_px:
                break
            if clear_segment(free_grid, path[i], path[j]):
                furthest = j
            j += 1
        out.append(path[furthest])
        i = furthest
    return out


def smooth_corners(path, free_grid, radius_px, required=()):
    """Bounded quadratic rounding with complete collision checks."""
    if not path or len(path) < 3 or radius_px <= 0:
        return list(path)
    required = set(required)
    out = [path[0]]
    for i in range(1, len(path)-1):
        a, b, c = path[i-1], path[i], path[i+1]
        ab, bc = heuristic(a,b), heuristic(b,c)
        if b in required or min(ab,bc) < 1e-6:
            out.append(b)
            continue
        # Don't create curves for almost straight segments.
        ux, uy = (b[0]-a[0])/ab, (b[1]-a[1])/ab
        vx, vy = (c[0]-b[0])/bc, (c[1]-b[1])/bc
        if ux*vx + uy*vy > 0.995:
            out.append(b)
            continue
        cut = min(radius_px, 0.35*ab, 0.35*bc)
        p = (b[0]-ux*cut, b[1]-uy*cut)
        q = (b[0]+vx*cut, b[1]+vy*cut)
        # Approximately one sample per 3 pixels, maximum 12 samples.
        samples = max(3, min(12, int(math.ceil(2*cut/3))))
        curve = [((1-t)**2*p[0]+2*(1-t)*t*b[0]+t*t*q[0],
                  (1-t)**2*p[1]+2*(1-t)*t*b[1]+t*t*q[1])
                 for t in (j/samples for j in range(samples+1))]
        chain = [out[-1]] + curve + [c]
        if all(clear_segment(free_grid, chain[k], chain[k+1])
               for k in range(len(chain)-1)):
            out.extend(curve)
        else:
            out.append(b)
    out.append(path[-1])
    return out


def adaptive_reduce_path(path, free_grid, required=(), tolerance_px=1.5,
                         max_spacing_px=28.0):
    """Remove redundant curve samples while retaining safe, accurately traced bends.

    Uses error-bounded Ramer-Douglas-Peucker separately between required stops,
    followed by bounded segment spacing. Every replacement chord must be free
    on the inflated occupancy grid. The endpoints and stop coordinates survive.
    """
    points = list(path)
    if len(points) <= 2:
        return points
    mandatory = set(required)
    fixed = [0] + [i for i in range(1, len(points)-1)
                   if points[i] in mandatory] + [len(points)-1]
    fixed = sorted(set(fixed))

    def deviation(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        d2 = dx*dx + dy*dy
        if d2 < 1e-12:
            return math.hypot(p[0]-a[0], p[1]-a[1])
        t = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d2))
        return math.hypot(p[0]-(a[0]+t*dx), p[1]-(a[1]+t*dy))

    def reduce_section(lo, hi):
        if hi <= lo + 1:
            return [lo, hi]
        a, b = points[lo], points[hi]
        far = max(range(lo+1, hi), key=lambda k: deviation(points[k], a, b))
        error = deviation(points[far], a, b)
        if error <= tolerance_px and clear_segment(free_grid, a, b):
            return [lo, hi]
        left = reduce_section(lo, far)
        right = reduce_section(far, hi)
        return left[:-1] + right

    indices = []
    for lo, hi in zip(fixed, fixed[1:]):
        section = reduce_section(lo, hi)
        indices.extend(section if not indices else section[1:])
    reduced = [points[i] for i in indices]
    result = [reduced[0]]
    for a, b in zip(reduced, reduced[1:]):
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        divisions = max(1, math.ceil(distance / max(1.0, max_spacing_px)))
        for j in range(1, divisions+1):
            if j == divisions:
                result.append(b)  # exact mandatory stop, if present
            else:
                t = j/divisions
                result.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
    return result


def mega_bridge_mix(vx, vy, omega):
    """Return FL, FR, RL, RR Arduino units, matching mega_bridge_node.py."""
    if not all(math.isfinite(v) for v in (vx, vy, omega)):
        return (0, 0, 0, 0)
    x, y, z = (vx * MEGA_TRANSLATION_GAIN,
               vy * MEGA_TRANSLATION_GAIN,
               omega * MEGA_ROTATION_GAIN)
    wheels = [x-y-z, x+y+z, x+y-z, x-y+z]
    peak = max(abs(w) for w in wheels)
    if peak > MEGA_COMMAND_LIMIT:
        wheels = [w * MEGA_COMMAND_LIMIT / peak for w in wheels]
    return tuple(int(round(w)) for w in wheels)


def virtual_mega_wheel_targets(commands, radius):
    """Uncalibrated nominal motor model; assumes 3500 units per m/s of wheel rim speed.

    This is a placeholder, NOT an observed Arduino motor response.
    """
    return tuple(c / (MEGA_TRANSLATION_GAIN * radius) for c in commands)


def mecanum_body_velocity(wheels, wheel_radius, lever):
    """Inverse of mecanum_wheel_rates; FL, FR, RL, RR (rad/s)."""
    fl, fr, rl, rr = wheels
    r = wheel_radius / 4.0
    return (r * (fl + fr + rl + rr),
            r * (-fl + fr + rl - rr),
            r * (-fl + fr - rl + rr) / lever)


def angle_difference(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


def approach(current, target, limit):
    return current + max(-limit, min(limit, target-current))


def mecanum_wheel_rates(vx, vy, omega, wheel_radius, lever):
    """FL, FR, RL, RR wheel angular velocities (rad/s), X configuration.

    Body x forward, y left, omega positive CCW.
    """
    return ((vx-vy-lever*omega)/wheel_radius,
            (vx+vy+lever*omega)/wheel_radius,
            (vx+vy-lever*omega)/wheel_radius,
            (vx-vy+lever*omega)/wheel_radius)


