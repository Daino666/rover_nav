#!/usr/bin/env python3
"""Global path planning for pure pursuit -- no ROS or hardware dependencies.

rover_nav's `omar` branch plans with a real Nav2 stack (map_server ->
global_costmap -> planner_server[SmacPlanner2D] -> smoother_server
[SimpleSmoother]) so pure pursuit tracks a dense path instead of jumping
goal-to-goal. This repo has no occupancy map, so there is no costmap to
search against -- but SmacPlanner2D's shortest path through open space is
just the straight line between waypoints, which is the degenerate case of
what hermite_leg() below produces.

Nav2's SimpleSmoother pass is deliberately NOT reproduced here: it rounds
corners through interior waypoints to make one continuous curve, which
cuts inside a waypoint enough that a rover following it may never get
within GOAL_TOLERANCE. This rover instead physically stops at each
waypoint (see rover_controller_pure_pursuit.py) before continuing, so
generate_waypoint_legs() plans one leg per waypoint, each pinned exactly
at both ends -- but shaped as a cubic Hermite spline (tangent to the
rover's current heading at the start, tangent to the straight-line
bearing at the end) rather than a hard-angled straight line, so pure
pursuit eases out of each stop instead of demanding an immediate sharp
lateral correction.

Kept dependency-free (stdlib only) so it can be imported for offline
visualization/testing without rclpy or the odrive_can hardware stack.
"""

import math

GLOBAL_RESOLUTION = 0.1  # global_costmap.resolution -- spacing between points on a leg

# Edit these to change the route. START/START_HEADING stand in for the
# rover's pose when it isn't coming from live odometry
# (rover_controller_pure_pursuit.py uses its actual position and heading
# instead once running; the offline CLI/RViz tools below use these as-is).
# WAYPOINTS are goal points to visit in order, in the map frame -- the
# competition's own axes, defined by its two given reference points, NOT
# this rover's local odom frame (which just starts wherever it happens to
# boot facing -- see MAP_TO_ODOM_* below for how those two get reconciled).
# The rover stops at each waypoint before continuing to the next.
START         = [0.0, 0.0]
START_HEADING = 0.0  # radians, 0 = facing +X
WAYPOINTS     = [[2.0, 3.0], [4.0, 2.0], [5.0, 1.0], [4.0, -2.0], [2.0, -3.0]]

# map -> odom alignment correction: how this rover's local odom frame sits
# relative to the competition's map frame (the frame WAYPOINTS above are
# expressed in). Published once at startup by map_odom_broadcaster.py.
# Defaults to identity (rover assumed to power on exactly at the
# competition's origin point, facing its +X axis) since the actual
# competition-day alignment procedure -- how these three numbers actually
# get determined from the two given reference points -- is still TBD; there
# is no absolute-position sensor (GPS/RTK/etc) on this rover to compute it
# automatically. Edit these directly before a run, same workflow as
# WAYPOINTS above, once the real numbers are known.
MAP_TO_ODOM_X       = 0.0  # metres
MAP_TO_ODOM_Y       = 0.0  # metres
MAP_TO_ODOM_YAW_DEG = 0.0  # degrees


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def hermite_leg(start, start_heading, target, resolution=GLOBAL_RESOLUTION):
    """Cubic Hermite spline from `start` to `target`, tangent to
    `start_heading` at the start and tangent to the straight-line bearing
    into `target` at the end. Both endpoints are exact (t=0 and t=1 pin
    them precisely), so the leg always lands on the waypoint -- but unlike
    a raw straight line, the path curves smoothly out of whatever
    direction the rover is currently facing instead of requiring an
    instant pivot. When start_heading already points straight at target,
    this reduces to the straight line (both tangents collinear with the
    chord)."""
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    chord = math.hypot(dx, dy)
    if chord < 1e-6:
        return [list(start)]

    bearing = math.atan2(dy, dx)
    m0 = [math.cos(start_heading) * chord, math.sin(start_heading) * chord]
    m1 = [math.cos(bearing) * chord, math.sin(bearing) * chord]

    n_samples = max(int(math.ceil(chord / resolution)), 2)
    pts = []
    for i in range(n_samples + 1):
        t = i / n_samples
        t2 = t * t
        t3 = t2 * t
        h00 = 2 * t3 - 3 * t2 + 1
        h10 = t3 - 2 * t2 + t
        h01 = -2 * t3 + 3 * t2
        h11 = t3 - t2
        x = h00 * start[0] + h10 * m0[0] + h01 * target[0] + h11 * m1[0]
        y = h00 * start[1] + h10 * m0[1] + h01 * target[1] + h11 * m1[1]
        pts.append([x, y])
    pts[0] = list(start)
    pts[-1] = list(target)
    return pts


def generate_waypoint_legs(start, waypoints, start_heading=None, resolution=GLOBAL_RESOLUTION):
    """One dense curved leg per waypoint: leg[i] eases from wherever the
    rover was after the previous stop (start, for leg 0) into waypoints[i],
    landing exactly on it. Each leg's start heading is the previous leg's
    arrival bearing (i.e. however the rover ended up facing), chained from
    `start_heading` (default START_HEADING) for leg 0. Returns a list of
    legs (each a list of [x, y] points) -- drive leg 0 to completion, stop,
    then leg 1, and so on, rather than following one continuous path."""
    if start_heading is None:
        start_heading = START_HEADING
    legs = []
    prev = list(start)
    heading = start_heading
    for wp in waypoints:
        legs.append(hermite_leg(prev, heading, wp, resolution))
        dx = wp[0] - prev[0]
        dy = wp[1] - prev[1]
        if math.hypot(dx, dy) > 1e-6:
            heading = math.atan2(dy, dx)
        prev = wp
    return legs
