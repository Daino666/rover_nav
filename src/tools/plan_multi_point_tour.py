#!/usr/bin/env python3
"""
Copy of Mars-rover/tools/plan_multi_point_tour.py, adapted for this
workspace's layout (paths point at rover_nav/maps/ directly instead of a
separate sibling marsyard/ directory) and changed so --start accepts a raw
"x,y" coordinate directly, not just a named survey point.

Give it a list of named points (or raw (x, y) tuples); it orders them
nearest-first (greedy nearest-neighbor) starting from START, optionally
returns to START at the end, and asks the real Nav2 stack (planner_server +
smoother_server, already running via
`ros2 launch rover_nav nav2_planning_real.launch.py`) for ONE continuous
smoothed path through all of them -- not a separate re-plan per leg.

HOW TO USE: make sure the Nav2 planning stack is running first:
  ros2 launch rover_nav nav2_planning_real.launch.py
Then:
  python3 plan_multi_point_tour.py --start 3,2 --points W6 W5 W7 W8 --use-sim-time false

START and POINTS can be survey point names ("W3", "L5", ...) or raw "x,y"
strings, mixed freely.
"""

import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

# ============================================================================
# PARAMETERS -- edit these for a different default run, or override on the
# command line (see HOW TO USE above / --help).
# ============================================================================

POINTS = ["W6", "W5", "W7", "W8"]
# The points to visit, in ANY order -- they'll be automatically sorted
# nearest-first starting from the robot's current position. Names or raw
# "x,y" strings, mixed OK.

RETURN_TO_START = True
# If True, appends the start point back onto the end of the route after the
# last point, so the final path is a full loop.

# ============================================================================
# End of parameters. Nothing below this line needs editing for normal use.
# ============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROVER_NAV_MAPS = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "rover_nav", "maps"))

DEFAULT_COORDS = os.path.join(
    ROVER_NAV_MAPS, "2026_MarsYard_3D_Model-20260812T165935Z-1-001",
    "2026_MarsYard_3D_Model", "Coordinates_MarsYard2026.txt",
)
OUT_WAYPOINTS = os.path.join(ROVER_NAV_MAPS, "marsyard2026_tour_waypoints.csv")
OUT_VIZ = os.path.join(ROVER_NAV_MAPS, "marsyard2026_tour_preview.png")
GROUP_COLOR = {"S": "#e6194B", "L": "#3b82f6", "W": "#f58231", "P": "#a020f0"}


def load_survey_points(path):
    points = {}
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split("\t") if p.strip() != ""]
            if len(parts) < 4:
                continue
            name = parts[0]
            if name.lower().startswith("point"):
                continue
            try:
                y, x, h = (float(v.replace(",", ".")) for v in parts[1:4])
            except ValueError:
                continue
            points[name] = (x, y)
    return points


def resolve(spec, survey_points):
    if "," in spec:
        x_str, y_str = spec.split(",")
        xy = (float(x_str), float(y_str))
        return f"({xy[0]:.2f},{xy[1]:.2f})", xy
    if spec not in survey_points:
        raise ValueError(f"'{spec}' not found in survey points ({sorted(survey_points)})")
    return spec, survey_points[spec]


def nearest_neighbor_order(start_xy, remaining):
    """remaining: list of (label, (x,y)). Returns the same items reordered
    greedily nearest-first from start_xy."""
    ordered = []
    pool = list(remaining)
    current = start_xy
    while pool:
        dists = [np.hypot(xy[0] - current[0], xy[1] - current[1]) for _, xy in pool]
        i = int(np.argmin(dists))
        label, xy = pool.pop(i)
        ordered.append((label, xy))
        current = xy
    return ordered


def to_pose(xy):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x = float(xy[0])
    p.pose.position.y = float(xy[1])
    p.pose.orientation.w = 1.0
    return p


def save_preview(route_labels, route_xy, path_world, survey_points, out_viz):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image
    import yaml as _yaml

    yaml_path = os.path.join(ROVER_NAV_MAPS, "marsyard2026_occupancy.yaml")
    with open(yaml_path) as f:
        meta = _yaml.safe_load(f)
    pgm = np.array(Image.open(os.path.join(ROVER_NAV_MAPS, meta["image"])))
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    H, W = pgm.shape
    xmax, ymax = ox + (W - 1) * res, oy + (H - 1) * res
    occupied = (1.0 - pgm.astype(float) / 255.0) > meta["occupied_thresh"]

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.imshow(np.where(occupied, 1, 0), cmap="Reds", extent=[ox, xmax, oy, ymax], origin="upper", alpha=0.6)

    xs = [p[0] for p in path_world]
    ys = [p[1] for p in path_world]
    ax.plot(xs, ys, "-", color="lime", linewidth=2.5, zorder=4, label="Nav2 path (through all points)")

    for name, (x, y) in survey_points.items():
        ax.scatter([x], [y], c=GROUP_COLOR.get(name[0], "#333"), s=12, alpha=0.35, zorder=2)

    for i, (label, (x, y)) in enumerate(route_xy):
        ax.scatter([x], [y], c="black", s=140, zorder=6)
        ax.scatter([x], [y], c="yellow", s=70, zorder=7)
        ax.annotate(f"{i}: {label}", (x, y), fontsize=9, weight="bold",
                    xytext=(6, 6), textcoords="offset points", zorder=8)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Multi-point tour: " + " -> ".join(route_labels))
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_viz, dpi=130)
    plt.close(fig)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Plan a continuous multi-point tour using Nav2.")
    parser.add_argument("--start", default="S1",
                         help="Starting survey point name (e.g. S1) or raw coordinate 'x,y' (e.g. 3,2)")
    parser.add_argument("--points", nargs="*", default=POINTS, help="Survey points to visit (e.g. W6 W5 W7 W8)")
    parser.add_argument("--no-loop", action="store_true", help="Do not return to start at the end")
    parser.add_argument("--use-sim-time", type=lambda x: (str(x).lower() in ["true", "1", "yes"]), default=True, help="Use simulation time (default: True)")
    parsed_args, _ = parser.parse_known_args()

    start_arg = parsed_args.start
    points_arg = parsed_args.points
    return_to_start = not parsed_args.no_loop
    use_sim = parsed_args.use_sim_time

    survey_points = load_survey_points(DEFAULT_COORDS)
    start_label, start_xy = resolve(start_arg, survey_points)
    to_visit = [resolve(p, survey_points) for p in points_arg]

    print(f"start: {start_label} {start_xy}")
    print(f"points to visit (unordered): {[l for l, _ in to_visit]}")

    rclpy_args = ["--ros-args", "-p", "use_sim_time:=true"] if use_sim else []
    rclpy.init(args=rclpy_args)
    nav = BasicNavigator()

    ordered = nearest_neighbor_order(start_xy, to_visit)
    if return_to_start:
        ordered = ordered + [(start_label, start_xy)]

    route_labels = [start_label] + [l for l, _ in ordered]
    print("nearest-neighbor route: " + " -> ".join(route_labels))

    start_pose = to_pose(start_xy)
    goal_poses = [to_pose(xy) for _, xy in ordered]

    print("waiting for Nav2 planner_server action server...")
    if not nav.compute_path_through_poses_client.wait_for_server(timeout_sec=10.0):
        print("Warning: compute_path_through_poses action server did not respond within 10s. Trying to query anyway...")

    print(f"requesting one continuous path through {len(goal_poses)} points from planner_server...")
    raw = nav.getPathThroughPoses(start_pose, goal_poses, planner_id="GridBased", use_start=True)
    if raw is None:
        print("\n*** Nav2 could not plan through this route. This almost always means one of the "
              "points sits on an occupied cell in the current occupancy grid (many L-group landmark "
              "points do, by design -- they mark rock features, not drivable ground). Check each "
              "point's clear/BLOCKED status with generate_occupancy_grid.py's console output before "
              "including it here. Route attempted: " + " -> ".join(route_labels) + " ***")
        nav.destroy_node()
        rclpy.shutdown()
        return
    print(f"raw path: {len(raw.poses)} poses")

    print("smoothing via smoother_server...")
    smooth = nav.smoothPath(raw, smoother_id="simple_smoother", check_for_collision=False)
    if smooth is not None:
        smooth_path = smooth.path if hasattr(smooth, "path") else smooth
        print(f"smoothed path: {len(smooth_path.poses)} poses")
    else:
        print("Warning: path smoothing skipped (using high-quality raw grid path)")
        smooth_path = raw

    path_world = [(p.pose.position.x, p.pose.position.y) for p in smooth_path.poses]
    length_m = sum(
        np.hypot(path_world[i + 1][0] - path_world[i][0], path_world[i + 1][1] - path_world[i][1])
        for i in range(len(path_world) - 1)
    )
    print(f"total tour length: {length_m:.2f}m")

    with open(OUT_WAYPOINTS, "w") as f:
        f.write("x,y\n")
        for x, y in path_world:
            f.write(f"{x:.3f},{y:.3f}\n")
    print("waypoints:", OUT_WAYPOINTS)

    save_preview(route_labels, [(start_label, start_xy)] + ordered, path_world, survey_points, OUT_VIZ)
    print("preview:", OUT_VIZ)

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
