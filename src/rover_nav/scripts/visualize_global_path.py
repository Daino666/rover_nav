#!/usr/bin/env python3
"""Plot the waypoint-by-waypoint path rover_controller_pure_pursuit.py drives
(and stops at each waypoint of), without needing ROS, hardware, or the rover
running.

Usage:
  python3 visualize_global_path.py
  python3 visualize_global_path.py --start 0,0 --waypoints 2,0 2,2 0,2
  python3 visualize_global_path.py --waypoints 3,1 5,4 2,6 --out my_path.png
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from global_path_planner import generate_waypoint_legs, START, WAYPOINTS  # noqa: E402


def parse_xy(s):
    x, y = s.split(",")
    return [float(x), float(y)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=parse_xy, default=START,
                         help=f"Start position 'x,y' (default: START in global_path_planner.py, {START})")
    parser.add_argument("--waypoints", type=parse_xy, nargs="+", default=WAYPOINTS,
                         help="Goal waypoints 'x,y' 'x,y' ... (default: WAYPOINTS in global_path_planner.py)")
    parser.add_argument("--out", default="global_path.png", help="Output PNG path")
    args = parser.parse_args()

    legs = generate_waypoint_legs(args.start, args.waypoints)
    total_pts = sum(len(leg) for leg in legs)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    for i, leg in enumerate(legs):
        leg_x = [p[0] for p in leg]
        leg_y = [p[1] for p in leg]
        ax.plot(leg_x, leg_y, "-", color="#2a78d6", linewidth=2.5, zorder=3,
                 label="Path (stops at each waypoint)" if i == 0 else None)

    wp_x = [args.start[0]] + [w[0] for w in args.waypoints]
    wp_y = [args.start[1]] + [w[1] for w in args.waypoints]
    ax.scatter(wp_x[1:], wp_y[1:], c="#0b0b0b", s=90, zorder=5, label="Waypoints (stop here)")
    ax.scatter([wp_x[0]], [wp_y[0]], c="#1baf7a", s=140, marker="*", zorder=6, label="Start")

    for i, (x, y) in enumerate(zip(wp_x, wp_y)):
        label = "start" if i == 0 else str(i)
        ax.annotate(label, (x, y), fontsize=9, weight="bold",
                    xytext=(8, 8), textcoords="offset points")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Pure pursuit waypoint path (open-space assumption, stop at each point)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="best", fontsize=9)

    plt.tight_layout()
    plt.savefig(args.out, dpi=150)
    print(f"legs:  {len(legs)}")
    print(f"points: {total_pts} total ({', '.join(str(len(leg)) for leg in legs)} per leg)")
    print(f"saved: {args.out}")


if __name__ == "__main__":
    main()
