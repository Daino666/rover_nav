#!/usr/bin/env python3
"""Print where a path CSV actually sends the rover, BEFORE driving it.

Every wrong run so far has been a frame question -- is the route rotated the
right way for how the rover is parked -- and the only way it got answered was
by driving into terrain. This answers it on the bench: it prints each waypoint
as plain FORWARD / LEFT-RIGHT from wherever the rover starts, which you can
check against the ground with a tape measure.

  # a map-frame path, with the alignment you intend to launch with
  ros2 run rover_nav preview_path.py --path marsyard/global_path_hybrid.csv \\
      --waypoints marsyard/global_path_hybrid_waypoints.csv --frame map --yaw 90

  # a rover-frame path, driven verbatim
  ros2 run rover_nav preview_path.py --path marsyard/global_path_hybrid_odom.csv \\
      --waypoints marsyard/global_path_hybrid_odom_waypoints.csv --frame odom

--yaw is MAP_TO_ODOM_YAW_DEG: the rover's heading in the map frame at the
moment localization starts. Try the candidates and see which one puts the
waypoints where they physically are.
"""
import argparse
import csv
import math
import sys


def load(path, cols=("x_m", "y_m")):
    with open(path, newline="") as f:
        return [(r, tuple(float(r[c]) for c in cols)) for r in csv.DictReader(f)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--path", required=True)
    ap.add_argument("--waypoints")
    ap.add_argument("--frame", choices=["map", "odom"], default="map")
    ap.add_argument("--yaw", type=float, default=90.0, help="MAP_TO_ODOM_YAW_DEG")
    ap.add_argument("--x", type=float, default=0.0, help="MAP_TO_ODOM_X")
    ap.add_argument("--y", type=float, default=0.0, help="MAP_TO_ODOM_Y")
    a = ap.parse_args()

    # odom <- map is the INVERSE of the map -> odom transform the broadcaster
    # publishes, which is what cmd_vel_arbiter looks up and applies.
    if a.frame == "map":
        inv = math.radians(-a.yaw)
        c, s = math.cos(inv), math.sin(inv)
        tx = -(a.x * c - a.y * s)
        ty = -(a.x * s + a.y * c)
        conv = lambda p: (p[0] * c - p[1] * s + tx, p[0] * s + p[1] * c + ty)
        note = f"map-frame path, map_to_odom yaw={a.yaw:g}deg x={a.x:g} y={a.y:g}"
    else:
        conv = lambda p: p
        note = "rover-frame path, driven verbatim (no transform)"

    pts = [conv(p) for _, p in load(a.path)]
    print(f"\n{note}")
    print(f"{len(pts)} poses.  Everything below is from the rover's START pose:"
          f"  +forward = nose, +left = its left.\n")

    rows = []
    if a.waypoints:
        rows = [(r["name"], conv(p)) for r, p in load(a.waypoints)]
    else:
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            i = min(len(pts) - 1, int(frac * (len(pts) - 1)))
            rows.append((f"{frac*100:.0f}%", pts[i]))

    print(f"  {'point':<8}{'forward':>10}{'sideways':>18}{'range':>9}{'bearing':>10}")
    for name, (fx, fy) in rows:
        side = "left" if fy >= 0 else "right"
        print(f"  {name:<8}{fx:>9.2f}m{abs(fy):>12.2f}m {side:<5}"
              f"{math.hypot(fx, fy):>8.2f}m{math.degrees(math.atan2(fy, fx)):>+9.1f}")

    behind = [n for n, (fx, _) in rows if fx < -0.5]
    if behind:
        print(f"\n  NOTE: {', '.join(behind)} come out BEHIND the rover. If they are"
              f"\n  physically in front of it, this rotation is wrong.")
    fx, fy = pts[min(len(pts) - 1, 20)]
    print(f"\n  First 2 m of path heads {math.degrees(math.atan2(fy, fx)):+.0f} deg "
          f"from the rover's nose ({'straight ahead' if abs(math.degrees(math.atan2(fy,fx)))<30 else 'OFF TO THE SIDE'}).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
