#!/usr/bin/env python3
"""Plan a global path across the Mars Yard costmap with the Hybrid-A* global
planner, and write it out as a drivable CSV.

Requires the planning stack to be up (it drives planner_server's action):

  ros2 launch rover_nav nav2_planning.launch.py rviz:=true
  ros2 run rover_nav plan_global_path.py --start S1 --points W6 W5 W7 W8 --loop

Why this exists rather than Mars-rover/tools/plan_multi_point_tour.py (the
`omar` branch's tour tool): that tool sets every goal pose's orientation to
identity (yaw = 0). SmacPlanner2D ignores goal orientation entirely, so that
was harmless there. SmacPlannerHybrid *enforces* it, and this nav2 build has
no `goal_heading_mode` to relax the constraint -- so an identity goal yaw
makes the planner add a loop at every waypoint purely to arrive facing +X.

This tool instead assigns each waypoint the heading the rover would naturally
carry through it (the bearing on to the next waypoint), and chains each leg's
start heading from the previous leg's arrival heading. The planner then only
spends curvature on getting between points, never on lining up with an
arbitrary yaw.

Output (to --out-dir, default the marsyard/ directory):
  <name>.csv      - x_m, y_m, yaw_rad, one row per pose
  <name>.png      - the path drawn over the occupancy grid, waypoints labelled
"""

import argparse
import csv
import math
import os
import re
import sys

DEFAULT_COORDS = os.path.expanduser(
    "~/jazzy_ws/marsyard/2026_MarsYard_3D_Model-20260812T165935Z-1-001/"
    "2026_MarsYard_3D_Model/Coordinates_MarsYard2026.txt")
DEFAULT_MAP_YAML = os.path.expanduser("~/jazzy_ws/marsyard/marsyard2026_occupancy.yaml")
DEFAULT_OUT_DIR = os.path.expanduser("~/jazzy_ws/marsyard")

# The curvature the planner is configured to respect. Keep in step with
# minimum_turning_radius in config/nav2_planning_params.yaml -- this value only
# drives the reporting below, so a mismatch silently mis-scores the path rather
# than changing what gets planned.
MIN_TURNING_RADIUS = 1.5


def load_survey_points(path):
    """Parse the organisers' coordinate table. Columns are Name, Y, X, H --
    note Y before X -- with comma decimal separators."""
    points = {}
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split("\t") if p.strip()]
            if len(parts) < 4 or parts[0].lower().startswith("point"):
                continue
            try:
                y, x, _h = (float(v.replace(",", ".")) for v in parts[1:4])
            except ValueError:
                continue
            points[parts[0]] = (x, y)
    return points


def resolve(spec, survey):
    """A waypoint is either a survey point name (W6) or a raw 'x,y' pair."""
    if re.match(r"^-?\d+(\.\d+)?,-?\d+(\.\d+)?$", spec):
        x, y = (float(v) for v in spec.split(","))
        return f"({x:.1f},{y:.1f})", (x, y)
    if spec not in survey:
        raise SystemExit(f"'{spec}' is not a survey point. Known: {', '.join(sorted(survey))}")
    return spec, survey[spec]


def nearest_first(start_xy, items):
    ordered, pool, cur = [], list(items), start_xy
    while pool:
        i = min(range(len(pool)), key=lambda k: math.dist(cur, pool[k][1]))
        label, xy = pool.pop(i)
        ordered.append((label, xy))
        cur = xy
    return ordered


def bearing(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def heading_candidates(start_xy, route_xy, i):
    """Goal headings to try at waypoint i, best first.

    SmacPlannerHybrid enforces the goal orientation exactly (this nav2 build
    has no goal_heading_mode to relax it), so a badly chosen yaw can make a
    perfectly open leg unplannable: demanding a 168-degree reversal at a point
    with 2 m of clearance needs a turning loop that simply does not fit inside
    a 2 m minimum radius.

    So rather than commit to one heading, offer several and let the caller
    take the first that plans:
      1. the bisector of the incoming and outgoing bearings -- the heading a
         vehicle would naturally carry through the corner, splitting the turn
         evenly across the two legs instead of dumping all of it on one;
      2. the incoming bearing -- arrive straight, turn on the next leg;
      3. the outgoing bearing -- turn on this leg, leave straight;
      4. +-45 degrees off the bisector, as slack for tight corners.
    """
    prev = route_xy[i - 1] if i > 0 else start_xy
    here = route_xy[i]
    nxt = route_xy[i + 1] if i + 1 < len(route_xy) else None

    incoming = bearing(prev, here)
    if nxt is None:
        preferred = [incoming, _wrap(incoming + math.pi / 4), _wrap(incoming - math.pi / 4)]
        return preferred + _sweep(preferred)

    outgoing = bearing(here, nxt)
    # Circular mean of the two bearings (correct across the +-pi wrap).
    bisector = math.atan2(
        (math.sin(incoming) + math.sin(outgoing)) / 2.0,
        (math.cos(incoming) + math.cos(outgoing)) / 2.0,
    )
    preferred = [bisector, incoming, outgoing,
                 _wrap(bisector + math.pi / 4), _wrap(bisector - math.pi / 4)]
    return preferred + _sweep(preferred)


def _sweep(preferred, step_deg=30):
    """Every heading on a coarse ring, minus any already in `preferred`.

    The preferred headings above are the *natural* ones, not the only feasible
    ones. Measured against this yard at a 2 m turning radius, the headings that
    actually plan into a waypoint form a narrow arc -- 4/12 of a 30-degree ring
    at W6, 3/12 at W3 -- and that arc frequently excludes all five preferred
    headings. Falling back to a full sweep turns "this route is impossible"
    into "this route needs an unusual approach angle", which is almost always
    the truth: every W point here is reachable from S1 at *some* heading.

    Only reached when the preferred headings all fail, so the common case pays
    nothing for it."""
    out = []
    for d in range(0, 360, step_deg):
        h = _wrap(math.radians(d))
        if all(abs(_wrap(h - p)) > math.radians(step_deg / 2.0) for p in preferred):
            out.append(h)
    return out


def curvature_profile(pts):
    """Menger curvature at each interior point, in 1/m."""
    ks = []
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
        if min(ab, bc, ca) < 1e-9:
            continue
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        ks.append(cross / (ab * bc * ca) * 2.0)
    return ks


def report_curvature(pts, limit_k):
    ks = curvature_profile(pts)
    length = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
    if not ks:
        print(f"  path: {len(pts)} poses, {length:.1f} m (too short to measure curvature)")
        return
    ks_s = sorted(ks)
    over = sum(1 for k in ks if k > limit_k * 1.05)   # 5% tolerance for sampling noise
    print(f"  poses:     {len(pts)}   length: {length:.1f} m")
    print(f"  curvature: median {ks_s[len(ks)//2]:.3f}   p95 {ks_s[int(0.95*len(ks))]:.3f}   "
          f"max {max(ks):.3f} 1/m")
    print(f"  tightest turn radius: {1.0/max(ks):.2f} m   (planner limit {1.0/limit_k:.2f} m)")
    print(f"  poses over the limit: {over}/{len(ks)} ({100.0*over/len(ks):.1f}%)")


def warn_blocked(map_yaml, named_points):
    """Flag requested waypoints that sit on, or very near, occupied cells.

    Several of the organisers' survey markers land on terrain the occupancy
    grid calls blocked -- they mark features (rocks, slopes) rather than
    parking spots. The planner's `tolerance` quietly absorbs this by stopping
    short, so without this warning a route can look like it planned fine
    while never actually reaching the point that was asked for."""
    try:
        import numpy as np
        import yaml
        from scipy import ndimage
    except ImportError:
        return
    try:
        meta = yaml.safe_load(open(map_yaml))
        pgm = os.path.join(os.path.dirname(map_yaml), meta["image"])
        with open(pgm, "rb") as f:
            assert f.readline().strip() == b"P5"
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            w, h = map(int, line.split())
            f.readline()
            img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    except Exception:
        return

    res, ox, oy = meta["resolution"], meta["origin"][0], meta["origin"][1]
    free = img > int((1.0 - meta["free_thresh"]) * 255)
    clearance = ndimage.distance_transform_edt(free) * res

    problems = []
    for name, (x, y) in named_points:
        col, row = int((x - ox) / res), int((y - oy) / res)
        # A PGM stores its rows top-first, but a ROS map's origin is its
        # BOTTOM-left corner -- so the map row for a world y is counted up
        # from the bottom of the image, i.e. flipped. Sampling img[row]
        # directly reads a point mirrored about the map's horizontal axis,
        # which silently reports the wrong cell rather than failing.
        row = h - 1 - row
        if not (0 <= col < w and 0 <= row < h):
            problems.append(f"    {name}: outside the map")
        elif not free[row, col]:
            problems.append(f"    {name}: on an OCCUPIED cell -- the planner will stop "
                            f"up to `tolerance` short of it")
        elif clearance[row, col] < 1.0:
            problems.append(f"    {name}: only {clearance[row, col]:.2f} m clearance -- "
                            f"tight for a {MIN_TURNING_RADIUS:.1f} m turning radius")
    if problems:
        print("  Waypoint clearance warnings:")
        print("\n".join(problems))
        print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", default="S1", help="Start point (survey name or 'x,y')")
    ap.add_argument("--points", nargs="+", default=["W6", "W5", "W7", "W8"],
                    help="Waypoints to visit (survey names or 'x,y')")
    ap.add_argument("--loop", action="store_true", help="Return to the start at the end")
    ap.add_argument("--in-order", action="store_true",
                    help="Visit --points as given instead of nearest-first")
    ap.add_argument("--name", default="global_path_hybrid", help="Output basename")
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    ap.add_argument("--coords", default=DEFAULT_COORDS)
    ap.add_argument("--map-yaml", default=DEFAULT_MAP_YAML)
    ap.add_argument("--planner-id", default="GridBased")
    ap.add_argument("--timeout", type=float, default=60.0, help="Per-leg planning timeout (s)")
    ap.add_argument("--publish", action="store_true",
                    help="After planning, publish the assembled path + waypoint markers "
                         "and stay alive so RViz keeps showing them (Ctrl+C to exit). "
                         "Without this the tool exits and RViz only ever saw each leg "
                         "flash past on /unsmoothed_plan.")
    ap.add_argument("--view-csv", metavar="FILE",
                    help="Skip planning entirely: publish a previously generated path CSV "
                         "for RViz and stay alive. Nothing else is required beyond RViz.")
    args = ap.parse_args()

    if args.view_csv:
        view_csv(args.view_csv)
        return

    survey = load_survey_points(args.coords)
    start_label, start_xy = resolve(args.start, survey)
    route = [resolve(p, survey) for p in args.points]
    if not args.in_order:
        route = nearest_first(start_xy, route)
    if args.loop:
        route.append((start_label, start_xy))

    route_labels = [lbl for lbl, _ in route]
    route_xy = [xy for _, xy in route]

    print(f"Route: {start_label} -> {' -> '.join(route_labels)}")
    print(f"Planner: {args.planner_id}\n")

    warn_blocked(args.map_yaml, [(start_label, start_xy)] + list(zip(route_labels, route_xy)))

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose

    def pose(xy, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = float(xy[0]), float(xy[1])
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    rclpy.init()
    node = Node("plan_global_path")
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        raise SystemExit("planner_server's compute_path_to_pose action never showed up -- "
                         "is nav2_planning.launch.py running?")

    all_pts, all_yaw, leg_breaks = [], [], []
    cur_xy = start_xy
    # The first leg's start heading is a free variable: this is a skid-steer
    # rover, so it can pivot in place on the spot before setting off. Every
    # later leg inherits its start heading from the previous leg's arrival and
    # cannot, which is why only this one gets swept. Measured on this yard, the
    # headings that plan into a given waypoint can be as narrow as 3 of 12, so
    # the straight-line bearing being one of them is not a safe assumption.
    natural_start = bearing(start_xy, route_xy[0])
    start_yaws = [natural_start] + _sweep([natural_start])
    cur_yaw = natural_start

    def plan_leg(from_xy, from_yaw, to_xy, to_yaw):
        goal = ComputePathToPose.Goal()
        goal.start = pose(from_xy, from_yaw)
        goal.goal = pose(to_xy, to_yaw)
        goal.use_start = True
        goal.planner_id = args.planner_id
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, rfut, timeout_sec=args.timeout)
        result = rfut.result()
        if result is None or not result.result.path.poses:
            return None
        return result.result.path.poses

    for i, (label, goal_xy) in enumerate(route):
        poses = None
        # Leg 0 may also vary where the rover points before it starts moving.
        start_options = start_yaws if i == 0 else [cur_yaw]
        planned_but_dead_ended = 0
        for attempt, (syaw, yaw) in enumerate(
                (sy, gy) for gy in heading_candidates(start_xy, route_xy, i)
                for sy in start_options):
            trial = plan_leg(cur_xy, syaw, goal_xy, yaw)
            if not trial:
                continue
            # One-step lookahead. A leg's start heading is fixed by however the
            # previous leg ended, so a heading that plans fine *into* a waypoint
            # can still leave the rover unable to get *out* of it -- with a
            # minimum turning radius, arriving at a tight point facing the wrong
            # way is a dead end. Committing only to headings the next leg can
            # actually depart on avoids failing a whole route over a choice made
            # one waypoint earlier.
            if i + 1 < len(route):
                # Probe from where this leg ACTUALLY ends, not from the nominal
                # waypoint. The planner lands up to ~6 cm off the requested pose
                # (0.1 m grid), and the next leg starts from that real endpoint --
                # so probing the nominal pose validates a position the rover never
                # occupies. In a tight corridor those few centimetres flip
                # feasibility, and the route then fails at the very leg the
                # lookahead was supposed to have cleared.
                tp = trial[-1].pose
                probe_xy = [tp.position.x, tp.position.y]
                probe_yaw = 2.0 * math.atan2(tp.orientation.z, tp.orientation.w)
                nxt_xy = route_xy[i + 1]
                nxt_yaws = heading_candidates(start_xy, route_xy, i + 1)
                if not any(plan_leg(probe_xy, probe_yaw, nxt_xy, ny) for ny in nxt_yaws):
                    planned_but_dead_ended += 1
                    continue
            poses = trial
            chosen_yaw = yaw
            if i == 0 and abs(_wrap(syaw - natural_start)) > 1e-6:
                print(f"    (pivot in place to {math.degrees(syaw):+.0f}deg before setting off "
                      f"-- the straight-line bearing could not reach this waypoint)")
            if attempt:
                print(f"    (arriving on heading {math.degrees(yaw):+.0f}deg -- the preferred "
                      f"heading was not reachable, or dead-ended the next leg)")
            break
        if not poses:
            tried = ", ".join(f"{math.degrees(h):+.0f}" for h in
                              heading_candidates(start_xy, route_xy, i))
            where = (f"from ({cur_xy[0]:.2f},{cur_xy[1]:.2f}) "
                     f"heading {math.degrees(cur_yaw):+.1f}deg")
            if planned_but_dead_ended:
                # Distinguishing these two matters: they have opposite fixes,
                # and conflating them sends you looking at the costmap when the
                # costmap is fine.
                nxt_label = route[i + 1][0] if i + 1 < len(route) else "?"
                raise SystemExit(
                    f"leg {i+1} ({label}): reachable {where}, but every arrival heading "
                    f"({planned_but_dead_ended} of them) leaves the rover unable to go on "
                    f"to {nxt_label}.\n"
                    f"  goal headings tried: {tried}\n"
                    f"  This is NOT a costmap problem -- {label} itself is reachable. The "
                    f"route order is the constraint: reaching {label} and then {nxt_label} "
                    f"needs an arrival heading that serves both, and none does.\n"
                    f"  Try: a different order (--in-order lets you choose), dropping "
                    f"{nxt_label} from this leg's neighbourhood, or a smaller "
                    f"minimum_turning_radius in config/nav2_planning_params.yaml.")
            raise SystemExit(
                f"leg {i+1} ({label}): no path found {where} at any candidate goal heading.\n"
                f"  goal headings tried: {tried}\n"
                f"  (the start pose is fixed by where the previous leg ended -- if that "
                f"heading sits in a dead zone, no goal heading can rescue this leg)\n"
                "A minimum turning radius can make a leg unplannable where a "
                "zero-radius planner would succeed -- check the clearance around "
                "this point, or lower minimum_turning_radius in "
                "config/nav2_planning_params.yaml.")
        pts = [(q.pose.position.x, q.pose.position.y) for q in poses]
        yaws = [2.0 * math.atan2(q.pose.orientation.z, q.pose.orientation.w) for q in poses]
        if all_pts:                      # drop the duplicated joint pose
            pts, yaws = pts[1:], yaws[1:]
        leg_breaks.append(len(all_pts))
        all_pts.extend(pts)
        all_yaw.extend(yaws)

        seg_len = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
        gap = math.dist(pts[-1], goal_xy)
        note = f"  (ends {gap*100:.1f} cm from {label})" if gap > 0.005 else ""
        print(f"  leg {i+1}/{len(route)} -> {label:<8} {len(pts):4d} poses, {seg_len:6.1f} m{note}")

        # Continue from where this leg ACTUALLY ended, not from the waypoint we
        # asked for. On a 0.1 m grid the planner lands up to ~5 cm off the
        # requested pose, and starting the next leg from the nominal waypoint
        # inserts exactly that much lateral jump between two points ~0.15 m
        # apart -- a corner of curvature ~2*delta/s^2, measured at 2.53 1/m
        # (a 0.40 m radius) at the leg joints of the S1-W5-W4-W3-W2 tour, on a
        # path every individual leg of which honoured the 2.0 m bound.
        # Chaining the real endpoint keeps the concatenated path C0-continuous
        # and makes the reported curvature the one the rover will actually be
        # asked to drive.
        cur_xy = list(pts[-1])
        cur_yaw = yaws[-1]

    node.destroy_node()
    rclpy.shutdown()

    print()
    report_curvature(all_pts, 1.0 / MIN_TURNING_RADIUS)

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, f"{args.name}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_m", "y_m", "yaw_rad"])
        for (x, y), yaw in zip(all_pts, all_yaw):
            w.writerow([f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}"])
    print(f"\n  -> {csv_path}")

    # The waypoints themselves, so the follower can report how close the rover
    # actually came to each one. The path CSV alone cannot answer that: it is a
    # dense pose list with nothing marking which poses were the goals.
    wp_path = os.path.join(args.out_dir, f"{args.name}_waypoints.csv")
    with open(wp_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["name", "x_m", "y_m"])
        w.writerow(["START" if start_label == args.start else start_label,
                    f"{start_xy[0]:.4f}", f"{start_xy[1]:.4f}"])
        for lbl, (wx, wy) in zip(route_labels, route_xy):
            w.writerow([lbl, f"{wx:.4f}", f"{wy:.4f}"])
    print(f"  -> {wp_path}")

    # How close the plan itself comes to each waypoint -- the floor on what the
    # rover can achieve, before any tracking error is added.
    print("\n  plan's closest approach to each waypoint:")
    for lbl, (wx, wy) in zip(route_labels, route_xy):
        d = min(math.hypot(x - wx, y - wy) for x, y in all_pts)
        print(f"    {lbl:<8} {d * 100:5.1f} cm")

    png_path = os.path.join(args.out_dir, f"{args.name}.png")
    try:
        save_preview(all_pts, start_xy, route_labels, route_xy, survey,
                     args.map_yaml, png_path)
        print(f"  -> {png_path}")
    except Exception as exc:                       # preview is a convenience, not the product
        print(f"  (preview skipped: {exc})")

    if args.publish:
        publish_and_spin(all_pts, [start_xy] + list(route_xy))


def publish_and_spin(pts, waypoints):
    """Publish the whole assembled tour on the topics rviz/nav2_path_view.rviz
    already shows, then block so the latched messages stay served.

    planner_server's own /unsmoothed_plan carries one leg at a time and is
    overwritten by the next, so RViz never shows the complete route. This
    publishes the concatenated path instead, transient-local, so it is there
    whenever RViz connects or is restarted."""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from visualization_msgs.msg import MarkerArray

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import test_path_viz

    rclpy.init()
    node = Node("global_path_view")
    path_pub = node.create_publisher(Path, test_path_viz.PATH_TOPIC, test_path_viz.latched_qos())
    mark_pub = node.create_publisher(MarkerArray, test_path_viz.MARKERS_TOPIC,
                                     test_path_viz.latched_qos())
    stamp = node.get_clock().now().to_msg()
    path_pub.publish(test_path_viz.path_msg(pts, stamp, "map"))
    mark_pub.publish(test_path_viz.marker_msgs(
        [(i, x, y) for i, (x, y) in enumerate(waypoints)], stamp, "map"))

    print(f"\n  Publishing on {test_path_viz.PATH_TOPIC} and {test_path_viz.MARKERS_TOPIC} "
          f"(frame 'map'). Set RViz's Fixed Frame to 'map'. Ctrl+C to stop.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def view_csv(path):
    """Publish a path CSV written by an earlier run, with no planner needed."""
    pts = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x_m"]), float(row["y_m"])))
    if not pts:
        raise SystemExit(f"{path} has no rows")
    print(f"{path}: {len(pts)} poses, "
          f"{sum(math.dist(a, b) for a, b in zip(pts, pts[1:])):.1f} m")
    report_curvature(pts, 1.0 / MIN_TURNING_RADIUS)
    # Waypoints aren't recorded in the CSV; show the endpoints so the route
    # start/finish are still identifiable in RViz.
    publish_and_spin(pts, [pts[0], pts[-1]])


def save_preview(pts, start_xy, labels, route_xy, survey, map_yaml, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import yaml

    meta = yaml.safe_load(open(map_yaml))
    pgm = os.path.join(os.path.dirname(map_yaml), meta["image"])
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

    res, ox, oy = meta["resolution"], meta["origin"][0], meta["origin"][1]
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(11, 12))
    # origin="upper", NOT "lower": a PGM stores row 0 as the TOP of the map
    # (maximum Y), which is also what the ROS map origin convention assumes.
    # With origin="lower" matplotlib draws row 0 at the bottom, flipping the
    # occupancy grid vertically -- while the path, drawn in world coordinates,
    # stays put. The result looks like a plan that runs straight over rocks.
    # generate_occupancy_grid.py's own preview uses "upper" for this reason.
    ax.imshow(img, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255)
    ax.plot([p[0] for p in pts], [p[1] for p in pts], "-", color="#1f5fd1",
            linewidth=2.2, zorder=4, label="Hybrid-A* global path")
    ax.plot(*start_xy, "*", color="#1baf7a", markersize=20, zorder=6, label="start")

    for name, (x, y) in survey.items():
        ax.plot(x, y, ".", color="#999999", markersize=4, zorder=2)
        ax.annotate(name, (x, y), fontsize=5, color="#777777",
                    xytext=(2, 2), textcoords="offset points")
    for i, ((x, y), lbl) in enumerate(zip(route_xy, labels), start=1):
        ax.plot(x, y, "o", color="#0b0b0b", markersize=9, zorder=5)
        ax.annotate(f"{i}. {lbl}", (x, y), fontsize=10, weight="bold", color="#b3005e",
                    xytext=(7, 7), textcoords="offset points", zorder=7)

    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m, map frame)")
    # Read the radius from the constant rather than hardcoding it: a figure that
    # states a planning parameter it did not use is worse than one that states
    # nothing, because it looks like evidence.
    ax.set_title("Global path over the Mars Yard costmap "
                 f"(Hybrid-A*, min turning radius {MIN_TURNING_RADIUS:g} m)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
