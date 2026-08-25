#!/usr/bin/env python3
"""Send a list of waypoints to the full Nav2 navigation stack and drive
them -- unlike tools/plan_multi_point_tour.py, which only asks
planner_server for a path and never moves the robot, this calls
BasicNavigator.goToPose() once per waypoint (NavigateToPose on
bt_navigator), each one followed live by controller_server and confirmed
reached before the next is sent. Deliberately NOT a single
goThroughPoses()/NavigateThroughPoses call for the whole route: that action
judges completion by proximity to the FINAL pose only, not by having
visited each intermediate one -- with RETURN_TO_START the route's last pose
lands back near the robot's own spawn point, so goThroughPoses() reports
"succeeded" within milliseconds without the robot ever moving. Requires the
full stack -- nav2_navigation_sim.launch.py / full_nav_sim.launch.py, not
the planning-only nav2_planning_sim.launch.py.

Waypoints are named survey points (see rover_nav/maps/.../
Coordinates_MarsYard2026.txt, e.g. "W6") or raw "x,y" strings, mixed freely
-- same convention as plan_multi_point_tour.py. Ordered nearest-first from
--start (a rough starting position for ordering purposes only; the actual
drive starts from wherever Nav2 currently has the robot localized).

Usage:
  ros2 launch rover_nav full_nav_sim.launch.py         # bring the stack up first
  ros2 run rover_nav send_waypoints.py --start 3,2 --points W6 W5 W7 W8
  ros2 run rover_nav send_waypoints.py --points W6 W5 --no-loop --use-sim-time false
"""

import argparse
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

POINTS = ["W6", "W5", "W7", "W8"]
# Default points to visit (nearest-first ordered from --start), overridden by --points.

RETURN_TO_START = True
# If True, appends --start back onto the end of the route as the final waypoint.

DEFAULT_COORDS_REL = os.path.join(
    "2026_MarsYard_3D_Model-20260812T165935Z-1-001",
    "2026_MarsYard_3D_Model", "Coordinates_MarsYard2026.txt",
)


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


def _wait_for_first_message(node, topic, msg_type, timeout_sec):
    """Blocks until one message arrives on topic, or timeout_sec elapses (with
    a warning -- proceeding without perception data is degraded, not fatal,
    e.g. if the camera pipeline genuinely isn't running)."""
    got = {"msg": None}

    def _cb(msg):
        got["msg"] = msg

    sub = node.create_subscription(msg_type, topic, _cb, 5)
    start = time.monotonic()
    while got["msg"] is None and time.monotonic() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if got["msg"] is None:
        node.get_logger().warn(
            f"no message received on {topic} within {timeout_sec}s -- proceeding anyway, "
            "but the local costmap's obstacle_layer may not have real data yet"
        )
    return got["msg"]


def to_pose(xy, stamp):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = stamp
    p.pose.position.x = float(xy[0])
    p.pose.position.y = float(xy[1])
    p.pose.orientation.w = 1.0
    return p


def main():
    parser = argparse.ArgumentParser(description="Drive a multi-point tour using the full Nav2 stack.")
    parser.add_argument("--start", default="S1",
                         help="Rough starting position for nearest-first ordering: survey point "
                              "name (e.g. S1) or raw 'x,y' (e.g. 3,2). Does not set the robot's "
                              "actual pose -- that comes from localization.")
    parser.add_argument("--points", nargs="*", default=POINTS, help="Survey points to visit (e.g. W6 W5 W7 W8)")
    parser.add_argument("--no-loop", action="store_true", help="Do not return to start at the end")
    parser.add_argument("--use-sim-time", type=lambda x: (str(x).lower() in ["true", "1", "yes"]),
                         default=True, help="Use simulation time (default: True)")
    args, _ = parser.parse_known_args()

    coords_path = os.path.join(
        get_package_share_directory("rover_nav"), "maps", DEFAULT_COORDS_REL
    )
    survey_points = load_survey_points(coords_path)
    start_label, start_xy = resolve(args.start, survey_points)
    to_visit = [resolve(p, survey_points) for p in args.points]

    ordered = nearest_neighbor_order(start_xy, to_visit)
    if not args.no_loop:
        ordered = ordered + [(start_label, start_xy)]
    route_labels = [start_label] + [label for label, _ in ordered]
    print("route (nearest-neighbor from --start): " + " -> ".join(route_labels))

    rclpy_args = ["--ros-args", "-p", "use_sim_time:=true"] if args.use_sim_time else []
    rclpy.init(args=rclpy_args)
    nav = BasicNavigator()

    print("waiting for Nav2 to become active (bt_navigator)...")
    # localizer='robot_localization' -- nav2_simple_commander special-cases this
    # string to skip waiting on a lifecycle get_state service, which is what this
    # stack needs: localization here is robot_localization's ekf_node (not a
    # lifecycle node) plus map_odom_broadcaster.py/odom_tf_broadcaster.py, not
    # AMCL. The default localizer='amcl' would otherwise wait forever on a
    # /amcl/get_state service that never comes up.
    nav.waitUntilNav2Active(localizer='robot_localization')

    # waitUntilNav2Active only confirms bt_navigator's LIFECYCLE state is
    # active -- it says nothing about whether local_costmap's obstacle_layer
    # has actually observed anything yet. That chain (camera ->
    # depth_to_pointcloud.py -> PassThrough -> SOR -> obstacle_layer) takes a
    # few cycles to populate real data after activation; sending a goal
    # immediately risks the controller's collision check running against a
    # still-empty local costmap for its first cycles right as it commits to
    # an initial turn -- exactly when a nearby obstacle is most likely to
    # matter. Wait for at least one real /pcl/denoised message, then give the
    # costmap's own update cycle (5 Hz, nav2_local_planner_params.yaml) a
    # couple of ticks to actually integrate it.
    print("waiting for the camera obstacle-avoidance chain to produce real data...")
    _wait_for_first_message(nav, "/pcl/denoised", PointCloud2, timeout_sec=8.0)
    time.sleep(0.5)

    # One goToPose() per waypoint (not a single goThroughPoses() call) -- deliberately.
    # NavigateThroughPoses judges completion by proximity to the FINAL pose only, it
    # does not require actually passing through the intermediate ones. With
    # RETURN_TO_START the route's final pose is back near the robot's own spawn
    # point, so goThroughPoses() would report "succeeded" within milliseconds
    # without ever visiting the waypoints in between (confirmed: bt_navigator's own
    # log showed "Goal succeeded" 60ms after "Begin navigating... through 5 poses").
    # Sequential goToPose() calls make each leg's arrival real and independently
    # verified before starting the next.
    all_succeeded = True
    for i, (label, xy) in enumerate(ordered):
        stamp = nav.get_clock().now().to_msg()
        goal_pose = to_pose(xy, stamp)
        print(f"[{i + 1}/{len(ordered)}] driving to {label} {xy}...")
        nav.goToPose(goal_pose)

        last_print = 0.0
        while not nav.isTaskComplete():
            now = time.monotonic()
            if now - last_print > 2.0:
                feedback = nav.getFeedback()
                if feedback:
                    remaining = getattr(feedback, "distance_remaining", None)
                    eta = getattr(feedback, "estimated_time_remaining", None)
                    eta_s = eta.sec + eta.nanosec * 1e-9 if eta is not None else None
                    print(f"  distance remaining: {remaining:.2f} m"
                          + (f", eta: {eta_s:.0f}s" if eta_s is not None else ""))
                last_print = now

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"  reached {label}")
        elif result == TaskResult.CANCELED:
            print(f"  canceled en route to {label}")
            all_succeeded = False
            break
        else:
            print(f"  failed to reach {label}")
            all_succeeded = False
            break

    if all_succeeded:
        print("navigation succeeded -- reached all waypoints")
    else:
        print("navigation stopped before reaching all waypoints")

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
