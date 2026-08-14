#!/usr/bin/env python3
"""
Pure pursuit path follower for the Gazebo sim.

Path source: loaded once at startup from PATH_CSV (plain "x,y" rows in
meters -- same format plan_multi_point_tour.py / plan_path.py write). The
old single hardcoded goal_point = [7, 11] is gone; the rover now follows a
lookahead search over the full loaded path (same pattern as
Testing_PPC_logic_matplotlib.py's clean prototype, generalized from a single
goal to a full route).

Odometry: /odometry/filtered (the EKF's output), was /odom.

Control output: /rover_controller/cmd_vel as geometry_msgs/TwistStamped
(was Twist on /cmd_vel) -- matches what the sim's diff_drive_controller
actually subscribes to (use_stamped_vel: true in diff_controller.yaml).
"""
import csv
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R

# ═══════════════════════════════════════════════════════════════
# GLOBAL VARIABLES
# ═══════════════════════════════════════════════════════════════

# Rover state (from odom)
car_yaw = None
car_global_axis = None

# Path to follow: "x,y" rows in meters, same S1-origin world frame as
# everything else (see plan_multi_point_tour.py / plan_path.py, which both
# write exactly this format).
PATH_CSV = os.path.expanduser("~/jazzy_ws/src/marsyard/marsyard2026_tour_waypoints.csv")
path = []
current_target_idx = 0

# Publisher
vel_pub = None

# ═══════════════════════════════════════════════════════════════
# PARAMETERS
# ═══════════════════════════════════════════════════════════════
# Lookahead distance is the only tunable knob (ROS parameter
# "lookahead_distance", default 1.0m) -- everything else below is fixed by
# the rover's actual geometry/desired behavior, not something to tune per
# run. Tune live with:
#   ros2 param set /pure_pursuit lookahead_distance 1.5

LA = 1.0                    # Lookahead distance default (m), overridden by the ROS param at startup
WHEEL_BASE = 0.7095         # m -- from URDF forward kinematics (was 0.6)

BASE_VELOCITY = 0.5         # Normal speed (m/s)
GOAL_TOLERANCE = 0.3        # Stop when this close to goal (m)
MAX_CURVATURE = 2.0         # 1/m -- caps angular command when the target is much farther than LA
                             # (e.g. right after startup if the rover isn't sitting at path[0])
STUCK_TICKS_LIMIT = 30      # control_loop runs at 10Hz -- 30 ticks = 3s with no index progress

target_idx_initialized = False
_last_seen_idx = None
_stuck_ticks = 0


# ═══════════════════════════════════════════════════════════════
# PATH LOADING
# ═══════════════════════════════════════════════════════════════

def load_path(csv_path):
    pts = []
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
    for row in rows:
        if not row or len(row) < 2:
            continue
        try:
            pts.append([float(row[0]), float(row[1])])
        except ValueError:
            continue  # header row
    return pts


# ═══════════════════════════════════════════════════════════════
# CALLBACKS
# ═══════════════════════════════════════════════════════════════

def odom_callback(odom):
    """Update rover position and heading from odometry."""
    global car_yaw
    global car_global_axis

    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w

    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y

    car_global_axis = [x, y]

    r = R.from_quat([qx, qy, qz, qw])
    _, _, car_yaw = r.as_euler('xyz')


# ═══════════════════════════════════════════════════════════════
# PURE PURSUIT FUNCTIONS
# ═══════════════════════════════════════════════════════════════

def distance(p1, p2):
    """Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def point_global_to_local(point_global_axis, car_yaw, car_axis):
    """Transform point from global to rover's local frame."""
    dx = point_global_axis[0] - car_axis[0]
    dy = point_global_axis[1] - car_axis[1]

    point_local_x = dy * np.sin(car_yaw) + dx * np.cos(car_yaw)
    point_local_y = dy * np.cos(car_yaw) - dx * np.sin(car_yaw)

    return point_local_x, point_local_y


def calc_curv(point_local_y, look_ahead=LA):
    """Calculate curvature for pure pursuit."""
    curvature = (2 * point_local_y) / (look_ahead * look_ahead)
    return max(-MAX_CURVATURE, min(MAX_CURVATURE, curvature))


def generate_command(curvature, linear_speed):
    """Generate a (linear, angular) command from curvature and speed."""
    angular_speed = curvature * linear_speed
    return float(linear_speed), float(angular_speed)


def publish_cmd_vel(node, linear, angular):
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_footprint"
    msg.twist.linear.x = float(linear)
    msg.twist.angular.z = float(angular)
    vel_pub.publish(msg)


def next_lookahead_point():
    """Advance current_target_idx and return the next point on `path` at
    least LA away from the rover, same lookahead-search pattern as
    Testing_PPC_logic_matplotlib.py's prototype."""
    global current_target_idx
    lookahead_point = None
    for i in range(current_target_idx, len(path)):
        if distance(car_global_axis, path[i]) >= LA:
            lookahead_point = path[i]
            current_target_idx = i
            break
    if lookahead_point is None:
        lookahead_point = path[-1]
    return lookahead_point


# ═══════════════════════════════════════════════════════════════
# MAIN CONTROL LOOP
# ═══════════════════════════════════════════════════════════════

def control_loop(node):
    """Main control loop - called by timer."""
    global car_yaw, car_global_axis, vel_pub, LA, current_target_idx, target_idx_initialized

    LA = node.get_parameter("lookahead_distance").value

    if car_yaw is None or car_global_axis is None:
        node.get_logger().warn("Waiting for odometry...")
        return

    if not path:
        node.get_logger().warn(f"Path is empty (checked {PATH_CSV})")
        return

    # The lookahead search below assumes current_target_idx tracks the
    # rover's actual progress along the path. If the rover isn't sitting at
    # path[0] when this node starts (e.g. it already drove somewhere before
    # this process launched), starting the search at index 0 would lock the
    # target onto a faraway point instead of the nearby path segment. Fix
    # that up once, on the first control tick, by snapping to the closest
    # point on the path.
    if not target_idx_initialized:
        current_target_idx = min(
            range(len(path)), key=lambda i: distance(car_global_axis, path[i])
        )
        target_idx_initialized = True

    # Only allow a goal-reached stop once the lookahead search has actually
    # worked its way to the final point on the path -- the tour path is a
    # closed loop that ends back near its own start, so checking distance to
    # path[-1] alone would trigger immediately at t=0 before the rover ever
    # moves.
    on_final_segment = current_target_idx >= len(path) - 1
    dist_to_goal = distance(car_global_axis, path[-1])
    if on_final_segment and dist_to_goal < GOAL_TOLERANCE:
        publish_cmd_vel(node, 0.0, 0.0)
        node.get_logger().info("Goal reached!")
        return

    target_x, target_y = next_lookahead_point()

    # Geometric curvature clamping alone isn't enough to guarantee progress:
    # if the clamped turn radius is too tight to actually reach the current
    # target (e.g. the sharp loop-closure segment where the tour path curves
    # back to meet its own start), the rover can lock into an indefinite
    # tight circle around a target it never gets close enough to advance
    # past. Sustained circling like that was traced back to a real rollover
    # in testing, so detect "no index progress" and force-advance instead of
    # letting it spin forever.
    global _last_seen_idx, _stuck_ticks
    if current_target_idx == _last_seen_idx:
        _stuck_ticks += 1
    else:
        _stuck_ticks = 0
        _last_seen_idx = current_target_idx

    if _stuck_ticks >= STUCK_TICKS_LIMIT:
        node.get_logger().warn(
            f"Target index {current_target_idx} hasn't advanced in "
            f"{STUCK_TICKS_LIMIT / 10:.0f}s (stuck circling) -- forcing it forward."
        )
        current_target_idx = min(current_target_idx + 1, len(path) - 1)
        target_x, target_y = path[current_target_idx]
        _stuck_ticks = 0
        _last_seen_idx = current_target_idx

    local_x, local_y = point_global_to_local(
        [target_x, target_y], car_yaw, car_global_axis
    )

    curv = calc_curv(local_y, look_ahead=LA)
    linear, angular = generate_command(curv, BASE_VELOCITY)

    publish_cmd_vel(node, linear, angular)

    node.get_logger().info(
        f"pos: {car_global_axis} | target: [{target_x:.2f}, {target_y:.2f}] "
        f"| vel: {linear:.2f}, ang: {angular:.2f}",
        throttle_duration_sec=0.5,
    )


def publish_path(node, path_pub):
    """Publish the loaded path as a nav_msgs/Path (in the "map" frame -- the
    world/S1-origin frame the CSV waypoints are already expressed in) so
    RViz can show the route the rover is actually following, since it's
    driving off this pre-planned CSV rather than a live Nav2 planning call."""
    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    for x, y in path:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    path_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    global vel_pub, path

    rclpy.init(args=args)
    node = rclpy.create_node("pure_pursuit")
    node.declare_parameter("lookahead_distance", LA)

    try:
        path = load_path(PATH_CSV)
        node.get_logger().info(f"Loaded {len(path)} path points from {PATH_CSV}")
    except FileNotFoundError:
        node.get_logger().error(
            f"Path file not found: {PATH_CSV} -- run plan_multi_point_tour.py or plan_path.py "
            "first, or point PATH_CSV at an existing waypoints CSV."
        )
        path = []

    node.create_subscription(Odometry, "/odometry/filtered", odom_callback, 10)
    vel_pub = node.create_publisher(TwistStamped, "/rover_controller/cmd_vel", 10)

    # Transient local so RViz gets the path even if it subscribes after this
    # one-shot publish.
    path_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(Path, "/pure_pursuit/path", path_qos)
    if path:
        publish_path(node, path_pub)

    timer = node.create_timer(0.1, lambda: control_loop(node))

    node.get_logger().info("Pure Pursuit started!")

    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
