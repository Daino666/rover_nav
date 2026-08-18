#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation as R
import math
import os
import time
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from global_path_planner import distance, hermite_leg, GLOBAL_RESOLUTION, WAYPOINTS

# ═══════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════

TRACK_WIDTH    = 0.65
BASE_VELOCITY  = 0.2
GOAL_TOLERANCE = 0.5
HALT_TIME      = 1.5  # seconds to stop at each waypoint before driving to the next
MAX_WHEEL_VEL  = 1.0
WHEEL_RADIUS   = 0.111
LA             = 0.5
MAX_CURVATURE  = 2.0

MAX_VELOCITY      = 1.5
TURN_BOOST        = 1.5
TURN_THRESHOLD    = 0.2
DEADZONE          = 0.08
ACCEL_LIMIT       = 3.0

# WAYPOINTS (goal points to visit in order, odom frame) lives in
# global_path_planner.py -- edit it there. The rover drives to WAYPOINTS[0],
# stops for HALT_TIME, then WAYPOINTS[1], and so on -- see ensure_leg_path()
# and the halt handling in pursuit_control().

current_wp_idx = 0      # index into WAYPOINTS of the leg currently being driven
leg_path       = []     # dense straight-line path for the current leg
leg_ready      = False
halt_until     = None   # monotonic timestamp to resume driving, or None if not halted

# Post-2026-08-12 reassembly mapping; see cmd_vel_odrive_bridge.yaml.
right_wheels = [0, 4, 3]
left_wheels  = [5, 1, 2]

try:
    _SOUND_DIR = os.path.join(get_package_share_directory("rover_nav"), "sounds", "Sounds")
except PackageNotFoundError:
    _SOUND_DIR = os.path.join(os.path.dirname(__file__), "Sounds")

Stop_sound  = os.path.join(_SOUND_DIR, "Stop.wav")
Start_sound = os.path.join(_SOUND_DIR, "Start.wav")

# ═══════════════════════════════════════════════════════════════
# STATE
# ═══════════════════════════════════════════════════════════════

car_yaw            = None
car_global_axis    = None
pubs               = []
pursuit_enabled    = True
current_target_idx = 0

target_right_velocity  = 0.0
target_left_velocity   = 0.0
current_right_velocity = 0.0
current_left_velocity  = 0.0
trigger          = 0
prev_Y_button    = 0
prev_X_button    = 0
node             = None

# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════

def play_sound(file_path):
    subprocess.Popen(["aplay", file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def apply_deadzone(value, threshold=DEADZONE):
    if abs(value) < threshold:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)

def ramp_velocity(current, target, dt):
    max_change = ACCEL_LIMIT * dt
    diff = target - current
    if abs(diff) < max_change:
        return target
    return current + (max_change if diff > 0 else -max_change)

def mps_to_revs(mps):
    return mps / (2 * math.pi * WHEEL_RADIUS)

# ═══════════════════════════════════════════════════════════════
# GLOBAL PATH PLANNING (stand-in for planner_server; one leg per waypoint)
# ═══════════════════════════════════════════════════════════════

def ensure_leg_path(target):
    """(Re)plan the current leg -- a Hermite curve from wherever the rover
    actually is right now (eased out of its current heading, car_yaw) to
    `target` -- once per leg. Re-planning from the live position and
    heading (rather than the previous waypoint) keeps the leg exact even
    if the rover drifted slightly during the halt, and avoids demanding an
    instant pivot at the start of each leg."""
    global leg_path, leg_ready, current_target_idx
    if leg_ready:
        return
    leg_path = hermite_leg(car_global_axis, car_yaw, target, GLOBAL_RESOLUTION)
    current_target_idx = 0
    leg_ready = True
    node.get_logger().info(
        f"➡️  Leg to waypoint {current_wp_idx + 1}/{len(WAYPOINTS)}: {len(leg_path)} points"
    )

def point_global_to_local(point_global, car_yaw, car_pos):
    dx = point_global[0] - car_pos[0]
    dy = point_global[1] - car_pos[1]
    local_x =  dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
    local_y = -dx * np.sin(car_yaw) + dy * np.cos(car_yaw)
    return local_x, local_y

def calc_curv(local_x, local_y):
    ld = math.sqrt(local_x**2 + local_y**2)
    if ld < 0.01:
        return 0.0
    return (2 * local_y) / (ld ** 2)

def publish_wheel_velocities(v_right, v_left):
    right_msg = ControlMessage()
    right_msg.control_mode = 2
    right_msg.input_mode   = 1
    right_msg.input_vel    = float(np.clip(v_right, -MAX_WHEEL_VEL, MAX_WHEEL_VEL))
    right_msg.input_pos    = 0.0
    right_msg.input_torque = 0.0

    left_msg = ControlMessage()
    left_msg.control_mode = 2
    left_msg.input_mode   = 1
    left_msg.input_vel    = float(np.clip(-v_left, -MAX_WHEEL_VEL, MAX_WHEEL_VEL))
    left_msg.input_pos    = 0.0
    left_msg.input_torque = 0.0

    for i in right_wheels:
        pubs[i].publish(right_msg)
    for i in left_wheels:
        pubs[i].publish(left_msg)

# ═══════════════════════════════════════════════════════════════
# CALLBACKS
# ═══════════════════════════════════════════════════════════════

def odom_callback(odom):
    global car_yaw, car_global_axis
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    car_global_axis = [x, y]
    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    r = R.from_quat([qx, qy, qz, qw])
    _, _, car_yaw = r.as_euler('xyz')

def joy_callback(joy_msg):
    global target_right_velocity, target_left_velocity
    global trigger
    global prev_Y_button, prev_X_button
    global pursuit_enabled, current_target_idx, node
    global current_wp_idx, leg_ready, halt_until

    vertical    = apply_deadzone(-joy_msg.axes[3])
    horizontal  = apply_deadzone( joy_msg.axes[2])
    Y_button    = joy_msg.buttons[4]
    X_button    = joy_msg.buttons[3]
    trigger     = joy_msg.buttons[7]

    if Y_button == 1 and prev_Y_button == 0:
        pursuit_enabled = False
        play_sound(Stop_sound)
        node.get_logger().info("🛑 Switched to MANUAL control")

    if X_button == 1 and prev_X_button == 0:
        pursuit_enabled = True
        current_wp_idx = 0
        leg_ready = False  # re-plan the first leg from wherever the rover is now
        halt_until = None
        play_sound(Start_sound)
        node.get_logger().info("🚀 Switched to AUTONOMOUS control")

    prev_Y_button = Y_button
    prev_X_button = X_button

    if abs(vertical) < TURN_THRESHOLD and abs(horizontal) > 0.1:
        turn_vel = horizontal * MAX_VELOCITY * TURN_BOOST
        target_right_velocity = turn_vel
        target_left_velocity  = turn_vel
    else:
        target_right_velocity = -(vertical - horizontal) * MAX_VELOCITY
        target_left_velocity  =  (vertical + horizontal) * MAX_VELOCITY

# ═══════════════════════════════════════════════════════════════
# PURE PURSUIT
# ═══════════════════════════════════════════════════════════════

def pursuit_control():
    global current_target_idx, current_wp_idx, leg_ready, halt_until

    if car_yaw is None or car_global_axis is None:
        node.get_logger().warn("Waiting for odometry...", throttle_duration_sec=2.0)
        return 0.0, 0.0

    if current_wp_idx >= len(WAYPOINTS):
        node.get_logger().info("✅ All waypoints reached!", throttle_duration_sec=2.0)
        return 0.0, 0.0

    # Sitting still at the waypoint we just reached -- wait out HALT_TIME,
    # then advance to the next one (or finish if that was the last).
    if halt_until is not None:
        if time.monotonic() < halt_until:
            return 0.0, 0.0
        halt_until = None
        current_wp_idx += 1
        leg_ready = False
        if current_wp_idx >= len(WAYPOINTS):
            node.get_logger().info("✅ All waypoints reached!")
            return 0.0, 0.0

    target = WAYPOINTS[current_wp_idx]
    ensure_leg_path(target)

    if distance(car_global_axis, target) < GOAL_TOLERANCE:
        node.get_logger().info(
            f"🛑 Reached waypoint {current_wp_idx + 1}/{len(WAYPOINTS)} -- pausing {HALT_TIME:.1f}s"
        )
        halt_until = time.monotonic() + HALT_TIME
        return 0.0, 0.0

    lookahead_point = None
    for i in range(current_target_idx, len(leg_path)):
        if distance(car_global_axis, leg_path[i]) >= LA:
            lookahead_point = leg_path[i]
            current_target_idx = i
            break

    if lookahead_point is None:
        lookahead_point = leg_path[-1]

    local_x, local_y = point_global_to_local(lookahead_point, car_yaw, car_global_axis)
    curvature = float(np.clip(calc_curv(local_x, local_y), -MAX_CURVATURE, MAX_CURVATURE))

    angular     = curvature * BASE_VELOCITY
    v_right_mps = BASE_VELOCITY + angular * (TRACK_WIDTH / 2)
    v_left_mps  = BASE_VELOCITY - angular * (TRACK_WIDTH / 2)

    node.get_logger().info(
        f"wp {current_wp_idx + 1}/{len(WAYPOINTS)} | pos: {car_global_axis} | "
        f"target: {lookahead_point} | curv: {curvature:.3f}",
        throttle_duration_sec=0.5
    )

    return mps_to_revs(v_right_mps), mps_to_revs(v_left_mps)

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    global pubs, node
    global current_right_velocity, current_left_velocity
    global trigger

    rclpy.init(args=args)
    node = Node("rover_controller")

    clients = []
    for i in range(6):
        clients.append(node.create_client(AxisState, f"/odrive_axis{i}/request_axis_state"))
        pubs.append(node.create_publisher(ControlMessage, f"/odrive_axis{i}/control_message", 10))

    for i, client in enumerate(clients):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for odrive_axis{i}...")

    for i, client in enumerate(clients):
        req = AxisState.Request()
        req.axis_requested_state = 8
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(0.1)

    node.get_logger().info("✅ All ODrive axes armed!")

    node.create_subscription(Odometry, "/odometry/filtered", odom_callback, 10)
    node.create_subscription(Joy, "/joy", joy_callback, 10)

    play_sound(Start_sound)
    node.get_logger().info("🚀 Rover controller started! (AUTONOMOUS mode)")

    publish_period = 1.0 / 20.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            dt = publish_period

            if pursuit_enabled:
                v_right, v_left = pursuit_control()
                publish_wheel_velocities(v_right, v_left)
            else:
                if trigger == 1:
                    target_vel_right = target_right_velocity
                    target_vel_left  = target_left_velocity
                else:
                    target_vel_right = 0.0
                    target_vel_left  = 0.0

                current_right_velocity = ramp_velocity(current_right_velocity, target_vel_right, dt)
                current_left_velocity  = ramp_velocity(current_left_velocity,  target_vel_left,  dt)
                publish_wheel_velocities(current_right_velocity, current_left_velocity)

            time.sleep(publish_period)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
