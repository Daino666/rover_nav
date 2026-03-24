#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import math
import time
import subprocess


# ═══════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════

LA = 0.5
WHEEL_BASE = 0.566
TRACK_WIDTH = 0.65

BASE_VELOCITY = 0.1
GOAL_TOLERANCE = 0.3
MAX_WHEEL_VEL = 1.0
WHEEL_RADIUS = 0.111

goal_point = [2.0, 2.0]

right_wheels = [0, 1, 2]
left_wheels  = [3, 4, 5]
Start_sound = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/Start.wav"

# ═══════════════════════════════════════════════════════════════
# STATE
# ═══════════════════════════════════════════════════════════════

car_yaw = None
car_global_axis = None
pubs = []
pursuit_enabled = True

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

def safety_callback(msg):
    global pursuit_enabled
    pursuit_enabled = msg.data

# ═══════════════════════════════════════════════════════════════
# PURE PURSUIT
# ═══════════════════════════════════════════════════════════════

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

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

# ═══════════════════════════════════════════════════════════════
# ODRIVE HELPERS
# ═══════════════════════════════════════════════════════════════
def play_sound(file_path):
    subprocess.Popen(["aplay", file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def mps_to_revs(mps):
    return mps / (2 * math.pi * WHEEL_RADIUS)

def publish_wheel_velocities(v_right, v_left):
    right_msg = ControlMessage()
    right_msg.control_mode = 2
    right_msg.input_mode = 1
    right_msg.input_vel = float(np.clip(v_right, -MAX_WHEEL_VEL, MAX_WHEEL_VEL))
    right_msg.input_pos = 0.0
    right_msg.input_torque = 0.0

    left_msg = ControlMessage()
    left_msg.control_mode = 2
    left_msg.input_mode = 1
    left_msg.input_vel = float(np.clip(-v_left, -MAX_WHEEL_VEL, MAX_WHEEL_VEL))
    left_msg.input_pos = 0.0
    left_msg.input_torque = 0.0

    for i in right_wheels:
        pubs[i].publish(right_msg)
    for i in left_wheels:
        pubs[i].publish(left_msg)

# ═══════════════════════════════════════════════════════════════
# CONTROL LOOP
# ═══════════════════════════════════════════════════════════════

def control_loop(node):
    global car_yaw, car_global_axis

    if not pursuit_enabled:
        publish_wheel_velocities(0.0, 0.0)
        node.get_logger().warn("🛑 Pursuit disabled!", throttle_duration_sec=2.0)
        return

    if car_yaw is None or car_global_axis is None:
        node.get_logger().warn("Waiting for odometry...", throttle_duration_sec=2.0)
        return

    if distance(car_global_axis, goal_point) < GOAL_TOLERANCE:
        publish_wheel_velocities(0.0, 0.0)
        node.get_logger().info("✅ Goal reached!", throttle_duration_sec=1.0)
        return

    local_x, local_y = point_global_to_local(goal_point, car_yaw, car_global_axis)

    if local_x < 0:
        publish_wheel_velocities(0.0, 0.0)
        node.get_logger().warn("⚠️ Goal is behind rover!", throttle_duration_sec=1.0)
        return

    curvature = calc_curv(local_x, local_y)

    angular = curvature * BASE_VELOCITY
    v_right_mps = BASE_VELOCITY + angular * (TRACK_WIDTH / 2)
    v_left_mps  = BASE_VELOCITY - angular * (TRACK_WIDTH / 2)

    v_right = mps_to_revs(v_right_mps)
    v_left  = mps_to_revs(v_left_mps)

    publish_wheel_velocities(v_right, v_left)

    node.get_logger().info(
        f"pos: {car_global_axis} | local: ({local_x:.2f}, {local_y:.2f}) | "
        f"curv: {curvature:.3f} | R: {v_right:.2f} | L: {v_left:.2f}",
        throttle_duration_sec=0.5
    )

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    global pubs

    rclpy.init(args=args)
    node = Node("pure_pursuit_odrive")

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
    node.create_subscription(Bool, "/pursuit_enabled", safety_callback, 10)
    node.create_timer(0.1, lambda: control_loop(node))

    node.get_logger().info("🚀 Pure Pursuit controller started!")
    play_sound(Start_sound)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()