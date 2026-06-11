#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
import math
import time
import subprocess

# ═══════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════

BASE_VELOCITY  = 0.2          # m/s   forward speed in autonomous mode
GOAL_TOLERANCE = 0.3          # m     how close counts as "reached" a waypoint
STOP_DURATION  = 1.0          # s     pause at each waypoint before moving on
MAX_CURVATURE  = 2.0          # 1/m

MAX_LINEAR     = 1.0          # m/s   manual full-stick forward speed
MAX_ANGULAR    = 2.0          # rad/s manual full-stick turn rate
TURN_DURATION  = 2.1          # s     duration of the timed 180° turn
TURN_RATE_180  = math.pi / TURN_DURATION   # rad/s to sweep ~180° in TURN_DURATION
TURN_THRESHOLD = 0.2
DEADZONE       = 0.08

CMD_VEL_TOPIC  = "rover_controller/cmd_vel"   # ros2_control diff_drive_controller (TwistStamped)

path = [[3.0, 0.0], [5.0, 1.425]]

Stop_sound  = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/Stop.wav"
Start_sound = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/Start.wav"

# ═══════════════════════════════════════════════════════════════
# STATE
# ═══════════════════════════════════════════════════════════════

car_yaw            = None
car_global_axis    = None
cmd_vel_pub        = None
pursuit_enabled    = True
current_target_idx = 0
waypoint_pause_until = 0.0

target_linear    = 0.0
target_angular   = 0.0
trigger          = 0
turn_button      = 0
prev_turn_button = 0
prev_Y_button    = 0
prev_X_button    = 0
is_turning       = False
turn_start_time  = 0.0
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

def publish_cmd_vel(linear, angular):
    if cmd_vel_pub is None:
        return
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.twist.linear.x  = float(linear)
    msg.twist.angular.z = float(angular)
    cmd_vel_pub.publish(msg)

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
    global target_linear, target_angular
    global trigger, turn_button, prev_turn_button
    global prev_Y_button, prev_X_button
    global is_turning, turn_start_time
    global pursuit_enabled, current_target_idx, node

    vertical    = apply_deadzone(-joy_msg.axes[3])   # forward / back
    horizontal  = apply_deadzone( joy_msg.axes[2])   # turn left / right
    Y_button    = joy_msg.buttons[4]
    X_button    = joy_msg.buttons[3]
    trigger     = joy_msg.buttons[7]
    turn_button = joy_msg.buttons[6]

    if Y_button == 1 and prev_Y_button == 0:
        pursuit_enabled = False
        play_sound(Stop_sound)
        node.get_logger().info("🛑 Switched to MANUAL control")

    if X_button == 1 and prev_X_button == 0:
        pursuit_enabled = True
        current_target_idx = 0
        play_sound(Start_sound)
        node.get_logger().info("🚀 Switched to AUTONOMOUS control")

    prev_Y_button = Y_button
    prev_X_button = X_button

    # Map joystick to a robot-frame velocity command.
    # linear.x > 0 = forward, angular.z > 0 = turn left (CCW). Flip the
    # horizontal sign here if the rover turns the wrong way.
    target_linear  = vertical   * MAX_LINEAR
    target_angular = horizontal * MAX_ANGULAR

    if turn_button == 1 and prev_turn_button == 0 and not is_turning:
        is_turning = True
        turn_start_time = time.time()
        node.get_logger().info("🔄 Starting 180° turn...")
    prev_turn_button = turn_button

# ═══════════════════════════════════════════════════════════════
# PURE PURSUIT
# ═══════════════════════════════════════════════════════════════

def pursuit_control():
    global current_target_idx, waypoint_pause_until

    if car_yaw is None or car_global_axis is None:
        node.get_logger().warn("Waiting for odometry...", throttle_duration_sec=2.0)
        publish_cmd_vel(0.0, 0.0)
        return

    # Finished the whole path
    if current_target_idx >= len(path):
        node.get_logger().info("✅ All waypoints reached!", throttle_duration_sec=1.0)
        publish_cmd_vel(0.0, 0.0)
        return

    # Pausing at a waypoint we just reached
    now = time.time()
    if now < waypoint_pause_until:
        publish_cmd_vel(0.0, 0.0)
        return

    # Drive toward the CURRENT waypoint (one at a time, in order)
    target = path[current_target_idx]
    dist   = distance(car_global_axis, target)

    if dist < GOAL_TOLERANCE:
        # Reached this waypoint: stop, pause, then advance to the next one
        publish_cmd_vel(0.0, 0.0)
        node.get_logger().info(f"🛑 Reached waypoint {current_target_idx}: {target}")
        waypoint_pause_until = now + STOP_DURATION
        current_target_idx += 1
        return

    local_x, local_y = point_global_to_local(target, car_yaw, car_global_axis)
    curvature = float(np.clip(calc_curv(local_x, local_y), -MAX_CURVATURE, MAX_CURVATURE))
    angular   = curvature * BASE_VELOCITY

    publish_cmd_vel(BASE_VELOCITY, angular)

    node.get_logger().info(
        f"pos: {car_global_axis} | target[{current_target_idx}]: {target} | "
        f"dist: {dist:.2f} | curv: {curvature:.3f}",
        throttle_duration_sec=0.5
    )

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    global node, cmd_vel_pub
    global is_turning, turn_start_time, trigger

    rclpy.init(args=args)
    node = Node("rover_controller")

    cmd_vel_pub = node.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

    node.create_subscription(Odometry, "/odometry/filtered", odom_callback, 10)
    node.create_subscription(Joy, "/joy", joy_callback, 10)

    play_sound(Start_sound)
    node.get_logger().info("🚀 Rover controller started! (AUTONOMOUS mode)")

    publish_period = 1.0 / 20.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if pursuit_enabled:
                pursuit_control()
            else:
                if is_turning:
                    elapsed = time.time() - turn_start_time
                    if elapsed < TURN_DURATION:
                        linear  = 0.0
                        angular = TURN_RATE_180
                    else:
                        is_turning = False
                        linear  = 0.0
                        angular = 0.0
                        node.get_logger().info("✅ 180° turn complete!")
                elif trigger == 1:
                    linear  = target_linear
                    angular = target_angular
                else:
                    linear  = 0.0
                    angular = 0.0

                publish_cmd_vel(linear, angular)

            time.sleep(publish_period)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
