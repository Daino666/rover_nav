#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import time
import subprocess

# Configuration
MAX_VELOCITY = 1.5
TURN_VELOCITY_180 = 1.0
TURN_DURATION = 2.1
TURN_BOOST = 1.5
TURN_THRESHOLD = 0.2
DEADZONE = 0.08
ACCEL_LIMIT = 3.0

Stop_sound = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/Stop.wav"
Start_sound = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/Start.wav"

# Global state
target_right_velocity = 0.0
target_left_velocity = 0.0
trigger = 0
turn_button = 0
prev_turn_button = 0
prev_B_button = 0
prev_Y_button = 0
prev_X_button = 0
is_turning = False
turn_start_time = 0.0
node = None
current_right_velocity = 0.0
current_left_velocity = 0.0
pursuit_pub = None

# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════

def apply_deadzone(value, threshold=DEADZONE):
    if abs(value) < threshold:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)

def play_sound(file_path):
    subprocess.Popen(["aplay", file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def ramp_velocity(current, target, dt, max_accel=ACCEL_LIMIT):
    max_change = max_accel * dt
    diff = target - current
    if abs(diff) < max_change:
        return target
    return current + (max_change if diff > 0 else -max_change)

# ═══════════════════════════════════════════════════════════════
# CALLBACKS
# ═══════════════════════════════════════════════════════════════

def joy_callback(joy_msg):
    global target_right_velocity, target_left_velocity, node
    global trigger, turn_button, prev_turn_button, is_turning, turn_start_time
    global prev_B_button, prev_Y_button, prev_X_button, pursuit_pub

    vertical   = apply_deadzone(-joy_msg.axes[3])
    horizontal = apply_deadzone(joy_msg.axes[2])
    Y_button   = joy_msg.buttons[4]
    X_button   = joy_msg.buttons[3]
    trigger    = joy_msg.buttons[7]

    # Safety stop
    if Y_button == 1 and prev_Y_button == 0:
        pursuit_pub.publish(Bool(data=False))
        play_sound(Stop_sound)
        node.get_logger().info("🛑 Pure Pursuit STOPPED")

    # Resume
    if X_button == 1 and prev_X_button == 0:
        pursuit_pub.publish(Bool(data=True))
        play_sound(Start_sound)
        node.get_logger().info("✅ Pure Pursuit Resumed")

    prev_Y_button = Y_button
    prev_X_button = X_button

    # Drive mode
    if abs(vertical) < TURN_THRESHOLD and abs(horizontal) > 0.1:
        turn_vel = horizontal * MAX_VELOCITY * TURN_BOOST
        target_right_velocity = turn_vel
        target_left_velocity  = turn_vel
        node.get_logger().info(f"🔄 SPIN: {turn_vel:.2f} rev/s", throttle_duration_sec=1.0)
    else:
        target_right_velocity = -(vertical - horizontal) * MAX_VELOCITY
        target_left_velocity  =  (vertical + horizontal) * MAX_VELOCITY

    # 180° turn
    if turn_button == 1 and prev_turn_button == 0 and not is_turning:
        is_turning = True
        turn_start_time = time.time()
        node.get_logger().info("🔄 Starting 180° turn...")
    prev_turn_button = turn_button




# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    global target_right_velocity, target_left_velocity, node
    global trigger, is_turning, turn_start_time
    global current_right_velocity, current_left_velocity
    global pursuit_pub

    rclpy.init(args=args)
    node = Node("six_wheel_controller")

    right_wheels = [0, 1, 2]
    left_wheels  = [3, 4, 5]

    pubs = []
    for i in range(6):
        pubs.append(node.create_publisher(ControlMessage, f"/odrive_axis{i}/control_message", 10))
    
    pursuit_pub = node.create_publisher(Bool, "/pursuit_enabled", 10)

    node.create_subscription(Joy, "/joy", joy_callback, 10)
    node.get_logger().info("✅ Six-wheel controller ready!")

    publish_rate_hz = 20.0
    publish_period  = 1.0 / publish_rate_hz

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            dt = publish_period

            if is_turning:
                elapsed = time.time() - turn_start_time
                if elapsed < TURN_DURATION:
                    target_vel_right = TURN_VELOCITY_180
                    target_vel_left  = TURN_VELOCITY_180
                else:
                    is_turning = False
                    target_vel_right = 0.0
                    target_vel_left  = 0.0
                    node.get_logger().info("✅ 180° turn complete!")
            elif trigger == 1:
                target_vel_right = target_right_velocity
                target_vel_left  = target_left_velocity
            else:
                target_vel_right = 0.0
                target_vel_left  = 0.0

            current_right_velocity = ramp_velocity(current_right_velocity, target_vel_right, dt)
            current_left_velocity  = ramp_velocity(current_left_velocity,  target_vel_left,  dt)

            right_msg = ControlMessage()
            right_msg.control_mode = 2
            right_msg.input_mode   = 1
            right_msg.input_pos    = 0.0
            right_msg.input_vel    = current_right_velocity
            right_msg.input_torque = 0.0

            left_msg = ControlMessage()
            left_msg.control_mode = 2
            left_msg.input_mode   = 1
            left_msg.input_pos    = 0.0
            left_msg.input_vel    = current_left_velocity
            left_msg.input_torque = 0.0

            for i in right_wheels:
                pubs[i].publish(right_msg)
            for i in left_wheels:
                pubs[i].publish(left_msg)

            time.sleep(publish_period)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()