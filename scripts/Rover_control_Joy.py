#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from sensor_msgs.msg import Joy
import time
import subprocess

# Configuration
MAX_VELOCITY = 1.5           # rev/s - increased from default
TURN_VELOCITY_180 = 1.0      # rev/s for 180° turn
TURN_DURATION = 2.1          # seconds for 180° turn
TURN_BOOST = 1.5              # Multiplier for joystick turns
TURN_THRESHOLD = 0.2         # Below this = spin in place
DEADZONE = 0.08              # Ignore small joystick drift

# Sound
B_SOUND_FILE = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/Merry_Chirstmas.wav"

# Global state
target_right_velocity = 0.0
target_left_velocity = 0.0
trigger = 0
turn_button = 0
prev_turn_button = 0
prev_B_button = 0
is_turning = False
turn_start_time = 0.0
node = None
current_right_velocity = 0.0
current_left_velocity = 0.0
ACCEL_LIMIT = 3.0  # rev/s² - adjust this for ramp speed

def apply_deadzone(value, threshold=DEADZONE):
    """Remove joystick drift"""
    if abs(value) < threshold:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)

def play_sound(file_path):
    subprocess.Popen(["aplay", file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def ramp_velocity(current, target, dt, max_accel=ACCEL_LIMIT):
    """Smooth acceleration"""
    max_change = max_accel * dt
    diff = target - current
    if abs(diff) < max_change:
        return target
    return current + (max_change if diff > 0 else -max_change)

def joy_callback(joy_msg):
    global target_right_velocity, target_left_velocity, node
    global trigger, turn_button, prev_turn_button, is_turning, turn_start_time, prev_B_button

    # Read joystick
    vertical = apply_deadzone(-joy_msg.axes[3])
    horizontal = apply_deadzone(joy_msg.axes[2])
    B_button = joy_msg.buttons[1]
    trigger = joy_msg.buttons[7]
    turn_button = joy_msg.buttons[6]

    # Determine mode: spin vs drive
    if abs(vertical) < TURN_THRESHOLD and abs(horizontal) > 0.1:
        # SPIN IN PLACE - same velocity, same sign
        turn_vel = horizontal * MAX_VELOCITY * TURN_BOOST
        target_right_velocity = -turn_vel
        target_left_velocity = -turn_vel
        node.get_logger().info(f"🔄 SPIN: {turn_vel:.2f} rev/s", throttle_duration_sec=1.0)
    else:
        # NORMAL DRIVE
        target_right_velocity = -(vertical - horizontal) * MAX_VELOCITY
        target_left_velocity = (vertical + horizontal) * MAX_VELOCITY

    # 180° turn button
    if turn_button == 1 and prev_turn_button == 0 and not is_turning:
        is_turning = True
        turn_start_time = time.time()
        node.get_logger().info("🔄 Starting 180° turn...")
    prev_turn_button = turn_button

    # Sound
    if B_button == 1 and prev_B_button == 0:
        play_sound(B_SOUND_FILE)
    prev_B_button = B_button

def main(args=None):
    global target_right_velocity, target_left_velocity, node
    global trigger, is_turning, turn_start_time
    global current_right_velocity, current_left_velocity


    rclpy.init(args=args)
    node = Node("six_wheel_controller")

    num_axes = 6
    right_wheels = [0, 1, 2]
    left_wheels = [3, 4, 5]

    # Setup ODrive clients and publishers
    clients = []
    pubs = []
    for i in range(num_axes):
        srv_name = f"/odrive_axis{i}/request_axis_state"
        topic_name = f"/odrive_axis{i}/control_message"
        clients.append(node.create_client(AxisState, srv_name))
        pubs.append(node.create_publisher(ControlMessage, topic_name, 10))

    # Wait for services
    for i, client in enumerate(clients):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for {client.srv_name}...")

    # Set to CLOSED_LOOP_CONTROL
    for i, client in enumerate(clients):
        req = AxisState.Request()
        req.axis_requested_state = 8
        node.get_logger().info(f"Setting odrive_axis{i} to CLOSED_LOOP_CONTROL...")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(0.1)

    node.create_subscription(Joy, "/joy", joy_callback, 10)
    node.get_logger().info("✅ Six-wheel controller ready!")
    node.get_logger().info(f"   Max velocity: {MAX_VELOCITY} rev/s")
    node.get_logger().info(f"   Turn boost: {TURN_BOOST}x")

    publish_rate_hz = 20.0
    publish_period = 1.0 / publish_rate_hz

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Calculate dt for ramping
            dt = publish_period
            
            # Determine target velocities
            if is_turning:
                elapsed = time.time() - turn_start_time
                if elapsed < TURN_DURATION:
                    target_vel_right = TURN_VELOCITY_180
                    target_vel_left = TURN_VELOCITY_180
                else:
                    is_turning = False
                    target_vel_right = 0.0
                    target_vel_left = 0.0
                    node.get_logger().info("✅ 180° turn complete!")
            elif trigger == 1:
                target_vel_right = target_right_velocity
                target_vel_left = target_left_velocity
            else:
                target_vel_right = 0.0
                target_vel_left = 0.0
            
            # Apply ramping
            current_right_velocity = ramp_velocity(current_right_velocity, target_vel_right, dt)
            current_left_velocity = ramp_velocity(current_left_velocity, target_vel_left, dt)
            
            # Create messages
            right_msg = ControlMessage()
            right_msg.control_mode = 2
            right_msg.input_mode = 1
            right_msg.input_pos = 0.0
            right_msg.input_vel = current_right_velocity  # Use ramped value
            right_msg.input_torque = 0.0

            left_msg = ControlMessage()
            left_msg.control_mode = 2
            left_msg.input_mode = 1
            left_msg.input_pos = 0.0
            left_msg.input_vel = current_left_velocity   # Use ramped value
            right_msg.input_torque = 0.0

            # Publish
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