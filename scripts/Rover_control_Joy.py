#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from sensor_msgs.msg import Joy
import time
import subprocess

# Global variables for velocities
right_velocity = 0.0
left_velocity = 0.0

# Path to sound file
B_SOUND_FILE = "/home/daino/colcon_ws/src/ERC/ERC/Sounds/walle.wav"

# Track previous button state
prev_B_button = 0

def play_sound(file_path):
    """Play a sound using aplay (non-blocking)."""
    subprocess.Popen(["aplay", file_path])

def joy_callback(joy_msg):
    global right_velocity, left_velocity, node, prev_B_button

    vertical = -joy_msg.axes[3]      # forward/backward
    horizontal = joy_msg.axes[2]     # turning
    B_button = joy_msg.buttons[1]

    node.get_logger().info(f"vertical={vertical:.3f}, horizontal={horizontal:.3f}")

    # Differential drive + inverted left side
    right_velocity = -(vertical - horizontal)
    left_velocity = (vertical + horizontal)

    # Play sound only when B button is pressed (0 → 1)
    if B_button == 1 and prev_B_button == 0:
        play_sound(B_SOUND_FILE)

    # Update previous button state
    prev_B_button = B_button

def main(args=None):
    global right_velocity, left_velocity, node

    rclpy.init(args=args)
    node = Node("six_wheel_controller")

    num_axes = 6
    right_wheels = [0, 1 , 2]
    left_wheels = [3, 4 ,5]

    # Create publishers and clients
    clients = []
    pubs = []
    for i in range(num_axes):
        srv_name = f"/odrive_axis{i}/request_axis_state"
        topic_name = f"/odrive_axis{i}/control_message"
        clients.append(node.create_client(AxisState, srv_name))
        pubs.append(node.create_publisher(ControlMessage, topic_name, 10))

    # Wait for all services
    for i, client in enumerate(clients):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for service {client.srv_name}...")

    # Set CLOSED_LOOP_CONTROL mode
    for i, client in enumerate(clients):
        req = AxisState.Request()
        req.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
        node.get_logger().info(f"Setting odrive_axis{i} to CLOSED_LOOP_CONTROL...")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(0.1)

    # Subscribe to joystick
    node.create_subscription(Joy, "/joy", joy_callback, 10)
    node.get_logger().info("✅ Ready to receive joystick input.")

    # Main loop: publish continuously
    publish_rate_hz = 20.0
    publish_period = 1.0 / publish_rate_hz

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            # Create messages
            right_msg = ControlMessage()
            right_msg.control_mode = 2
            right_msg.input_mode = 1
            right_msg.input_pos = 0.0
            right_msg.input_vel = right_velocity
            right_msg.input_torque = 0.0

            left_msg = ControlMessage()
            left_msg.control_mode = 2
            left_msg.input_mode = 1
            left_msg.input_pos = 0.0
            left_msg.input_vel = left_velocity
            left_msg.input_torque = 0.0

            # Publish to both sides
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
