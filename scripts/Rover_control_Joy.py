#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from sensor_msgs.msg import Joy
import time
import subprocess

# Global variables for velocities
target_right_velocity = 0.0
target_left_velocity = 0.0
trigger = 0
stop_button = 0

# Path to sound file
B_SOUND_FILE = "/home/daino/colcon_ws/src/rover_nav/scripts/Sounds/walle.wav"

# Track previous button state
prev_B_button = 0

def play_sound(file_path):
    subprocess.Popen(["aplay", file_path])

def joy_callback(joy_msg):
    global target_right_velocity, target_left_velocity, node, prev_B_button, trigger, stop_button

    vertical = -joy_msg.axes[3]
    horizontal =    0 #joy_msg.axes[]
    B_button = joy_msg.buttons[1]
    trigger = joy_msg.buttons[7]
    stop_button = joy_msg.buttons[6]
    
    node.get_logger().info(f"vertical={vertical:.3f}, horizontal={horizontal:.3f}, trigger={trigger}, STOP={stop_button}")

    target_right_velocity = -(vertical - horizontal)
    target_left_velocity = (vertical + horizontal)

    target_right_velocity *= 1.5
    target_left_velocity *= 1.5

    if B_button == 1 and prev_B_button == 0:
        play_sound(B_SOUND_FILE)

    prev_B_button = B_button

def main(args=None):
    global target_right_velocity, target_left_velocity, node, trigger, stop_button

    rclpy.init(args=args)
    node = Node("six_wheel_controller")

    num_axes = 6
    right_wheels = [0, 1, 2]
    left_wheels = [3, 4, 5]

    clients = []
    pubs = []
    for i in range(num_axes):
        srv_name = f"/odrive_axis{i}/request_axis_state"
        topic_name = f"/odrive_axis{i}/control_message"
        clients.append(node.create_client(AxisState, srv_name))
        pubs.append(node.create_publisher(ControlMessage, topic_name, 10))

    for i, client in enumerate(clients):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for service {client.srv_name}...")

    for i, client in enumerate(clients):
        req = AxisState.Request()
        req.axis_requested_state = 8
        node.get_logger().info(f"Setting odrive_axis{i} to CLOSED_LOOP_CONTROL...")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(0.1)

    node.create_subscription(Joy, "/joy", joy_callback, 10)
    node.get_logger().info("✅ Ready to receive joystick input.")

    publish_rate_hz = 20.0
    publish_period = 1.0 / publish_rate_hz

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Determine velocity to send
            if stop_button == 1:
                vel_right = 0.0
                vel_left = 0.0
                node.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED!")
            elif trigger == 1:
                vel_right = target_right_velocity
                vel_left = target_left_velocity
            else:
                vel_right = 0.0
                vel_left = 0.0
            
            # Send directly - ODrive handles ramping
            right_msg = ControlMessage()
            right_msg.control_mode = 2   # Velocity control
            right_msg.input_mode = 2     # VEL_RAMP (ODrive handles smoothing)
            right_msg.input_pos = 0.0
            right_msg.input_vel = vel_right
            right_msg.input_torque = 0.0

            left_msg = ControlMessage()
            left_msg.control_mode = 2
            left_msg.input_mode = 2
            left_msg.input_pos = 0.0
            left_msg.input_vel = vel_left
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