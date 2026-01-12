#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from odrive_can.msg import ControllerStatus
from tf_transformations import quaternion_from_euler
import math

# Constants
WHEEL_CIRCUMFERENCE = 0.695  # meters
WHEELBASE = 0.665  # meters

# Global state
x = 0.0
y = 0.0
theta = 0.0

# Previous encoder positions (revolutions)
prev_left_pos = [0.0, 0.0, 0.0]
prev_right_pos = [0.0, 0.0, 0.0]

# Current deltas (revolutions) - NOT meters yet
delta_left_rev = [0.0, 0.0, 0.0]
delta_right_rev = [0.0, 0.0, 0.0]

wheels_updated = [False] * 6
first_read = [True] * 6
node = None
odom_pub = None


def calc_distance(d_left, d_right):
    return (d_left + d_right) / 2


def calc_delta_theta(d_left, d_right):
    return (d_right - d_left) / WHEELBASE


def update_pose(d_center, delta_theta):
    global x, y, theta
    
    # Update heading first
    theta += delta_theta
    
    # Calculate movement in current heading direction
    delta_x = d_center * math.cos(theta)
    delta_y = d_center * math.sin(theta)
    
    x += delta_x
    y += delta_y


def publish_odom():
    global x, y, theta, node, odom_pub
    
    odom = Odometry()
    odom.header.stamp = node.get_clock().now().to_msg()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    
    q = quaternion_from_euler(0, 0, theta)
    odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
    odom.pose.covariance[14] = 999999.0
    odom.pose.covariance[21] = 999999.0
    odom.pose.covariance[28] = 999999.0
    
    odom_pub.publish(odom)


def feedback_callback(msg, axis):
    global prev_left_pos, prev_right_pos, first_read
    global delta_left_rev, delta_right_rev, wheels_updated
    
    current_pos = msg.pos_estimate  # revolutions
    
    # Initialize
    if first_read[axis]:
        if axis in [0, 1, 2]:
            prev_right_pos[axis] = current_pos
        else:
            prev_left_pos[axis - 3] = current_pos
        first_read[axis] = False
        return
    
    # Calculate delta in REVOLUTIONS
    if axis in [0, 1, 2]:  # Right wheels
        delta_rev = current_pos - prev_right_pos[axis]
        prev_right_pos[axis] = current_pos
        delta_right_rev[axis] = delta_rev
        
    else:  # Left wheels - NEGATE because opposite mounting
        delta_rev = current_pos - prev_left_pos[axis - 3]
        prev_left_pos[axis - 3] = current_pos
        delta_left_rev[axis - 3] = -delta_rev  # NEGATE HERE
    
    wheels_updated[axis] = True
    
    # Only calculate when ALL wheels updated
    if all(wheels_updated):
        calculate_odometry()
        wheels_updated[:] = [False] * 6


def calculate_odometry():
    global delta_left_rev, delta_right_rev, node
    
    # Average revolutions per side
    avg_left_rev = sum(delta_left_rev) / 3
    avg_right_rev = sum(delta_right_rev) / 3
    
    # Convert to meters
    d_left = avg_left_rev * WHEEL_CIRCUMFERENCE
    d_right = avg_right_rev * WHEEL_CIRCUMFERENCE
    
    # Debug logging
    node.get_logger().info(f"L_rev={avg_left_rev:.4f}, R_rev={avg_right_rev:.4f}, d_L={d_left:.4f}m, d_R={d_right:.4f}m")
    
    # Calculate movement
    d_center = calc_distance(d_left, d_right)
    delta_theta = calc_delta_theta(d_left, d_right)
    
    node.get_logger().info(f"d_center={d_center:.4f}m, delta_theta={delta_theta:.4f}rad")
    
    # Update pose
    update_pose(d_center, delta_theta)
    
    # Log final position
    node.get_logger().info(f"Position: x={x:.3f}, y={y:.3f}, theta={theta:.3f}\n")
    
    # Reset deltas
    delta_left_rev[:] = [0.0, 0.0, 0.0]
    delta_right_rev[:] = [0.0, 0.0, 0.0]
    
    publish_odom()


def main(args=None):
    global node, odom_pub
    
    rclpy.init(args=args)
    node = Node("odom_node")
    
    for i in range(6):
        node.create_subscription(
            ControllerStatus,
            f'/odrive_axis{i}/controller_status',
            lambda msg, axis=i: feedback_callback(msg, axis),
            10
        )
    
    odom_pub = node.create_publisher(Odometry, 'odom', 10)
    
    node.get_logger().info("✅ Odometry node started")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()