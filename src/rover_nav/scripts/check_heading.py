#!/usr/bin/env python3
"""Live-prints fused EKF yaw and position from /odometry/filtered, for
diagnosing an IMU-mounting/heading offset. See "Diagnosing an IMU/heading
mounting offset" in README.md for the full procedure -- in short: drive the
rover straight forward a couple meters and check which axis (x or y) actually
changes, since yaw alone can be misleading (it typically reads ~0 at launch
regardless of true physical orientation, since there's no absolute compass
reference -- 0 just means "wherever it was facing at boot").

  ros2 run rover_nav check_heading.py
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingCheck(Node):
    def __init__(self):
        super().__init__("heading_check")
        self.create_subscription(Odometry, "/odometry/filtered", self._cb, 10)
        self.get_logger().info("Waiting for /odometry/filtered ...")

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        yaw_deg = math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print(f"yaw: {yaw_deg:7.2f} deg | x: {x:7.3f} m | y: {y:7.3f} m", end="\r")


def main():
    rclpy.init()
    node = HeadingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
