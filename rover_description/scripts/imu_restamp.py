#!/usr/bin/env python3
"""
imu_restamp.py

The MPU6050 firmware talks to ROS through the micro-ROS agent and stamps
/imu with its own boot-relative clock (seconds since MCU power-on), not
synced ROS wall-clock time. robot_localization's ekf_node computes its
predict-step dt from message timestamps, so mixing these boot-relative
/imu stamps with real-time-stamped /rover_controller/odom produces a
multi-billion-second dt, blows up the filter covariance, and the very
next correction snaps /odometry/filtered's x/y to huge garbage values
even while the rover is stationary.

This node re-stamps every /imu message with the host's synced clock
before EKF sees it. Proper fix is syncing the MCU clock in firmware
(rmw_uros_sync_session), but that firmware lives outside this repo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRestamp(Node):
    def __init__(self):
        super().__init__("imu_restamp")
        self.publisher = self.create_publisher(Imu, "/imu/synced", 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.get_logger().info("Re-stamping /imu -> /imu/synced with host clock")

    def imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRestamp()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
