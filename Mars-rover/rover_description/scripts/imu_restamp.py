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

It also guards against all-zero covariance matrices (the Gazebo IMU
sensor plugin reports these in sim; some firmware/drivers do on real
hardware too). robot_localization treats an all-zero covariance as
near-infinite confidence: the EKF's internal covariance for that
variable collapses toward zero after a few updates, and its outlier
Mahalanobis-distance gate then rejects every subsequent measurement as
an outlier -- the filter's estimate silently freezes while the real
sensor keeps moving. Rows that come in all-zero are replaced with a
small fixed variance so the filter keeps accepting updates.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Fallback variance (rad^2 or (rad/s)^2) substituted for any 3x3 orientation/
# angular_velocity/linear_acceleration covariance block that arrives all-zero.
DEFAULT_COVARIANCE_VALUE = 1e-3


def _fix_covariance(covariance):
    if all(v == 0.0 for v in covariance):
        fixed = [0.0] * 9
        fixed[0] = fixed[4] = fixed[8] = DEFAULT_COVARIANCE_VALUE
        return fixed
    return covariance


class ImuRestamp(Node):
    def __init__(self):
        super().__init__("imu_restamp")
        self.publisher = self.create_publisher(Imu, "/imu/synced", 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.get_logger().info("Re-stamping /imu -> /imu/synced with host clock")

    def imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation_covariance = _fix_covariance(msg.orientation_covariance)
        msg.angular_velocity_covariance = _fix_covariance(msg.angular_velocity_covariance)
        msg.linear_acceleration_covariance = _fix_covariance(msg.linear_acceleration_covariance)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRestamp()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
