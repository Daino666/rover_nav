#!/usr/bin/env python3
"""Live-prints yaw from three sources side by side, to localize WHERE drift
actually originates instead of only seeing the end result:

  raw   /microstrain/imu/data      -- raw sensor + onboard AHRS orientation
  dev   /microstrain/ekf/imu/data  -- MicroStrain driver's own internal
                                       estimation filter (not ours)
  fused /odometry/filtered         -- robot_localization's EKF output (also
                                       prints x/y position from this one)

If `raw` and `dev` already drift while driving, the problem is at the sensor
(e.g. motor-current EMI corrupting the magnetometer). If they're stable but
`fused` still drifts, the problem is in robot_localization's own fusion
instead. See "Diagnosing an IMU/heading mounting offset" in README.md for
the full offset-check procedure.

  ros2 run rover_nav check_heading.py
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingCheck(Node):
    def __init__(self):
        super().__init__("heading_check")
        self.raw_yaw = None
        self.dev_yaw = None
        self.fused_yaw = None
        self.x = None
        self.y = None

        self.create_subscription(Imu, "/microstrain/imu/data", self._on_raw, 10)
        self.create_subscription(Imu, "/microstrain/ekf/imu/data", self._on_dev, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_fused, 10)
        self.get_logger().info(
            "Waiting for /microstrain/imu/data, /microstrain/ekf/imu/data, "
            "/odometry/filtered ..."
        )

    def _on_raw(self, msg):
        q = msg.orientation
        self.raw_yaw = math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))
        self._print()

    def _on_dev(self, msg):
        q = msg.orientation
        self.dev_yaw = math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))
        self._print()

    def _on_fused(self, msg):
        q = msg.pose.pose.orientation
        self.fused_yaw = math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self._print()

    def _print(self):
        def fmt(v):
            return f"{v:7.2f}" if v is not None else "   n/a "

        x = f"{self.x:7.3f}" if self.x is not None else "   n/a "
        y = f"{self.y:7.3f}" if self.y is not None else "   n/a "
        print(
            f"raw: {fmt(self.raw_yaw)} deg | dev: {fmt(self.dev_yaw)} deg | "
            f"fused: {fmt(self.fused_yaw)} deg | x: {x} m | y: {y} m",
            end="\r",
        )


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
