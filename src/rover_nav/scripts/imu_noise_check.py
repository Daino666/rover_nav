#!/usr/bin/env python3
"""Records yaw/vyaw from /odometry/filtered for a fixed window while the
rover sits stationary, then reports the empirical noise (variance) against
ekf_config.yaml's process_noise_covariance yaw/vyaw terms -- to check
whether there's real room to tighten them now that the IMU is confirmed
high-quality, rather than guessing.

  ros2 run rover_nav imu_noise_check.py
  ros2 run rover_nav imu_noise_check.py --ros-args -p duration_s:=60.0

Keep the rover completely still for the whole window -- any real motion
(even someone bumping it) will inflate the numbers and make the IMU look
noisier than it actually is.
"""
import math
import statistics

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ekf_config.yaml's process_noise_covariance diagonal, yaw (row 5) and vyaw
# (row 11) of the 15x15 [x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,
# ax,ay,az] state -- keep in sync with that file if it changes.
CONFIGURED_YAW_VARIANCE = 0.004
CONFIGURED_VYAW_VARIANCE = 0.0015


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuNoiseCheck(Node):
    def __init__(self):
        super().__init__("imu_noise_check")
        self.declare_parameter("duration_s", 30.0)
        self.duration_s = float(self.get_parameter("duration_s").value)

        self.yaw_samples = []
        self.vyaw_samples = []
        self.start_time = None

        self.create_subscription(Odometry, "/odometry/filtered", self._cb, 10)
        self.timer = self.create_timer(0.5, self._check_done)
        self.get_logger().info(
            f"Recording yaw/vyaw from /odometry/filtered for {self.duration_s:.0f}s "
            "-- keep the rover completely still."
        )

    def _cb(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        q = msg.pose.pose.orientation
        self.yaw_samples.append(quat_to_yaw(q.x, q.y, q.z, q.w))
        self.vyaw_samples.append(msg.twist.twist.angular.z)

    def _check_done(self):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration_s:
            remaining = self.duration_s - elapsed
            self.get_logger().info(
                f"...{remaining:.0f}s left, {len(self.yaw_samples)} samples so far",
                throttle_duration_sec=5.0,
            )
            return
        self._report()
        self.timer.cancel()
        rclpy.shutdown()

    def _report(self):
        n = len(self.yaw_samples)
        if n < 2:
            self.get_logger().error(f"Only {n} sample(s) received -- can't compute variance.")
            return

        yaw_var = statistics.variance(self.yaw_samples)
        vyaw_var = statistics.variance(self.vyaw_samples)
        yaw_std_deg = math.degrees(math.sqrt(yaw_var))
        vyaw_std_degs = math.degrees(math.sqrt(vyaw_var))

        def verdict(measured, configured):
            if measured < configured * 0.3:
                return "measured << configured -- real room to tighten"
            if measured > configured * 0.8:
                return "measured close to (or above) configured -- already about right"
            return "measured somewhat below configured -- some room, not dramatic"

        self.get_logger().info(
            "\n"
            f"=== {n} samples over {self.duration_s:.0f}s ===\n"
            f"yaw:  variance={yaw_var:.3e} rad^2 (std={yaw_std_deg:.4f} deg) | "
            f"configured process noise={CONFIGURED_YAW_VARIANCE} -> {verdict(yaw_var, CONFIGURED_YAW_VARIANCE)}\n"
            f"vyaw: variance={vyaw_var:.3e} (rad/s)^2 (std={vyaw_std_degs:.4f} deg/s) | "
            f"configured process noise={CONFIGURED_VYAW_VARIANCE} -> {verdict(vyaw_var, CONFIGURED_VYAW_VARIANCE)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuNoiseCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
