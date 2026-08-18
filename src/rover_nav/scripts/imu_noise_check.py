#!/usr/bin/env python3
"""Records yaw/vyaw/x/y from /odometry/filtered over a fixed window and saves
them to CSV, for characterizing IMU behavior while the rover is actually
driving -- as opposed to the original stationary-only noise check. A
stationary IMU can look excellent (see CONFIGURED_YAW_VARIANCE /
CONFIGURED_VYAW_VARIANCE, measured this way) while still drifting
meaningfully once real driving vibration and motor current are involved,
which a variance-only stationary test can't see.

  ros2 run rover_nav imu_noise_check.py
  ros2 run rover_nav imu_noise_check.py --ros-args -p duration_s:=60.0 -p csv_path:=/tmp/run1.csv

Start the node, then drive (or keep the rover still, for a baseline) for the
whole window. The CSV has one row per /odometry/filtered message:
t_s,yaw_rad,yaw_deg,vyaw_rad_s,vyaw_deg_s,x_m,y_m -- t_s is seconds since the
first message. A steady, non-zero slope in yaw over t during a straight-line
drive (no turning) is a bias, not noise -- the linear-fit report at the end
estimates that slope directly.
"""
import csv
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ekf_config.yaml's process_noise_covariance diagonal, yaw (row 5) and vyaw
# (row 11) of the 15x15 [x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,
# ax,ay,az] state -- measured stationary. Kept here as a reference point for
# how much worse (if at all) driving noise/bias is than the stationary
# baseline they were tuned from.
CONFIGURED_YAW_VARIANCE = 0.004
CONFIGURED_VYAW_VARIANCE = 0.0015


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def linfit_slope(xs, ys):
    """Least-squares slope of ys vs xs (no numpy dependency)."""
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    den = sum((x - mean_x) ** 2 for x in xs)
    if den == 0:
        return 0.0
    return num / den


class ImuNoiseCheck(Node):
    def __init__(self):
        super().__init__("imu_noise_check")
        self.declare_parameter("duration_s", 60.0)
        self.declare_parameter(
            "csv_path", f"/tmp/imu_noise_check_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.t = []
        self.yaw_samples = []
        self.vyaw_samples = []
        self.x_samples = []
        self.y_samples = []
        self.start_time = None

        self.create_subscription(Odometry, "/odometry/filtered", self._cb, 10)
        self.timer = self.create_timer(0.5, self._check_done)
        self.get_logger().info(
            f"Recording /odometry/filtered for {self.duration_s:.0f}s -> {self.csv_path}"
        )

    def _cb(self, msg):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        q = msg.pose.pose.orientation
        self.t.append((now - self.start_time).nanoseconds / 1e9)
        self.yaw_samples.append(quat_to_yaw(q.x, q.y, q.z, q.w))
        self.vyaw_samples.append(msg.twist.twist.angular.z)
        self.x_samples.append(msg.pose.pose.position.x)
        self.y_samples.append(msg.pose.pose.position.y)

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
        self._save_csv()
        self._report()
        self.timer.cancel()
        rclpy.shutdown()

    def _save_csv(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_s", "yaw_rad", "yaw_deg", "vyaw_rad_s", "vyaw_deg_s", "x_m", "y_m"])
            for t, yaw, vyaw, x, y in zip(
                self.t, self.yaw_samples, self.vyaw_samples, self.x_samples, self.y_samples
            ):
                writer.writerow(
                    [f"{t:.3f}", f"{yaw:.6f}", f"{math.degrees(yaw):.4f}",
                     f"{vyaw:.6f}", f"{math.degrees(vyaw):.4f}",
                     f"{x:.4f}", f"{y:.4f}"]
                )
        self.get_logger().info(f"Saved {len(self.t)} rows to {self.csv_path}")

    def _report(self):
        n = len(self.yaw_samples)
        if n < 2:
            self.get_logger().error(f"Only {n} sample(s) received -- can't compute stats.")
            return

        yaw_var = statistics.variance(self.yaw_samples)
        vyaw_var = statistics.variance(self.vyaw_samples)
        yaw_std_deg = math.degrees(math.sqrt(yaw_var))
        vyaw_std_degs = math.degrees(math.sqrt(vyaw_var))
        yaw_slope_deg_s = math.degrees(linfit_slope(self.t, self.yaw_samples))
        elapsed = self.t[-1] - self.t[0]

        self.get_logger().info(
            "\n"
            f"=== {n} samples over {elapsed:.1f}s -> {self.csv_path} ===\n"
            f"yaw:  variance={yaw_var:.3e} rad^2 (std={yaw_std_deg:.4f} deg) | "
            f"stationary reference={CONFIGURED_YAW_VARIANCE}\n"
            f"vyaw: variance={vyaw_var:.3e} (rad/s)^2 (std={vyaw_std_degs:.4f} deg/s) | "
            f"stationary reference={CONFIGURED_VYAW_VARIANCE}\n"
            f"yaw linear drift rate: {yaw_slope_deg_s:.4f} deg/s "
            f"(-> {yaw_slope_deg_s * elapsed:.3f} deg over this run's {elapsed:.1f}s -- "
            "meaningful only if the rover was driving straight, no turning, for this whole window)"
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
