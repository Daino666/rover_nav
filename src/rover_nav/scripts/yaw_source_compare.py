#!/usr/bin/env python3
"""Logs yaw from the IMU's own AHRS output (pre-robot_localization) alongside
the EKF's fused yaw, side by side, to tell apart two different explanations
for yaw swinging away from true while driving and recovering once stopped:

  - the AHRS/magnetometer itself gets disturbed while driving (sensor-level,
    e.g. motor-current EMI -- already anticipated in aries_imu/config/
    microstrain.yaml's comments), or
  - robot_localization's own EKF fusion introduces or amplifies something
    beyond what the AHRS already reports (filter-level).

If both curves swing together, it's the sensor. If the AHRS stays much
flatter while only the fused output swings, it's the filter.

  ros2 run rover_nav yaw_source_compare.py
  ros2 run rover_nav yaw_source_compare.py --ros-args -p duration_s:=60.0 -p csv_path:=/tmp/yaw_compare1.csv

Run this alongside the same driving test as imu_noise_check.py -- ideally at
the same time, on the same drive, so both CSVs cover the same window.
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class YawSourceCompare(Node):
    def __init__(self):
        super().__init__("yaw_source_compare")
        self.declare_parameter("duration_s", 60.0)
        self.declare_parameter("ahrs_topic", "/microstrain/ekf/imu/data")
        self.declare_parameter(
            "csv_path", f"/tmp/yaw_compare_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.ahrs_topic = str(self.get_parameter("ahrs_topic").value)
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.start_time = None
        self.rows = []  # (t, fused_yaw, ahrs_yaw)
        self._latest_ahrs_yaw = None

        self.create_subscription(Imu, self.ahrs_topic, self._ahrs_cb, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._fused_cb, 10)
        self.timer = self.create_timer(0.5, self._check_done)
        self.get_logger().info(
            f"Comparing {self.ahrs_topic} (AHRS, pre-EKF) vs /odometry/filtered "
            f"(fused) for {self.duration_s:.0f}s -> {self.csv_path}"
        )

    def _ahrs_cb(self, msg):
        q = msg.orientation
        self._latest_ahrs_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _fused_cb(self, msg):
        if self._latest_ahrs_yaw is None:
            return  # wait until we have at least one AHRS sample to pair with
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        q = msg.pose.pose.orientation
        fused_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        t = (now - self.start_time).nanoseconds / 1e9
        self.rows.append((t, fused_yaw, self._latest_ahrs_yaw))

    def _check_done(self):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration_s:
            remaining = self.duration_s - elapsed
            self.get_logger().info(
                f"...{remaining:.0f}s left, {len(self.rows)} samples so far",
                throttle_duration_sec=5.0,
            )
            return
        self._save_and_report()
        self.timer.cancel()
        rclpy.shutdown()

    def _save_and_report(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_s", "fused_yaw_deg", "ahrs_yaw_deg", "diff_deg"])
            for t, fused, ahrs in self.rows:
                diff = math.degrees(fused - ahrs)
                writer.writerow(
                    [f"{t:.3f}", f"{math.degrees(fused):.4f}",
                     f"{math.degrees(ahrs):.4f}", f"{diff:.4f}"]
                )
        n = len(self.rows)
        if n == 0:
            self.get_logger().error("No paired samples recorded.")
            return
        diffs = [math.degrees(f - a) for _, f, a in self.rows]
        max_abs_diff = max(abs(d) for d in diffs)
        fused_swing = math.degrees(max(f for _, f, _ in self.rows) - min(f for _, f, _ in self.rows))
        ahrs_swing = math.degrees(max(a for _, _, a in self.rows) - min(a for _, _, a in self.rows))
        self.get_logger().info(
            "\n"
            f"=== {n} samples -> {self.csv_path} ===\n"
            f"fused yaw total swing: {fused_swing:.2f} deg\n"
            f"AHRS yaw total swing:  {ahrs_swing:.2f} deg\n"
            f"max |fused - ahrs| divergence: {max_abs_diff:.2f} deg\n"
            "If fused and AHRS swings are similar, the disturbance is at the "
            "sensor level. If fused swings much more than AHRS, the EKF "
            "fusion itself is introducing/amplifying it."
        )


def main(args=None):
    rclpy.init(args=args)
    node = YawSourceCompare()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
