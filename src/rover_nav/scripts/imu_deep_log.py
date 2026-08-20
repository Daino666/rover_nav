#!/usr/bin/env python3
"""Deep IMU diagnostic logger: raw (non-AHRS) gyro rate, AHRS-fused gyro
rate, AHRS yaw, raw magnetometer vector, and the final EKF-fused output, all
in one CSV, timestamped together. Optionally also pairs ground-truth physical
waypoints against the EKF's reported position at the moment you stop there.

Built because loosening imu_orientation_cov's yaw term (microstrain.yaml)
had zero measurable effect on how closely /odometry/filtered's yaw tracks
/microstrain/ekf/imu/data's raw AHRS yaw -- confirmed the new covariance
value was actually live on the running driver, so the lack of effect is
real, not a deployment miss. That means the disturbance may not be entering
purely through the absolute-yaw channel: /microstrain/ekf/imu/data's
angular_velocity (the vyaw robot_localization actually fuses at high trust,
imu_angular_cov unchanged) is itself AHRS/Kalman-filtered by the device, not
a raw gyro reading -- if the device's internal filter blends magnetometer
disturbance into more than just the orientation it reports, de-weighting
only orientation wouldn't touch it.

This logs /microstrain/imu/data (calibrated, NOT AHRS-fused -- a candidate
"clean" gyro rate) alongside /microstrain/ekf/imu/data (AHRS-fused, what's
actually being fused as imu0) so the two can be compared directly: if raw
gyro stays flat while the AHRS-fused rate swings, the disturbance is
entering through the device's internal fusion, not the raw sensor itself.
The magnetometer vector's magnitude is also logged -- Earth's field
magnitude should stay roughly constant regardless of orientation, so a
change in magnitude (not just direction) is direct evidence of real EMI
rather than just normal heading change.

Requires imu_mag_data_rate: 100 in microstrain.yaml (already added) for the
magnetometer topic to actually publish.

GROUND-TRUTH CHECKPOINTS (optional)
------------------------------------
If you know in advance which physical points you're going to drive to and
stop at, pass them as ground_truth_waypoints and call the mark_checkpoint
service each time you physically stop the rover there. That row gets the
EKF's reported (x, y) at that instant plus the known-true (x, y) and the
resulting error, right alongside the IMU data in the same CSV -- no need to
line up timestamps against a separate ground-truth log by hand afterward.

  ros2 run rover_nav imu_deep_log.py --ros-args \\
    -p duration_s:=120.0 \\
    -p ground_truth_waypoints:="-5.0,-1.0;3.0,2.0;-2.0,4.0"

  # each time you stop the rover at the next physical point in that list:
  ros2 service call /imu_deep_log/mark_checkpoint std_srvs/srv/Trigger

Plain IMU-only logging, no checkpoints:
  ros2 run rover_nav imu_deep_log.py --ros-args -p duration_s:=90.0

CLOSED-LOOP DISTANCE CHECK (optional)
--------------------------------------
For a "start and end at the same physical point" test: this always tracks
the EKF's cumulative path length (sum of consecutive /odometry/filtered
position deltas -- total distance traveled, not net displacement) and how
far the reported end position landed from the reported start position
(loop-closure error). If you also know the real distance you drove (tape
measure, measuring wheel, or a pre-planned course length), set it -- before
or any time during the run, since you often only know it after measuring
the physical loop:

  ros2 param set /imu_deep_log real_distance_m 18.4

and the final report will include the EKF-vs-real delta and percent error.
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_waypoints(raw: str):
    """Parses 'x1,y1;x2,y2;...' into [(x1,y1), (x2,y2), ...]. Empty -> []."""
    raw = raw.strip()
    if not raw:
        return []
    points = []
    for chunk in raw.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        x_str, y_str = chunk.split(',')
        points.append((float(x_str), float(y_str)))
    return points


class ImuDeepLog(Node):
    def __init__(self):
        super().__init__("imu_deep_log")
        self.declare_parameter("duration_s", 90.0)
        self.declare_parameter("raw_imu_topic", "/microstrain/imu/data")
        self.declare_parameter("ahrs_imu_topic", "/microstrain/ekf/imu/data")
        self.declare_parameter("mag_topic", "/microstrain/imu/mag")
        self.declare_parameter(
            "csv_path",
            f"/home/rhitik/test/src/imu_deep_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.declare_parameter("ground_truth_waypoints", "")
        self.declare_parameter("real_distance_m", 0.0)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.csv_path = str(self.get_parameter("csv_path").value)
        self.ground_truth_waypoints = parse_waypoints(
            str(self.get_parameter("ground_truth_waypoints").value)
        )
        self.checkpoint_idx = 0
        self.real_distance_m = float(self.get_parameter("real_distance_m").value)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.start_time = None
        self.rows = []

        self._raw_vyaw = None
        self._ahrs_yaw = None
        self._ahrs_vyaw = None
        self._mag = None  # (x, y, z)
        self._fused_yaw = None
        self._fused_pos = None  # (x, y)

        # Closed-loop distance check: total path length walked by the EKF
        # (sum of consecutive position deltas, not net displacement), plus
        # how far the last reported position is from the first -- the loop-
        # closure error for a start==end test.
        self._first_fused_pos = None
        self._prev_fused_pos = None
        self.ekf_path_length_m = 0.0
        self._saved = False

        self.create_subscription(
            Imu, str(self.get_parameter("raw_imu_topic").value), self._raw_cb, 10
        )
        self.create_subscription(
            Imu, str(self.get_parameter("ahrs_imu_topic").value), self._ahrs_cb, 10
        )
        self.create_subscription(
            MagneticField, str(self.get_parameter("mag_topic").value), self._mag_cb, 10
        )
        self.create_subscription(Odometry, "/odometry/filtered", self._fused_cb, 10)
        self.create_service(Trigger, "/imu_deep_log/mark_checkpoint", self._mark_checkpoint_srv)
        self.timer = self.create_timer(0.5, self._check_done)
        self.get_logger().info(
            f"Deep-logging raw/AHRS gyro, magnetometer, and fused yaw for "
            f"{self.duration_s:.0f}s -> {self.csv_path}"
        )
        if self.ground_truth_waypoints:
            self.get_logger().info(
                f"  {len(self.ground_truth_waypoints)} ground-truth checkpoints configured -- "
                "call `ros2 service call /imu_deep_log/mark_checkpoint std_srvs/srv/Trigger` "
                "each time you physically stop the rover at the next one, in order: "
                + " -> ".join(f"({x:.2f},{y:.2f})" for x, y in self.ground_truth_waypoints)
            )

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "real_distance_m":
                self.real_distance_m = float(p.value)
                self.get_logger().info(
                    f"real_distance_m set to {self.real_distance_m:.3f} m -- "
                    f"EKF path length so far: {self.ekf_path_length_m:.3f} m"
                )
        return SetParametersResult(successful=True)

    def _raw_cb(self, msg):
        self._raw_vyaw = msg.angular_velocity.z

    def _ahrs_cb(self, msg):
        q = msg.orientation
        self._ahrs_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._ahrs_vyaw = msg.angular_velocity.z

    def _mag_cb(self, msg):
        self._mag = (msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    def _elapsed(self) -> float:
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        return (now - self.start_time).nanoseconds / 1e9

    def _fused_cb(self, msg):
        q = msg.pose.pose.orientation
        self._fused_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._fused_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self._first_fused_pos is None:
            self._first_fused_pos = self._fused_pos
        if self._prev_fused_pos is not None:
            self.ekf_path_length_m += math.hypot(
                self._fused_pos[0] - self._prev_fused_pos[0],
                self._fused_pos[1] - self._prev_fused_pos[1],
            )
        self._prev_fused_pos = self._fused_pos

        # Pace regular rows off the fused output (~20Hz) -- wait until every
        # other source has produced at least one sample to pair with.
        if self._raw_vyaw is None or self._ahrs_yaw is None or self._mag is None:
            return
        t = self._elapsed()
        mag_mag = math.sqrt(sum(c * c for c in self._mag))
        self.rows.append((
            t, self._fused_yaw, self._ahrs_yaw, self._raw_vyaw, self._ahrs_vyaw,
            self._mag[0], self._mag[1], self._mag[2], mag_mag,
            self._fused_pos[0], self._fused_pos[1],
            False, None, None, None, None,
        ))

    def _mark_checkpoint_srv(self, request, response):
        if not self.ground_truth_waypoints:
            response.success = False
            response.message = "no ground-truth waypoints configured (set the 'ground_truth_waypoints' param)"
            return response
        if self.checkpoint_idx >= len(self.ground_truth_waypoints):
            response.success = False
            response.message = f"all {len(self.ground_truth_waypoints)} checkpoints already marked"
            return response
        if (self._fused_pos is None or self._raw_vyaw is None
                or self._ahrs_yaw is None or self._mag is None):
            response.success = False
            response.message = "sensors not ready yet -- no odometry/IMU/mag samples received"
            return response

        gt_x, gt_y = self.ground_truth_waypoints[self.checkpoint_idx]
        ekf_x, ekf_y = self._fused_pos
        error_m = math.hypot(ekf_x - gt_x, ekf_y - gt_y)

        t = self._elapsed()
        mag_mag = math.sqrt(sum(c * c for c in self._mag))
        self.rows.append((
            t, self._fused_yaw, self._ahrs_yaw, self._raw_vyaw, self._ahrs_vyaw,
            self._mag[0], self._mag[1], self._mag[2], mag_mag,
            ekf_x, ekf_y,
            True, self.checkpoint_idx, gt_x, gt_y, error_m,
        ))

        self.get_logger().info(
            f"📍 Checkpoint {self.checkpoint_idx} marked: "
            f"ground truth=({gt_x:.2f}, {gt_y:.2f}) reported=({ekf_x:.2f}, {ekf_y:.2f}) "
            f"error={error_m * 100:.1f} cm"
        )
        response.success = True
        response.message = f"checkpoint {self.checkpoint_idx} marked, error={error_m:.3f} m"
        self.checkpoint_idx += 1
        return response

    def _check_done(self):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration_s:
            remaining = self.duration_s - elapsed
            self.get_logger().info(
                f"...{remaining:.0f}s left, {len(self.rows)} samples so far "
                f"({self.checkpoint_idx}/{len(self.ground_truth_waypoints)} checkpoints marked)",
                throttle_duration_sec=5.0,
            )
            return
        self._save_and_report()
        self.timer.cancel()
        rclpy.shutdown()

    def _save_and_report(self):
        self._saved = True
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t_s", "fused_yaw_deg", "ahrs_yaw_deg",
                "raw_vyaw_deg_s", "ahrs_vyaw_deg_s",
                "mag_x", "mag_y", "mag_z", "mag_magnitude",
                "x_m", "y_m",
                "is_checkpoint", "checkpoint_idx", "gt_x", "gt_y", "gt_error_m",
            ])
            for (t, fused_yaw, ahrs_yaw, raw_vyaw, ahrs_vyaw,
                 mx, my, mz, mmag, x, y,
                 is_checkpoint, checkpoint_idx, gt_x, gt_y, gt_error_m) in self.rows:
                writer.writerow([
                    f"{t:.3f}", f"{math.degrees(fused_yaw):.4f}", f"{math.degrees(ahrs_yaw):.4f}",
                    f"{math.degrees(raw_vyaw):.4f}", f"{math.degrees(ahrs_vyaw):.4f}",
                    f"{mx:.6f}", f"{my:.6f}", f"{mz:.6f}", f"{mmag:.6f}",
                    f"{x:.4f}", f"{y:.4f}",
                    int(is_checkpoint),
                    "" if checkpoint_idx is None else checkpoint_idx,
                    "" if gt_x is None else f"{gt_x:.4f}",
                    "" if gt_y is None else f"{gt_y:.4f}",
                    "" if gt_error_m is None else f"{gt_error_m:.4f}",
                ])
        n = len(self.rows)
        if n == 0:
            self.get_logger().error("No paired samples recorded -- check all 4 topics are publishing.")
            return
        mags = [r[8] for r in self.rows]
        raw_vyaws = [math.degrees(r[3]) for r in self.rows]
        ahrs_vyaws = [math.degrees(r[4]) for r in self.rows]
        checkpoint_errors = [r[15] for r in self.rows if r[11]]
        summary = (
            "\n"
            f"=== {n} samples -> {self.csv_path} ===\n"
            f"mag magnitude: min={min(mags):.4f} max={max(mags):.4f} "
            f"range={max(mags)-min(mags):.4f} (stable magnitude = no real EMI; "
            f"large range = something is disturbing the field, not just heading)\n"
            f"raw  vyaw range: {max(raw_vyaws)-min(raw_vyaws):.2f} deg/s\n"
            f"AHRS vyaw range: {max(ahrs_vyaws)-min(ahrs_vyaws):.2f} deg/s "
            "(if AHRS range is much bigger than raw, the device's own internal "
            "filter is where the disturbance is entering, not the raw gyro)"
        )
        if checkpoint_errors:
            summary += (
                f"\n{len(checkpoint_errors)}/{len(self.ground_truth_waypoints)} checkpoints marked, "
                f"landing error: min={min(checkpoint_errors)*100:.1f}cm "
                f"max={max(checkpoint_errors)*100:.1f}cm "
                f"mean={sum(checkpoint_errors)/len(checkpoint_errors)*100:.1f}cm"
            )
        if self._first_fused_pos is not None and self._prev_fused_pos is not None:
            closure_error_m = math.hypot(
                self._prev_fused_pos[0] - self._first_fused_pos[0],
                self._prev_fused_pos[1] - self._first_fused_pos[1],
            )
            summary += (
                f"\nEKF path length (total distance traveled): {self.ekf_path_length_m:.3f} m\n"
                f"loop closure error (end vs. start position): {closure_error_m * 100:.1f} cm"
            )
            if self.real_distance_m > 0.0:
                diff_m = self.ekf_path_length_m - self.real_distance_m
                pct = (diff_m / self.real_distance_m) * 100.0
                summary += (
                    f"\nreal distance: {self.real_distance_m:.3f} m | "
                    f"EKF - real: {diff_m:+.3f} m ({pct:+.1f}%)"
                )
            else:
                summary += (
                    "\n(no real_distance_m set -- `ros2 param set /imu_deep_log "
                    "real_distance_m <meters>` to get an EKF-vs-real percent error)"
                )
        self.get_logger().info(summary)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDeepLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Ctrl-C before duration_s elapses -- save whatever was collected
        # rather than silently discarding the run.
        if not node._saved:
            node.get_logger().warn("interrupted -- saving partial data collected so far")
            node._save_and_report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
