#!/usr/bin/env python3
"""Relays the IMU topic with yaw zeroed at the first message *after the
MicroStrain's own AHRS filter has converged*, deterministically, in code we
control -- instead of trusting ekf_config.yaml's imu0_relative.

Confirmed on hardware (two different boot orientations) that fused yaw
(/odometry/filtered) is NOT ~0 at startup even with imu0_relative: true, and
is off from the raw IMU heading by a constant ~90 degrees both times -- a
repeatable structural offset, not the boot-to-boot-varying residual
originally suspected (see git history for that investigation). robot_localization's relative-orientation handling is closed-source to us here
(binary install, no local source checkout) so the exact internal cause
isn't provable from this box, and an external /set_pose correction after
the fact was also confirmed NOT to hold -- subsequent IMU fusion measures
yaw relative to robot_localization's OWN internal reference and pulls the
corrected state back away from 0 within a few updates.

Sidesteps the whole mechanism instead: capture the raw yaw of the first
message seen *after* /microstrain/ekf/status reports filter_state ==
"Solution Valid" (confirmed on hardware to matter -- capturing from message
#1 unconditionally reproducibly locked onto a stale pre-convergence heading,
off from the settled value by a consistent ~90 degrees every boot: the same
size gap regardless of which way the rover was actually facing, i.e. an
artifact of grabbing the reference too early, not a real geometric offset).
Republishes every message (including that first post-convergence one) with
yaw replaced by (raw_yaw - reference), i.e. a plain, self-computed relative
heading. ekf_config.yaml's imu0 now points at this relayed topic with
imu0_relative: false (see localization.launch.py) so the EKF fuses it as a
plain absolute-yaw measurement that already reads 0 at the first sample --
no reliance on robot_localization's own relative-mode bookkeeping at all.

Falls back to capturing from whatever messages have arrived after
startup_grace_s if filter_state never reports Solution Valid (e.g. the
status topic is unavailable), so this can't hang forever -- logged loudly
since that means the zero reference may still land on a not-fully-converged
reading.

The reference itself is a circular mean over reference_avg_samples
consecutive messages once ready (default 20, ~0.2s at the driver's 100Hz
imu_data_rate) rather than a single sample -- a lone reading still carries
ordinary sensor noise even post-convergence (observed ~0.04 degree residual
error), and averaging N independent noisy samples cuts that down by
roughly sqrt(N).

Only yaw is touched. Roll/pitch are irrelevant here: ekf_config.yaml's
imu0_config only fuses yaw (2D mode), and two_d_mode forces the filter's
own roll/pitch state to 0 regardless of what's republished. Angular
velocity / linear acceleration are body-frame and untouched by a
heading-reference shift, so they're passed through as-is.

  ros2 run rover_nav imu_yaw_zero.py --ros-args -p input_topic:=/microstrain/ekf/imu/data
"""
import math
import time

import rclpy
from microstrain_inertial_msgs.msg import HumanReadableStatus
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ImuYawZero(Node):
    def __init__(self):
        super().__init__("imu_yaw_zero")
        self.declare_parameter("input_topic", "/microstrain/ekf/imu/data")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("status_topic", "/microstrain/ekf/status")
        self.declare_parameter("startup_grace_s", 15.0)
        self.declare_parameter("reference_avg_samples", 20)
        # What the boot heading should READ once the reference is subtracted.
        # 0 keeps odom conventional: "forward at launch" is yaw=0 (+x), which
        # check_heading.py, the test courses' odom_origin anchoring and this
        # file's own name all assume. The rover's heading *within the
        # competition map* is expressed separately, by MAP_TO_ODOM_YAW_DEG in
        # global_path_planner.py -- a static transform, so exact, rather than
        # a filter state that wanders. Only change this if you specifically
        # want odom itself rotated.
        self.declare_parameter("initial_yaw_deg", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if not output_topic:
            output_topic = input_topic + "_yaw_zeroed"
        self.output_topic = output_topic
        status_topic = str(self.get_parameter("status_topic").value)
        self.startup_grace_s = float(self.get_parameter("startup_grace_s").value)
        self.reference_avg_samples = int(self.get_parameter("reference_avg_samples").value)
        self.initial_yaw = math.radians(float(self.get_parameter("initial_yaw_deg").value))

        self.reference_yaw = None
        self.filter_ready = False
        self._first_imu_seen_at = None
        self._sin_sum = 0.0
        self._cos_sum = 0.0
        self._collected = 0

        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(HumanReadableStatus, status_topic, self._on_status, 10)
        self.create_subscription(Imu, input_topic, self._on_imu, 10)
        self.get_logger().info(
            f"relaying {input_topic} -> {output_topic}, will zero yaw once "
            f"{status_topic} reports filter_state=Solution Valid "
            f"(or after {self.startup_grace_s:.0f}s, whichever comes first)"
        )

    def _on_status(self, msg):
        if not self.filter_ready and msg.filter_state == HumanReadableStatus.FILTER_STATE_GX5_RUN_SOLUTION_VALID:
            self.filter_ready = True
            self.get_logger().info(
                f"{self.get_parameter('status_topic').value} reports "
                "Solution Valid -- ready to capture yaw reference"
            )

    def _on_imu(self, msg):
        now = time.monotonic()
        if self._first_imu_seen_at is None:
            self._first_imu_seen_at = now

        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.reference_yaw is None:
            grace_expired = (now - self._first_imu_seen_at) >= self.startup_grace_s
            if not self.filter_ready and not grace_expired:
                return
            if not self.filter_ready and self._collected == 0:
                self.get_logger().warn(
                    f"filter_state never reported Solution Valid within "
                    f"{self.startup_grace_s:.0f}s -- averaging a yaw reference "
                    "anyway, it may not be fully converged"
                )

            self._sin_sum += math.sin(yaw)
            self._cos_sum += math.cos(yaw)
            self._collected += 1
            if self._collected < self.reference_avg_samples:
                return

            self.reference_yaw = math.atan2(self._sin_sum, self._cos_sum)
            self.get_logger().info(
                f"yaw reference captured (mean of {self._collected} samples): "
                f"{math.degrees(self.reference_yaw):.2f} deg (will read as "
                f"{math.degrees(self.initial_yaw):.1f} from here on)"
            )

        corrected_yaw = normalize_angle(yaw - self.reference_yaw + self.initial_yaw)

        out = Imu()
        out.header = msg.header
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = (
            yaw_to_quat(corrected_yaw)
        )
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawZero()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
