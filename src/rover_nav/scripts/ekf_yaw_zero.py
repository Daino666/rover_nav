#!/usr/bin/env python3
"""One-shot: zeroes /odometry/filtered's yaw at startup via robot_localization's
own /set_pose service.

ekf_config.yaml's imu0_relative: true is supposed to make the EKF's fused
yaw start at 0 regardless of the IMU's absolute (magnetometer-referenced)
heading, but this was confirmed on hardware to not reliably hold:
robot_localization's ros_filter.cpp preparePose() composes a second, live TF
lookup (target_frame_trans, built from the filter's own currently-published
odom->base_link) on top of the relative-mode zeroing, so the first fused
reading isn't deterministically 0 even with imu0_relative on -- e.g. dev
(the IMU's own absolute heading) reading 30 deg while fused reads -60 deg
instead of ~0.

Instead of trusting imu0_relative alone, this node waits for the first
/odometry/filtered message, then calls ekf_filter_node's /set_pose service
to explicitly reset the filter's internal state to yaw=0 at the position it
already reported. Fully deterministic, using robot_localization's own
supported mechanism, so every consumer of /odometry/filtered -- not just
one node -- sees the corrected value.

Deliberately one-shot, not re-triggered by /planner/start: re-zeroing there
would silently redefine what "forward"/WAYPOINTS mean if the rover gets
restarted mid-route facing a different direction than it started in.

  ros2 run rover_nav ekf_yaw_zero.py
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import SetPose


class EkfYawZero(Node):
    def __init__(self):
        super().__init__("ekf_yaw_zero")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("set_pose_service", "/set_pose")
        odom_topic = str(self.get_parameter("odom_topic").value)
        service_name = str(self.get_parameter("set_pose_service").value)

        self._client = self.create_client(SetPose, service_name)
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_first_odom, 10)
        self.get_logger().info(f"waiting for first {odom_topic} message to zero fused yaw...")

    def _on_first_odom(self, msg):
        self.destroy_subscription(self._sub)

        # two_d_mode in ekf_config.yaml already forces z/roll/pitch to 0 in
        # the filter's own state, so an identity-yaw quaternion here is safe
        # -- position is kept as-is, only yaw is corrected.
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = msg.header.frame_id
        pose.pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.pose.position.z = msg.pose.pose.position.z
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance = list(msg.pose.covariance)

        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"{self._client.srv_name} service unavailable -- fused yaw "
                "NOT zeroed, still reporting whatever the EKF started at"
            )
            return

        future = self._client.call_async(SetPose.Request(pose=pose))
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"set_pose call failed: {exc}")
            return
        self.get_logger().info("fused yaw zeroed via /set_pose")


def main(args=None):
    rclpy.init(args=args)
    node = EkfYawZero()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
