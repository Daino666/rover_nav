#!/usr/bin/env python3
"""Snap the EKF's fused yaw to an exact heading, without moving its position.

  ros2 run rover_nav set_heading.py                  # snap odom yaw to 0 (default)
  ros2 run rover_nav set_heading.py --ros-args -p heading_deg:=90.0

Why this exists: /odometry/filtered's yaw wanders. Measured stationary over
three minutes it sat at 90.12 +-0.04 deg with a 0.19 deg band, and twenty
minutes after boot it read 90.33 -- bounded, but not repeatable enough to
correct with a constant. ekf_config.yaml's `initial_state` only fixes the
heading the filter STARTS from; by the time a run actually begins, minutes of
planning later, it has wandered off again.

So correct it at the moment it matters instead: call this immediately before
`ros2 service call /planner/start`, and the run begins from an exact heading
with whatever drifted away since boot removed.

This works because ekf_config.yaml sets imu0_differential: true, so the filter
fuses the IMU's orientation as a yaw RATE and never as an absolute heading --
nothing pulls a corrected state back. (rover_nav's own history records
/set_pose being tried and NOT holding; that was under absolute yaw fusion,
where each subsequent measurement dragged the state back within a few updates.
Verified on hardware in differential mode: snapped from 90.29 to 89.99, still
90.04 fifty seconds later.) If imu0_differential is ever set false, re-check
that this still holds before relying on it.

Position is read from the current estimate and written back unchanged --
robot_localization's SetPose sets the WHOLE pose, so sending a default-
constructed request silently teleports the rover to the origin.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = Node("set_heading")
    node.declare_parameter("heading_deg", 0.0)
    node.declare_parameter("odom_topic", "/odometry/filtered")
    node.declare_parameter("service", "/set_pose")
    node.declare_parameter("frame_id", "odom")

    heading = math.radians(float(node.get_parameter("heading_deg").value))
    odom_topic = str(node.get_parameter("odom_topic").value)
    service = str(node.get_parameter("service").value)
    frame_id = str(node.get_parameter("frame_id").value)

    latest = {}

    def on_odom(msg):
        latest["msg"] = msg

    node.create_subscription(Odometry, odom_topic, on_odom, 10)

    # Wait for a current estimate -- the position has to be carried over.
    for _ in range(400):
        rclpy.spin_once(node, timeout_sec=0.05)
        if "msg" in latest:
            break
    if "msg" not in latest:
        node.get_logger().error(f"no {odom_topic} in 20s -- is localization running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    cur = latest["msg"].pose.pose
    before = math.degrees(quat_to_yaw(cur.orientation))

    client = node.create_client(SetPose, service)
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{service} not available -- is ekf_filter_node running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    req = SetPose.Request()
    req.pose.header.frame_id = frame_id
    req.pose.header.stamp = node.get_clock().now().to_msg()
    req.pose.pose.pose.position.x = cur.position.x     # keep where we are
    req.pose.pose.pose.position.y = cur.position.y
    req.pose.pose.pose.position.z = cur.position.z
    req.pose.pose.pose.orientation.z = math.sin(heading / 2.0)
    req.pose.pose.pose.orientation.w = math.cos(heading / 2.0)
    # Tight covariance: this is an assertion about where the rover is pointing,
    # not a noisy measurement to be blended with the current estimate.
    cov = [0.0] * 36
    for i in (0, 7, 14, 21, 28, 35):
        cov[i] = 1e-9
    req.pose.pose.covariance = cov

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        node.get_logger().error(f"{service} did not respond")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Read back, so the number reported is the filter's, not our request's.
    # Settle first: messages published just before the service call are already
    # in flight, and grabbing the next one to arrive reports the OLD state and
    # makes a successful correction look like it did nothing.
    import time as _time
    settle_until = _time.monotonic() + 1.0
    while _time.monotonic() < settle_until and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
    latest.clear()
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.05)
        if "msg" in latest:
            break
    after = (math.degrees(quat_to_yaw(latest["msg"].pose.pose.orientation))
             if "msg" in latest else float("nan"))

    node.get_logger().info(
        f"yaw {before:.3f} -> {after:.3f} deg (asked for {math.degrees(heading):.1f}), "
        f"position held at x={cur.position.x:.3f} y={cur.position.y:.3f}"
    )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
