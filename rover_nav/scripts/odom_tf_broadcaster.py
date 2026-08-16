#!/usr/bin/env python3
"""
Broadcasts the odom->base_footprint TF from /odometry/filtered (the EKF's
output), plus a one-time static map->odom identity transform.

Why this exists: ekf_filter_node has publish_tf forced False (see
my_robot.launch.py -- it was self-consuming its own TF broadcast through its
own internal listener, which caused a "jump back in time" loop and stalled
position integration entirely; disabling its TF publish fixed the filter
itself but means nothing publishes odom->base_footprint anymore). This node is a
plain one-way broadcaster with no internal listener of its own, so it
doesn't hit that same problem.

map->odom is published once as a static identity transform because S1 (the
survey/world-frame origin) is also where odom starts from at launch (the
rover spawns at S1) -- there's no separate localization step correcting map
vs odom drift here, so treating them as coincident is the simplest correct
choice for visualization purposes.

This makes RViz's RobotModel + TF displays show the rover at its real,
moving position instead of pinned at a static transform.

Monotonic guard: Gazebo's /clock topic transiently publishes backward
timestamps during physics startup/reset, which makes the EKF momentarily
output odometry messages with stale stamps. Re-emitting a TF with an earlier
stamp would trigger "jump back in time" warnings in every TF consumer
(EKF, costmap, PCL filters). We guard by skipping any odometry message
whose stamp is not strictly newer than the last TF we published.

Usage:
  ros2 run rover_nav odom_tf_broadcaster.py
"""
import math
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _stamp_to_ns(stamp: Time) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.0)
        self.declare_parameter("spawn_yaw", 1.5707963267948966)

        self._spawn_x = float(self.get_parameter("spawn_x").value)
        self._spawn_y = float(self.get_parameter("spawn_y").value)
        self._spawn_z = float(self.get_parameter("spawn_z").value)
        self._spawn_yaw = float(self.get_parameter("spawn_yaw").value)

        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._last_stamp_ns: int = 0
        self.publish_static_map_to_odom()
        self.create_timer(1.0, self.publish_static_map_to_odom)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.get_logger().info(
            f"Broadcasting map->odom (static: x={self._spawn_x:.2f}, y={self._spawn_y:.2f}, "
            f"yaw={self._spawn_yaw:.3f} rad) and odom->base_footprint (from /odometry/filtered). "
            "Monotonic stamp guard active to prevent TF time-jump cascades."
        )

    def publish_static_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = self._spawn_x
        t.transform.translation.y = self._spawn_y
        t.transform.translation.z = self._spawn_z

        half_yaw = self._spawn_yaw * 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(math.sin(half_yaw))
        t.transform.rotation.w = float(math.cos(half_yaw))
        self.static_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        stamp_ns = _stamp_to_ns(msg.header.stamp)

        # Drop messages with backward or equal timestamps to prevent
        # "jump back in time" cascades during Gazebo clock transients.
        if stamp_ns <= self._last_stamp_ns:
            return

        self._last_stamp_ns = stamp_ns

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = float(msg.pose.pose.position.z)
        t.transform.rotation.x = float(msg.pose.pose.orientation.x)
        t.transform.rotation.y = float(msg.pose.pose.orientation.y)
        t.transform.rotation.z = float(msg.pose.pose.orientation.z)
        t.transform.rotation.w = float(msg.pose.pose.orientation.w)
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

