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

Usage:
  ros2 run rover_nav odom_tf_broadcaster.py
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_map_to_odom()
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.get_logger().info("Broadcasting map->odom (static) and odom->base_footprint (from /odometry/filtered)")

    def publish_static_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
