#!/usr/bin/env python3
"""Publish the waypoint-by-waypoint path for RViz, with no map, hardware, or
joystick needed -- just rclpy + standard message packages.

Edit START/WAYPOINTS in global_path_planner.py, then run this node. It
publishes once on startup (latched, so RViz sees it whenever it connects)
and stays alive to keep serving late subscribers:

  ros2 run rover_nav publish_global_path.py
  rviz2 -d $(ros2 pkg prefix rover_nav)/share/rover_nav/rviz/global_path_view.rviz

Or both together:
  ros2 launch rover_nav visualize_global_path.launch.py

Publishes the same path pure pursuit actually drives -- straight legs that
land exactly on each waypoint, where the rover stops before continuing --
on /pure_pursuit/path, matching the name rover_nav's `omar` branch's RViz
config already uses for "the followed route".
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from global_path_planner import START, WAYPOINTS, generate_waypoint_legs  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

FRAME_ID = "odom"  # set RViz's Fixed Frame to this -- no TF needed, it's self-referential


def to_path_msg(points, node, frame_id=FRAME_ID):
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    for x, y in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def waypoint_markers(start, waypoints, node, frame_id=FRAME_ID):
    markers = MarkerArray()
    stamp = node.get_clock().now().to_msg()

    def sphere(idx, x, y, color, scale):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "global_path"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        return m

    def label(idx, x, y, text):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "global_path_labels"
        m.id = idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = text
        return m

    start_color = ColorRGBA(r=0.11, g=0.69, b=0.48, a=1.0)  # aqua-green
    wp_color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)     # near-black

    markers.markers.append(sphere(0, start[0], start[1], start_color, 0.35))
    markers.markers.append(label(0, start[0], start[1], "start"))
    for i, (x, y) in enumerate(waypoints, start=1):
        markers.markers.append(sphere(i, x, y, wp_color, 0.28))
        markers.markers.append(label(i, x, y, str(i)))
    return markers


def main(args=None):
    rclpy.init(args=args)
    node = Node("publish_global_path")

    latched_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    path_pub = node.create_publisher(Path, "/pure_pursuit/path", latched_qos)
    markers_pub = node.create_publisher(MarkerArray, "/global_path/waypoints", latched_qos)

    legs = generate_waypoint_legs(START, WAYPOINTS)
    all_points = [pt for leg in legs for pt in leg]

    path_pub.publish(to_path_msg(all_points, node))
    markers_pub.publish(waypoint_markers(START, WAYPOINTS, node))

    node.get_logger().info(
        f"Published waypoint path: {len(legs)} legs, {len(all_points)} points total, "
        f"through {len(WAYPOINTS)} waypoint(s), frame_id='{FRAME_ID}'"
    )
    node.get_logger().info(
        "Topics: /pure_pursuit/path, /global_path/waypoints -- "
        "set RViz's Fixed Frame to 'odom'. Staying alive to serve late subscribers (Ctrl+C to exit)."
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
