#!/usr/bin/env python3
"""Publish one real-world test path for RViz, with no rover, hardware, or
joystick needed -- the preview step before actually driving it.

  ros2 run rover_nav publish_test_path.py --ros-args -p test_path:=circle
  rviz2 -d $(ros2 pkg prefix rover_nav)/share/rover_nav/rviz/global_path_view.rviz

Or both together:
  ros2 launch rover_nav view_test_path.launch.py test_path:=circle

Publishes on the same latched topics the RViz config already shows
(/pure_pursuit/path, /global_path/waypoints), in frame `odom`, from the CSVs
in scripts/test_paths/output/. The course is drawn at its nominal placement
-- origin at odom's origin, +x along odom's +x -- which is where it really
lands only if the rover hasn't moved since localization came up. During an
actual run cmd_vel_arbiter.py republishes the same topics with the course
anchored to the rover's live pose at run start, which supersedes this.

Use it to check the shape and the marker numbering against <name>.jpg before
pacing the course out on the ground.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from test_path_loader import (  # noqa: E402
    TEST_PATHS, available, coerce_name, load_markers, load_test_path, path_length,
)
import test_path_viz  # noqa: E402

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

FRAME_ID = "odom"  # matches rviz/global_path_view.rviz's Fixed Frame, and is the
                   # frame the CSVs are natively in (see test_path_loader.py)


def main(args=None):
    rclpy.init(args=args)
    node = Node("publish_test_path")

    # dynamic_typing: the value can legitimately arrive as a DOUBLE -- see
    # coerce_name() for why "infinity" does exactly that.
    node.declare_parameter(
        "test_path", "straight_line", ParameterDescriptor(dynamic_typing=True))
    name = coerce_name(node.get_parameter("test_path").value)

    if name not in TEST_PATHS:
        node.get_logger().error(
            f"unknown test_path '{name}'. Available on disk: {', '.join(available()) or 'none'}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    points = load_test_path(name)
    markers = [(mid, x, y) for mid, x, y, _ in load_markers(name)]

    path_pub = node.create_publisher(Path, test_path_viz.PATH_TOPIC, test_path_viz.latched_qos())
    markers_pub = node.create_publisher(
        MarkerArray, test_path_viz.MARKERS_TOPIC, test_path_viz.latched_qos())

    stamp = node.get_clock().now().to_msg()
    path_pub.publish(test_path_viz.path_msg(points, stamp, FRAME_ID))
    markers_pub.publish(test_path_viz.marker_msgs(markers, stamp, FRAME_ID))

    node.get_logger().info(
        f"Published test path '{name}': {len(points)} points, {path_length(points):.1f} m, "
        f"{len(markers)} ground marker(s), frame_id='{FRAME_ID}'. {TEST_PATHS[name]}"
    )
    node.get_logger().info(
        f"Topics: {test_path_viz.PATH_TOPIC}, {test_path_viz.MARKERS_TOPIC} -- set RViz's "
        "Fixed Frame to 'odom'. Staying alive to serve late subscribers (Ctrl+C to exit)."
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
