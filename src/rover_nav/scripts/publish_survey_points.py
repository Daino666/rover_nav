#!/usr/bin/env python3
"""Publish every named survey point -- start lines (S#), ArUco landmarks
(L#), waypoints (W#), and anything else in the organisers' coordinate table
-- as labelled RViz markers, visible on the map the same way every time,
independent of whatever route is currently planned or being driven.

Publishes once (latched, transient-local -- a late-joining RViz still sees
it) and stays alive to keep serving late subscribers, same pattern as
publish_global_path.py:

  ros2 run rover_nav publish_survey_points.py

Already included in nav2_planning.launch.py (and therefore
global_local_view.launch.py, which includes that), so it comes up
automatically with RViz whenever you're planning or previewing a route.
Topic: /marsyard/survey_points -- nav2_path_view.rviz, global_path_view.rviz
and global_local_view.rviz all already have a MarkerArray display for it;
add one yourself on any other config.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from plan_global_path import DEFAULT_COORDS, load_survey_points  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

TOPIC = "/marsyard/survey_points"

# Category prefix -> (marker color, sphere diameter m). Matched against the
# first letter of each point's name in Coordinates_MarsYard2026.txt.
CATEGORIES = {
    "S": (ColorRGBA(r=0.11, g=0.69, b=0.48, a=1.0), 0.34),  # start lines, aqua-green
    "L": (ColorRGBA(r=0.90, g=0.10, b=0.10, a=1.0), 0.24),  # ArUco landmarks, red
    "W": (ColorRGBA(r=0.10, g=0.40, b=0.95, a=1.0), 0.24),  # waypoints, blue
}
OTHER_CATEGORY = (ColorRGBA(r=0.55, g=0.55, b=0.55, a=1.0), 0.22)  # anything else, grey


def latched_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def category_for(name):
    prefix = name[0].upper() if name else ""
    return CATEGORIES.get(prefix, OTHER_CATEGORY)


def build_markers(points, frame_id, stamp):
    array = MarkerArray()
    for i, (name, (x, y)) in enumerate(sorted(points.items())):
        color, diameter = category_for(name)

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "survey_points"
        sphere.id = i
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(x)
        sphere.pose.position.y = float(y)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = diameter
        sphere.color = color
        array.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "survey_point_labels"
        label.id = i
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(x)
        label.pose.position.y = float(y)
        label.pose.position.z = 0.4
        label.scale.z = 0.35
        # Black, not white -- these RViz configs use a white background
        # (Global Options > Background Color: 255; 255; 255), so white text
        # was invisible.
        label.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        label.text = name
        array.markers.append(label)
    return array


class PublishSurveyPoints(Node):

    def __init__(self):
        super().__init__("publish_survey_points")
        self.declare_parameter("coords", DEFAULT_COORDS)
        self.declare_parameter("frame_id", "map")

        coords_path = os.path.expanduser(str(self.get_parameter("coords").value))
        frame_id = str(self.get_parameter("frame_id").value)

        points = load_survey_points(coords_path)
        if not points:
            self.get_logger().error(f"no survey points loaded from {coords_path}")
            return

        # Kept as an instance attribute -- letting the publisher get
        # garbage-collected after __init__ returns would drop transient-local
        # history for anyone who subscribes later.
        self._pub = self.create_publisher(MarkerArray, TOPIC, latched_qos())
        msg = build_markers(points, frame_id, self.get_clock().now().to_msg())
        self._pub.publish(msg)

        n_s = sum(1 for n in points if n.upper().startswith("S"))
        n_l = sum(1 for n in points if n.upper().startswith("L"))
        n_w = sum(1 for n in points if n.upper().startswith("W"))
        self.get_logger().info(
            f"published {len(points)} survey points ({n_s} start lines, {n_l} landmarks, "
            f"{n_w} waypoints, {len(points) - n_s - n_l - n_w} other) on {TOPIC}, latched"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PublishSurveyPoints()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
