#!/usr/bin/env python3
"""RViz message builders for the real-world test paths -- shared by
publish_test_path.py (offline preview) and cmd_vel_arbiter.py (which
publishes the *anchored* route it is actually driving, so RViz shows the
course where the rover really placed it rather than where the CSV nominally
put it).

Topics are deliberately the ones rviz/global_path_view.rviz already
subscribes to (/pure_pursuit/path, /global_path/waypoints), so the same RViz
config serves both the waypoint route and a test path with no edits.
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

PATH_TOPIC = "/pure_pursuit/path"
MARKERS_TOPIC = "/global_path/waypoints"


def latched_qos():
    """Transient-local depth 1: publish once, and RViz still sees it whenever
    it connects (the test path is static for the whole run)."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def path_msg(points, stamp, frame_id):
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    for p in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(p[0])
        pose.pose.position.y = float(p[1])
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def marker_msgs(markers, stamp, frame_id):
    """One sphere + one floating number label per ground marker, numbered to
    match `marker_id` in <name>_markers.csv and the labels on <name>.jpg --
    so a cone in the field, a row in the CSV, and a dot in RViz are all the
    same identifiable point."""
    array = MarkerArray()
    start_color = ColorRGBA(r=0.11, g=0.69, b=0.48, a=1.0)  # aqua-green, marker 0
    marker_color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)  # near-black

    for mid, x, y in markers:
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "test_path_markers"
        sphere.id = int(mid)
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(x)
        sphere.pose.position.y = float(y)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35 if mid == 0 else 0.28
        sphere.color = start_color if mid == 0 else marker_color
        array.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "test_path_marker_labels"
        label.id = int(mid)
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(x)
        label.pose.position.y = float(y)
        label.pose.position.z = 0.3
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = str(mid)
        array.markers.append(label)

    return array
