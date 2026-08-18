#!/usr/bin/env python3
"""Publishes the one-time static map -> odom transform: the alignment
correction between this rover's local odom frame (starts wherever it
happens to boot facing -- see ekf_config.yaml's imu0_relative) and the
competition's own map frame (the frame WAYPOINTS in global_path_planner.py
are given in, defined by the competition's two given reference points).

Defaults to identity (map == odom) via MAP_TO_ODOM_X/Y/YAW_DEG in
global_path_planner.py, since the actual competition-day alignment
procedure is still TBD and there's no absolute-position sensor on this
rover to compute it automatically. Override at launch for testing:

  ros2 run rover_nav map_odom_broadcaster.py \
    --ros-args -p map_to_odom_yaw_deg:=37.0

Deliberately does NOT publish odom -> base_footprint -- that stays solely
owned by robot_localization (ekf_config.yaml's publish_tf: true). Publishing
it here too would be exactly the kind of dual-publisher conflict this
codebase avoids elsewhere (see cmd_vel_arbiter.py's _check_ownership).

  ros2 run rover_nav map_odom_broadcaster.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from global_path_planner import MAP_TO_ODOM_X, MAP_TO_ODOM_Y, MAP_TO_ODOM_YAW_DEG  # noqa: E402

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class MapOdomBroadcaster(Node):
    def __init__(self):
        super().__init__("map_odom_broadcaster")

        self.declare_parameter("map_to_odom_x", MAP_TO_ODOM_X)
        self.declare_parameter("map_to_odom_y", MAP_TO_ODOM_Y)
        self.declare_parameter("map_to_odom_yaw_deg", MAP_TO_ODOM_YAW_DEG)

        x = float(self.get_parameter("map_to_odom_x").value)
        y = float(self.get_parameter("map_to_odom_y").value)
        yaw_deg = float(self.get_parameter("map_to_odom_yaw_deg").value)
        yaw = math.radians(yaw_deg)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish(x, y, yaw)

        self.get_logger().info(
            f"map -> odom static transform published: x={x:.3f} y={y:.3f} "
            f"yaw={yaw_deg:.2f}deg"
        )

    def _publish(self, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
