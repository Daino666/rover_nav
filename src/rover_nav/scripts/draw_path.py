#!/usr/bin/env python3
"""Draw a route on the map in RViz, then refine it into a drivable,
collision-checked, curvature-bounded path through the same Hybrid-A* planner
plan_global_path.py already uses for named waypoints.

Requires the planning stack up (needs planner_server's compute_path_to_pose,
exactly like plan_global_path.py):

  ros2 launch rover_nav nav2_planning.launch.py rviz:=true
  ros2 run rover_nav draw_path.py

Then in RViz:
  1. Use the "Publish Point" tool (in nav2_path_view.rviz's toolbar already;
     add it via Tools > + > rviz_default_plugins/PublishPoint on any other
     config) and click a rough route across the map, in order.
  2. A live preview (yellow line + orange dots) shows what you've clicked so
     far, on /draw_path/preview -- add a MarkerArray display for it if it's
     not already in your RViz config (nav2_path_view.rviz has it).
  3. Refine it:
       ros2 service call /draw_path/refine std_srvs/srv/Trigger
     This hands your clicked points to plan_global_path.py as an ordered
     --points list -- Nav2 connects them exactly like it connects named
     waypoints: collision-checked against the costmap, curvature-bounded to
     minimum_turning_radius, heading resolved automatically at each point.
     Output goes to <out_dir>/drawn_path.csv (+ _waypoints.csv, + .png) and
     is republished on /pure_pursuit/path for an immediate RViz check.
  4. Happy with it? Drive it:
       ros2 launch aries_bringup full_hardware.launch.py \\
         path_csv:=<out_dir>/drawn_path.csv \\
         waypoints_csv:=<out_dir>/drawn_path_waypoints.csv
     Not happy? Clear and redraw:
       ros2 service call /draw_path/clear std_srvs/srv/Trigger

Each /draw_path/refine call re-plans whatever is CURRENTLY accumulated --
clicking more points after a refine and calling it again extends and
re-plans the same route, it does not start over. /draw_path/clear is the
only way to start over.
"""

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
from plan_global_path import DEFAULT_COORDS, DEFAULT_OUT_DIR  # noqa: E402

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class DrawPath(Node):

    def __init__(self):
        super().__init__("draw_path")

        self.declare_parameter("out_dir", DEFAULT_OUT_DIR)
        self.declare_parameter("coords", DEFAULT_COORDS)
        self.declare_parameter("name", "drawn_path")
        self.declare_parameter("frame_id", "map")
        # Mouse clicks land close together (often ~1 m apart) -- much closer
        # than minimum_turning_radius (1.5 m, a real wheel-scrub limit, not
        # tunable per-route). Two consecutive points that close, needing a
        # sharp heading change, in a narrow corridor, cannot be connected
        # directly by a 1.5 m-radius planner -- it instead loops way out into
        # open ground to build the turning room it needs. Measured on a real
        # drawn route: 4 of 26 legs between raw clicks ballooned to
        # 16-30 m each (vs ~1 m straight-line) this way. Dropping clicks
        # closer than this to the last KEPT one before planning avoids
        # forcing those tiny, tight legs in the first place.
        self.declare_parameter("min_spacing_m", 1.5)
        # Hybrid-A* runs per LEG (one search per consecutive pair of clicked
        # points), each already bounded by nav2_planning_params.yaml's own
        # max_planning_time -- this is the ceiling on the whole subprocess,
        # covering every leg together, not any single search.
        self.declare_parameter("planning_timeout_s", 120.0)

        self.out_dir = os.path.expanduser(str(self.get_parameter("out_dir").value))
        self.coords = os.path.expanduser(str(self.get_parameter("coords").value))
        self.name = str(self.get_parameter("name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.planning_timeout = float(self.get_parameter("planning_timeout_s").value)
        self.min_spacing = float(self.get_parameter("min_spacing_m").value)

        self.points = []  # ordered [(x, y), ...] clicked so far, map frame

        self.create_subscription(PointStamped, "/clicked_point", self._on_click, 10)
        self.preview_pub = self.create_publisher(MarkerArray, "/draw_path/preview", 10)
        self.create_service(Trigger, "/draw_path/clear", self._on_clear)
        self.create_service(Trigger, "/draw_path/refine", self._on_refine)

        self.get_logger().info(
            "draw_path up. Click points on the map with RViz's 'Publish Point' tool, "
            "then: ros2 service call /draw_path/refine std_srvs/srv/Trigger"
        )

    def _on_click(self, msg: PointStamped):
        self.points.append((msg.point.x, msg.point.y))
        self.get_logger().info(
            f"point {len(self.points)}: ({msg.point.x:.2f}, {msg.point.y:.2f})"
        )
        self._publish_preview()

    def _publish_preview(self, clear_all=False):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        if clear_all:
            delete_all = Marker()
            delete_all.header.frame_id = self.frame_id
            delete_all.header.stamp = stamp
            delete_all.action = Marker.DELETEALL
            array.markers.append(delete_all)
            self.preview_pub.publish(array)
            return

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = "draw_path_preview"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0)
        line.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in self.points]
        array.markers.append(line)

        for i, (x, y) in enumerate(self.points):
            dot = Marker()
            dot.header.frame_id = self.frame_id
            dot.header.stamp = stamp
            dot.ns = "draw_path_preview_points"
            dot.id = i
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = float(x)
            dot.pose.position.y = float(y)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.2
            dot.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0)
            array.markers.append(dot)

        self.preview_pub.publish(array)

    def _on_clear(self, request, response):
        n = len(self.points)
        self.points = []
        self._publish_preview(clear_all=True)
        response.success = True
        response.message = f"cleared {n} clicked point(s)"
        self.get_logger().info(response.message)
        return response

    def _decimate(self, points):
        """Drop points closer than min_spacing_m to the last KEPT point --
        always keeping the first and last so the route still starts and ends
        exactly where drawn. See min_spacing_m's declare_parameter comment
        for why this matters."""
        if len(points) < 2 or self.min_spacing <= 0.0:
            return list(points)
        kept = [points[0]]
        for p in points[1:-1]:
            last = kept[-1]
            if math.hypot(p[0] - last[0], p[1] - last[1]) >= self.min_spacing:
                kept.append(p)
        kept.append(points[-1])
        return kept

    def _on_refine(self, request, response):
        if len(self.points) < 2:
            response.success = False
            response.message = f"need at least 2 clicked points, have {len(self.points)}"
            self.get_logger().warn(response.message)
            return response

        route = self._decimate(self.points)
        if len(route) != len(self.points):
            self.get_logger().info(
                f"decimated {len(self.points)} clicked points -> {len(route)} "
                f"(min_spacing_m={self.min_spacing:.1f}); raise/lower it and refine "
                f"again for more/less simplification"
            )
        if len(route) < 2:
            response.success = False
            response.message = "all clicked points collapsed under min_spacing_m -- click farther apart"
            self.get_logger().warn(response.message)
            return response

        # Leading space on every token: argparse reads a bare "-0.2,1.3" as
        # an unrecognized flag (it starts with "-" and isn't a pure negative
        # number), which silently truncates --points right there. A token
        # containing a space is exempt from that check -- plan_global_path.py
        # strips it back off before parsing. See resolve()'s docstring there.
        def fmt(xy):
            return f" {xy[0]:.3f},{xy[1]:.3f}"

        start = fmt(route[0])
        rest = [fmt(p) for p in route[1:]]

        cmd = [
            "ros2", "run", "rover_nav", "plan_global_path.py",
            "--start", start, "--points", *rest, "--in-order",
            "--name", self.name, "--out-dir", self.out_dir,
            "--coords", self.coords, "--publish",
        ]
        self.get_logger().info(
            f"refining {len(route)} points via Nav2: {' '.join(cmd)}"
        )
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=self.planning_timeout)
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = f"planning timed out after {self.planning_timeout:.0f}s"
            self.get_logger().error(response.message)
            return response

        if result.returncode != 0:
            response.success = False
            response.message = (
                f"planning failed (exit {result.returncode}): {result.stderr[-800:]}"
            )
            self.get_logger().error(response.message)
            return response

        csv_path = os.path.join(self.out_dir, f"{self.name}.csv")
        waypoints_csv = os.path.join(self.out_dir, f"{self.name}_waypoints.csv")
        response.success = True
        response.message = (
            f"refined -> {csv_path}. Drive it: "
            f"ros2 launch aries_bringup full_hardware.launch.py "
            f"path_csv:={csv_path} waypoints_csv:={waypoints_csv}"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DrawPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
