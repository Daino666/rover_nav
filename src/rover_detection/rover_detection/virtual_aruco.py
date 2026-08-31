"""Publishes SYNTHETIC landmark fixes, as if markers were out there.

For testing the half of the pipeline that does not need a camera: the pose
snap, the map->odom conversion, and how the path follower recovers from a
correction. Detection, depth ranging and the camera mount offset are NOT
exercised here -- those need a real marker in front of a real camera.

WHAT IT CANNOT DO: reproduce a real marker's correction of genuine odometry
drift. A real marker is valuable precisely because it is an independent
measurement of where the rover truly is; in simulation odometry is the only
position that exists, so a fix derived from it would agree with it exactly
and correct nothing. What this node does instead is inject a KNOWN error --
"believe you are 0.30 m to the left of where you think" -- which is what makes
the follower do something measurable.

Each virtual landmark fires once, when the rover first drives within
trigger_range_m of its trigger point. The fix published is the rover's own
map-frame position plus the configured error, so the correction the rover then
makes is exactly the error you asked for, in a direction you chose.

CSV columns: name, trigger_x, trigger_y, landmark_id, error_x, error_y
  trigger_x/y  where along the route (map frame) the sighting happens
  error_x/y    the lie, in map-frame metres, added to the rover's true position

  ros2 run rover_detection virtual_aruco --ros-args \\
      -p landmarks_csv:=/path/to/virtual_landmarks.csv
"""

import csv
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rover_perception_msgs.msg import RoverPositionFix


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VirtualAruco(Node):

    def __init__(self):
        super().__init__('virtual_aruco')
        self.declare_parameter('landmarks_csv', '')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('fix_topic', '/aruco_localization/rover_position_fix')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('trigger_range_m', 1.0)
        # Route the triggers are measured along. With it, a landmark fires at a
        # DISTANCE INTO THE ROUTE (trigger_s), which is unambiguous on a loop.
        # Without it the node falls back to proximity, which is not.
        self.declare_parameter('route_csv', '')

        path = str(self.get_parameter('landmarks_csv').value)
        if not path:
            raise SystemExit('landmarks_csv is required')
        self.trigger_range = float(self.get_parameter('trigger_range_m').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        self.route = None
        self.cum = []
        self._progress_i = 0
        route_csv = str(self.get_parameter('route_csv').value)
        if route_csv:
            with open(route_csv, newline='') as f:
                self.route = [(float(r['x_m']), float(r['y_m'])) for r in csv.DictReader(f)]
            self.cum = [0.0]
            for a, b in zip(self.route, self.route[1:]):
                self.cum.append(self.cum[-1] + math.dist(a, b))

        self.marks = []
        with open(path, newline='') as f:
            for r in csv.DictReader(f):
                self.marks.append({
                    'name': r['name'],
                    'trigger': (float(r['trigger_x']), float(r['trigger_y'])),
                    'id': int(r['landmark_id']),
                    'error': (float(r['error_x']), float(r['error_y'])),
                    'trigger_s': float(r['trigger_s']) if r.get('trigger_s') else None,
                    'fired': False,
                })

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            RoverPositionFix, str(self.get_parameter('fix_topic').value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 20)

        self.get_logger().warn(
            f'VIRTUAL ArUco: {len(self.marks)} synthetic landmark(s) from {path}. '
            f'These are NOT real detections -- no camera is involved. Each fires once '
            f'within {self.trigger_range:.1f} m of its trigger point.')
        for m in self.marks:
            err = math.hypot(*m['error'])
            self.get_logger().info(
                f"  {m['name']} (id {m['id']}) at map ({m['trigger'][0]:.2f},"
                f"{m['trigger'][1]:.2f}) -> injects {err:.2f} m "
                f"(dx={m['error'][0]:+.2f}, dy={m['error'][1]:+.2f})")

    def _progress_m(self, mx, my):
        """How far along the route the rover is, in metres.

        Triggering on Euclidean proximity alone is ambiguous on a closed loop:
        the route can pass near the same spot twice, so a sighting fires on
        whichever pass happens to come within range -- or, if the intended pass
        misses by slightly more than the trigger radius, only much later on the
        return leg. Observed on the 4-point loop: L10 fired correctly at 63.9 m
        while L11, meant for 16.7 m, fired sixty metres late. Progress along
        the route disambiguates them."""
        if self.route is None:
            return None
        best_i, best_d = 0, float('inf')
        # search forward only, so a loop closing on its own start cannot snap
        # the progress estimate back to zero
        for i in range(self._progress_i, len(self.route)):
            d = math.dist((mx, my), self.route[i])
            if d < best_d:
                best_d, best_i = d, i
        self._progress_i = best_i
        return self.cum[best_i]

    def _on_odom(self, msg):
        # odom -> map, so triggers and errors are specified in the same frame
        # the real detector publishes in.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException:
            self.get_logger().warn(
                f'no {self.map_frame} <- {self.odom_frame} transform yet',
                throttle_duration_sec=5.0)
            return
        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = msg.pose.pose.position.x, msg.pose.pose.position.y
        mx = ox * c - oy * s + t.x
        my = ox * s + oy * c + t.y

        progress = self._progress_m(mx, my)
        for m in self.marks:
            if m['fired']:
                continue
            if m['trigger_s'] is not None and progress is not None:
                # fire once the rover has driven past this distance into the route
                if progress < m['trigger_s']:
                    continue
            elif math.dist((mx, my), m['trigger']) > self.trigger_range:
                continue
            m['fired'] = True
            fx, fy = mx + m['error'][0], my + m['error'][1]
            out = RoverPositionFix()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.map_frame
            out.landmark_id = m['id']
            out.position = Point(x=fx, y=fy, z=0.0)
            out.distance = 2.0     # plausible sighting range; nothing consumes it
            self.pub.publish(out)
            self.get_logger().warn(
                f"VIRTUAL sighting {m['name']} (id {m['id']}): rover really at map "
                f"({mx:.3f},{my:.3f}), reporting ({fx:.3f},{fy:.3f}) -- injecting "
                f"{math.hypot(*m['error']):.2f} m of error for the follower to correct")


def main(args=None):
    rclpy.init(args=args)
    node = VirtualAruco()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
