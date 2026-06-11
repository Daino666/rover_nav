#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)


# ═══════════════════════════════════════════════════════════════
# PARAMETERS
# ═══════════════════════════════════════════════════════════════

BASE_VELOCITY  = 1.0   # normal cruising speed (m/s)
AVOID_VELOCITY = 0.5   # speed while dodging an obstacle (m/s)
MAX_CURVATURE  = 2.0

AVOID_RANGE    = 2.0   # only react to obstacles within this distance ahead (m)
PATH_WIDTH     = 0.5   # obstacle must be within this lateral band to count as blocking (m)
AVOID_OFFSET   = 0.8   # lateral shift of the avoidance waypoint (m)

goal_point = [7.0, 11.0]


# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def global_to_local(point, yaw, origin):
    dx = point[0] - origin[0]
    dy = point[1] - origin[1]
    lx =  dx * math.cos(yaw) + dy * math.sin(yaw)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return lx, ly


def local_to_global(lx, ly, yaw, origin):
    dx = lx * math.cos(yaw) - ly * math.sin(yaw)
    dy = lx * math.sin(yaw) + ly * math.cos(yaw)
    return [origin[0] + dx, origin[1] + dy]


def pure_pursuit_angular(target, yaw, pos, max_curv):
    lx, ly = global_to_local(target, yaw, pos)
    ld = math.sqrt(lx**2 + ly**2)
    if ld < 0.01:
        return 0.0
    curv = (2.0 * ly) / (ld ** 2)
    return float(np.clip(curv, -max_curv, max_curv))


# ═══════════════════════════════════════════════════════════════
# NODE
# ═══════════════════════════════════════════════════════════════

class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_gazebo',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])

        self.declare_parameter('goal_tolerance', 0.5)

        self.car_pos  = None
        self.car_yaw  = None
        self.obstacles = []   # list of [x, y] in odom frame

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry,    '/odom',              self._odom_cb,      10)
        self.create_subscription(MarkerArray, '/obstacles/markers', self._obstacles_cb, 10)

        self.vel_pub = self.create_publisher(TwistStamped, '/rover_controller/cmd_vel', 10)
        self.create_timer(0.1, self._loop)

        self.get_logger().info(f'Pure pursuit started — goal: {goal_point}')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.car_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        _, _, self.car_yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')

    def _obstacles_cb(self, msg: MarkerArray):
        """Convert marker bounding-box centroids into odom-frame [x, y] points."""
        obstacles = []
        for marker in msg.markers:
            if marker.action == Marker.DELETE or not marker.points:   # skip DELETE markers
                continue
            pts = np.array([[p.x, p.y, p.z] for p in marker.points])
            cx, cy, cz = pts.mean(axis=0)

            pt = PointStamped()
            pt.header = marker.header
            pt.point.x, pt.point.y, pt.point.z = float(cx), float(cy), float(cz)

            try:
                pt_odom = self.tf_buffer.transform(pt, 'odom', timeout=rclpy.duration.Duration(seconds=0.05))
                obstacles.append([pt_odom.point.x, pt_odom.point.y])
            except Exception:
                pass

        self.obstacles = obstacles

    # ── control loop ──────────────────────────────────────────────────────────

    def _loop(self):
        if self.car_pos is None or self.car_yaw is None:
            self.get_logger().warn('Waiting for odometry…', throttle_duration_sec=2.0)
            return

        tol = self.get_parameter('goal_tolerance').value
        if distance(self.car_pos, goal_point) < tol:
            self._publish(0.0, 0.0)
            self.get_logger().info('Goal reached!', throttle_duration_sec=1.0)
            return

        blocking = self._find_blocking_obstacle()

        if blocking is not None:
            target   = self._avoidance_waypoint(blocking)
            velocity = AVOID_VELOCITY
            self.get_logger().info(
                f'AVOIDING obstacle at local ({blocking[0]:.2f}, {blocking[1]:.2f}) '
                f'→ waypoint {target}',
                throttle_duration_sec=0.5)
        else:
            target   = goal_point
            velocity = BASE_VELOCITY

        angular = pure_pursuit_angular(target, self.car_yaw, self.car_pos, MAX_CURVATURE)
        self._publish(velocity, angular * velocity)

    # ── avoidance ─────────────────────────────────────────────────────────────

    def _find_blocking_obstacle(self):
        """Return (local_x, local_y) of the closest obstacle blocking the path, or None."""
        if not self.obstacles or self.car_pos is None:
            return None

        closest, closest_dist = None, float('inf')
        for obs in self.obstacles:
            lx, ly = global_to_local(obs, self.car_yaw, self.car_pos)
            if lx <= 0 or lx > AVOID_RANGE:          # not ahead or too far
                continue
            if abs(ly) > PATH_WIDTH:                  # outside the rover's path corridor
                continue
            d = math.sqrt(lx**2 + ly**2)
            if d < closest_dist:
                closest_dist = d
                closest = (lx, ly)

        return closest

    def _avoidance_waypoint(self, obstacle_local):
        """
        Place a waypoint just past the obstacle, offset to the clear side.
        Obstacle perfectly centred → default to going left.
        """
        obs_lx, obs_ly = obstacle_local
        side = -math.copysign(1.0, obs_ly) if obs_ly != 0 else 1.0

        wp_lx = obs_lx + 0.5            # slightly past the obstacle
        wp_ly = side * AVOID_OFFSET     # step to the clear side

        return local_to_global(wp_lx, wp_ly, self.car_yaw, self.car_pos)

    # ── publisher ─────────────────────────────────────────────────────────────

    def _publish(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x  = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.vel_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
