#!/usr/bin/env python3
"""
Lightweight Kinematic & Sensor Simulator for Mars Rover

Provides a complete, zero-GPU simulation environment for testing Nav2 planning,
Pure Pursuit, 3D obstacle perception, and obstacle avoidance in RViz2 without
running Gazebo.
"""

import math
import random
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

CAMERA_X_OFFSET = 0.245
CAMERA_Z_OFFSET = 0.225
FOV_H = math.radians(86.0)
FOV_V = math.radians(57.0)
MAX_RANGE = 4.0

# ── Purposeful Path Obstacles ───────────────────────────────────────────────

def _rand_obstacles(seed=None):
    """
    Places clean, purposeful obstacles directly in the rover's tour path
    to clearly showcase 3D perception and dynamic avoidance in RViz2,
    with zero background clutter.
    """
    obstacles = [
        # ═════════════════════════════════════════════════════════════════
        # IN-PATH AVOIDANCE BOULDERS (Open Flat Ground Only - No Hill Passes)
        # ═════════════════════════════════════════════════════════════════
        # 1. Leg 1: S1 (0, 0) -> W7 (-2.1, 7.7) [Open Southern Flat]
        {'x': -0.9, 'y':  3.8, 'r': 0.30, 'h': 0.45, 'shape': 'boulder'},

        # 2. Leg 2: W7 (-2.1, 7.7) -> W6 (-7.7, 8.2) [Open Southwest Corridor]
        {'x': -4.5, 'y':  7.8, 'r': 0.30, 'h': 0.45, 'shape': 'boulder'},

        # 3. Leg 3: W6 (-7.7, 8.2) -> W5 (-5.3, 19.0) [Open Western Valley Plain]
        {'x': -7.2, 'y': 12.5, 'r': 0.30, 'h': 0.45, 'shape': 'boulder'},

        # 4. Leg 4: W5 (-5.3, 19.0) -> W8 (5.9, 6.8) [Open North Saddle & East Basin]
        {'x': -1.5, 'y': 19.5, 'r': 0.30, 'h': 0.45, 'shape': 'boulder'},
        {'x':  5.2, 'y':  8.5, 'r': 0.30, 'h': 0.45, 'shape': 'boulder'},

        # 5. Leg 5: W8 (5.9, 6.8) -> S1 (0, 0) [Open Southeast Plain]
        {'x':  3.2, 'y':  3.5, 'r': 0.30, 'h': 0.45, 'shape': 'boulder'},
    ]

    # ═════════════════════════════════════════════════════════════════
    # IMPASSABLE BARRIER WALL (Showcases unreachable path / dead-end)
    # ═════════════════════════════════════════════════════════════════
    # Northeast canyon wall blocking access past Y=21.0
    for k in range(12):
        wx = 3.5 + k * 0.42
        wy = 21.0
        obstacles.append({'x': wx, 'y': wy, 'r': 0.28, 'h': 0.65, 'shape': 'barrier_wall'})

    return obstacles


class KinematicMockSim(Node):

    def __init__(self):
        super().__init__('kinematic_mock_sim')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 1.5707963267948966) # 90 deg facing Mars Yard

        # Rover Kinematic State
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        self.vx = 0.0
        self.wz = 0.0
        self.last_time = self.get_clock().now()

        # Generate structured obstacle field
        self.obstacles = _rand_obstacles(seed=None)
        nav_count = sum(1 for o in self.obstacles if o['shape'] in ('boulder', 'cluster'))
        blocked_count = sum(1 for o in self.obstacles if o['shape'] == 'barrier_wall')
        self.get_logger().info(
            f"🗿 Obstacle field generated: {len(self.obstacles)} total primitives "
            f"({nav_count} navigable bypass rocks, {blocked_count} impassable barrier segments in 3 dead-end zones)"
        )

        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # Publishers (Standard RELIABLE QoS matching ROS 2 defaults)
        self.odom_pub = self.create_publisher(Odometry, '/rover_controller/odom', 10)
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)
        self.virtual_obs_viz_pub = self.create_publisher(MarkerArray, '/simulation/virtual_obstacles', 10)

        # Subscriptions
        self.create_subscription(TwistStamped, '/rover_controller/cmd_vel', self._cmd_vel_stamped_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._clicked_point_cb, 10)

        # Timers
        self.sim_timer = self.create_timer(0.05, self._update_sim)  # 20 Hz
        self.sensor_timer = self.create_timer(0.1, self._publish_sensors)  # 10 Hz

        self.get_logger().info('🚀 Lightweight Kinematic & Camera Simulator active! Place obstacles in RViz via "Publish Point".')

    def _publish_static_tfs(self):
        """Publish map->odom and base_footprint->camera_depth_frame static TFs."""
        static_tfs = []

        # 1. map → odom
        t0 = TransformStamped()
        t0.header.stamp = self.get_clock().now().to_msg()
        t0.header.frame_id = 'map'
        t0.child_frame_id = 'odom'
        t0.transform.rotation.w = 1.0
        static_tfs.append(t0)

        # 2. base_footprint → camera_depth_frame (matches d435i mounting)
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_footprint'
        t1.child_frame_id = 'camera_depth_frame'
        t1.transform.translation.x = CAMERA_X_OFFSET
        t1.transform.translation.z = CAMERA_Z_OFFSET
        t1.transform.rotation.w = 1.0
        static_tfs.append(t1)

        self.static_tf_broadcaster.sendTransform(static_tfs)

    def _cmd_vel_stamped_cb(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.wz = msg.twist.angular.z

    def _cmd_vel_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def _clicked_point_cb(self, msg: PointStamped):
        """Allow user to click in RViz to spawn interactive test rocks!"""
        new_obs = {'x': float(msg.point.x), 'y': float(msg.point.y), 'r': 0.30, 'h': 0.45, 'shape': 'boulder'}
        self.obstacles.append(new_obs)
        self.get_logger().info(f"✨ Spawned new virtual obstacle at [{new_obs['x']:.2f}, {new_obs['y']:.2f}] via RViz click!")

    def _update_sim(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        # Unicycle kinematics
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Broadcast odom -> base_footprint
        stamp = now.to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_footprint'
        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation.z = qz
        t_odom_base.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t_odom_base)

        # Publish Odometry
        odom_msg = self._create_odom_msg(stamp)
        self.odom_pub.publish(odom_msg)
        self.filtered_odom_pub.publish(odom_msg)

    def _create_odom_msg(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        return msg

    def _publish_sensors(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)

        # Camera origin in world coordinates
        cam_wx = self.x + CAMERA_X_OFFSET * cy
        cam_wy = self.y + CAMERA_X_OFFSET * sy

        # 1. Ground plane in camera_depth_frame (fast numpy meshgrid)
        gx = np.linspace(0.4, 3.5, 10, dtype=np.float32)
        gy = np.linspace(-1.2, 1.2, 10, dtype=np.float32)
        GX, GY = np.meshgrid(gx, gy)
        GZ = np.full_like(GX, -CAMERA_Z_OFFSET, dtype=np.float32)
        GZ += np.random.uniform(-0.008, 0.008, GZ.shape).astype(np.float32)
        ground_pts = np.stack([GX.ravel(), GY.ravel(), GZ.ravel()], axis=1)

        # 2. Obstacle 3D points in camera_depth_frame
        obs_parts = [ground_pts]
        for obs in self.obstacles:
            dx = obs['x'] - cam_wx
            dy = obs['y'] - cam_wy
            dist = math.hypot(dx, dy)
            if dist > MAX_RANGE + obs['r']:
                continue

            lx =  dx * cy + dy * sy
            ly = -dx * sy + dy * cy
            if lx < 0.2 or abs(ly) > (lx * math.tan(FOV_H / 2.0) + obs['r']):
                continue

            n = 100
            theta  = np.random.uniform(0, 2 * np.pi, n).astype(np.float32)
            r_rand = np.random.uniform(0, obs['r'],   n).astype(np.float32)
            px = (lx + r_rand * np.cos(theta)).astype(np.float32)
            py = (ly + r_rand * np.sin(theta)).astype(np.float32)
            pz = (-CAMERA_Z_OFFSET + np.random.uniform(0.06, obs['h'], n)).astype(np.float32)
            obs_parts.append(np.stack([px, py, pz], axis=1))

        all_pts = np.vstack(obs_parts).astype(np.float32)
        cloud_msg = self._make_pointcloud2_np(all_pts, stamp)
        self.points_pub.publish(cloud_msg)

        # 3. Ground-truth visual markers in map frame
        SHAPE_COLOR = {
            'boulder':      (0.92, 0.48, 0.08),  # Terracotta Orange (Navigable)
            'cluster':      (0.85, 0.35, 0.15),  # Rust Orange (Navigable)
            'barrier_wall': (0.95, 0.08, 0.08),  # Crimson Red (Impassable / Blocked)
            'slab':         (0.95, 0.80, 0.10),
            'ridge':        (0.10, 0.85, 0.90),
        }
        viz_array = MarkerArray()
        for idx, obs in enumerate(self.obstacles):
            shape = obs.get('shape', 'boulder')
            cr, cg, cb = SHAPE_COLOR.get(shape, (0.8, 0.5, 0.2))
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = f'gt_{shape}'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(obs['x'])
            m.pose.position.y = float(obs['y'])
            m.pose.position.z = float(obs['h'] / 2.0)
            m.pose.orientation.w = 1.0
            m.scale.x = float(obs['r'] * 2.0)
            m.scale.y = float(obs['r'] * 2.0)
            m.scale.z = float(obs['h'])
            m.color.r = cr
            m.color.g = cg
            m.color.b = cb
            m.color.a = 0.88
            viz_array.markers.append(m)
        self.virtual_obs_viz_pub.publish(viz_array)

    def _make_pointcloud2_np(self, pts: np.ndarray, stamp) -> PointCloud2:
        """Zero-copy fast PointCloud2 creation."""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_depth_frame'
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = KinematicMockSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
