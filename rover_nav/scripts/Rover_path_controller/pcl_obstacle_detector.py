#!/usr/bin/env python3
"""
Robust 3D PointCloud Obstacle Detector for Mars Rover

Transforms incoming point clouds into the base_link robot frame via TF2,
removes the ground plane using RANSAC, clusters obstacles with DBSCAN,
and publishes ObstacleArray, RViz markers, and clean obstacle point clouds.
"""

import math
import numpy as np
import open3d as o3d
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rover_nav.msg import Obstacle, ObstacleArray

# ── Tuneable Perception Parameters ────────────────────────────────────────────
VOXEL_SIZE        = 0.04   # Downsample leaf size (m)
PLANE_DIST_THRESH = 0.08   # RANSAC ground removal threshold (m)
DBSCAN_EPS        = 0.28   # Cluster neighbourhood radius (m)
DBSCAN_MIN_PTS    = 5      # Min points to form a cluster
MIN_CLUSTER_PTS   = 8      # Discard clusters smaller than this
MAX_CENTROID_Z    = 1.2    # Discard clusters above rover sensor height in base_link (m)
MIN_CENTROID_Z    = -0.15  # Discard clusters below wheel contact plane in base_link (m)
# Fallback offsets if TF lookup fails early at launch
CAMERA_TO_BASE_X  = 0.245
CAMERA_TO_BASE_Z  = 0.225
# ──────────────────────────────────────────────────────────────────────────────


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, '/pcl/denoised', self._cb, 10)
        self._markers_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self._obstacles_pub = self.create_publisher(ObstacleArray, '/obstacles', 10)
        self._flag_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self._clean_pts_pub = self.create_publisher(PointCloud2, '/obstacles/points', 10)

        self._prev_marker_count = 0
        self.get_logger().info('✅ 3D Obstacle Detector initialized with robust TF2 base_link transform.')

    def _transform_to_base_link(self, pts: np.ndarray, frame_id: str, stamp) -> np.ndarray:
        """Transform (N, 3) point array from sensor frame_id to base_link frame."""
        if frame_id == 'base_link':
            return pts

        try:
            if self.tf_buffer.can_transform('base_link', frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)):
                t = self.tf_buffer.lookup_transform('base_link', frame_id, rclpy.time.Time())
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z

                rot = R.from_quat([qx, qy, qz, qw])
                pts_transformed = rot.apply(pts) + np.array([tx, ty, tz])
                return pts_transformed
        except Exception:
            pass

        # Fallback transform if TF is not ready
        if 'optical' in frame_id:
            # Optical frame: X right, Y down, Z forward
            x_base = pts[:, 2] + CAMERA_TO_BASE_X
            y_base = -pts[:, 0]
            z_base = -pts[:, 1] + CAMERA_TO_BASE_Z
            return np.column_stack([x_base, y_base, z_base])
        else:
            # Robot-aligned camera frame (camera_depth_frame / camera_link: X forward, Y left, Z up)
            x_base = pts[:, 0] + CAMERA_TO_BASE_X
            y_base = pts[:, 1]
            z_base = pts[:, 2] + CAMERA_TO_BASE_Z
            return np.column_stack([x_base, y_base, z_base])

    def _cb(self, msg: PointCloud2):
        raw_pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if raw_pts.shape[0] == 0:
            self._publish([], np.empty((0, 3)), msg.header.stamp)
            return

        # 1. Transform to base_link
        pts_base = self._transform_to_base_link(raw_pts.astype(np.float64), msg.header.frame_id, msg.header.stamp)

        # 2. Filter non-finite and bounding region in base_link frame
        finite = np.isfinite(pts_base[:, 0]) & np.isfinite(pts_base[:, 1]) & np.isfinite(pts_base[:, 2])
        pts_base = pts_base[finite]

        # Crop to forward FOV box in front of rover in base_link frame
        in_fov = (
            (pts_base[:, 0] >= 0.20) & (pts_base[:, 0] <= 3.0) &
            (pts_base[:, 1] >= -1.2) & (pts_base[:, 1] <= 1.2) &
            (pts_base[:, 2] >= -0.25) & (pts_base[:, 2] <= 1.5)
        )
        pts_base = pts_base[in_fov]

        if pts_base.shape[0] < 10:
            self._publish([], np.empty((0, 3)), msg.header.stamp)
            return

        clusters, obs_pts = self._detect(pts_base)
        self._publish(clusters, obs_pts, msg.header.stamp)

    def _detect(self, xyz: np.ndarray):
        # 1. Voxel downsample
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.voxel_down_sample(VOXEL_SIZE)

        if len(pcd.points) < 10:
            return [], np.empty((0, 3))

        # 2. Remove ground plane via RANSAC
        plane_model, inliers = pcd.segment_plane(PLANE_DIST_THRESH, ransac_n=3, num_iterations=50)
        pcd_obstacles = pcd.select_by_index(inliers, invert=True)
        if len(pcd_obstacles.points) < MIN_CLUSTER_PTS:
            return [], np.empty((0, 3))

        pts = np.asarray(pcd_obstacles.points)

        # 3. Open3D native C++ DBSCAN clustering
        labels = np.array(pcd_obstacles.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_PTS, print_progress=False))

        # 4. Extract clusters
        clusters = []
        valid_obs_pts = []
        for lbl in set(labels):
            if lbl == -1:
                continue
            c = pts[labels == lbl]
            if len(c) < MIN_CLUSTER_PTS:
                continue

            # Check 3D vertical extent: genuine rocks/obstacles have physical height
            c_min = c.min(axis=0)
            c_max = c.max(axis=0)
            height_extent = c_max[2] - c_min[2]
            if height_extent < 0.08:
                continue

            cz = c[:, 2].mean()
            if cz > MAX_CENTROID_Z or cz < MIN_CENTROID_Z:
                continue
            clusters.append(c)
            valid_obs_pts.append(c)

        all_obs_pts = np.vstack(valid_obs_pts) if valid_obs_pts else np.empty((0, 3))
        return clusters, all_obs_pts

    def _publish(self, clusters: list, obs_pts: np.ndarray, stamp):
        markers = MarkerArray()
        obs_array_msg = ObstacleArray()
        frame_id = 'base_link'
        marker_idx = 0

        # Publish clean obstacle points in base_link frame
        if obs_pts.shape[0] > 0:
            cloud_msg = self._make_pointcloud2_np(obs_pts.astype(np.float32), frame_id, stamp)
            self._clean_pts_pub.publish(cloud_msg)

        has_danger_obs = False

        for i, c in enumerate(clusters):
            mn, mx = c.min(axis=0), c.max(axis=0)
            centroid = c.mean(axis=0)
            size = mx - mn
            radius = float(max(size[0], size[1]) / 2.0)
            dist = math.sqrt(centroid[0]**2 + centroid[1]**2)

            x_base = float(centroid[0])
            y_base = float(centroid[1])

            if dist < 2.5:
                has_danger_obs = True

            # 1. Populate ObstacleArray message for Pure Pursuit & planners
            obs = Obstacle()
            obs.x = float(x_base)
            obs.y = float(y_base)
            obs.radius = float(radius)
            obs_array_msg.obstacles.append(obs)

            # Determine danger color
            if dist < 1.2:
                r_col, g_col, b_col = 1.0, 0.1, 0.1  # Red (Immediate danger)
            elif dist < 2.2:
                r_col, g_col, b_col = 1.0, 0.6, 0.0  # Orange (Caution)
            else:
                r_col, g_col, b_col = 0.1, 0.9, 0.3  # Green (Safe)

            # 2. 3D Wireframe Bounding Box Marker
            bbox_marker = self._create_bbox(mn, mx, marker_idx, frame_id, r_col, g_col, b_col)
            markers.markers.append(bbox_marker)
            marker_idx += 1

            # 3. Ground Safety Clearance Disk (Cylinder)
            disk_marker = Marker()
            disk_marker.header.frame_id = frame_id
            disk_marker.header.stamp = self.get_clock().now().to_msg()
            disk_marker.ns = 'obstacle_clearance'
            disk_marker.id = marker_idx
            disk_marker.type = Marker.CYLINDER
            disk_marker.action = Marker.ADD
            disk_marker.pose.position.x = float(centroid[0])
            disk_marker.pose.position.y = float(centroid[1])
            disk_marker.pose.position.z = float(mn[2])
            disk_marker.scale.x = float(radius * 2.0 + 0.3)
            disk_marker.scale.y = float(radius * 2.0 + 0.3)
            disk_marker.scale.z = 0.02
            disk_marker.color.r = r_col
            disk_marker.color.g = g_col
            disk_marker.color.b = b_col
            disk_marker.color.a = 0.35
            disk_marker.lifetime = Duration(sec=0, nanosec=300_000_000)
            markers.markers.append(disk_marker)
            marker_idx += 1

            # 4. Floating 3D Text HUD
            hud_marker = Marker()
            hud_marker.header.frame_id = frame_id
            hud_marker.header.stamp = self.get_clock().now().to_msg()
            hud_marker.ns = 'obstacle_hud'
            hud_marker.id = marker_idx
            hud_marker.type = Marker.TEXT_VIEW_FACING
            hud_marker.action = Marker.ADD
            hud_marker.pose.position.x = float(centroid[0])
            hud_marker.pose.position.y = float(centroid[1])
            hud_marker.pose.position.z = float(mx[2] + 0.25)
            hud_marker.scale.z = 0.12
            hud_marker.color.r = 1.0
            hud_marker.color.g = 1.0
            hud_marker.color.b = 1.0
            hud_marker.color.a = 1.0
            hud_marker.text = f"Obs #{i+1}\nd={dist:.2f}m | r={radius:.2f}m"
            hud_marker.lifetime = Duration(sec=0, nanosec=300_000_000)
            markers.markers.append(hud_marker)
            marker_idx += 1

        # Delete leftover markers from previous frame
        for i in range(marker_idx, self._prev_marker_count):
            m = Marker()
            m.header.frame_id = frame_id
            m.ns, m.id, m.action = 'obstacles', i, Marker.DELETE
            markers.markers.append(m)

        self._prev_marker_count = marker_idx
        self._markers_pub.publish(markers)
        self._obstacles_pub.publish(obs_array_msg)
        self._flag_pub.publish(Bool(data=has_danger_obs))

    def _create_bbox(self, mn: np.ndarray, mx: np.ndarray, mid: int, frame_id: str, r: float, g: float, b: float) -> Marker:
        corners = [
            [mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]],
            [mx[0], mx[1], mn[2]], [mn[0], mx[1], mn[2]],
            [mn[0], mn[1], mx[2]], [mx[0], mn[1], mx[2]],
            [mx[0], mx[1], mx[2]], [mn[0], mx[1], mx[2]],
        ]
        edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'obstacles', mid
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.lifetime = Duration(sec=0, nanosec=300_000_000)
        for a, b_idx in edges:
            m.points.append(Point(x=float(corners[a][0]), y=float(corners[a][1]), z=float(corners[a][2])))
            m.points.append(Point(x=float(corners[b_idx][0]), y=float(corners[b_idx][1]), z=float(corners[b_idx][2])))
        return m

    def _make_pointcloud2_np(self, pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
