#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


# ── tuneable ──────────────────────────────────────────────────────────────────
VOXEL_SIZE        = 0.05   # downsample leaf size (m) — speeds up DBSCAN
PLANE_DIST_THRESH = 0.05   # RANSAC ground removal threshold (m)
DBSCAN_EPS        = 0.80   # cluster neighbourhood radius (m)
DBSCAN_MIN_PTS    = 5     # min points to form a cluster
MIN_CLUSTER_PTS   = 30     # discard clusters smaller than this after downsampling
MAX_CENTROID_Z    = 1.5    # discard clusters whose centroid Z exceeds this (m)
# ──────────────────────────────────────────────────────────────────────────────


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.create_subscription(PointCloud2, '/pcl/denoised', self._cb, 10)
        self._markers_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self._flag_pub    = self.create_publisher(Bool, '/obstacle_detected', 10)
        self._prev_count  = 0
        self.get_logger().info('Obstacle detector ready.')

    def _cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            self._publish([], msg.header.frame_id)
            return

        clusters = self._detect(pts.astype(np.float64))
        self._publish(clusters, msg.header.frame_id)

    def _detect(self, xyz: np.ndarray) -> list:
        # 1. Voxel downsample
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.voxel_down_sample(VOXEL_SIZE)

        # 2. Remove ground plane
        if len(pcd.points) < 3:
            return []
        _, inliers = pcd.segment_plane(PLANE_DIST_THRESH, ransac_n=3, num_iterations=100)
        pcd = pcd.select_by_index(inliers, invert=True)
        if len(pcd.points) == 0:
            return []

        pts = np.asarray(pcd.points)

        # 3. DBSCAN clustering
        labels = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_PTS, n_jobs=-1).fit_predict(pts)

        # 4. Filter clusters
        clusters = []
        for lbl in set(labels):
            if lbl == -1:
                continue
            c = pts[labels == lbl]
            if len(c) < MIN_CLUSTER_PTS:
                continue
            if c[:, 2].mean() > MAX_CENTROID_Z:
                continue
            clusters.append(c)

        return clusters

    def _publish(self, clusters: list, frame_id: str):
        markers = MarkerArray()

        for i, c in enumerate(clusters):
            markers.markers.append(self._bbox(c, i, frame_id))

        # delete leftover markers from previous frame
        for i in range(len(clusters), self._prev_count):
            m = Marker()
            m.header.frame_id = frame_id
            m.ns, m.id, m.action = 'obstacles', i, Marker.DELETE
            markers.markers.append(m)

        self._prev_count = len(clusters)
        self._markers_pub.publish(markers)
        self._flag_pub.publish(Bool(data=len(clusters) > 0))

    def _bbox(self, pts: np.ndarray, mid: int, frame_id: str) -> Marker:
        mn, mx = pts.min(axis=0), pts.max(axis=0)
        corners = [
            [mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]],
            [mx[0], mx[1], mn[2]], [mn[0], mx[1], mn[2]],
            [mn[0], mn[1], mx[2]], [mx[0], mn[1], mx[2]],
            [mx[0], mx[1], mx[2]], [mn[0], mx[1], mx[2]],
        ]
        edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]

        m = Marker()
        m.header.frame_id = frame_id
        m.ns, m.id   = 'obstacles', mid
        m.type       = Marker.LINE_LIST
        m.action     = Marker.ADD
        m.scale.x    = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.0, 1.0
        m.lifetime   = Duration(sec=0, nanosec=300_000_000)
        for a, b in edges:
            m.points.append(Point(x=corners[a][0], y=corners[a][1], z=corners[a][2]))
            m.points.append(Point(x=corners[b][0], y=corners[b][1], z=corners[b][2]))
        return m


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
