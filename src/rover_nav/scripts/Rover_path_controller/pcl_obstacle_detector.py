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
VOXEL_SIZE        = 0.03   # downsample leaf size (m) — smaller keeps more points
                           # per object so short rocks clear MIN_CLUSTER_PTS
PLANE_DIST_THRESH = 0.05   # ground plane fit tolerance (m)
DBSCAN_EPS        = 0.80   # cluster neighbourhood radius (m)
DBSCAN_MIN_PTS    = 5     # min points to form a cluster
MIN_CLUSTER_PTS   = 15     # discard clusters smaller than this after downsampling
MAX_CENTROID_Z    = 1.5    # (legacy, unused) superseded by MAX_RANGE

# ── local-planner gating ──────────────────────────────────────────────────────
# This node feeds the local planner, so only obstacles the rover could actually
# hit are worth publishing. Three gates, all applied per cluster:
#
#   range   -- nearest point of the cluster, measured as optical Z (forward from
#              the CAMERA). The camera sits 0.2756 m ahead of base_link, so a
#              cluster at MAX_RANGE is ~2.78 m from the rover centre.
#   height  -- tallest point above the ground surface. Anything shorter is
#              driveable, not an obstacle. Note this is the cluster TOP, while
#              GROUND_MARGIN stays low so the object's base is still captured
#              and its true height can be measured.
#   corridor-- lateral overlap with the swept path straight ahead. Rover is
#              0.759 m track + 0.070 m wheel = 0.829 m wide, so half-width is
#              0.4145 m; CORRIDOR_HALF_WIDTH adds clearance on top. Tested as an
#              OVERLAP, not a centroid test, so a rock poking into the path from
#              the side still counts.
MAX_RANGE            = 2.0    # ignore clusters whose nearest point is beyond this (m)
MIN_OBSTACLE_HEIGHT  = 0.14   # ignore clusters shorter than this above ground (m)
CORRIDOR_HALF_WIDTH  = 0.50   # half-width of the rover's path (m); 0.4145 = bare rover
CORRIDOR_ENABLED     = True   # False publishes everything in range, ignoring the corridor

# Ground removal. The cloud is in camera_depth_optical_frame: +X right,
# +Y DOWN, +Z forward. The URDF mounts the camera with rpy="0 0 0" relative
# to base_link, so optical +Y is gravity-aligned and height above ground is
# simply (CAMERA_HEIGHT - y). These are ROS parameters so they can be tuned
# live with `ros2 param set /obstacle_detector <name> <value>`.
CAMERA_HEIGHT     = 0.43   # optical centre above ground (m), measured on the rover
GROUND_MARGIN     = 0.04   # keep points this far above the ground surface (m).
                           # 0.08 hid anything under ~20 cm; 0.04 drops the
                           # detection floor to ~10 cm at the cost of needing
                           # CAMERA_HEIGHT accurate to a couple of cm.
GROUND_BAND       = 0.15   # points below this height are candidates for the plane fit (m)
MAX_HEIGHT        = 2.00   # ignore anything higher than this above ground (m)
FIT_GROUND_PLANE  = True   # fit the ground within the band instead of assuming it flat
NORMAL_TOL_DEG    = 25.0   # reject a fitted plane tilted more than this from horizontal
# ──────────────────────────────────────────────────────────────────────────────


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.create_subscription(PointCloud2, '/pcl/denoised', self._cb, 10)
        self._markers_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self._flag_pub    = self.create_publisher(Bool, '/obstacle_detected', 10)
        self._prev_count  = 0

        for name, default in (
            ('camera_height', CAMERA_HEIGHT),
            ('ground_margin', GROUND_MARGIN),
            ('ground_band', GROUND_BAND),
            ('max_height', MAX_HEIGHT),
            ('plane_dist_thresh', PLANE_DIST_THRESH),
            ('normal_tol_deg', NORMAL_TOL_DEG),
            ('voxel_size', VOXEL_SIZE),
            ('dbscan_eps', DBSCAN_EPS),
            ('max_centroid_z', MAX_CENTROID_Z),
            ('max_range', MAX_RANGE),
            ('min_obstacle_height', MIN_OBSTACLE_HEIGHT),
            ('corridor_half_width', CORRIDOR_HALF_WIDTH),
        ):
            self.declare_parameter(name, float(default))
        for name, default in (
            ('min_cluster_pts', MIN_CLUSTER_PTS),
            ('dbscan_min_pts', DBSCAN_MIN_PTS),
        ):
            self.declare_parameter(name, int(default))
        self.declare_parameter('fit_ground_plane', FIT_GROUND_PLANE)
        self.declare_parameter('corridor_enabled', CORRIDOR_ENABLED)

        self._log_every = 30
        self._frames = 0
        self.get_logger().info('Obstacle detector ready.')

    def _p(self, name):
        return self.get_parameter(name).value

    def _cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            self._publish([], msg.header.frame_id)
            return

        clusters = self._detect(pts.astype(np.float64))
        self._publish(clusters, msg.header.frame_id)

    def _ground_normal_and_offset(self, pts, height):
        """Plane (n, d) for the ground with n oriented UP, or None to fall back.

        Fitted only to points already near the ground (height < ground_band), so
        a tall object can never dominate the fit -- the old code fitted the whole
        cloud and happily deleted an object face instead of the floor. The normal
        is then checked against vertical, so a mis-fit is rejected rather than
        silently removing the wrong plane.
        """
        band = pts[height < self._p('ground_band')]
        if band.shape[0] < 50:
            return None

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(band)
        try:
            model, _ = cloud.segment_plane(
                self._p('plane_dist_thresh'), ransac_n=3, num_iterations=200
            )
        except RuntimeError:
            return None

        n = np.asarray(model[:3], dtype=np.float64)
        d = float(model[3])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            return None
        n, d = n / norm, d / norm

        # Optical +Y is down, so "up" is -Y. Orient the normal upwards.
        if n[1] > 0:
            n, d = -n, -d

        # Reject anything too far from horizontal (a wall, or a bad fit).
        tilt = np.degrees(np.arccos(np.clip(np.dot(n, np.array([0.0, -1.0, 0.0])), -1.0, 1.0)))
        if tilt > self._p('normal_tol_deg'):
            return None
        return n, d

    def _detect(self, xyz: np.ndarray) -> list:
        # 1. Voxel downsample
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.voxel_down_sample(self._p('voxel_size'))
        if len(pcd.points) < 3:
            return []
        pts = np.asarray(pcd.points)

        # 2. Remove the ground.
        #
        # Height above ground, using the known mounting geometry rather than
        # "whichever plane has the most inliers". Optical +Y is down, so a point
        # at y == camera_height sits exactly on the nominal ground.
        height = self._p('camera_height') - pts[:, 1]

        plane = self._ground_normal_and_offset(pts, height) if self._p('fit_ground_plane') else None
        if plane is not None:
            # Signed height above the *fitted* ground, which tracks slope.
            n, d = plane
            height = pts @ n + d
            mode = 'fitted'
        else:
            mode = 'nominal'

        keep = (height > self._p('ground_margin')) & (height < self._p('max_height'))
        pts = pts[keep]
        heights = height[keep]   # height above ground, per surviving point

        # Discard everything outside the rover's path before clustering. Doing
        # this at the POINT level (not just per cluster) is what stops a rock at
        # the edge of view, or a side wall, from being joined to something in
        # the path by DBSCAN and dragged into one oversized box.
        if self._p('corridor_enabled'):
            inside = np.abs(pts[:, 0]) <= self._p('corridor_half_width')
            pts = pts[inside]
            heights = heights[inside]

        self._frames += 1
        if self._frames % self._log_every == 0:
            self.get_logger().info(
                f'ground={mode}: {int(keep.sum())}/{keep.size} pts kept above '
                f"{self._p('ground_margin'):.2f} m"
            )

        if pts.shape[0] == 0:
            return []

        # 3. DBSCAN clustering
        labels = DBSCAN(eps=self._p('dbscan_eps'),
                        min_samples=self._p('dbscan_min_pts'),
                        n_jobs=-1).fit_predict(pts)

        # 4. Keep only clusters the rover could actually hit.
        half_w = self._p('corridor_half_width')
        use_corridor = self._p('corridor_enabled')
        max_range = self._p('max_range')
        min_h = self._p('min_obstacle_height')

        clusters = []
        rejected = {'small': 0, 'far': 0, 'short': 0, 'aside': 0}
        for lbl in set(labels):
            if lbl == -1:
                continue
            sel = labels == lbl
            c = pts[sel]
            if len(c) < self._p('min_cluster_pts'):
                rejected['small'] += 1
                continue

            # Nearest point, not the centroid: a wide obstacle whose centre is
            # past the limit can still have an edge inside braking distance.
            if c[:, 2].min() > max_range:
                rejected['far'] += 1
                continue

            # Tallest point above the ground surface.
            if heights[sel].max() < min_h:
                rejected['short'] += 1
                continue

            # Lateral overlap with the path, so something intruding from the
            # side counts even though its centroid sits outside the corridor.
            if use_corridor and (c[:, 0].min() > half_w or c[:, 0].max() < -half_w):
                rejected['aside'] += 1
                continue

            clusters.append(c)

        if self._frames % self._log_every == 0 and any(rejected.values()):
            self.get_logger().info(
                f"kept {len(clusters)} | rejected "
                f"small={rejected['small']} far={rejected['far']} "
                f"short={rejected['short']} off-path={rejected['aside']}"
            )

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
        # 0 = persist until explicitly replaced or DELETEd, which _publish
        # already does for stale ids. A finite lifetime flickers: the pipeline
        # runs ~3 Hz (0.33 s), so the old 0.3 s timeout expired markers before
        # the next frame arrived.
        m.lifetime   = Duration(sec=0, nanosec=0)
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
