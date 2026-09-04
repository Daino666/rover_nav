#!/usr/bin/env python3
import json
import math

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import tf2_ros
from rclpy.time import Time
from rover_nav.msg import Obstacle, ObstacleArray


# ── tuneable ──────────────────────────────────────────────────────────────────
# Every constant below is declared as a ROS parameter and re-read each frame, so
# it can be tuned live:
#   ros2 run rover_nav tune_obstacle_detection.py     (interactive tuner)
#   ros2 param set /obstacle_detector <name> <value>  (one-off)
VOXEL_SIZE        = 0.03   # downsample leaf size (m) — smaller keeps more points
                           # per object so short rocks clear MIN_CLUSTER_PTS
PLANE_DIST_THRESH = 0.05   # ground plane fit tolerance (m)
DBSCAN_EPS        = 0.20   # cluster neighbourhood radius (m). At VOXEL_SIZE 0.03
                           # this is a ~7-voxel gap. The old 0.80 tolerated 26
                           # voxels, so separate objects chained into one cluster
                           # -- observed live as a single box spanning a person
                           # and a wall section 0.5 m to their side.
DBSCAN_MIN_PTS    = 12     # min points to form a cluster. At 5, five stray noise
                           # points are enough to seed one.
MIN_CLUSTER_PTS   = 15     # discard clusters smaller than this after downsampling
MAX_RANGE         = 2.0    # ignore obstacles whose NEAREST point is beyond this (m).
                           # Gating on the centroid instead hid anything straddling the
                           # limit -- a wall spanning 1.3-2.0 m has its centroid at 1.65
                           # and vanished even though part of it was well inside range.
# Ignore anything shorter than this. Measured as the cluster's TALLEST point
# above the ground surface -- GROUND_MARGIN stays low so the object's base is
# still captured and its true height can be measured. Raising GROUND_MARGIN to
# 0.15 instead would lop the bottom off every object and under-report heights.
MIN_OBSTACLE_HEIGHT = 0.25  # discard clusters shorter than this (m).
# Raised from 0.15 2026-09-04: only BIG obstacles should stop the rover.
#
# Height is NOT what keeps terrain out -- it cannot be. Approaching a hill the
# ground fit locks onto the flat ground at the rover's feet, so the hill is
# measured against that and a 12 deg slope has already risen 0.25 m by the
# 1.2 m stop distance. Any height threshold low enough to catch real rocks also
# catches hills. That job belongs to TERRAIN_MAX_SLOPE_DEG below, which
# classifies by rise-over-run instead, and to the FITTED ground plane.
#
# So this is purely a "don't bother with small stuff" floor. The trade is
# explicit: anything shorter than this the rover will drive into.

# Ground removal. The cloud is in camera_depth_optical_frame: +X right,
# +Y DOWN, +Z forward. The URDF mounts the camera with rpy="0 0 0" relative
# to base_link, so optical +Y is gravity-aligned and height above ground is
# simply (CAMERA_HEIGHT - y).
CAMERA_HEIGHT     = 0.489  # optical centre above ground (m). Measured from a live
                           # ground-plane fit on the rover, not taped: a hand estimate
                           # of 0.43 was ~6 cm low. Only used when the plane fit is
                           # rejected, but it also selects the band the fit runs on.
GROUND_MARGIN     = 0.04   # keep points this far above the ground surface (m).
                           # 0.08 hid anything under ~20 cm; 0.04 drops the
                           # detection floor to ~10 cm at the cost of needing
                           # CAMERA_HEIGHT accurate to a couple of cm.
GROUND_BAND       = 0.15   # points below this height are candidates for the plane fit (m)
MAX_HEIGHT        = 2.00   # ignore anything higher than this above ground (m)
FIT_GROUND_PLANE  = True   # fit the ground within the band instead of assuming it flat
NORMAL_TOL_DEG    = 25.0   # reject a fitted plane tilted more than this from horizontal

# Terrain rejection by STEEPNESS (see the filter in _cluster). A cluster whose
# height gain over its own depth extent is shallower than this is ground the
# rover can drive -- a hill, a bank, a ramp -- not something to stop for.
# 30 deg sits above the yard's drivable slopes and well below the near-vertical
# face of a rock or wall. terrain_min_depth is the depth a cluster must span
# before its slope means anything at all; below it, no slope is measurable and
# the cluster stays an obstacle.
TERRAIN_MAX_SLOPE_DEG = 30.0
TERRAIN_MIN_DEPTH     = 0.30

# Frame the ObstacleArray is published in. The markers and the Bool stay in the
# camera's own optical frame (RViz and the stop-gate neither need nor want a
# transform), but anything that plans has to receive obstacles in the frame it
# plans in -- cmd_vel_arbiter.py drives in odom. Empty disables the array
# entirely and leaves only the markers + Bool.
OUTPUT_FRAME      = 'odom'
# ──────────────────────────────────────────────────────────────────────────────


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.create_subscription(PointCloud2, '/pcl/denoised', self._cb, 10)
        self._markers_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self._flag_pub    = self.create_publisher(Bool, '/obstacle_detected', 10)
        # Per-frame counters for the tuner: which stage discarded what. Without
        # this, "my rock disappeared" is indistinguishable from "no rock seen".
        self._stats_pub   = self.create_publisher(String, '/obstacles/stats', 10)
        # Planner-facing output: centroid + radius in a planning frame, unlike
        # the RViz markers which stay in the optical frame as raw line lists.
        self._array_pub   = self.create_publisher(ObstacleArray, '/obstacles/array', 10)
        self._prev_count  = 0

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_warned   = False

        for name, default in (
            ('camera_height', CAMERA_HEIGHT),
            ('ground_margin', GROUND_MARGIN),
            ('ground_band', GROUND_BAND),
            ('max_height', MAX_HEIGHT),
            ('plane_dist_thresh', PLANE_DIST_THRESH),
            ('normal_tol_deg', NORMAL_TOL_DEG),
            ('voxel_size', VOXEL_SIZE),
            ('dbscan_eps', DBSCAN_EPS),
            ('max_range', MAX_RANGE),
            ('min_obstacle_height', MIN_OBSTACLE_HEIGHT),
            ('terrain_max_slope_deg', TERRAIN_MAX_SLOPE_DEG),
            ('terrain_min_depth', TERRAIN_MIN_DEPTH),
        ):
            self.declare_parameter(name, float(default))
        for name, default in (
            ('min_cluster_pts', MIN_CLUSTER_PTS),
            ('dbscan_min_pts', DBSCAN_MIN_PTS),
        ):
            self.declare_parameter(name, int(default))
        self.declare_parameter('fit_ground_plane', FIT_GROUND_PLANE)
        self.declare_parameter('output_frame', OUTPUT_FRAME)

        self._log_every = 30
        self._frames = 0
        self._stats = {}
        self.get_logger().info('Obstacle detector ready.')

    def _p(self, name):
        return self.get_parameter(name).value

    def _cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self._stats = {'in_pts': int(pts.shape[0])}
        if pts.shape[0] == 0:
            self._publish([], msg.header)
            return

        clusters = self._detect(pts.astype(np.float64))
        self._publish(clusters, msg.header)

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
        self._stats['voxel_pts'] = int(pts.shape[0])

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
        self._stats['ground'] = mode
        self._stats['kept_pts'] = int(keep.sum())

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

        # 4. Filter clusters
        clusters = []
        n_small = n_far = n_short = n_terrain = 0
        for lbl in set(labels):
            if lbl == -1:
                continue
            sel = labels == lbl
            c = pts[sel]
            if len(c) < self._p('min_cluster_pts'):
                n_small += 1
                continue
            if c[:, 2].min() > self._p('max_range'):
                n_far += 1
                continue
            # voxel_down_sample averages each voxel, so the topmost voxel sits
            # ~half a leaf below the object's real top. Add that back, otherwise
            # a true 15 cm rock measures ~14.2 cm and is wrongly discarded --
            # i.e. the parameter means ACTUAL object height, not measured height.
            top = heights[sel].max() + 0.5 * self._p('voxel_size')
            if top < self._p('min_obstacle_height'):
                n_short += 1
                continue

            # TERRAIN vs OBSTACLE, by steepness rather than height.
            #
            # Height alone cannot tell a hill from a rock, and raising
            # min_obstacle_height does not fix it. The ground plane is fitted
            # only from points within ground_band of the local ground, so
            # approaching a hill the fit locks onto the flat ground at the
            # rover's feet and the hill is measured against THAT: a 20 deg
            # slope has risen 0.44 m by 1.2 m out, clearing any sane height
            # threshold. Push the threshold high enough to ignore hills and
            # you ignore real rocks too.
            #
            # What actually separates them is rise over run. A hill climbs
            # gradually and the camera sees its surface receding in Z; a rock
            # or a wall presents a near-vertical face at roughly constant Z.
            # So: measure the cluster's height gain against its depth extent
            # and treat anything shallower than terrain_max_slope as ground.
            #
            # Fails toward DETECTING: a cluster with little depth extent has no
            # measurable slope, so it stays an obstacle. That is the right bias
            # -- a rock seen face-on is exactly that shape.
            depth = float(c[:, 2].max() - c[:, 2].min())
            if depth >= self._p('terrain_min_depth'):
                rise = float(heights[sel].max() - heights[sel].min())
                slope_deg = math.degrees(math.atan2(rise, depth))
                if slope_deg < self._p('terrain_max_slope_deg'):
                    n_terrain += 1
                    continue
            clusters.append(c)

        self._stats.update({
            'raw_clusters': int(len({l for l in labels if l != -1})),
            'terrain_rejected': int(n_terrain),
            'drop_small': n_small,
            'drop_far': n_far,
            'drop_short': n_short,
        })

        if n_short and self._frames % self._log_every == 0:
            self.get_logger().info(
                f"dropped {n_short} cluster(s) shorter than "
                f"{self._p('min_obstacle_height'):.2f} m"
            )

        return clusters

    def _publish(self, clusters: list, header):
        frame_id = header.frame_id
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
        # Published before the array, and never gated on TF: the stop-gate in
        # cmd_vel_arbiter.py rides on this Bool, so a missing transform must
        # not silently disarm the brake.
        self._flag_pub.publish(Bool(data=len(clusters) > 0))
        self._publish_array(clusters, header)
        self._publish_stats(clusters)

    def _lookup(self, target, source, stamp):
        """Transform (R, t) from `source` into `target`, or None.

        Falls back to the latest available transform when the cloud's own stamp
        has already been dropped from the buffer. At rover speeds the few tens
        of ms of skew costs far less accuracy than refusing to publish at all,
        which is what a strict exact-stamp lookup would do most frames.
        """
        for when in (Time.from_msg(stamp), Time()):
            try:
                tf = self._tf_buffer.lookup_transform(target, source, when)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, tf2_ros.TransformException):
                continue
            q = tf.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
                [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
            ])
            v = tf.transform.translation
            return R, np.array([v.x, v.y, v.z])
        return None

    def _publish_array(self, clusters: list, header):
        """Centroid + enclosing radius per cluster, in the planning frame.

        Points are transformed and only THEN flattened to x/y. Doing it the
        other way round would mean hand-remapping the optical frame's
        (X right, Y down, Z forward) onto the planner's (X forward, Y left),
        which is exactly the kind of axis bookkeeping TF exists to remove.
        """
        target = str(self._p('output_frame')).strip()
        if not target:
            return

        msg = ObstacleArray()
        if clusters:
            tf = self._lookup(target, header.frame_id, header.stamp)
            if tf is None:
                if not self._tf_warned:
                    self._tf_warned = True
                    self.get_logger().warn(
                        f'no transform {header.frame_id} -> {target}; '
                        f'/obstacles/array stays empty while markers and '
                        f'/obstacle_detected keep publishing. Start '
                        f'robot_state_publisher, or set output_frame to "".'
                    )
                return
            self._tf_warned = False
            R, t = tf
            for c in clusters:
                pts = c @ R.T + t
                cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
                radius = float(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy).max())
                msg.obstacles.append(Obstacle(x=cx, y=cy, radius=radius))

        self._array_pub.publish(msg)

    def _publish_stats(self, clusters: list):
        s = dict(self._stats)
        s['published'] = len(clusters)
        s['obstacles'] = [
            {
                'range': round(float(c[:, 2].min()), 3),
                'pts': int(len(c)),
                'size': [round(float(v), 3) for v in (c.max(axis=0) - c.min(axis=0))],
            }
            for c in clusters
        ]
        self._stats_pub.publish(String(data=json.dumps(s)))

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
