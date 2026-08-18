import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray

DBSCAN_EPS = 0.20
# Was 100 against the raw cloud; lowered since VOXEL_SIZE downsampling below
# cuts points-per-object by roughly the same factor it cuts total points.
# Needs live tuning against actual observed post-downsample cluster sizes.
DBSCAN_MIN_SAMPLES = 20

MAX_CENTROID_Z = 3.0

# Cheap numpy voxel downsample before DBSCAN -- DBSCAN's cost scales with
# point count and density, so cutting input size here is the biggest lever
# on speed. Picks one representative point per voxel rather than a true
# centroid average (cheaper, good enough for this purpose).
VOXEL_SIZE = 0.03


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


class ObstacleClustering(Node):

    def __init__(self):
        super().__init__('obstacle_clustering')
        self.sub = self.create_subscription(
            PointCloud2,
            '/obstacles/filtered',
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(MarkerArray, '/obstacles/detected', 10)
        self.get_logger().info('subscribed, waiting for first cloud message...')
        self._frame_count = 0

    def cloud_callback(self, msg: PointCloud2):
        t0 = time.perf_counter()
        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info(f'first message received: {msg.width * msg.height} points')

        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        raw_count = points.shape[0]
        points = voxel_downsample(points, VOXEL_SIZE)

        markers = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if points.shape[0] >= DBSCAN_MIN_SAMPLES:
            labels = DBSCAN(
                eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES, n_jobs=-1
            ).fit_predict(points)

            marker_id = 0
            for label in set(labels):
                if label == -1:
                    continue  # noise

                cluster = points[labels == label]
                centroid = cluster.mean(axis=0)
                if centroid[2] > MAX_CENTROID_Z:
                    continue

                mins = cluster.min(axis=0)
                maxs = cluster.max(axis=0)
                size = np.maximum(maxs - mins, 0.02)  # avoid zero-volume markers

                marker = Marker()
                marker.header = msg.header
                marker.ns = 'obstacles'
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(centroid[0])
                marker.pose.position.y = float(centroid[1])
                marker.pose.position.z = float(centroid[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = float(size[0])
                marker.scale.y = float(size[1])
                marker.scale.z = float(size[2])
                marker.color.r = 1.0
                marker.color.g = 0.3
                marker.color.b = 0.0
                marker.color.a = 0.5
                # 0 duration = persist until explicitly replaced/deleted, which we
                # already do via DELETEALL at the top of every callback. A fixed
                # timeout here would expire markers during the camera's known
                # irregular publish gaps, causing flicker unrelated to detection quality.
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 0
                markers.markers.append(marker)
                marker_id += 1

        self.pub.publish(markers)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        self.get_logger().info(
            f'frame {self._frame_count}: {raw_count} raw -> {points.shape[0]} downsampled -> '
            f'{len(markers.markers) - 1} clusters, {elapsed_ms:.0f} ms'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleClustering()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
