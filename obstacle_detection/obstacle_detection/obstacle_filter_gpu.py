import time
import traceback

import numpy as np
import open3d as o3d
import open3d.core as o3c
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

Z_MIN = 0.1
Z_MAX = 3.0
VOXEL_SIZE = 0.03
SOR_NB_NEIGHBORS = 20
SOR_STD_RATIO = 1.0

XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class ObstacleFilterGPU(Node):

    def __init__(self):
        super().__init__('obstacle_filter_gpu')
        self.device = o3c.Device('CUDA:0') if o3c.cuda.is_available() else o3c.Device('CPU:0')
        self.get_logger().info(f'open3d device: {self.device}')

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(PointCloud2, '/obstacles/filtered', 10)
        self._field_dtype = None
        self._frame_count = 0
        self.get_logger().info('subscribed, waiting for first cloud message...')

    def cloud_callback(self, msg: PointCloud2):
        try:
            self._cloud_callback_impl(msg)
        except Exception:
            self.get_logger().error('cloud_callback crashed:\n' + traceback.format_exc())

    def _cloud_callback_impl(self, msg: PointCloud2):
        t0 = time.perf_counter()
        if self._frame_count == 0:
            self.get_logger().info(
                f'first message received: width={msg.width} height={msg.height} '
                f'point_step={msg.point_step} fields={[f.name for f in msg.fields]}'
            )

        if self._field_dtype is None:
            offsets = {f.name: f.offset for f in msg.fields}
            self._field_dtype = np.dtype({
                'names': ['x', 'y', 'z'],
                'formats': [np.float32, np.float32, np.float32],
                'offsets': [offsets['x'], offsets['y'], offsets['z']],
                'itemsize': msg.point_step,
            })

        count = msg.width * msg.height
        arr = np.frombuffer(msg.data, dtype=self._field_dtype, count=count)

        z = arr['z']
        mask = (z >= Z_MIN) & (z <= Z_MAX)
        n = int(mask.sum())
        if n == 0:
            if self._frame_count < 5:
                self.get_logger().warn(
                    f'passthrough dropped all {count} points this frame '
                    f'(z range seen: {np.nanmin(z):.3f} to {np.nanmax(z):.3f})'
                )
            self._frame_count += 1
            return

        xyz = np.empty((n, 3), dtype=np.float32)
        xyz[:, 0] = arr['x'][mask]
        xyz[:, 1] = arr['y'][mask]
        xyz[:, 2] = z[mask]

        # VoxelGrid downsample on GPU
        pcd = o3d.t.geometry.PointCloud(self.device)
        pcd.point.positions = o3c.Tensor(xyz, device=self.device)
        pcd_down = pcd.voxel_down_sample(VOXEL_SIZE)
        down_xyz = pcd_down.point.positions.cpu().numpy()

        # SOR on CPU (open3d docs: not recommended on GPU), but on the
        # much smaller downsampled set instead of the raw cloud.
        if down_xyz.shape[0] > SOR_NB_NEIGHBORS:
            legacy = o3d.geometry.PointCloud()
            legacy.points = o3d.utility.Vector3dVector(down_xyz.astype(np.float64))
            legacy_filtered, _ = legacy.remove_statistical_outlier(
                nb_neighbors=SOR_NB_NEIGHBORS, std_ratio=SOR_STD_RATIO
            )
            filtered = np.asarray(legacy_filtered.points, dtype=np.float32)
        else:
            filtered = down_xyz.astype(np.float32)

        self._frame_count += 1
        elapsed_ms = (time.perf_counter() - t0) * 1000
        if self._frame_count <= 5 or self._frame_count % 30 == 0:
            self.get_logger().info(
                f'frame {self._frame_count}: raw {n} -> downsampled {down_xyz.shape[0]} '
                f'-> filtered {filtered.shape[0]}, {elapsed_ms:.1f} ms'
            )

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = filtered.shape[0]
        out_msg.fields = XYZ_FIELDS
        out_msg.is_bigendian = False
        out_msg.point_step = 12
        out_msg.row_step = 12 * filtered.shape[0]
        out_msg.is_dense = True
        out_msg.data = np.ascontiguousarray(filtered, dtype=np.float32).tobytes()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFilterGPU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
