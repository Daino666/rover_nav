import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

Z_MIN = 0.1
Z_MAX = 3.0

XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class ObstacleFilter(Node):

    def __init__(self):
        super().__init__('obstacle_filter')
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(PointCloud2, '/obstacles/filtered', 10)
        self._frame_count = 0
        self._field_offsets = None

    def cloud_callback(self, msg: PointCloud2):
        t0 = time.perf_counter()

        if self._field_offsets is None:
            offsets = {f.name: f.offset for f in msg.fields}
            self._field_offsets = np.dtype({
                'names': ['x', 'y', 'z'],
                'formats': [np.float32, np.float32, np.float32],
                'offsets': [offsets['x'], offsets['y'], offsets['z']],
                'itemsize': msg.point_step,
            })

        count = msg.width * msg.height
        arr = np.frombuffer(msg.data, dtype=self._field_offsets, count=count)

        z = arr['z']
        # NaN comparisons are always False in numpy, so invalid/no-return
        # points (NaN x/y/z) are dropped by this range check with no
        # separate isnan pass needed.
        mask = (z >= Z_MIN) & (z <= Z_MAX)
        n_kept = int(mask.sum())
        if n_kept == 0:
            return

        filtered = np.empty((n_kept, 3), dtype=np.float32)
        filtered[:, 0] = arr['x'][mask]
        filtered[:, 1] = arr['y'][mask]
        filtered[:, 2] = z[mask]

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            self.get_logger().info(
                f'kept {n_kept}/{count} pts, z range [{filtered[:, 2].min():.2f}, '
                f'{filtered[:, 2].max():.2f}], {elapsed_ms:.1f} ms/frame'
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
        out_msg.data = filtered.tobytes()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
