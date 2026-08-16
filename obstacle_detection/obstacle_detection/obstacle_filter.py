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
        self.declare_parameter('input_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('depth_axis', 'auto')  # 'auto', 'x', 'z'

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.depth_axis_param = self.get_parameter('depth_axis').get_parameter_value().string_value

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(PointCloud2, '/obstacles/filtered', 10)
        self._frame_count = 0
        self._field_offsets = None
        self.get_logger().info(f'ObstacleFilter subscribed to {input_topic}')

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

        finite = np.isfinite(arr['x']) & np.isfinite(arr['y']) & np.isfinite(arr['z'])
        if not np.any(finite):
            return

        # Determine depth axis if auto
        depth_axis = self.depth_axis_param
        if depth_axis == 'auto':
            if 'optical' in msg.header.frame_id:
                depth_axis = 'z'
            else:
                # Body frame or standard camera frame: check which axis is positive forward
                if np.nanmax(arr['x'][finite]) > 0.5 and np.nanmax(arr['z'][finite]) <= 0.1:
                    depth_axis = 'x'
                else:
                    depth_axis = 'z'

        d = arr[depth_axis]
        mask = finite & (d >= self.min_range) & (d <= self.max_range)
        n_kept = int(mask.sum())
        if n_kept == 0:
            return

        filtered = np.empty((n_kept, 3), dtype=np.float32)
        filtered[:, 0] = arr['x'][mask]
        filtered[:, 1] = arr['y'][mask]
        filtered[:, 2] = arr['z'][mask]

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            self.get_logger().info(
                f'kept {n_kept}/{count} pts ({depth_axis}-axis [{self.min_range:.2f}, {self.max_range:.2f}]), '
                f'{elapsed_ms:.1f} ms/frame'
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
