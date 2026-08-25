#!/usr/bin/env python3
"""Converts the rover's front depth camera into a PointCloud2, so the local
costmap's obstacle_layer has something to mark/clear from -- the sim never
bridges a native points topic (see aries/config/gazebo_bridge.yaml: "The
/camera/.../points ... bridges were removed: ... nothing consumed the point
cloud"), only /camera/depth/image_rect_raw + /camera/depth/camera_info.
depth_image_proc would normally do this conversion, but it isn't installed
on this machine and the point is to keep obstacle avoidance self-contained
in rover_nav, so this is a small hand-rolled equivalent.

Publishes on /camera/depth/points (PointCloud2, in the camera's own optical
frame -- X right, Y down, Z forward/depth) at a downsampled resolution
(stride, default 4 -> 160x120 from the sim's 640x480 depth image). That
feeds straight into obstacle_detection_sim.launch.py's existing PassThrough
(range-crop) + Statistical Outlier Removal chain, whose output
(/pcl/denoised) is what local_costmap's obstacle_layer actually subscribes
to -- see nav2_local_planner_params.yaml.

Usage:
  ros2 run rover_nav depth_to_pointcloud.py
  ros2 run rover_nav depth_to_pointcloud.py --ros-args -p stride:=2 -p max_range:=3.0
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge


class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__("depth_to_pointcloud")

        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "/camera/depth/points")
        self.declare_parameter("stride", 4)
        # 2.0 m matches obstacle_detection_sim.launch.py's PassThrough filter
        # limit downstream -- generating points beyond what that stage keeps
        # is wasted numpy/serialization work every frame for nothing.
        self.declare_parameter("max_range", 2.0)

        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.stride = int(self.get_parameter("stride").value)
        self.max_range = float(self.get_parameter("max_range").value)

        self.bridge = CvBridge()
        self._uv_cache = None  # (u_grid, v_grid) for the current image size, built once

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(PointCloud2, output_topic, 5)

        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=sensor_qos)
        info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic, qos_profile=sensor_qos)
        # slop generous on purpose: the sim's depth sensor is only nominally
        # 15Hz and can be considerably jitterier under load (observed as low
        # as ~2Hz with >1s gaps in some conditions) -- a tight slop would just
        # silently drop every frame pair instead of degrading gracefully.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, info_sub], queue_size=10, slop=0.3
        )
        self.sync.registerCallback(self.on_frame)

        self.get_logger().info(
            f"depth_to_pointcloud: {depth_topic} + {camera_info_topic} -> {output_topic} "
            f"(stride={self.stride}, max_range={self.max_range}m)"
        )

    def on_frame(self, depth_msg, info_msg):
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.asarray(depth, dtype=np.float32)
        h, w = depth.shape

        if self._uv_cache is None or self._uv_cache[2] != (h, w):
            u = np.arange(0, w, self.stride)
            v = np.arange(0, h, self.stride)
            u_grid, v_grid = np.meshgrid(u, v)
            self._uv_cache = (u_grid, v_grid, (h, w))
        u_grid, v_grid, _ = self._uv_cache

        z = depth[v_grid, u_grid]
        valid = np.isfinite(z) & (z > 0.0) & (z <= self.max_range)
        if not np.any(valid):
            return

        fx, fy = info_msg.k[0], info_msg.k[4]
        cx, cy = info_msg.k[2], info_msg.k[5]

        u_valid = u_grid[valid].astype(np.float32)
        v_valid = v_grid[valid].astype(np.float32)
        z_valid = z[valid]

        x = (u_valid - cx) * z_valid / fx
        y = (v_valid - cy) * z_valid / fy
        points = np.stack([x, y, z_valid], axis=-1).astype(np.float32)

        cloud_msg = self._to_pointcloud2(points, depth_msg.header)
        self.pub.publish(cloud_msg)

    @staticmethod
    def _to_pointcloud2(points, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        return msg


def main():
    rclpy.init()
    node = DepthToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
