#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuToTf(Node):
    def __init__(self):
        super().__init__("imu_to_tf")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_footprint")
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.get_logger().info(
            f"Broadcasting {self.parent_frame} -> {self.child_frame} from /imu yaw"
        )

    def imu_callback(self, msg):
        q = msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToTf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
