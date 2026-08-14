#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class Trajectory(Node):
    def __init__(self):
        super().__init__("path_projection")
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.trajectory_pub = self.create_publisher(Path, "/rover_controller/trajectory", 10)
        self.path = Path()
        self.get_logger().info("Trajectory node started")

    def odom_callback(self, msg):
        self.path.header = msg.header

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.path.poses.append(pose_stamped)
        self.trajectory_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
