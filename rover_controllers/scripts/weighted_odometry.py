#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS     = 0.110   # m
WHEEL_SEPARATION = 0.665   # m

LEFT_JOINTS  = ['L_1_Wheel_Joint', 'L_2_Wheel_Joint', 'L_3_Wheel_Joint']
RIGHT_JOINTS = ['R_1_Wheel_Joint', 'R_2_Wheel_Joint', 'R_3_Wheel_Joint']

# Middle wheel weighted higher — stays grounded most on rocker-bogie
LEFT_WEIGHTS  = [0.25, 0.50, 0.25]
RIGHT_WEIGHTS = [0.25, 0.50, 0.25]


class WeightedOdometry(Node):
    def __init__(self):
        super().__init__('weighted_odometry')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_pos  = None
        self.prev_right_pos = None
        self.last_time      = None

    def weighted_avg(self, positions, weights):
        return sum(p * w for p, w in zip(positions, weights))

    def joint_states_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        if not all(j in name_to_pos for j in LEFT_JOINTS + RIGHT_JOINTS):
            return

        left_pos  = self.weighted_avg([name_to_pos[j] for j in LEFT_JOINTS],  LEFT_WEIGHTS)
        right_pos = self.weighted_avg([name_to_pos[j] for j in RIGHT_JOINTS], RIGHT_WEIGHTS)

        now = self.get_clock().now()

        if self.prev_left_pos is None:
            self.prev_left_pos  = left_pos
            self.prev_right_pos = right_pos
            self.last_time = now
            return

        delta_left  = (left_pos  - self.prev_left_pos)  * WHEEL_RADIUS
        delta_right = (right_pos - self.prev_right_pos) * WHEEL_RADIUS

        self.prev_left_pos  = left_pos
        self.prev_right_pos = right_pos

        delta_d     = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / WHEEL_SEPARATION

        self.x     += delta_d * math.cos(self.theta + delta_theta / 2.0)
        self.y     += delta_d * math.sin(self.theta + delta_theta / 2.0)
        self.theta  = math.atan2(
            math.sin(self.theta + delta_theta),
            math.cos(self.theta + delta_theta)
        )

        dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-9)
        self.last_time = now

        v = delta_d / dt
        w = delta_theta / dt

        self._publish(now, v, w)

    def _publish(self, stamp, v, w):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp    = stamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WeightedOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
