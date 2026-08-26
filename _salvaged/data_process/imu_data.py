import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import csv
import math
import time


class ImuLogger(Node):

    def __init__(self):
        super().__init__('imu_logger')

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        self.yaw_rates = []
        self.yaws = []
        self.start_time = time.time()

        self.get_logger().info("IMU Logger Started... Keep rover stationary.")

    def quat_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg):

        if time.time() - self.start_time > 60:
            self.compute_and_save()
            rclpy.shutdown()
            return

        yaw_rate = msg.angular_velocity.z
        yaw = self.quat_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        self.yaw_rates.append(yaw_rate)
        self.yaws.append(yaw)

    def compute_and_save(self):

        yaw_rate_var = np.var(self.yaw_rates)
        yaw_var = np.var(self.yaws)

        self.get_logger().info(f"Samples: {len(self.yaw_rates)}")
        self.get_logger().info(f"Yaw rate variance: {yaw_rate_var}")
        self.get_logger().info(f"Yaw variance: {yaw_var}")

        with open("imu_stationary_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["yaw_rate", "yaw"])
            for yr, y in zip(self.yaw_rates, self.yaws):
                writer.writerow([yr, y])

        self.get_logger().info("CSV saved.")


def main(args=None):
    rclpy.init(args=args)
    node = ImuLogger()
    rclpy.spin(node)


if __name__ == '__main__':
    main()                                                                  