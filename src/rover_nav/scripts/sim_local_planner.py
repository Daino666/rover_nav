#!/usr/bin/env python3
"""Desktop test rig for local_planner.py -- no rover, no camera, no ODrives.

Closes the loop in software so the detour logic can be exercised and watched in
RViz before anything with wheels is switched on:

  * integrates /cmd_vel into /odometry/filtered with unicycle kinematics, so
    the rover actually moves in response to the arbiter;
  * publishes /aries_drive/enabled true, standing in for the ODrive bridge's
    arm state;
  * drops fake rocks onto /obstacles/array at chosen arclengths ALONG the real
    global route (it subscribes to /pure_pursuit/path and measures), which is
    what makes the test meaningful -- a rock at a fixed coordinate usually
    turns out to be nowhere near the path.

It never publishes /cmd_vel, so cmd_vel_arbiter's single-owner check stays happy.

  ros2 run rover_nav sim_local_planner.py --ros-args -p rocks:="[3.0, 8.0]"

`rocks` is a list of arclengths in metres along the route. `rock_radius` sizes
them; `drift` offsets them sideways from the centreline to test a graze rather
than a head-on block.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from rover_nav.msg import Obstacle, ObstacleArray


class SimRig(Node):
    def __init__(self):
        super().__init__("sim_local_planner")

        self.declare_parameter("rocks", [3.0])
        self.declare_parameter("rock_radius", 0.35)
        self.declare_parameter("drift", 0.0)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("detector_rate_hz", 10.0)
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("sensor_range", 6.0)
        self.declare_parameter("sensor_fov_deg", 87.0)

        self.rocks_at = [float(v) for v in self.get_parameter("rocks").value]
        self.rock_radius = float(self.get_parameter("rock_radius").value)
        self.drift = float(self.get_parameter("drift").value)
        self.sensor_range = float(self.get_parameter("sensor_range").value)
        self.sensor_half_fov = math.radians(
            float(self.get_parameter("sensor_fov_deg").value)) / 2.0

        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.yaw = float(self.get_parameter("start_yaw").value)
        self.v = 0.0
        self.w = 0.0
        self.last = time.monotonic()
        self.rocks = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.obs_pub = self.create_publisher(ObstacleArray, "/obstacles/array", 10)
        self.arm_pub = self.create_publisher(Bool, "/aries_drive/enabled", latched)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Path, "/pure_pursuit/path", self._on_path, latched)

        self.arm_pub.publish(Bool(data=True))
        self.create_timer(1.0, lambda: self.arm_pub.publish(Bool(data=True)))
        self.create_timer(1.0 / float(self.get_parameter("odom_rate_hz").value), self._odom)
        self.create_timer(1.0 / float(self.get_parameter("detector_rate_hz").value), self._obs)

        self.get_logger().info(
            f"sim rig up at ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f}deg). "
            f"Waiting for /pure_pursuit/path to place {len(self.rocks_at)} rock(s) at "
            f"{self.rocks_at} m along it, r={self.rock_radius} m, drift {self.drift} m. "
            f"Publishing /odometry/filtered and /aries_drive/enabled=true.")

    def _on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2 or self.rocks is not None:
            return
        # Only the FIRST path is used. The local planner republishes detours on
        # its own topic, but the arbiter republishes nothing -- so if that ever
        # changes, rocks that move with the plan would make the test meaningless.
        rocks, s, ri = [], 0.0, 0
        for i in range(1, len(pts)):
            s += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            while ri < len(self.rocks_at) and s >= self.rocks_at[ri]:
                hx = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
                rocks.append((pts[i][0] - math.sin(hx) * self.drift,
                              pts[i][1] + math.cos(hx) * self.drift))
                ri += 1
        self.rocks = rocks
        self.get_logger().info(
            f"route of {len(pts)} poses ({s:.1f} m) received; rocks placed at "
            + ", ".join(f"({rx:.2f}, {ry:.2f})" for rx, ry in rocks))

    def _on_cmd(self, msg):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def _odom(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.2)
        self.last = now
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_footprint"
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.odom_pub.publish(m)

    def _obs(self):
        """Only rocks within sensor_range AND the forward cone are reported -- a detector that saw the
        whole field at once would let the planner solve the course in one shot
        and never exercise the replan-as-you-go path the real camera forces."""
        msg = ObstacleArray()
        for rx, ry in (self.rocks or []):
            if math.hypot(rx - self.x, ry - self.y) > self.sensor_range:
                continue
            # Forward cone, not a disc. A depth camera does not see behind the
            # rover, and a rig that reports rocks it has driven past hides
            # exactly the memory behaviour the planner needs to get right.
            bearing = math.atan2(ry - self.y, rx - self.x) - self.yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(bearing) > self.sensor_half_fov:
                continue
            msg.obstacles.append(Obstacle(x=rx, y=ry, radius=self.rock_radius))
        self.obs_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimRig()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
