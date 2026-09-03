"""Snaps the EKF's pose onto an ArUco landmark fix, instead of fusing it.

Wheel odometry accumulates error without bound; a landmark sighting is an
absolute position that does not. This node takes the second and overwrites
the first with it -- one /set_pose call per accepted sighting, after which
the rover carries on dead-reckoning from a corrected position rather than a
drifted one.

WHY NOT FUSE IT: fusing needs a covariance, and no one has measured what this
pipeline's actual error is at range (the detector's own docstring refuses to
invent one, correctly). A snap needs no covariance -- it is an assertion, not
a measurement to be weighed. The cost is that a bad detection moves the
estimate all at once instead of being averaged away, which is what the gates
below are for.

FRAMES -- the easy thing to get wrong here. RoverPositionFix is published in
the MAP frame. robot_localization's /set_pose expects a pose in the EKF's
`world_frame`, which is `odom` in rover_nav/config/ekf_config.yaml. Those two
differ by MAP_TO_ODOM_YAW_DEG, re-measured every run. So the fix is
transformed map -> odom through TF before being sent; sending map coordinates
straight to /set_pose would displace the rover by exactly that rotation.

HEADING IS PRESERVED, NOT SET. A single landmark sighting gives position
only -- the marker's own orientation is not used, and one pole is
orientation-ambiguous anyway. The current fused yaw is read back and rewritten
unchanged, because /set_pose sets the WHOLE pose: a default-constructed
request silently snaps the rover's heading to zero as well as moving it. Use
rover_nav's set_heading.py if you want to correct yaw.

WHY THIS HOLDS: ekf_config.yaml sets imu0_differential: true, so the IMU is
fused as a yaw RATE and never as an absolute pose -- nothing pulls a corrected
state back. Verified on hardware for yaw (snapped 90.29 -> 89.99, still 90.04
fifty seconds later). Position has no absolute measurement source at all, so
it holds for the same reason. If imu0_differential is ever set false, re-check
this before relying on it.

  ros2 run rover_detection aruco_pose_reset
"""

import math

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from rover_perception_msgs.msg import RoverPositionFix

# Matches START_SEED_LANDMARK_ID in aruco_detect_roverpos.py -- the one-time
# startup seed is a configured guess, not a sighting.
SEED_LANDMARK_ID = -1


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ArucoPoseReset(Node):

    def __init__(self):
        super().__init__('aruco_pose_reset')

        self.declare_parameter('fix_topic', '/aruco_localization/rover_position_fix')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('set_pose_service', '/set_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        # Rate limit. The detector runs at camera rate (~30 Hz); snapping that
        # often would keep restarting the filter's own settling and fight the
        # path follower, which sees every snap as a step change in position.
        self.declare_parameter('min_interval_s', 2.0)
        # Sanity gate. Field-validated against the real global-planner run:
        # genuine landmark corrections land in 0-0.7 m (see lookahead_max /
        # lookahead_curvature_sample_m in cmd_vel_arbiter.py, tuned around
        # that same 0.7 m ceiling). 0.8 m gives that a little headroom for
        # measurement noise. A correction bigger than this is far more likely
        # to be a misdetection, a wrong landmark table entry, or a bad
        # map->odom than a real drift of that size -- refuse it loudly rather
        # than teleporting the rover mid-run. Raise it deliberately if genuine
        # drift ever exceeds this (and re-check lookahead_max, which was
        # tuned assuming corrections stay under 0.7 m).
        self.declare_parameter('max_correction_m', 0.8)
        # Ignore corrections smaller than this: below the pipeline's own noise
        # there is nothing to correct, and snapping anyway just adds jitter.
        self.declare_parameter('min_correction_m', 0.05)
        self.declare_parameter('accept_startup_seed', False)
        self.declare_parameter('enabled', True)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.min_interval = float(self.get_parameter('min_interval_s').value)
        self.max_correction = float(self.get_parameter('max_correction_m').value)
        self.min_correction = float(self.get_parameter('min_correction_m').value)
        self.accept_seed = bool(self.get_parameter('accept_startup_seed').value)
        self.enabled = bool(self.get_parameter('enabled').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_odom = None
        self.last_snap_time = None
        self.snaps = 0

        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self._on_odom, 10)
        self.create_subscription(RoverPositionFix, str(self.get_parameter('fix_topic').value),
                                 self._on_fix, 10)
        self.client = self.create_client(SetPose, str(self.get_parameter('set_pose_service').value))

        self.get_logger().info(
            f'aruco_pose_reset up ({"ENABLED" if self.enabled else "DISABLED, observing only"}): '
            f'snapping the EKF onto landmark fixes at most every {self.min_interval:.1f}s, '
            f'accepting corrections {self.min_correction:.2f}-{self.max_correction:.1f}m. '
            f'Heading is preserved, not set.'
        )

    def _on_odom(self, msg):
        self.latest_odom = msg

    def _on_fix(self, msg):
        if msg.landmark_id == SEED_LANDMARK_ID and not self.accept_seed:
            return
        if self.latest_odom is None:
            self.get_logger().warn('fix received but no odometry yet -- ignoring',
                                   throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        if self.last_snap_time is not None:
            if (now - self.last_snap_time) < Duration(seconds=self.min_interval):
                return

        # map -> odom, so the fix lands in the frame /set_pose actually uses.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.map_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'no {self.odom_frame} <- {self.map_frame} transform ({exc}) -- cannot place '
                'the fix in the EKF\'s frame, ignoring it', throttle_duration_sec=5.0)
            return

        t = tf.transform.translation
        yaw_mo = quat_to_yaw(tf.transform.rotation)
        c, s = math.cos(yaw_mo), math.sin(yaw_mo)
        fx, fy = msg.position.x, msg.position.y
        target_x = fx * c - fy * s + t.x
        target_y = fx * s + fy * c + t.y

        cur = self.latest_odom.pose.pose
        correction = math.hypot(target_x - cur.position.x, target_y - cur.position.y)

        if correction > self.max_correction:
            self.get_logger().warn(
                f'landmark {msg.landmark_id} implies a {correction:.2f}m correction, over '
                f'max_correction_m={self.max_correction:.1f} -- REFUSING. Check the landmark '
                f'table, the marker ID, and map->odom before trusting this.',
                throttle_duration_sec=5.0)
            return
        if correction < self.min_correction:
            self.get_logger().info(
                f'landmark {msg.landmark_id}: {correction*100:.1f}cm off, below '
                f'min_correction_m -- nothing worth snapping',
                throttle_duration_sec=10.0)
            return

        if not self.enabled:
            self.get_logger().info(
                f'[observing only] landmark {msg.landmark_id} would correct '
                f'{correction:.3f}m: ({cur.position.x:.3f},{cur.position.y:.3f}) -> '
                f'({target_x:.3f},{target_y:.3f})')
            self.last_snap_time = now
            return

        if not self.client.service_is_ready():
            self.get_logger().warn('/set_pose not available -- is ekf_filter_node running?',
                                   throttle_duration_sec=5.0)
            return

        req = SetPose.Request()
        req.pose.header.frame_id = self.odom_frame
        req.pose.header.stamp = now.to_msg()
        req.pose.pose.pose.position.x = target_x
        req.pose.pose.pose.position.y = target_y
        req.pose.pose.pose.position.z = 0.0
        req.pose.pose.pose.orientation = cur.orientation   # heading preserved, see docstring
        cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            cov[i] = 1e-9
        req.pose.pose.covariance = cov
        self.client.call_async(req)

        self.last_snap_time = now
        self.snaps += 1
        self.get_logger().info(
            f'SNAP #{self.snaps} on landmark {msg.landmark_id} (seen at {msg.distance:.2f}m): '
            f'({cur.position.x:.3f},{cur.position.y:.3f}) -> ({target_x:.3f},{target_y:.3f}) '
            f'in {self.odom_frame}, correcting {correction:.3f}m of accumulated drift. '
            f'yaw {math.degrees(quat_to_yaw(cur.orientation)):.2f}deg kept.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseReset()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
