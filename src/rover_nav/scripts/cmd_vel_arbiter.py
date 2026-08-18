#!/usr/bin/env python3
"""Pure pursuit waypoint follower for the real rover -- the "waypoint stack's
cmd_vel_arbiter" that aries_teleop/nodes/cmd_vel_teleop_relay.py already
expects and yields to.

Runs the same stop-and-go, Hermite-curved waypoint sequencing as
global_path_planner.py (edit START/WAYPOINTS there), but drives the real
rover the way full_hardware.launch.py's stack actually expects: by
publishing geometry_msgs/Twist on /cmd_vel, not by talking to the six
ODrive axes directly. cmd_vel_odrive_bridge (already running as part of
full_hardware.launch.py) owns the ODrive axes, does the differential-drive
conversion, ramping, and the command-timeout fail-safe, and only drives once
armed via /aries_drive/enable -- none of that is duplicated here.

/cmd_vel ownership contract (matches cmd_vel_teleop_relay.py's docstring):
  - This node always advertises its /cmd_vel publisher while running, which
    is what makes cmd_vel_teleop_relay yield (destroy its own publisher) so
    there is only ever one real motor-command owner.
  - This node forwards /cmd_vel/teleop itself while a stick is active (LB-
    gated joystick, same as the relay would), so manual override still
    works with the relay silent.
  - This node periodically checks for any OTHER /cmd_vel publisher besides
    itself (get_publishers_info_by_topic, endpoints not traffic, same check
    the relay's docstring describes) and fails closed -- refuses to drive --
    if it finds one. This is what catches the legacy
    rover_controller_pure_pursuit.py (or two copies of this node) being run
    at the same time by mistake.

Motion is additionally gated on /aries_drive/enabled (the ODrive bridge's
own arm state) and on run_started, which defaults to false and is set by
calling /planner/start -- same safety posture as drive_auto_arm defaulting
to false elsewhere in this stack. Call:
  ros2 service call /planner/start std_srvs/srv/Trigger
to actually start driving once the drive is armed.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from global_path_planner import distance, hermite_leg, GLOBAL_RESOLUTION, WAYPOINTS  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def point_global_to_local(point, yaw, pos):
    dx = point[0] - pos[0]
    dy = point[1] - pos[1]
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return local_x, local_y


def calc_curvature(local_x, local_y):
    ld = math.hypot(local_x, local_y)
    if ld < 0.01:
        return 0.0
    return (2.0 * local_y) / (ld ** 2)


class CmdVelArbiter(Node):
    def __init__(self):
        super().__init__("cmd_vel_arbiter")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("teleop_topic", "/cmd_vel/teleop")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("drive_enabled_topic", "/aries_drive/enabled")
        self.declare_parameter("base_velocity", 0.2)
        self.declare_parameter("goal_tolerance", 0.5)
        self.declare_parameter("halt_time", 1.5)
        self.declare_parameter("lookahead_distance", 0.5)
        self.declare_parameter("max_curvature", 2.0)
        self.declare_parameter("autostart", False)
        self.declare_parameter("teleop_hold_s", 1.0)
        self.declare_parameter("teleop_deadband", 0.02)
        self.declare_parameter("odom_timeout_s", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("stuck_timeout_s", 3.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.teleop_topic = str(self.get_parameter("teleop_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.drive_enabled_topic = str(self.get_parameter("drive_enabled_topic").value)
        self.base_velocity = float(self.get_parameter("base_velocity").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.halt_time = float(self.get_parameter("halt_time").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.max_curvature = float(self.get_parameter("max_curvature").value)
        self.run_started = bool(self.get_parameter("autostart").value)
        self.teleop_hold_s = float(self.get_parameter("teleop_hold_s").value)
        self.teleop_deadband = float(self.get_parameter("teleop_deadband").value)
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        control_rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.stuck_timeout_s = float(self.get_parameter("stuck_timeout_s").value)

        # ---- pose / drive state
        self.car_pos = None
        self.car_yaw = None
        self.last_odom_time = None

        # ---- yaw zeroing, done here rather than relying solely on
        # ekf_config.yaml's imu0_relative: true. robot_localization's own
        # relative-mode zeroing (ros_filter.cpp preparePose(): step 7g)
        # correctly zeros the *sensor's* contribution against its first
        # reading, but a *separate* step (7h) then applies target_frame_trans
        # -- a live TF lookup through the filter's own currently-published
        # odom->base_link -- on top of that. Since that transform reflects
        # whatever the filter's own state happens to be at that exact
        # processing instant (tick timing/settling, not a fixed value), the
        # final fused yaw isn't reliably 0 on the first reading even with
        # imu0_relative: true. Zeroing again here, against the first reading
        # this node itself sees, is fully within code we control.
        self._yaw_offset = None
        self.drive_enabled = False
        self.drive_enabled_seen = False
        self.estopped = False
        self.gate_reason = "waiting for first odometry"

        # ---- teleop override
        self.last_teleop_cmd = None
        self.last_teleop_active_time = 0.0
        self.force_replan = False

        # ---- waypoint sequencer (see global_path_planner.py)
        self.current_wp_idx = 0
        self.leg_path = []
        self.leg_ready = False
        self.halt_until = None
        self.current_target_idx = 0

        # ---- stuck/circling detection (ported from omar's
        # Pure_pursuit_Gazebo.py after it was traced back to a real rollover
        # there: geometric curvature clamping alone doesn't guarantee
        # progress toward the lookahead target, so a stalled index needs to
        # be force-advanced rather than left to circle indefinitely)
        self._last_seen_idx = None
        self._stuck_since = None

        # ---- /cmd_vel ownership conflict tracking
        self.conflict = False

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(Twist, self.teleop_topic, self._on_teleop, 10)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, self.drive_enabled_topic, self._on_drive_enabled, latched_qos)

        self.create_service(Trigger, "/planner/start", self._on_start_srv)
        self.create_service(Trigger, "/planner/stop", self._on_stop_srv)

        self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info(
            f"cmd_vel_arbiter ready: {len(WAYPOINTS)} waypoint(s) -> {self.cmd_vel_topic} | "
            f"teleop override: {self.teleop_topic} | armed state: {self.drive_enabled_topic}"
        )
        if self.run_started:
            self.get_logger().warn("autostart=true -- will drive as soon as armed and localized")
        else:
            self.get_logger().info(
                "run is HELD. Start it with: ros2 service call /planner/start std_srvs/srv/Trigger"
            )

    # ---------- callbacks ----------

    def _on_odom(self, msg):
        self.car_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        raw_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self._yaw_offset is None:
            self._yaw_offset = raw_yaw
            self.get_logger().info(
                f"zeroing yaw: first fused reading was {math.degrees(raw_yaw):.1f} deg, "
                "treating that as forward from here on"
            )

        self.car_yaw = normalize_angle(raw_yaw - self._yaw_offset)
        self.last_odom_time = time.monotonic()

    def _on_teleop(self, msg):
        self.last_teleop_cmd = msg
        if abs(msg.linear.x) > self.teleop_deadband or abs(msg.angular.z) > self.teleop_deadband:
            self.last_teleop_active_time = time.monotonic()

    def _on_drive_enabled(self, msg):
        enabled = bool(msg.data)
        if self.drive_enabled_seen and enabled != self.drive_enabled:
            if enabled:
                self.get_logger().info("drive armed (aries_drive/enabled=true)")
            else:
                self.get_logger().warn("drive DISARMED -- autonomous motion blocked")
        self.drive_enabled = enabled
        self.drive_enabled_seen = True

    def _on_start_srv(self, request, response):
        self.estopped = False
        self.run_started = True
        self.current_wp_idx = 0
        self.leg_ready = False
        self.halt_until = None
        self.get_logger().info("run STARTED -- planning from the current pose")
        response.success = True
        response.message = "run started"
        return response

    def _on_stop_srv(self, request, response):
        self.estopped = True
        self.run_started = False
        self._publish(0.0, 0.0)
        self.get_logger().warn("run STOPPED by service call -- zero twist latched")
        response.success = True
        response.message = "stopped"
        return response

    # ---------- /cmd_vel ownership ----------

    def _foreign_publishers(self):
        own = f'/{self.get_namespace().strip("/")}/{self.get_name()}'.replace("//", "/")
        found = set()
        for info in self.get_publishers_info_by_topic(self.cmd_vel_topic):
            name = str(getattr(info, "node_name", "") or "").strip("/")
            namespace = str(getattr(info, "node_namespace", "") or "").strip("/")
            source = f"/{namespace}/{name}" if namespace else f"/{name}"
            if source != own:
                found.add(source)
        return sorted(found)

    def _check_ownership(self):
        foreign = self._foreign_publishers()
        if foreign and not self.conflict:
            self.conflict = True
            self.get_logger().error(
                f"{self.cmd_vel_topic} conflict: also published by {', '.join(foreign)} -- "
                "refusing to drive until this clears"
            )
        elif not foreign and self.conflict:
            self.conflict = False
            self.get_logger().info(f"{self.cmd_vel_topic} ownership restored, conflict cleared")

    # ---------- waypoint leg planning ----------

    def _ensure_leg_path(self, target):
        if self.leg_ready:
            return
        self.leg_path = hermite_leg(self.car_pos, self.car_yaw, target, GLOBAL_RESOLUTION)
        self.current_target_idx = 0
        self.leg_ready = True
        self._last_seen_idx = None
        self._stuck_since = None
        self.get_logger().info(
            f"leg to waypoint {self.current_wp_idx + 1}/{len(WAYPOINTS)}: {len(self.leg_path)} points"
        )

    # ---------- control loop ----------

    def _publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.pub.publish(cmd)

    def _teleop_override_active(self, now):
        return (now - self.last_teleop_active_time) < self.teleop_hold_s

    def _safety_gate(self, now):
        if self.conflict:
            return False, f"{self.cmd_vel_topic} owned by another publisher"
        if self.estopped:
            return False, "e-stopped"
        if not self.run_started:
            return False, "run not started (call /planner/start)"
        if self.car_pos is None or self.car_yaw is None or self.last_odom_time is None:
            return False, f"no odometry on {self.odom_topic}"
        if now - self.last_odom_time > self.odom_timeout_s:
            return False, f"odometry stale on {self.odom_topic}"
        if not self.drive_enabled:
            reason = "drive not armed" if self.drive_enabled_seen else "drive arm state unknown"
            return False, reason
        return True, ""

    def _hold(self, reason):
        if reason != self.gate_reason:
            self.gate_reason = reason
            self.get_logger().warn(f"holding: {reason}")
        self._publish(0.0, 0.0)

    def _control_loop(self):
        now = time.monotonic()

        self._check_ownership()
        if self.conflict:
            self._publish(0.0, 0.0)
            return

        if self._teleop_override_active(now):
            cmd = self.last_teleop_cmd if self.last_teleop_cmd is not None else Twist()
            self.pub.publish(cmd)
            self.force_replan = True
            self.gate_reason = "manual teleop override"
            return

        allowed, reason = self._safety_gate(now)
        if not allowed:
            self._hold(reason)
            return

        if self.gate_reason:
            self.get_logger().info("cleared to drive")
            self.gate_reason = ""

        if self.force_replan:
            self.leg_ready = False
            self.force_replan = False

        if self.current_wp_idx >= len(WAYPOINTS):
            self._publish(0.0, 0.0)
            return

        if self.halt_until is not None:
            if now < self.halt_until:
                self._publish(0.0, 0.0)
                return
            self.halt_until = None
            self.current_wp_idx += 1
            self.leg_ready = False
            if self.current_wp_idx >= len(WAYPOINTS):
                self.get_logger().info("all waypoints reached!")
                self._publish(0.0, 0.0)
                return

        target = WAYPOINTS[self.current_wp_idx]
        self._ensure_leg_path(target)

        if distance(self.car_pos, target) < self.goal_tolerance:
            self.get_logger().info(
                f"reached waypoint {self.current_wp_idx + 1}/{len(WAYPOINTS)} -- "
                f"pausing {self.halt_time:.1f}s"
            )
            self.halt_until = now + self.halt_time
            self._publish(0.0, 0.0)
            return

        lookahead_point = None
        for i in range(self.current_target_idx, len(self.leg_path)):
            if distance(self.car_pos, self.leg_path[i]) >= self.lookahead_distance:
                lookahead_point = self.leg_path[i]
                self.current_target_idx = i
                break
        if lookahead_point is None:
            lookahead_point = self.leg_path[-1]

        # Stuck/circling detection: if the lookahead index hasn't advanced
        # since last tick, the rover may be locked into a tight circle it
        # can never geometrically close (e.g. MAX_CURVATURE clamped tighter
        # than this leg's tangent requires). Force the index forward after
        # stuck_timeout_s rather than letting it spin indefinitely.
        if self.current_target_idx == self._last_seen_idx:
            if self._stuck_since is None:
                self._stuck_since = now
            elif now - self._stuck_since >= self.stuck_timeout_s:
                self.get_logger().warn(
                    f"lookahead index {self.current_target_idx} hasn't advanced in "
                    f"{self.stuck_timeout_s:.1f}s (stuck circling) -- forcing it forward."
                )
                self.current_target_idx = min(self.current_target_idx + 1, len(self.leg_path) - 1)
                lookahead_point = self.leg_path[self.current_target_idx]
                self._stuck_since = None
                self._last_seen_idx = self.current_target_idx
        else:
            self._stuck_since = None
            self._last_seen_idx = self.current_target_idx

        local_x, local_y = point_global_to_local(lookahead_point, self.car_yaw, self.car_pos)
        curvature = max(-self.max_curvature, min(self.max_curvature, calc_curvature(local_x, local_y)))
        angular = curvature * self.base_velocity

        self.get_logger().info(
            f"wp {self.current_wp_idx + 1}/{len(WAYPOINTS)} | pos: {self.car_pos} | "
            f"target: {lookahead_point} | curv: {curvature:.3f}",
            throttle_duration_sec=0.5,
        )
        self._publish(self.base_velocity, angular)

    def stop(self):
        self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
