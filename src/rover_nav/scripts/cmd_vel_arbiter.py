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

Two things can be driven:
  - the WAYPOINTS sequence in global_path_planner.py (default) -- stop-and-go,
    map-frame, one Hermite leg per waypoint;
  - one of the real-world test paths in scripts/test_paths/output/, selected
    with the `test_path` parameter (e.g. test_path:=circle). Those are dense,
    already-shaped courses in the rover's own start-relative frame, driven as
    one continuous run with no per-waypoint stops -- the point of them is to
    measure how well continuous curvature is tracked, which stopping every
    couple of metres would hide. See test_path_loader.py for the frame, and
    the "Real-world path-tracking tests" section of README.md for the field
    procedure. Tracking error against the reference is reported live and
    summarised at the end of the run.

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
from test_path_loader import (  # noqa: E402
    TEST_PATHS, anchor_path, coerce_name, load_markers, load_test_path, path_length,
)
import test_path_viz  # noqa: E402

import rclpy
import tf2_ros
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_point(point, tx, ty, yaw):
    """Apply a 2D rigid transform (translation + rotation) to a point."""
    x, y = point[0], point[1]
    return [
        x * math.cos(yaw) - y * math.sin(yaw) + tx,
        x * math.sin(yaw) + y * math.cos(yaw) + ty,
    ]


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
        # dynamic_typing: the value can legitimately arrive as a DOUBLE --
        # see test_path_loader.coerce_name() for why "infinity" does exactly that.
        self.declare_parameter("test_path", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("test_path_anchor", "start_pose")
        self.declare_parameter("test_path_goal_tolerance", 0.25)
        self.declare_parameter("path_goal_tolerance", 0.05)
        self.declare_parameter("path_csv", "")
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("waypoints_csv", "")

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
        self.test_path_name = coerce_name(self.get_parameter("test_path").value)
        self.test_path_anchor = str(self.get_parameter("test_path_anchor").value).strip()
        # Deliberately tighter than goal_tolerance. That one is sized for
        # stopping *at* each of several waypoints, where half a metre of slop
        # costs nothing; a test course is a fixed length being measured, so
        # ending half a metre short would leave the last stretch of every
        # course -- and on the closed loops, the loop closure itself -- never
        # actually driven.
        self.test_path_goal_tolerance = float(
            self.get_parameter("test_path_goal_tolerance").value)
        # Much tighter than test_path_goal_tolerance (0.25). A test course is
        # judged on the shape driven, so ending a handspan short of the last
        # point costs nothing. A global path's final point IS a waypoint the
        # rover is required to cross -- stopping 25 cm short of it fails the
        # one thing the run exists to do. Measured: with 0.25 the final
        # waypoint was missed by 24.7 cm while every crossed waypoint before it
        # came within 4.3 cm.
        self.path_goal_tolerance = float(self.get_parameter("path_goal_tolerance").value)
        self.path_csv = str(self.get_parameter("path_csv").value).strip()
        self.path_frame = str(self.get_parameter("path_frame").value).strip()
        self.waypoints_csv = str(self.get_parameter("waypoints_csv").value).strip()

        # ---- pose / drive state
        self.car_pos = None
        self.car_yaw = None
        self.last_odom_time = None
        self.drive_enabled = False
        self.drive_enabled_seen = False
        self.estopped = False
        self.gate_reason = "waiting for first odometry"

        # ---- teleop override
        self.last_teleop_cmd = None
        self.last_teleop_active_time = 0.0
        self.force_replan = False

        # ---- waypoint sequencer (see global_path_planner.py)
        # WAYPOINTS (imported below) is in the competition's map frame;
        # self.waypoints is the same list transformed once into this node's
        # odom frame via the static map->odom transform (see
        # _try_load_waypoints), which everything below actually drives
        # against. None until that transform becomes available -- see
        # _safety_gate, which holds rather than assuming an identity
        # transform in the meantime.
        self.waypoints = None
        self.current_wp_idx = 0
        self.leg_path = []
        self.leg_ready = False
        self.halt_until = None
        self.current_target_idx = 0

        # ---- test-path mode (see _load_test_path / _drive_test_path)
        # self.test_path_points is the CSV as generated, in the path's own
        # rover-start-relative frame; self.route is that same course placed
        # into odom (identity, or re-anchored to the pose at run start), which
        # is what actually gets driven. route is rebuilt on every
        # /planner/start so a repeat run re-anchors to where the rover now is.
        self.test_path_points = None
        self.test_markers = None
        # Global-path mode: a dense path CSV from plan_global_path.py, driven
        # straight through without stopping at the waypoints -- the waypoint
        # sequencer's stop-and-go is the wrong shape for this, since the goal
        # is to CROSS each point, not arrive at it. cross_best records the
        # closest the rover's centre actually got to each one.
        self.path_points = None
        self.cross_names = []
        self.cross_xy = []
        self.cross_best = []
        self.route = None
        self.route_idx = 0
        self.run_finished = False
        self.err_max = 0.0
        self.err_sum = 0.0
        self.err_count = 0
        if self.test_path_name:
            self._load_test_path()
        elif self.path_csv:
            self._load_path_csv()

        # ---- map -> odom lookup (see _try_load_waypoints)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # Latched so RViz (rviz/global_path_view.rviz, already subscribed to
        # both) picks the course up whenever it connects. Only used in
        # test-path mode -- the waypoint route has publish_global_path.py.
        self.route_pub = None
        self.route_markers_pub = None
        if self.test_path_name or self.path_csv:
            self.route_pub = self.create_publisher(
                Path, test_path_viz.PATH_TOPIC, test_path_viz.latched_qos())
            self.route_markers_pub = self.create_publisher(
                MarkerArray, test_path_viz.MARKERS_TOPIC, test_path_viz.latched_qos())

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

        if self.path_csv:
            what = (f"global path '{os.path.basename(self.path_csv)}' "
                    f"({len(self.path_points)} poses, {path_length(self.path_points):.1f} m, "
                    f"frame={self.path_frame}, {len(self.cross_xy)} waypoint(s) to cross)")
        elif self.test_path_name:
            what = (f"test path '{self.test_path_name}' ({len(self.test_path_points)} points, "
                    f"{path_length(self.test_path_points):.1f} m, anchor={self.test_path_anchor})")
        else:
            what = f"{len(WAYPOINTS)} waypoint(s)"
        self.get_logger().info(
            f"cmd_vel_arbiter ready: {what} -> {self.cmd_vel_topic} | "
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
        self.car_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
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
        # Dropping the route re-anchors the test path (and re-zeroes the
        # tracking-error stats) on the next tick, so a second /planner/start
        # drives the course from wherever the rover is now rather than
        # replaying the first run's placement.
        self.route = None
        self.run_finished = False
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

    def _try_load_waypoints(self):
        """One-time: look up the static map->odom transform (published by
        map_odom_broadcaster.py) and transform WAYPOINTS (map frame) into
        self.waypoints (odom frame, what the rest of this file drives
        against). map->odom never changes during a run, so this only needs
        to succeed once -- cheap to retry every tick from _control_loop
        until it does."""
        if self.waypoints is not None or self.test_path_name or self.path_csv:
            return
        try:
            tfm = self.tf_buffer.lookup_transform(
                "odom", "map", rclpy.time.Time(), timeout=RclpyDuration(seconds=0.1)
            )
        except tf2_ros.TransformException:
            return
        t = tfm.transform.translation
        yaw = quat_to_yaw(
            tfm.transform.rotation.x, tfm.transform.rotation.y,
            tfm.transform.rotation.z, tfm.transform.rotation.w,
        )
        self.waypoints = [transform_point(wp, t.x, t.y, yaw) for wp in WAYPOINTS]
        self.get_logger().info(
            f"map->odom transform found (x={t.x:.3f} y={t.y:.3f} yaw={math.degrees(yaw):.2f}deg) "
            f"-- {len(self.waypoints)} waypoint(s) ready in odom frame"
        )

    def _ensure_leg_path(self, target):
        if self.leg_ready:
            return
        self.leg_path = hermite_leg(self.car_pos, self.car_yaw, target, GLOBAL_RESOLUTION)
        self.current_target_idx = 0
        self.leg_ready = True
        self._last_seen_idx = None
        self._stuck_since = None
        self.get_logger().info(
            f"leg to waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}: {len(self.leg_path)} points"
        )

    # ---------- test-path mode ----------

    def _load_test_path(self):
        """Read the selected course off disk once, at construction. A bad
        name or a missing CSV is fatal here rather than a hold later: the
        whole node was started to drive that specific course, and a rover
        that comes up looking armed and ready but silently has no path is
        exactly the failure mode the safety gating exists to avoid."""
        if self.test_path_name not in TEST_PATHS:
            raise SystemExit(
                f"unknown test_path '{self.test_path_name}'. Known: {', '.join(TEST_PATHS)}"
            )
        if self.test_path_anchor not in ("start_pose", "odom_origin"):
            raise SystemExit(
                f"unknown test_path_anchor '{self.test_path_anchor}' -- "
                "expected 'start_pose' or 'odom_origin'"
            )
        self.test_path_points = load_test_path(self.test_path_name)
        self.test_markers = [(mid, x, y) for mid, x, y, _ in load_markers(self.test_path_name)]

    def _load_path_csv(self):
        """Read a dense path CSV (and its waypoint list) written by
        plan_global_path.py. Fatal on a bad file for the same reason
        _load_test_path is: a rover that comes up armed but silently pathless
        is the failure the gating exists to prevent."""
        import csv as _csv
        if self.path_frame not in ("map", "odom"):
            raise SystemExit(f"path_frame must be 'map' or 'odom', got '{self.path_frame}'")
        try:
            with open(self.path_csv, newline="") as f:
                self.path_points = [[float(r["x_m"]), float(r["y_m"])]
                                    for r in _csv.DictReader(f)]
        except OSError as exc:
            raise SystemExit(f"cannot read path_csv '{self.path_csv}': {exc}")
        if len(self.path_points) < 2:
            raise SystemExit(f"path_csv '{self.path_csv}' has fewer than 2 poses")

        if self.waypoints_csv:
            try:
                with open(self.waypoints_csv, newline="") as f:
                    for r in _csv.DictReader(f):
                        self.cross_names.append(r["name"])
                        self.cross_xy.append([float(r["x_m"]), float(r["y_m"])])
            except OSError as exc:
                raise SystemExit(f"cannot read waypoints_csv '{self.waypoints_csv}': {exc}")
            self.cross_best = [float("inf")] * len(self.cross_xy)

    def _ensure_route(self):
        """Place the loaded course into odom, once per run.

        anchor='start_pose' (default) rotates and translates it onto the
        rover's live pose, so the CSV's (0, 0)/+x lands under the rover
        wherever it currently sits -- which is what the field procedure
        assumes (mark the origin, park the rover on it, start). Anchoring at
        run start rather than at node start also means the follower doesn't
        care that the rover was driven to the test site after localization
        came up.

        anchor='odom_origin' drives the CSV coordinates verbatim in odom --
        correct only when the rover has not moved since localization started,
        and useful mainly for replaying an exact previous run."""
        if self.route is not None:
            return
        if self.car_pos is None or self.car_yaw is None:
            return

        if self.path_csv:
            # A global path is in the competition's map frame, not the rover's
            # start-relative frame, so unlike a test path it genuinely needs the
            # map->odom correction -- the same one the waypoint sequencer waits
            # on. Fail closed until it is available rather than assuming identity.
            if self.path_frame == "map":
                try:
                    tfm = self.tf_buffer.lookup_transform(
                        "odom", "map", rclpy.time.Time(), timeout=RclpyDuration(seconds=0.1))
                except tf2_ros.TransformException:
                    return
                t = tfm.transform.translation
                yaw = quat_to_yaw(tfm.transform.rotation.x, tfm.transform.rotation.y,
                                  tfm.transform.rotation.z, tfm.transform.rotation.w)
                self.route = [transform_point(p, t.x, t.y, yaw) for p in self.path_points]
                self.cross_xy = [transform_point(p, t.x, t.y, yaw) for p in self.cross_xy]
            else:
                self.route = [list(p) for p in self.path_points]
            self.cross_best = [float("inf")] * len(self.cross_xy)
            self.route_idx = 0
            self._last_seen_idx = None
            self._stuck_since = None
            self.err_max = self.err_sum = 0.0
            self.err_count = 0
            stamp = self.get_clock().now().to_msg()
            self.route_pub.publish(test_path_viz.path_msg(self.route, stamp, "odom"))
            if self.cross_xy:
                self.route_markers_pub.publish(test_path_viz.marker_msgs(
                    [(i, x, y) for i, (x, y) in enumerate(self.cross_xy)], stamp, "odom"))
            self.get_logger().info(
                f"global path '{os.path.basename(self.path_csv)}' ready in odom: "
                f"{len(self.route)} poses, {path_length(self.route):.1f} m, "
                f"{len(self.cross_xy)} waypoint(s) to cross. Driving straight through -- no stops.")
            return

        if self.test_path_anchor == "start_pose":
            self.route = anchor_path(self.test_path_points, self.car_pos, self.car_yaw)
            markers = anchor_path(
                [[x, y] for _, x, y in self.test_markers], self.car_pos, self.car_yaw)
            marker_ids = [mid for mid, _, _ in self.test_markers]
        else:
            self.route = [list(p) for p in self.test_path_points]
            markers = [[x, y] for _, x, y in self.test_markers]
            marker_ids = [mid for mid, _, _ in self.test_markers]

        self.route_idx = 0
        self._last_seen_idx = None
        self._stuck_since = None
        self.err_max = 0.0
        self.err_sum = 0.0
        self.err_count = 0

        stamp = self.get_clock().now().to_msg()
        self.route_pub.publish(test_path_viz.path_msg(self.route, stamp, "odom"))
        self.route_markers_pub.publish(
            test_path_viz.marker_msgs(list(zip(marker_ids, [m[0] for m in markers],
                                               [m[1] for m in markers])), stamp, "odom"))

        self.get_logger().info(
            f"test path '{self.test_path_name}' anchored at "
            f"x={self.car_pos[0]:.2f} y={self.car_pos[1]:.2f} "
            f"yaw={math.degrees(self.car_yaw):.1f}deg ({self.test_path_anchor}) -- "
            f"{len(self.route)} points, {path_length(self.route):.1f} m, "
            f"{len(markers)} ground marker(s). Published on "
            f"{test_path_viz.PATH_TOPIC} + {test_path_viz.MARKERS_TOPIC}."
        )

    def _record_tracking_error(self):
        """Distance from the rover to the nearest reference point in a window
        around the index being chased -- the same cross-track miss the ground
        markers are eyeballed for, but measured. Windowed rather than searched
        globally so a self-crossing course (infinity, circle_transition) can't
        score itself against the *other* pass through the same spot."""
        lo = max(0, self.route_idx - 20)
        hi = min(len(self.route), self.route_idx + 40)
        err = min(distance(self.car_pos, self.route[i]) for i in range(lo, hi))
        # Closest approach per waypoint, sampled continuously. The rover is
        # meant to CROSS these, not stop at them, so there is no arrival event
        # to measure at -- the smallest distance seen over the whole run is the
        # measurement.
        for k, wp in enumerate(self.cross_xy):
            d = distance(self.car_pos, wp)
            if d < self.cross_best[k]:
                self.cross_best[k] = d
        self.err_max = max(self.err_max, err)
        self.err_sum += err
        self.err_count += 1
        return err

    def _finish_run(self):
        mean = self.err_sum / self.err_count if self.err_count else 0.0
        self.run_finished = True

        if self.path_csv:
            lines = [f"    {n:<8} {d * 100:6.1f} cm" for n, d in
                     zip(self.cross_names, self.cross_best)]
            worst = max(self.cross_best) if self.cross_best else float("nan")
            self.get_logger().info(
                f"global path COMPLETE -- tracking error vs plan: mean {mean:.3f} m, "
                f"max {self.err_max:.3f} m over {self.err_count} samples.\n"
                f"  closest the rover's centre came to each waypoint:\n"
                + "\n".join(lines)
                + f"\n    worst: {worst * 100:.1f} cm. Re-run with /planner/start.")
            self._publish(0.0, 0.0)
            return

        self.get_logger().info(
            f"test path '{self.test_path_name}' COMPLETE -- tracking error vs reference: "
            f"mean {mean:.3f} m, max {self.err_max:.3f} m over {self.err_count} samples. "
            f"Compare against the {len(self.test_markers)} ground markers in "
            f"{self.test_path_name}_markers.csv. Re-run with /planner/start."
        )
        self._publish(0.0, 0.0)

    def _drive_test_path(self, now):
        """Continuous pure pursuit along the whole anchored course -- no
        per-waypoint stop-and-go (that is what _control_loop's waypoint branch
        does), because these paths exist to measure uninterrupted curvature
        tracking."""
        self._ensure_route()
        if self.route is None:
            self._hold("waiting for odometry / map->odom to place the path")
            return
        if self.run_finished:
            self._publish(0.0, 0.0)
            return

        err = self._record_tracking_error()

        lookahead_point = None
        for i in range(self.route_idx, len(self.route)):
            if distance(self.car_pos, self.route[i]) >= self.lookahead_distance:
                lookahead_point = self.route[i]
                self.route_idx = i
                break

        # No point far enough ahead left in the course: the rover is inside
        # one lookahead of the end. Closed courses (circle, infinity) return
        # to their own start, so "near the final point" alone would fire on
        # the first tick -- requiring the index to have run out first is what
        # makes completion mean "drove the whole thing", not "stands near the
        # end".
        if lookahead_point is None:
            self.route_idx = len(self.route) - 1
            done_radius = (self.path_goal_tolerance if self.path_csv
                           else self.test_path_goal_tolerance)
            if distance(self.car_pos, self.route[-1]) < done_radius:
                self._finish_run()
                return
            lookahead_point = self.route[-1]

        if self.route_idx == self._last_seen_idx:
            if self._stuck_since is None:
                self._stuck_since = now
            elif now - self._stuck_since >= self.stuck_timeout_s:
                self.get_logger().warn(
                    f"lookahead index {self.route_idx} hasn't advanced in "
                    f"{self.stuck_timeout_s:.1f}s (stuck circling) -- forcing it forward."
                )
                self.route_idx = min(self.route_idx + 1, len(self.route) - 1)
                lookahead_point = self.route[self.route_idx]
                self._stuck_since = None
                self._last_seen_idx = self.route_idx
        else:
            self._stuck_since = None
            self._last_seen_idx = self.route_idx

        local_x, local_y = point_global_to_local(lookahead_point, self.car_yaw, self.car_pos)
        curvature = max(-self.max_curvature, min(self.max_curvature, calc_curvature(local_x, local_y)))
        angular = curvature * self.base_velocity

        self.get_logger().info(
            f"{self.test_path_name} {self.route_idx + 1}/{len(self.route)} | "
            f"pos: [{self.car_pos[0]:.2f}, {self.car_pos[1]:.2f}] | "
            f"err: {err:.3f} m (max {self.err_max:.3f}) | curv: {curvature:.3f}",
            throttle_duration_sec=0.5,
        )
        self._publish(self.base_velocity, angular)

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
        if self.path_csv:
            # Anchoring needs the map->odom transform; _ensure_route holds
            # until it arrives, and _drive_test_path reports that as a hold.
            pass
        elif self.test_path_name:
            # Test paths are natively in the rover's own start-relative frame
            # (== odom here, see test_path_loader.py), so map->odom never
            # enters into it -- only the route being anchored, which needs a
            # pose and so is checked just below.
            pass
        elif self.waypoints is None:
            # Fail closed: hold rather than assume an identity map->odom
            # transform. A control loop that confidently drives on a wrong
            # assumed-zero correction is worse than one that just waits.
            return False, "map->odom transform not available yet"
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

        self._try_load_waypoints()
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

        if self.test_path_name or self.path_csv:
            # A teleop nudge mid-course must NOT re-anchor: the course is a
            # fixed piece of ground being measured against, so the rover
            # resumes chasing it from where the nudge left it rather than
            # restarting the whole thing under its new pose.
            self.force_replan = False
            self._drive_test_path(now)
            return

        if self.force_replan:
            self.leg_ready = False
            self.force_replan = False

        if self.current_wp_idx >= len(self.waypoints):
            self._publish(0.0, 0.0)
            return

        if self.halt_until is not None:
            if now < self.halt_until:
                self._publish(0.0, 0.0)
                return
            self.halt_until = None
            self.current_wp_idx += 1
            self.leg_ready = False
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info("all waypoints reached!")
                self._publish(0.0, 0.0)
                return

        target = self.waypoints[self.current_wp_idx]
        self._ensure_leg_path(target)

        if distance(self.car_pos, target) < self.goal_tolerance:
            self.get_logger().info(
                f"reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} -- "
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
            f"wp {self.current_wp_idx + 1}/{len(self.waypoints)} | pos: {self.car_pos} | "
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
