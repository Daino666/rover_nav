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

Forward obstacles gate motion too: /obstacle_detected from
Rover_path_controller/pcl_obstacle_detector.py holds the rover exactly the way
an unarmed drive or stale odometry does. Like those, it fails CLOSED -- a
detector that is not running, or has gone silent for obstacle_timeout_s, holds
rather than being read as "clear". Runs with no camera must therefore say so:

  ros2 run rover_nav cmd_vel_arbiter.py --ros-args -p obstacle_stop_enabled:=false

The gate sits inside _safety_gate, so it does NOT block the teleop override --
_control_loop forwards a live stick before the gate is consulted, and a human
on the sticks is assumed to be able to see the obstacle themselves.
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


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


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


def final_heading_vector(path):
    """Unit vector of the path's travel direction at its last point.

    Walks back from the end until it finds a point far enough away to give a
    meaningful direction -- consecutive poses can be coincident (or a few
    millimetres apart) where a planner packs points tightly near a goal, and
    normalising that gives a direction made of rounding noise."""
    end = path[-1]
    for i in range(len(path) - 2, -1, -1):
        dx, dy = end[0] - path[i][0], end[1] - path[i][1]
        d = math.hypot(dx, dy)
        if d > 0.05:
            return dx / d, dy / d
    return None


def extend_past_goal(route, extra_m, step=0.1):
    """Continue the route straight on past its final point.

    Pure pursuit chasing a lone final point is degenerate: curvature is
    2*lateral/distance^2, so as the rover closes on it the term explodes,
    clamps at max_curvature, and the rover flies a circle around the goal it
    can never tighten. That is the observed "loops around the last point and
    never stops".

    Tracking a LINE has no such singularity. Extending the route about one
    lookahead beyond the goal keeps an ordinary lookahead point ahead of the
    rover the whole way in, so it drives THROUGH the goal instead of trying to
    converge onto it, and the run ends when it crosses the goal rather than
    when it manages to get inside some radius.

    Measured over 300 realistic arrivals (<=15 cm lateral, <=15 deg heading):
    stopping distance median 0.087 m / worst 0.189 m, versus 0.140 m / 0.320 m
    for chasing the endpoint -- and no orbits in either case, where the tight
    0.05 m radius alone orbited outright.

    The extension is drive-through only. It is never a goal, never a waypoint,
    and the rover is stopped before reaching its end."""
    f = final_heading_vector(route)
    if f is None or extra_m <= 0.0:
        return list(route), list(route[-1])
    goal = list(route[-1])
    out = list(route)
    n = max(1, int(round(extra_m / step)))
    for i in range(1, n + 1):
        out.append([goal[0] + f[0] * step * i, goal[1] + f[1] * step * i])
    return out, goal


def calc_curvature(local_x, local_y):
    ld = math.hypot(local_x, local_y)
    if ld < 0.01:
        return 0.0
    return (2.0 * local_y) / (ld ** 2)


def path_curvature_ahead(route, idx, sample_gap_m):
    """Menger curvature (1/m) of `route` just ahead of `idx` -- same formula
    plan_global_path.py's curvature_profile() uses, but sampled live off
    three points spaced ~sample_gap_m apart in arc length, starting AT idx
    and walking forward. Deliberately anticipatory (samples the stretch the
    rover is about to enter, not the one it's already on): the point of this
    is to shorten the lookahead BEFORE a corner, not react once already
    cutting it. Arc-length stepping rather than a fixed index stride so this
    works the same regardless of a given path CSV's point spacing.

    Returns 0.0 (== "straight, no cap needed") if the route doesn't have
    enough points left ahead of idx to sample three."""
    pts = [route[idx]]
    i = idx
    for _ in range(2):
        acc = 0.0
        j = i
        while j + 1 < len(route) and acc < sample_gap_m:
            acc += distance(route[j], route[j + 1])
            j += 1
        if j == i:
            return 0.0
        pts.append(route[j])
        i = j

    a, b, c = pts
    ab, bc, ca = distance(a, b), distance(b, c), distance(c, a)
    if min(ab, bc, ca) < 1e-9:
        return 0.0
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    return cross / (ab * bc * ca) * 2.0


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
        # Cross-track error above which the stuck recovery must NOT force the
        # lookahead index forward -- see the guard in _drive_test_path. Sized
        # at ~2x lookahead_max (0.7): inside that the rover is essentially on
        # the path and merely grinding a corner, beyond it the rover is off
        # the path and advancing the target only walks it further away.
        self.declare_parameter("stuck_force_max_error_m", 1.5)
        self.declare_parameter("obstacle_stop_enabled", True)
        self.declare_parameter("obstacle_topic", "/obstacle_detected")
        self.declare_parameter("obstacle_timeout_s", 1.0)
        self.declare_parameter("obstacle_clear_s", 0.5)
        # Local planner hand-off. When enabled, cmd_vel_arbiter stops deciding for
        # itself what an obstacle means: local_planner.py owns the stop/detour
        # decision and this node executes it. The /obstacle_detected Bool gate is
        # replaced by /local_plan/hold, which fails closed identically.
        self.declare_parameter("local_plan_enabled", False)
        self.declare_parameter("local_plan_topic", "/local_plan")
        self.declare_parameter("local_hold_topic", "/local_plan/hold")
        self.declare_parameter("local_cmd_topic", "/local_plan/cmd")
        self.declare_parameter("local_plan_timeout_s", 1.0)
        self.declare_parameter("local_cmd_timeout_s", 0.3)
        # dynamic_typing: the value can legitimately arrive as a DOUBLE --
        # see test_path_loader.coerce_name() for why "infinity" does exactly that.
        self.declare_parameter("test_path", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("test_path_anchor", "start_pose")
        self.declare_parameter("test_path_goal_tolerance", 0.25)
        # Back to 0.05 m now that the route runs out past the goal (see
        # extend_past_goal). The loose 0.40 m this briefly used was a
        # workaround for the endpoint-chasing orbit, and with the run-out it
        # actively hurts: measured over 400 realistic arrivals it fired first
        # in 303 of them, stopping the rover up to 0.40 m short instead of
        # letting it drive through the goal. At 0.05 m the crossing test does
        # the work and the stop distance drops from 0.139 m median / 0.320 m
        # worst to 0.088 m / 0.189 m. Tightening below 0.05 changes nothing --
        # the crossing test already handles everything by then.
        self.declare_parameter("path_goal_tolerance", 0.05)
        # How long to let the radius test try before the finish-line fallback
        # is allowed to end the run. Sized from measurement, not taste: a
        # recoverable bad arrival converges in 3-13 s, while a genuine orbit
        # crosses the finish line within 0-1.5 s and never converges. Firing on
        # first crossing therefore cuts short approaches that would have
        # reached 0.05 m, ending them ~0.4 m out instead. Waiting past the
        # slowest recovery keeps the radius test as the real answer and leaves
        # the line purely as the thing that stops an orbit -- at 0.2 m/s this
        # bounds an orbit at well under one revolution. Shortened from 20 s
        # once path_goal_tolerance was loosened to 0.40: a looser radius
        # converges far sooner, so less patience is needed before concluding
        # the rover is circling rather than still arriving.
        self.declare_parameter("endgame_grace_s", 10.0)
        # How far the route is continued past its final point, as a multiple of
        # lookahead_distance. Needs to exceed one lookahead or the rover runs
        # out of extension before it reaches the goal and the endpoint-chasing
        # degeneracy returns. 1.5x leaves margin.
        self.declare_parameter("goal_runout_lookaheads", 1.5)
        # Dynamic lookahead. Tracking wants it SHORT (hugs the path, corners
        # accurately); recovering from a large lateral error wants it LONG
        # (keeps the demanded curvature under max_curvature instead of pinning
        # the clamp and circling, which is what a 0.7 m ArUco snap did at
        # 0.5 m fixed). One constant cannot serve both, so scale it with the
        # error actually being corrected:
        #
        #   L = clamp(L_min + gain * cross_track_error, L_min, L_max)
        #
        # Self-damping: as the error shrinks the lookahead shrinks with it, so
        # the rover eases back onto the path and then tracks tightly, rather
        # than weaving the way a permanently-long lookahead does.
        #
        # L_max is deliberately modest. A long lookahead cuts corners -- the
        # rover deviates to the INSIDE of curves -- and this follower has no
        # costmap of its own (/obstacle_detected is a stop/go flag, not a map),
        # so nothing here can see that it is cutting into an inflated obstacle.
        # Bounding L_max bounds how far off the planned, collision-checked path
        # the rover can stray while correcting.
        #
        # The error term alone has a feedback problem on curved sections: a
        # long lookahead cuts a corner, cutting the corner IS cross-track
        # error, and the error term responds to that error by growing the
        # lookahead further -- cutting the corner harder. So the error term is
        # capped by a second, independent term driven by the path's own
        # upcoming curvature (path_curvature_ahead(), same Menger-curvature
        # formula plan_global_path.py already uses), which shortens the
        # lookahead approaching a turn regardless of how much error has built
        # up. Final lookahead = min(error term, curvature cap), both already
        # clamped to [lookahead_min, lookahead_max] -- so on a straight the
        # curvature cap sits at lookahead_max and stays out of the way, and
        # entering a turn it pulls the ceiling down toward lookahead_min.
        self.declare_parameter("lookahead_dynamic", True)
        # This rover drives these paths at base_velocity ~0.2 m/s -- slow
        # enough that a long lookahead buys nothing in loop-stability margin,
        # it only costs tracking accuracy (a longer lookahead is a wider
        # corner-cut and a slower-to-correct cross-track error). 0.4 m stays
        # safely above the route CSVs' own point spacing (~0.23 m on
        # loop4_rot270.csv) so lookahead target selection doesn't get jumpy
        # at the floor.
        self.declare_parameter("lookahead_min", 0.4)
        self.declare_parameter("lookahead_max", 0.7)
        self.declare_parameter("lookahead_error_gain", 1.0)
        # curvature_cap = clamp(lookahead_min, lookahead_max,
        #                        lookahead_curvature_gain / kappa_ahead).
        # Calibrated against MIN_TURNING_RADIUS in plan_global_path.py -- what
        # the Hybrid-A* planned paths this follower actually drives top out
        # at.
        #
        # 2026-09-03, twice in one day:
        #   1.5 m radius (0.667 1/m): gain 0.267 (marsyard/loop4_rot270.csv,
        #     max 0.672 1/m -- the cap almost never engaged on that route).
        #   0.5 m radius (2.0 1/m): gain raised to 0.8 for the W1/W9/W3/W5
        #     route (p95 2.00 1/m). This one made things WORSE, not better --
        #     see MIN_TURNING_RADIUS's config/nav2_planning_params.yaml
        #     comment for the full story: 0.5 m gave the PLANNER zero
        #     tracking margin under this node's own max_curvature=2.0 clamp,
        #     and the rover was observed in sim circling at points where the
        #     path's own curvature was already pinned near 2.0, independent
        #     of this gain. Fixing the margin (MIN_TURNING_RADIUS -> 0.7 m)
        #     is what actually resolved it.
        #   0.7 m radius (1.43 1/m): gain here retuned to match -- p95 1.429
        #     1/m on the re-planned route (pivot joints excluded). lookahead_min
        #     * 1.429 = 0.4 * 1.429 = 0.572 puts the cap at exactly
        #     lookahead_min at that ceiling, same formula as always.
        # Retune again (lookahead_min * (1.0 / MIN_TURNING_RADIUS)) if
        # lookahead_min changes, or if max_curvature/MIN_TURNING_RADIUS
        # change again -- and if the cap ever seems to be fighting recovery
        # again, check MIN_TURNING_RADIUS's margin under max_curvature
        # FIRST, not just this gain.
        self.declare_parameter("lookahead_curvature_gain", 0.572)
        # How far ahead (arc length along the route, metres) to sample
        # curvature from -- anticipatory, not reactive. Set to lookahead_max's
        # default: the region a full-length lookahead arc would actually try
        # to cut through, so the cap has already dropped by the time the
        # rover would otherwise start cutting the corner.
        self.declare_parameter("lookahead_curvature_sample_m", 0.7)
        self.declare_parameter("path_csv", "")
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("waypoints_csv", "")
        # Optional <name>_pivots.csv from plan_global_path.py: joints where no
        # arrival heading served both getting there and getting to the next
        # point, so the route needs the rover to physically stop and rotate
        # in place rather than curve through. Pure pursuit alone cannot do
        # this -- it always drives forward at base_velocity while turning
        # (see calc_curvature/angular below) -- so _pending_pivot() overrides
        # it with a pure rotation (linear=0) until the heading is reached.
        # Empty (default): no pivots, path_csv drives exactly as before.
        self.declare_parameter("pivots_csv", "")
        # Lowered from 0.4 2026-09-04: real hardware wheels are slightly
        # shaky/uneven, and a bang-bang rotation at 0.4 rad/s (~23 deg/s)
        # visibly shook the rover during in-place pivots. This is closed-loop
        # (see _pending_pivot below, checks car_yaw every tick against
        # pivot_tolerance_deg) so a slower rate costs time, not accuracy --
        # each control tick just takes a smaller angular step.
        self.declare_parameter("pivot_max_angular_rps", 0.2)
        self.declare_parameter("pivot_tolerance_deg", 3.0)
        # Brake to a real stop before starting to rotate. wheel_accel_rps2
        # (cmd_vel_odrive_bridge.py) ramps each wheel toward its target
        # independently -- it guarantees the FINAL differential matches what
        # was commanded, not that it matches DURING the ramp. Going straight
        # from "driving forward" to "rotate in place" asks both wheels to
        # swap to opposite-signed targets at once; while they ramp there
        # independently, the actual differential (and so the actual turn
        # rate) doesn't match the commanded pivot, and the rover drifts/arcs
        # instead of cleanly stopping first. Waiting here for measured speed
        # to actually settle removes that ambiguous window instead of just
        # shrinking it.
        self.declare_parameter("pivot_brake_speed_mps", 0.03)
        # How close the rover must get to a pivot point before the "have I
        # passed the closest approach" test starts running. Same role as
        # waypoint_stop_arm_radius_m: far from the point, "closest so far" is
        # measuring noise rather than progress.
        self.declare_parameter("pivot_arrive_radius_m", 2.0)
        # Lookahead used while within waypoint_lookahead_radius_m of the next
        # waypoint -- shorter than the cruising lookahead so pure pursuit cuts
        # less of the corner at the points that are actually scored. See the
        # L^2/2R note in _drive_test_path. 0.0 disables the whole behaviour.
        self.declare_parameter("waypoint_lookahead_m", 0.35)
        self.declare_parameter("waypoint_lookahead_radius_m", 2.0)
        # How many route poses BEFORE a waypoint's own pose the stop may arm.
        # Poses run ~0.175 m apart, so 20 is roughly the last 3.5 m of the
        # approach -- comfortably more than waypoint_stop_arm_radius_m, while
        # still excluding an earlier fly-past on a looping leg.
        self.declare_parameter("waypoint_stop_arm_poses", 20)
        # Distance from a waypoint at which pure pursuit stops chasing the path
        # and aims straight at the waypoint itself, so it lands on the point
        # instead of cutting the corner inside it. 0.0 disables.
        self.declare_parameter("waypoint_final_approach_m", 1.0)
        # Fallback only, in case the speed estimate never quite settles
        # under the threshold (sensor noise/bias) -- proceed anyway rather
        # than waiting forever.
        self.declare_parameter("pivot_brake_timeout_s", 3.0)
        # Stop-and-wait mode: path_csv normally drives straight through with
        # no stops (see _drive_test_path's docstring) -- this overrides that
        # for competition runs that need a human confirmation between
        # waypoints. Needs waypoints_csv (the named stop points); the START
        # row and the final/loop-closing point are never stop-and-wait
        # points -- the rover is already there at t=0 for the first, and the
        # existing goal-arrival logic (_finish_run) already handles the last.
        self.declare_parameter("stop_at_waypoints", False)
        # How far PAST a waypoint's closest approach (tracked continuously in
        # cross_best, same measurement _finish_run reports) before treating
        # it as "passed" and stopping -- not a radius to stop WITHIN, so
        # "as close as possible" holds regardless of how much tracking error
        # the run actually has, rather than picking a fixed tolerance that
        # might never be reached.
        self.declare_parameter("waypoint_stop_margin_m", 0.05)
        # Don't evaluate "have we passed the closest approach" at all until
        # within this radius -- see _pending_waypoint_stop's BUG FOUND IN SIM
        # docstring note. Comfortably bigger than any real closest-approach
        # distance (the plan itself lands within cm of every waypoint) and
        # comfortably smaller than "still working through an earlier part of
        # a multi-waypoint route."
        self.declare_parameter("waypoint_stop_arm_radius_m", 2.0)

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
        self.stuck_force_max_error_m = float(
            self.get_parameter("stuck_force_max_error_m").value)
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
        self.endgame_grace_s = float(self.get_parameter("endgame_grace_s").value)
        self.endgame_since = None
        self.goal_runout = float(self.get_parameter('goal_runout_lookaheads').value)
        self.lookahead_dynamic = bool(self.get_parameter("lookahead_dynamic").value)
        self.lookahead_min = float(self.get_parameter("lookahead_min").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.lookahead_error_gain = float(self.get_parameter("lookahead_error_gain").value)
        self.lookahead_curvature_gain = float(
            self.get_parameter("lookahead_curvature_gain").value)
        self.lookahead_curvature_sample_m = float(
            self.get_parameter("lookahead_curvature_sample_m").value)
        self.route_goal = None   # true final point, before the run-out
        self.goal_armed = False  # see the goal test in _drive_test_path
        self.path_csv = str(self.get_parameter("path_csv").value).strip()
        self.path_frame = str(self.get_parameter("path_frame").value).strip()
        self.waypoints_csv = str(self.get_parameter("waypoints_csv").value).strip()
        self.pivots_csv = str(self.get_parameter("pivots_csv").value).strip()
        self.pivot_max_angular_rps = float(self.get_parameter("pivot_max_angular_rps").value)
        self.pivot_tolerance_rad = math.radians(
            float(self.get_parameter("pivot_tolerance_deg").value))
        self.pivot_brake_speed_mps = float(
            self.get_parameter("pivot_brake_speed_mps").value)
        self.pivot_arrive_radius_m = float(
            self.get_parameter("pivot_arrive_radius_m").value)
        self.waypoint_lookahead_m = float(
            self.get_parameter("waypoint_lookahead_m").value)
        self.waypoint_lookahead_radius_m = float(
            self.get_parameter("waypoint_lookahead_radius_m").value)
        self.waypoint_stop_arm_poses = int(
            self.get_parameter("waypoint_stop_arm_poses").value)
        self.waypoint_final_approach_m = float(
            self.get_parameter("waypoint_final_approach_m").value)
        self.pivot_brake_timeout_s = float(
            self.get_parameter("pivot_brake_timeout_s").value)
        self.stop_at_waypoints = bool(self.get_parameter("stop_at_waypoints").value)
        self.waypoint_stop_margin_m = float(
            self.get_parameter("waypoint_stop_margin_m").value)
        self.waypoint_stop_arm_radius_m = float(
            self.get_parameter("waypoint_stop_arm_radius_m").value)

        # ---- pose / drive state
        self.car_pos = None
        self.car_yaw = None
        self.car_speed = None
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
        # cross_xy_raw is the pristine map-frame waypoint list straight from
        # waypoints_csv; cross_xy is the per-run copy placed into odom by
        # _ensure_route. Same split as path_points/route and pivots_raw/pivots,
        # and for the same reason -- _ensure_route runs AGAIN on every
        # /planner/start (which sets self.route = None to re-anchor), so a
        # transform that read cross_xy and wrote back to cross_xy applied
        # map->odom a second time on the already-converted list.
        #
        # BUG FOUND IN SIM (2026-09-04): that is exactly what it did. With a
        # 90deg map->odom yaw, waypoint (2.0, 10.3) landed at (-2.0, -10.3)
        # instead of (10.3, -2.0) on the second pass, so the rover never came
        # within waypoint_stop_arm_radius_m of any waypoint and NOT ONE
        # stop_at_waypoints stop fired for a whole route. It also silently
        # corrupted _finish_run's per-waypoint accuracy report and the RViz
        # landmark markers, which are both derived from cross_xy.
        self.cross_xy_raw = []
        self.cross_xy = []
        self.cross_best = []
        self.cross_tangent = []
        self.cross_idx = []
        # Raw (pre map->odom) pivots loaded from pivots_csv: (route_index, x,
        # y, to_yaw_rad), route_index verbatim into path_points/route.
        # self.pivots is the live, per-run queue built from these in
        # _ensure_route -- transformed into odom, sorted by index, and popped
        # off (in _pending_pivot) as each is completed. Kept separate from
        # pivots_raw so a re-anchor (_on_start_srv resetting self.route)
        # rebuilds the queue from the same source instead of resuming a
        # half-consumed one from the previous run.
        self.pivots_raw = []
        self.pivots = []
        self.pivot_braking = False
        self.pivot_brake_since = None
        self.pivot_arrival_armed = False
        self.pivot_best_d = float("inf")
        # Stop-and-wait state. next_stop_idx indexes into cross_xy/cross_names
        # -- starts at 1 to skip the START row (index 0), reset to 1 whenever
        # _ensure_route (re)builds the route, same as cross_best.
        self.next_stop_idx = 1
        self.awaiting_continue = False
        self.continue_requested = False
        self.waypoint_stop_armed = False
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

        # ---- forward obstacle stop-gate (pcl_obstacle_detector.py)
        # Deliberately fail-closed, the same posture as drive_enabled below: an
        # absent detector must not read as "the way is clear". A run with no
        # camera therefore needs obstacle_stop_enabled:=false, which is an
        # explicit decision rather than a silent one.
        self.obstacle_stop_enabled = bool(self.get_parameter("obstacle_stop_enabled").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.obstacle_timeout_s = float(self.get_parameter("obstacle_timeout_s").value)
        # Hold clear for this long before releasing the brake. The detector is a
        # per-frame classifier with no temporal filtering, so a single dropout
        # frame on a real obstacle would otherwise lurch the rover forward.
        self.obstacle_clear_s = float(self.get_parameter("obstacle_clear_s").value)
        self.obstacle_blocked = False
        self.obstacle_seen = False
        self.last_obstacle_time = None
        self.obstacle_clear_since = None

        self.local_plan_enabled = bool(self.get_parameter("local_plan_enabled").value)
        self.local_plan_topic = str(self.get_parameter("local_plan_topic").value)
        self.local_hold_topic = str(self.get_parameter("local_hold_topic").value)
        self.local_cmd_topic = str(self.get_parameter("local_cmd_topic").value)
        self.local_plan_timeout_s = float(self.get_parameter("local_plan_timeout_s").value)
        self.local_cmd_timeout_s = float(self.get_parameter("local_cmd_timeout_s").value)
        # local_route is what the rover actually chases while the local planner
        # is driving. self.route stays the GLOBAL route throughout, because that
        # is what tracking error and waypoint crossings are measured against --
        # a detour is a deliberate departure from it, not a new definition of it.
        self.local_route = None
        self.local_idx = 0
        self.local_hold = True
        self.local_hold_seen = False
        self.last_local_plan_time = None
        self.last_local_hold_time = None
        self.local_cmd = None
        self.last_local_cmd_time = 0.0

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

        if self.obstacle_stop_enabled and not self.local_plan_enabled:
            self.create_subscription(Bool, self.obstacle_topic, self._on_obstacle, 10)

        if self.local_plan_enabled:
            self.create_subscription(Path, self.local_plan_topic, self._on_local_plan, latched_qos)
            self.create_subscription(Bool, self.local_hold_topic, self._on_local_hold, 10)
            self.create_subscription(Twist, self.local_cmd_topic, self._on_local_cmd, 10)

        self.create_service(Trigger, "/planner/start", self._on_start_srv)
        self.create_service(Trigger, "/planner/stop", self._on_stop_srv)
        self.create_service(Trigger, "/planner/continue", self._on_continue_srv)

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
        obstacle_note = (
            f"obstacle stop: {self.obstacle_topic}" if self.obstacle_stop_enabled
            else "obstacle stop: DISABLED"
        )
        self.get_logger().info(
            f"cmd_vel_arbiter ready: {what} -> {self.cmd_vel_topic} | "
            f"teleop override: {self.teleop_topic} | armed state: {self.drive_enabled_topic} | "
            f"{obstacle_note}"
        )
        if not self.obstacle_stop_enabled:
            self.get_logger().warn(
                "obstacle_stop_enabled=false -- forward obstacles will NOT stop the rover"
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
        self.car_speed = msg.twist.twist.linear.x
        self.last_odom_time = time.monotonic()

    def _on_teleop(self, msg):
        self.last_teleop_cmd = msg
        if abs(msg.linear.x) > self.teleop_deadband or abs(msg.angular.z) > self.teleop_deadband:
            self.last_teleop_active_time = time.monotonic()

    def _on_obstacle(self, msg):
        """Latch the forward-obstacle flag from pcl_obstacle_detector.py.

        `obstacle_clear_s` is applied on the falling edge only. Blocking is
        immediate; unblocking has to be earned by a run of consecutive clear
        frames, so one dropped detection cannot release the brake.
        """
        now = time.monotonic()
        self.last_obstacle_time = now
        self.obstacle_seen = True

        if msg.data:
            self.obstacle_clear_since = None
            self.obstacle_blocked = True
            return

        if self.obstacle_clear_since is None:
            self.obstacle_clear_since = now
        if now - self.obstacle_clear_since >= self.obstacle_clear_s:
            self.obstacle_blocked = False

    def _obstacle_gate(self, now):
        """(allowed, reason) for the obstacle stop-gate alone."""
        if not self.obstacle_stop_enabled:
            return True, ""
        if not self.obstacle_seen:
            return False, (
                f"no obstacle detector on {self.obstacle_topic} "
                f"(start pcl_obstacle_detector.py, or set obstacle_stop_enabled:=false)"
            )
        if now - self.last_obstacle_time > self.obstacle_timeout_s:
            return False, f"obstacle detector stale on {self.obstacle_topic}"
        if self.obstacle_blocked:
            return False, "obstacle ahead"
        return True, ""

    def _on_local_plan(self, msg):
        """Adopt the route local_planner.py says to drive right now -- the global
        route passed through, or a detour around something on it.

        The index is snapped to the nearest point rather than reset to 0: a
        detour is handed over mid-course, and restarting at the far end of a
        path whose first point is behind the rover would drive it backwards
        through the obstacle it is avoiding."""
        pts = [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]
        if len(pts) < 2:
            return
        changed = self.local_route is None or len(pts) != len(self.local_route)
        self.local_route = pts
        self.last_local_plan_time = time.monotonic()
        if changed and self.car_pos is not None:
            self.local_idx = min(
                range(len(pts)), key=lambda i: distance(self.car_pos, pts[i]))
            self._last_seen_idx = None
            self._stuck_since = None

    def _on_local_hold(self, msg):
        self.local_hold = bool(msg.data)
        self.local_hold_seen = True
        self.last_local_hold_time = time.monotonic()

    def _on_local_cmd(self, msg):
        """A direct Twist from the local planner, used only to reverse out of an
        obstacle that closed inside min_distance. Forwarded through THIS node's
        publisher in _control_loop so /cmd_vel keeps exactly one owner."""
        self.local_cmd = msg
        self.last_local_cmd_time = time.monotonic()

    def _local_plan_gate(self, now):
        """(allowed, reason) when local_planner.py owns the obstacle decision.

        Fails closed on the same three things the Bool gate did: planner absent,
        planner silent, planner saying stop."""
        if not self.local_hold_seen or self.last_local_hold_time is None:
            return False, (
                f"no local planner on {self.local_hold_topic} "
                f"(start local_planner.py, or drop local_plan_enabled)")
        if now - self.last_local_hold_time > self.local_plan_timeout_s:
            return False, f"local planner stale on {self.local_hold_topic}"
        if self.local_hold:
            return False, "local planner holding (no viable path)"
        if self._local_cmd_active(now):
            # Reversing out of a too-close obstacle. There is deliberately no
            # plan during a backoff -- the one that was being driven is what got
            # the rover into this -- so requiring a fresh /local_plan here would
            # block the escape and strand the rover against the obstacle. Every
            # other check above (armed, started, odom fresh, planner alive and
            # not holding) still applies.
            return True, ""
        if self.local_route is None or self.last_local_plan_time is None:
            return False, f"no route yet on {self.local_plan_topic}"
        if now - self.last_local_plan_time > self.local_plan_timeout_s:
            return False, f"local plan stale on {self.local_plan_topic}"
        return True, ""

    def _local_cmd_active(self, now):
        return (self.local_cmd is not None
                and (now - self.last_local_cmd_time) < self.local_cmd_timeout_s)

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
        self.endgame_since = None
        self.goal_armed = False
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

    def _on_continue_srv(self, request, response):
        if not self.awaiting_continue:
            response.success = False
            response.message = "not currently waiting at a waypoint -- nothing to continue"
        else:
            self.continue_requested = True
            response.success = True
            response.message = f"continuing from {self.cross_names[self.next_stop_idx]}"
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

    def _runout_lookahead(self):
        """Lookahead the run-out is sized against.

        Must be the LARGEST lookahead the run will ever use, not whatever it
        happens to be when the route is built. With lookahead_dynamic the value
        grows with tracking error, so a run-out sized on the 0.5 m minimum is
        too short once the lookahead widens to 1.0 m near the goal -- the
        lookahead search then finds no point far enough ahead, the index falls
        off the end, and the endgame branch takes over exactly where the
        run-out was supposed to prevent it."""
        return self.lookahead_max if self.lookahead_dynamic else self.lookahead_distance

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
                        self.cross_xy_raw.append([float(r["x_m"]), float(r["y_m"])])
            except OSError as exc:
                raise SystemExit(f"cannot read waypoints_csv '{self.waypoints_csv}': {exc}")
            self.cross_xy = [list(p) for p in self.cross_xy_raw]
            self.cross_best = [float("inf")] * len(self.cross_xy)

        if self.pivots_csv:
            try:
                with open(self.pivots_csv, newline="") as f:
                    for r in _csv.DictReader(f):
                        self.pivots_raw.append((
                            int(r["route_index"]), float(r["x_m"]), float(r["y_m"]),
                            float(r["to_yaw_rad"]),
                        ))
            except OSError as exc:
                raise SystemExit(f"cannot read pivots_csv '{self.pivots_csv}': {exc}")
            n_idx = len(self.path_points)
            bad = [idx for idx, *_ in self.pivots_raw if not (0 <= idx < n_idx)]
            if bad:
                raise SystemExit(
                    f"pivots_csv '{self.pivots_csv}' has route_index {bad} outside "
                    f"path_csv's range [0, {n_idx - 1}] -- mismatched with path_csv?")

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
                self.route, self.route_goal = extend_past_goal(
                    self.route, self.goal_runout * self._runout_lookahead())
                # From cross_xy_RAW, never from cross_xy: this runs again on
                # every /planner/start, and reading the already-converted list
                # applied map->odom twice (see cross_xy_raw's comment above).
                self.cross_xy = [transform_point(p, t.x, t.y, yaw)
                                 for p in self.cross_xy_raw]
                pivot_yaw_offset = yaw
                pivot_t = (t.x, t.y)
            else:
                self.route = [list(p) for p in self.path_points]
                self.route, self.route_goal = extend_past_goal(
                    self.route, self.goal_runout * self._runout_lookahead())
                self.cross_xy = [list(p) for p in self.cross_xy_raw]
                pivot_yaw_offset = 0.0
                pivot_t = None
            self.cross_best = [float("inf")] * len(self.cross_xy)
            self.next_stop_idx = 1
            self.awaiting_continue = False
            self.continue_requested = False
            self.waypoint_stop_armed = False
            # Unit path tangent at the route point nearest each waypoint.
            # _pending_waypoint_stop uses it to ask "has the rover crossed the
            # line through this waypoint, perpendicular to the path?" -- an
            # exact geometric "am I past it" test.
            #
            # This replaces a "distance stopped improving" test that fired on
            # ANY transient increase. On a curving approach the distance
            # wobbles before the true minimum, so it stopped at the first local
            # minimum: measured in sim, it halted 31 cm from W9 on a pass that
            # actually reached 6.0 cm, and 52 cm from W3 on a pass that reached
            # 28.8 cm. cross_best still records the true minimum for reporting.
            self.cross_tangent = []
            self.cross_idx = []
            for wp in self.cross_xy:
                best_i, best_d = 0, float("inf")
                for i, p in enumerate(self.route):
                    dd = distance(p, wp)
                    if dd < best_d:
                        best_d, best_i = dd, i
                i0 = max(best_i - 1, 0)
                i1 = min(best_i + 1, len(self.route) - 1)
                tx = self.route[i1][0] - self.route[i0][0]
                ty = self.route[i1][1] - self.route[i0][1]
                n = math.hypot(tx, ty)
                self.cross_tangent.append((tx / n, ty / n) if n > 1e-9 else (0.0, 0.0))
                self.cross_idx.append(best_i)
            # Pivots are keyed by route_index into path_points, which
            # extend_past_goal only ever APPENDS after -- so those indices
            # still point at the right pose in self.route post-extension.
            # Sorted so _pending_pivot only ever has to look at [0].
            # "xy" is the pivot point placed into odom the same way the route
            # and cross_xy are -- _pending_pivot waits for the ROVER to reach
            # it rather than firing when the lookahead index does.
            self.pivots = sorted((
                {"idx": idx,
                 "target_yaw": wrap_angle(to_yaw + pivot_yaw_offset),
                 "xy": (transform_point([px, py], pivot_t[0], pivot_t[1], pivot_yaw_offset)
                        if pivot_t is not None else [px, py])}
                for idx, px, py, to_yaw in self.pivots_raw
            ), key=lambda p: p["idx"])
            self.pivot_braking = False
            self.pivot_brake_since = None
            self.pivot_arrival_armed = False
            self.pivot_best_d = float("inf")
            if self.pivots:
                self.get_logger().info(
                    f"{len(self.pivots)} in-place pivot(s) loaded from "
                    f"{os.path.basename(self.pivots_csv)}"
                )
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
            stop_note = (
                f"stopping at {max(len(self.cross_xy) - 2, 0)} of them, waiting for "
                f"/planner/continue each time" if self.stop_at_waypoints
                else "driving straight through -- no stops"
            )
            self.get_logger().info(
                f"global path '{os.path.basename(self.path_csv)}' ready in odom: "
                f"{len(self.route)} poses, {path_length(self.route):.1f} m, "
                f"{len(self.cross_xy)} waypoint(s) to cross. {stop_note.capitalize()}.")
            return

        if self.test_path_anchor == "start_pose":
            self.route = anchor_path(self.test_path_points, self.car_pos, self.car_yaw)
            markers = anchor_path(
                [[x, y] for _, x, y in self.test_markers], self.car_pos, self.car_yaw)
            marker_ids = [mid for mid, _, _ in self.test_markers]
        else:
            self.route = [list(p) for p in self.test_path_points]
            self.route, self.route_goal = extend_past_goal(
                self.route, self.goal_runout * self._runout_lookahead())
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

    def _advance_global_idx(self):
        """Keep route_idx tracking progress along the GLOBAL route even while a
        detour is being driven, so tracking error stays measured against the
        route the run is judged on. Windowed forward search -- a global one
        would snap to the wrong pass on a self-crossing course."""
        lo = self.route_idx
        hi = min(len(self.route), self.route_idx + 60)
        best = min(range(lo, hi), key=lambda i: distance(self.car_pos, self.route[i]))
        self.route_idx = best

    def _pending_pivot(self):
        """Execute an in-place rotation if the route has one due at or before
        route_idx and it isn't done yet. Returns True while pivoting -- the
        caller should do nothing else this tick -- False once clear to resume
        normal pure pursuit.

        Only ever consults route_idx (the GLOBAL route), never local_idx: a
        pivot is a property of the planned route's own joints, not something
        a local-planner detour should trigger, so this is skipped entirely
        while detouring (see the call site) rather than taught to reason
        about a second index.

        Pure pursuit cannot express this itself -- angular = curvature *
        base_velocity is always paired with nonzero forward motion (see the
        bottom of this method vs. this one's linear=0.0) -- so a route that
        truly needs a stop-and-turn has no way to ask for it except this.

        Two phases, tracked by pivot_braking. cmd_vel_odrive_bridge.py's
        wheel_accel_rps2 ramps each wheel toward its target INDEPENDENTLY --
        it guarantees the final differential matches what's commanded, not
        that it matches during the ramp. Commanding the rotation immediately
        while still carrying forward speed asks both wheels to swap to
        opposite-signed targets at once; while they ramp there independently
        the actual differential doesn't match the commanded pivot, and the
        rover drifts/arcs instead of cleanly stopping first. So: brake and
        wait for measured speed to actually settle (phase 1) before
        commanding any rotation (phase 2), rather than trusting the ramp to
        sort it out."""
        if not self.pivots:
            return False
        nxt = self.pivots[0]
        if self.route_idx < nxt["idx"]:
            return False

        # route_idx is the LOOKAHEAD index -- it leads the rover by
        # lookahead_distance. Firing the pivot the moment it reaches the pivot
        # index therefore rotates the rover a full lookahead SHORT of the pivot
        # point, and since the rover then departs on the new heading it never
        # reaches that point at all.
        #
        # Measured in sim: the two pivot waypoints came out at 50.3 cm and
        # 52.1 cm closest approach against 31.4 cm at the non-pivot waypoint --
        # the ~20 cm difference is exactly this early trigger (lookahead runs
        # 0.4-0.7 m). So wait for the ROVER to actually reach the pivot point:
        # arm on approach, then fire once distance stops improving, i.e. at
        # genuine closest approach. Same "passed the minimum" test as
        # _pending_waypoint_stop, and for the same reason -- it lands as close
        # as the tracking error of the day allows instead of at a fixed radius
        # that might never be satisfied.
        if not self.pivot_braking and nxt.get("xy") is not None:
            d = distance(self.car_pos, nxt["xy"])
            if not self.pivot_arrival_armed:
                if d <= self.pivot_arrive_radius_m:
                    self.pivot_arrival_armed = True
                    self.pivot_best_d = d
                else:
                    return False  # still approaching -- keep driving the path
            if d < self.pivot_best_d:
                self.pivot_best_d = d
                return False      # still closing on the point
            if d <= self.pivot_best_d + self.waypoint_stop_margin_m:
                return False      # inside the noise band, not yet past it
            # Past the closest approach: this is the moment to stop and rotate.

        if not self.pivot_braking:
            speed = abs(self.car_speed) if self.car_speed is not None else 0.0
            if self.pivot_brake_since is None:
                self.pivot_brake_since = time.monotonic()
            stopped = speed <= self.pivot_brake_speed_mps
            timed_out = (time.monotonic() - self.pivot_brake_since) >= self.pivot_brake_timeout_s
            if stopped or timed_out:
                if timed_out and not stopped:
                    self.get_logger().warn(
                        f"pivot brake timed out after {self.pivot_brake_timeout_s:.1f}s "
                        f"(still {speed:.3f} m/s) -- rotating anyway",
                        throttle_duration_sec=5.0)
                self.pivot_braking = True
            else:
                self._publish(0.0, 0.0)
                self.get_logger().info(
                    f"braking to a stop before pivoting (speed {speed:.3f} m/s)",
                    throttle_duration_sec=0.5)
                return True

        err = wrap_angle(nxt["target_yaw"] - self.car_yaw)
        if abs(err) <= self.pivot_tolerance_rad:
            self.get_logger().info(
                f"pivot complete: now facing {math.degrees(self.car_yaw):+.0f}deg "
                f"(target {math.degrees(nxt['target_yaw']):+.0f}deg)")
            self.pivots.pop(0)
            self.pivot_braking = False
            self.pivot_brake_since = None
            # Re-arm for the NEXT pivot, which is a different point.
            self.pivot_arrival_armed = False
            self.pivot_best_d = float("inf")
            return False
        # Bang-bang, not proportional: a fixed rate is simpler to reason
        # about and verify on the bench than tuning a gain for a maneuver
        # that only runs a handful of times per competition run.
        rate = math.copysign(self.pivot_max_angular_rps, err)
        self._publish(0.0, rate)
        self.get_logger().info(
            f"pivoting in place: {math.degrees(self.car_yaw):+.0f}deg -> "
            f"{math.degrees(nxt['target_yaw']):+.0f}deg (err {math.degrees(err):+.1f}deg)",
            throttle_duration_sec=0.5)
        return True

    def _pending_waypoint_stop(self):
        """Stop-and-wait mode (stop_at_waypoints:=true): halt at each named
        waypoint in cross_xy/cross_names (skipping the START row at index 0
        and the final/loop-closing point, see stop_at_waypoints's
        declare_parameter comment) until /planner/continue is called.

        "As close as possible" is measured the same way _finish_run reports
        it -- cross_best[k], the smallest distance to waypoint k seen over
        the whole approach, updated every tick in _record_tracking_error
        (which MUST run before this each tick, see the call order in
        _drive_test_path). Waiting for the rover to move waypoint_stop_margin_m
        past that minimum, rather than testing a fixed radius, means this
        stops at the true closest point reached whatever the real tracking
        error turns out to be, instead of a tolerance that might never be
        satisfied.

        BUG FOUND IN SIM (2026-09-03), fixed by waypoint_stop_arm_radius_m:
        cross_best[k] starts at inf and gets set to whatever d happens to be
        on the very first tick, from wherever the rover starts -- 10+ metres
        away for a waypoint late in the route. Ordinary EKF settling noise
        of a few cm was then enough for the NEXT tick's d to exceed that by
        more than waypoint_stop_margin_m, declaring "reached" before the
        rover had moved from the start line at all. "Closest approach so
        far" is only a meaningful signal once genuinely closing in on the
        point -- far away it's measuring noise, not progress. So: the
        passed-the-minimum test below only runs once armed (d has come
        within waypoint_stop_arm_radius_m at least once); cross_best itself
        still tracks every tick regardless, same as always, this only gates
        when it gets ACTED on."""
        if not self.stop_at_waypoints:
            return False
        if self.next_stop_idx >= len(self.cross_xy) - 1:
            return False  # no interior stops left; final point is _finish_run's job

        if self.awaiting_continue:
            if not self.continue_requested:
                self._publish(0.0, 0.0)
                return True
            name = self.cross_names[self.next_stop_idx]
            self.get_logger().info(f"continuing from {name}")
            self.awaiting_continue = False
            self.continue_requested = False
            self.next_stop_idx += 1
            self.waypoint_stop_armed = False
            return False

        k = self.next_stop_idx
        d = distance(self.car_pos, self.cross_xy[k])
        # Waypoint position relative to the rover's heading: negative = ahead,
        # positive = behind. Needed by BOTH the arm gate and the trigger below.
        _tx, _ty = math.cos(self.car_yaw), math.sin(self.car_yaw)
        _wp = self.cross_xy[k]
        _along = (self.car_pos[0] - _wp[0]) * _tx + (self.car_pos[1] - _wp[1]) * _ty
        if not self.waypoint_stop_armed:
            # Arming needs BOTH "close to the waypoint" and "actually up to
            # this part of the route". Distance alone is not enough: legs here
            # loop hard (leg 2 is 12.3 m of path for a 4.16 m straight-line
            # gap), so the rover passes within the arm radius of a waypoint
            # EARLY, while still looping toward it, from a direction where it
            # already reads as "past" the perpendicular.
            #
            # Measured in sim: distance-only arming fired the instant it armed
            # and reported W9 at 198.9 cm -- essentially the arm radius itself.
            # The index gate makes "near W9" mean "near W9 on final approach".
            near_in_route = (k >= len(self.cross_idx) or
                             self.route_idx >= self.cross_idx[k] - self.waypoint_stop_arm_poses)
            # ...and the waypoint must still be AHEAD of the rover when we arm.
            # Legs here loop hard enough to fly PAST a waypoint and come back:
            # measured at W9, the route passes at 31 cm (idx ~127), recedes to
            # 1.45 m, then returns to 4.9 cm (idx ~154). The index gate opens
            # ~20 poses before 154, i.e. mid-loop, where the point is already
            # BEHIND -- so the abeam trigger fired in the same tick it armed and
            # reported the fly-past's 30.9 cm. Identical to the centimetre
            # across three different trigger designs, which is what gave it away.
            # Requiring "ahead" at arm time means only a real ahead->behind
            # transition can fire, which is the actual closest approach.
            if d <= self.waypoint_stop_arm_radius_m and near_in_route and _along < 0.0:
                self.waypoint_stop_armed = True
            else:
                return False
        # Has the waypoint gone abeam? Project the waypoint->rover vector onto
        # the ROVER'S OWN heading: negative while the point is ahead, positive
        # once it is behind. That is exactly the closest-approach condition for
        # the trajectory the rover is actually flying.
        #
        # Deliberately NOT the path tangent at the waypoint, which was tried
        # first: mid-corner the rover's heading and the path's tangent differ,
        # so the perpendicular-to-tangent plane is crossed before the rover's
        # real closest approach. Measured in sim, that stopped at W9 at 30.9 cm
        # on a pass that went on to reach 6.0 cm -- ~25 cm early, every time.
        if _along > self.waypoint_stop_margin_m:
            self.awaiting_continue = True
            self.get_logger().info(
                f"reached {self.cross_names[k]} ({self.cross_best[k] * 100:.1f} cm closest "
                f"approach) -- waiting for /planner/continue"
            )
            self._publish(0.0, 0.0)
            return True
        return False

    def _drive_test_path(self, now):
        """Continuous pure pursuit along the whole anchored course -- no
        per-waypoint stop-and-go (that is what _control_loop's waypoint branch
        does), because these paths exist to measure uninterrupted curvature
        tracking.

        With local_plan_enabled, the route CHASED may be a detour published by
        local_planner.py instead of the global route. self.route stays global
        throughout: the detour is a departure from the run's reference, not a
        replacement for it, and scoring it against itself would report a
        perfectly tracked detour as a perfectly tracked run."""
        self._ensure_route()
        if self.route is None:
            self._hold("waiting for odometry / map->odom to place the path")
            return
        if self.run_finished:
            self._publish(0.0, 0.0)
            return

        detouring = self.local_plan_enabled and self.local_route is not None
        if not detouring and self._pending_pivot():
            # The stuck/circling detector below measures "the lookahead index
            # has not advanced for stuck_timeout_s". A pivot or a waypoint stop
            # holds the rover deliberately still, so the index legitimately
            # does not advance -- but those branches return before the detector
            # runs, leaving _stuck_since frozen at a timestamp from before the
            # hold. The moment driving resumed, now - _stuck_since already
            # exceeded the timeout and the detector fired instantly.
            #
            # BUG FOUND IN SIM (2026-09-04): that false trigger, landing right
            # as the rover departed W3 into the tightest arc on the route,
            # force-advanced the index; the rover could not reach the receding
            # target, so the index never advanced on its own, so it fired again
            # every 3 s, marching the goal away. Cross-track error ran 5 m ->
            # 13 m and the run was unrecoverable. Clearing the timer on every
            # deliberate hold is what stops the cascade at its source.
            self._stuck_since = None
            return
        if detouring:
            self._advance_global_idx()
        err = self._record_tracking_error()
        if not detouring and self._pending_waypoint_stop():
            self._stuck_since = None
            return

        chase = self.local_route if detouring else self.route
        idx = self.local_idx if detouring else self.route_idx

        # Widen the lookahead in proportion to how far off the path we are, so
        # a large correction is asked for gently rather than at the curvature
        # clamp. Uses the cross-track error measured this tick, so it responds
        # to a pose snap immediately and relaxes as the rover converges. Capped
        # by a second, independent term driven by the path's own upcoming
        # curvature -- see the lookahead_curvature_gain declaration above for
        # why the error term alone isn't enough.
        if self.lookahead_dynamic:
            error_term = min(
                self.lookahead_max,
                max(self.lookahead_min,
                    self.lookahead_min + self.lookahead_error_gain * err))

            # TRIED (2026-09-03) making curvature_cap step aside entirely
            # once err > lookahead_min, reasoning that it has nothing useful
            # to say once the rover is already off the path by more than the
            # shortest lookahead reaches. That worked for the circling this
            # was built to fix (see MIN_TURNING_RADIUS's real fix in
            # nav2_planning_params.yaml -- this bypass was not actually
            # load-bearing for that one), but on its own it caused a NEW
            # problem: jumping lookahead straight to lookahead_max the
            # moment err crosses the threshold, while the rover's heading
            # hasn't converged yet, searches for a point far along the path
            # from a position/heading that doesn't reach one -- observed in
            # sim as the lookahead index advancing only ~1 per 3s
            # (stuck-circling firing repeatedly) with err growing
            # continuously, moderate but never-clamped curvature the whole
            # time (unlike genuine clamped circling, which pins at
            # max_curvature). Reverted -- kappa_ahead governs curvature_cap
            # unconditionally again, same as before 2026-09-03.
            kappa_ahead = path_curvature_ahead(chase, idx, self.lookahead_curvature_sample_m)
            curvature_cap = self.lookahead_max
            if kappa_ahead > 1e-6:
                curvature_cap = min(
                    self.lookahead_max,
                    max(self.lookahead_min, self.lookahead_curvature_gain / kappa_ahead))

            self.lookahead_distance = min(error_term, curvature_cap)

            # Shorten the lookahead while closing on a waypoint. Pure pursuit
            # always cuts the inside of a corner by roughly L^2/2R, so the
            # lookahead that gives smooth tracking down a leg is the same
            # thing that makes the rover miss the apex -- and the apexes here
            # ARE the scored waypoints. Measured in sim at L=0.7, R=0.7:
            # 0.7^2/(2*0.7) = 0.35 m of cut, against 31.4 cm actually observed
            # at W9. Dropping to 0.35 m over the approach predicts ~9 cm.
            #
            # Only applied within waypoint_lookahead_radius_m of the next
            # waypoint, so it buys accuracy exactly where it is scored and
            # leaves the long legs on the full lookahead they track best with.
            # Nearest waypoint, NOT cross_xy[next_stop_idx]: next_stop_idx only
            # ever advances inside _pending_waypoint_stop, so with
            # stop_at_waypoints=false it stays pinned at 1 and the shrink would
            # follow one waypoint for the whole run.
            if self.waypoint_lookahead_m > 0.0 and self.cross_xy:
                d_wp = min(distance(self.car_pos, wp) for wp in self.cross_xy)
                if d_wp <= self.waypoint_lookahead_radius_m:
                    self.lookahead_distance = min(
                        self.lookahead_distance, self.waypoint_lookahead_m)

        # A detour ends at its rejoin point, not at the goal. Only the route
        # whose end IS the global goal may declare the run complete.
        can_finish = (not detouring) or distance(chase[-1], self.route[-1]) < 0.1

        # ---- goal test, every tick ----
        # This has to run BEFORE the lookahead search, not inside the
        # "index exhausted" branch below. The run-out appended past the goal
        # keeps supplying points ahead, so the index never runs out while the
        # rover is approaching -- nesting the test in that branch meant it was
        # never evaluated at the moment of crossing. The rover drove through
        # the goal, carried on down the run-out, and only stopped once the
        # run-out itself was exhausted, roughly one run-out length past the
        # goal. Measured on hardware as a 0.6 m overshoot on a path it had
        # passed within centimetres of.
        goal = (self.route_goal if (not detouring and self.route_goal is not None)
                else chase[-1])
        gap = distance(self.car_pos, goal)

        # Armed only after the rover has genuinely been away from the goal.
        # A closed loop starts ON its own final point, so an unguarded test
        # fires at t=0 and the run ends before it begins. A latch handles open
        # and closed routes alike without any index arithmetic.
        if not self.goal_armed and gap > max(3.0 * self.lookahead_distance, 2.0):
            self.goal_armed = True

        if can_finish and self.goal_armed:
            done_radius = (self.path_goal_tolerance if self.path_csv
                           else self.test_path_goal_tolerance)
            f_end = final_heading_vector(chase)
            crossed = False
            if f_end is not None and gap < max(2.0 * self.lookahead_distance, 1.0):
                to_goal = (goal[0] - self.car_pos[0], goal[1] - self.car_pos[1])
                crossed = (to_goal[0] * f_end[0] + to_goal[1] * f_end[1]) <= 0.0
            if gap < done_radius or crossed:
                if detouring:
                    self.local_idx = min(idx, len(chase) - 1)
                else:
                    self.route_idx = min(idx, len(chase) - 1)
                self.get_logger().info(
                    f"finish: {'crossed the goal' if crossed else 'within tolerance'} "
                    f"({gap:.3f} m from it)")
                self._finish_run()
                return

        lookahead_point = None
        for i in range(idx, len(chase)):
            if distance(self.car_pos, chase[i]) >= self.lookahead_distance:
                lookahead_point = chase[i]
                idx = i
                break

        # No point far enough ahead left in the course: the rover is inside
        # one lookahead of the end. Closed courses (circle, infinity) return
        # to their own start, so "near the final point" alone would fire on
        # the first tick -- requiring the index to have run out first is what
        # makes completion mean "drove the whole thing", not "stands near the
        # end".
        if lookahead_point is None:
            # Index exhausted: the rover is past the end of even the run-out.
            # The goal test above already handles arrival, so reaching here at
            # all means it never crossed -- circling wide, or it never came
            # within arming range. Nothing here can be stopped by hand, so end
            # the run rather than drive on forever.
            idx = len(chase) - 1
            if self.endgame_since is None:
                self.endgame_since = now
            if can_finish and (now - self.endgame_since) >= self.endgame_grace_s:
                if detouring:
                    self.local_idx = idx
                else:
                    self.route_idx = idx
                self.get_logger().warn(
                    f"finish: gave up after {now - self.endgame_since:.0f}s past the end "
                    f"of the route without crossing the goal ({gap:.2f} m from it)")
                self._finish_run()
                return
            lookahead_point = chase[-1]

        if idx == self._last_seen_idx:
            if self._stuck_since is None:
                self._stuck_since = now
            elif now - self._stuck_since >= self.stuck_timeout_s:
                # Forcing the index forward only makes sense for a rover that
                # is ON the path but grinding against a corner it keeps
                # re-approaching. If it is far OFF the path, the index is not
                # advancing because the rover cannot reach the path at all,
                # and advancing the target moves the goal further away -- the
                # rover then chases a receding point in a straight line and
                # never comes back.
                #
                # Measured in sim: after a false trigger at W3 this ran every
                # 3 s for 25+ iterations, walking cross-track error from 5 m to
                # 13 m with no recovery possible. Past this threshold the right
                # move is to leave the lookahead where it is and let pure
                # pursuit steer back toward the path it is already chasing.
                if err > self.stuck_force_max_error_m:
                    self.get_logger().warn(
                        f"{err:.2f} m off the path (> {self.stuck_force_max_error_m:.2f} m) and "
                        f"index {idx} is not advancing -- NOT forcing forward, that would drive "
                        f"the rover further away. Steering back toward the path instead.",
                        throttle_duration_sec=3.0,
                    )
                else:
                    self.get_logger().warn(
                        f"lookahead index {idx} hasn't advanced in "
                        f"{self.stuck_timeout_s:.1f}s (stuck circling) -- forcing it forward."
                    )
                    idx = min(idx + 1, len(chase) - 1)
                    lookahead_point = chase[idx]
                    self._last_seen_idx = idx
                self._stuck_since = None
        else:
            self._stuck_since = None
            self._last_seen_idx = idx

        # ---- final approach: aim AT the waypoint, not at the path ----
        # Pure pursuit chasing a point on the path always cuts the inside of a
        # corner by ~L^2/2R, and the corners here ARE the scored waypoints, so
        # there is a hard floor on how close it can stop: 8.75 cm even at
        # L=0.35, R=0.7. Aiming the pursuit target directly at the waypoint
        # over the last stretch removes that term by construction.
        #
        # Safe against the collision-checked plan: the waypoint lies ON the
        # planned path, and corner-cutting is what pulls the rover OFF it
        # (toward the inside). Steering at the point moves the rover back
        # toward the route, never further from it -- which matters here, where
        # some waypoints have only 0.40 m of clearance.
        #
        # Index-gated the same way the stop is, so a leg that loops within
        # waypoint_final_approach_m of a waypoint on its way somewhere else
        # does not get dragged toward it.
        if (not detouring and self.waypoint_final_approach_m > 0.0
                and self.cross_xy and self.cross_idx):
            k = self.next_stop_idx if self.stop_at_waypoints else None
            if k is None:
                k = next((i for i, ci in enumerate(self.cross_idx)
                          if ci >= self.route_idx), None)
            if k is not None and 0 < k < len(self.cross_xy):
                d_wp = distance(self.car_pos, self.cross_xy[k])
                near_in_route = (self.route_idx >=
                                 self.cross_idx[k] - self.waypoint_stop_arm_poses)
                # Only aim at it while it is genuinely AHEAD -- same fly-past
                # hazard as the stop's arm gate (see _pending_waypoint_stop).
                # Steering toward a waypoint the rover has already passed would
                # drag it off the planned path mid-loop.
                wx, wy = self.cross_xy[k]
                ahead = ((wx - self.car_pos[0]) * math.cos(self.car_yaw) +
                         (wy - self.car_pos[1]) * math.sin(self.car_yaw)) > 0.0
                if d_wp <= self.waypoint_final_approach_m and near_in_route and ahead:
                    lookahead_point = self.cross_xy[k]

        if detouring:
            self.local_idx = idx
        else:
            self.route_idx = idx

        local_x, local_y = point_global_to_local(lookahead_point, self.car_yaw, self.car_pos)
        curvature = max(-self.max_curvature, min(self.max_curvature, calc_curvature(local_x, local_y)))
        angular = curvature * self.base_velocity

        what = "local_plan" if detouring else (self.test_path_name or "global path")
        self.get_logger().info(
            f"{what} {idx + 1}/{len(chase)} | "
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
        if self.local_plan_enabled:
            return self._local_plan_gate(now)
        return self._obstacle_gate(now)

    def _hold(self, reason):
        """Log on change, then keep repeating at a low rate.

        Logging only on change makes a PERSISTENT hold invisible: the reason
        prints once at startup, scrolls off, and from then on the node sits
        silently publishing zero twist with no clue why. That is indisting-
        uishable from "driving but not moving" when you are looking at a
        terminal full of other nodes, and it cost a debugging session.
        Repeating every 10s stays quiet enough to read while keeping the
        current reason always on screen."""
        if reason != self.gate_reason:
            self.gate_reason = reason
            self.get_logger().warn(f"holding: {reason}")
        else:
            self.get_logger().warn(f"still holding: {reason}", throttle_duration_sec=10.0)
        self._publish(0.0, 0.0)

    def _control_loop(self):
        now = time.monotonic()

        self._try_load_waypoints()
        # Place and publish the global route before the gate is consulted, not
        # after. local_planner.py reads /pure_pursuit/path to know what route to
        # keep clear, and the gate below waits on local_planner -- so leaving
        # this inside the post-gate drive path made each node wait for the
        # other and neither ever moved.
        if self.path_csv:
            self._ensure_route()
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

        # The local planner is reversing out of something that closed inside
        # min_distance. Pure pursuit cannot express that (it only ever drives
        # forward), so the Twist is forwarded verbatim -- through this node's
        # publisher, so /cmd_vel still has exactly one owner.
        if self.local_plan_enabled and self._local_cmd_active(now):
            self.pub.publish(self.local_cmd)
            return

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
