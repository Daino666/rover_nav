#!/usr/bin/env python3
"""Reactive local planner: stop, plan a costed detour, drive it, rejoin.

Implements the local-planner specification: trigger on an obstacle threatening
the ACTIVE path, size unknown obstacles as at least rover-sized, search for the
lowest-cost detour that clears every known obstacle and keeps the triggering one
in camera view, plan it to completion while stopped, and treat every previously
seen obstacle as a hard constraint on every subsequent replan.

WHY A SEPARATE NODE THAT DOES NOT TOUCH /cmd_vel
    cmd_vel_arbiter.py fails closed if any OTHER publisher appears on /cmd_vel
    (_check_ownership), which is what stops two controllers fighting over the
    motors. So this node never publishes /cmd_vel. It publishes GEOMETRY and
    INTENT, and the arbiter -- still the single motor-command owner -- executes
    it. Run the arbiter with local_plan_enabled:=true to make it listen.

INTERFACE (all odom frame)
    /local_plan         nav_msgs/Path    route to drive: global route, or a detour
    /local_plan/hold    std_msgs/Bool    true = stop. Also held for the whole of
                                         STOPPING and PLANNING, which is what
                                         makes "never plan while moving" true
                                         rather than merely intended.
    /local_plan/cmd     Twist            reverse-out override; off by default
    /local_plan/state   String           state, clearances, costs
    /local_plan/markers MarkerArray      keepout discs + the camera cone

SPEC POINT 2 -- UNKNOWN SIZE MEANS ROVER-SIZED
    A single depth cluster tells you where something is, not how big it is: the
    camera sees the near face of a rock and nothing behind it. So the planned
    extent is floored at the rover's own radius before any clearance is added:

        keepout = max(detected_radius, rover_radius) + rover_radius + margin

    The first term is the obstacle size floor, the second is the room the
    rover's own body needs. This roughly doubles the keepout of a small
    detection, which is the point -- a 0.25 m cluster that is really a 0.8 m
    boulder seen edge-on is the case that ends a run.

SPEC POINT 3 -- THREE CONSTRAINTS, ONE SEARCH
    Clearance is a hard constraint: cells inside any known keepout are not
    traversable, full stop. Cost and field-of-view are scored TOGETHER in one
    objective rather than filtered in sequence:

        step_cost = length * (1 + cost_weight*costmap01 + fov_weight*out_of_view)

    The specification flags that "lowest cost" and "keep it in view" can pull
    opposite ways. They are combined rather than ranked because making FOV a
    hard filter can empty the candidate set and leave the rover with no plan at
    all -- and a planner with no plan stops forever, which is worse than a plan
    that briefly loses sight of a rock it has already measured and remembered.
    Set fov_mode:=hard to make it a filter anyway, or fov_mode:=off to disable
    the term; fov_weight tunes who wins when they disagree.

SPEC POINTS 4, 5, 7 -- STOP, THEN PLAN, THEN MOVE
    Every trigger -- new obstacle on the path, another obstacle during a detour,
    the depth check invalidating the path ahead -- runs the same sequence:
    FOLLOW/DETOUR -> STOPPING (hold, wait for the wheels to actually stop, not
    just for a zero command to be sent) -> PLANNING (hold, search) -> DETOUR.
    The hold is released only once a complete path from the current pose to the
    reattachment point exists.

SPEC POINT 6 -- EVERY OBSTACLE, EVERY REPLAN
    Obstacles persist for the run. Each replan is checked against all of them,
    not just the one that triggered it, so a detour can never route back through
    something already avoided. They are dropped only once genuinely behind the
    rover AND off the remaining route -- not on a timer, because the camera
    stops seeing a rock the moment the rover turns to go around it.
"""

import heapq
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from global_path_planner import distance  # noqa: E402
import test_path_viz  # noqa: E402

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from rover_nav.msg import ObstacleArray

STATE_FOLLOW = "FOLLOW"
STATE_STOPPING = "STOPPING"
STATE_PLANNING = "PLANNING"
STATE_DETOUR = "DETOUR"
STATE_BACKOFF = "BACKOFF"
STATE_BLOCKED = "BLOCKED"


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_point(point, tx, ty, yaw):
    x, y = point[0], point[1]
    return [x * math.cos(yaw) - y * math.sin(yaw) + tx,
            x * math.sin(yaw) + y * math.cos(yaw) + ty]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Obstacle2D:
    """An obstacle disc with TWO radii around it, doing two different jobs.

    `collide` is the hard one, built from the MEASURED radius: the rover's
    centre inside it means contact. Nothing may ever enter it, and it is what
    decides whether an obstacle is worth avoiding at all.

    `keepout` is the soft one, built from the size floor (an unseen rock is
    assumed rover-sized): the clearance the planner would LIKE. Entering it
    costs, it does not forbid.

    Keeping them separate is what stops the rover detouring around every pebble
    within two metres of the route. One conservative radius doing both jobs
    meant a 0.1 m detection became a 1.81 m no-go zone, so anything vaguely near
    the path triggered a full stop-and-replan."""

    __slots__ = ("x", "y", "radius", "collide", "keepout", "last_seen")

    def __init__(self, x, y, radius, collide, keepout, last_seen):
        self.x, self.y = x, y
        self.radius, self.collide, self.keepout = radius, collide, keepout
        self.last_seen = last_seen

    def centre_dist(self, p):
        return math.hypot(p[0] - self.x, p[1] - self.y)

    def surface_clearance(self, p, rover_radius):
        return self.centre_dist(p) - self.radius - rover_radius


class LocalPlanner(Node):
    def __init__(self):
        super().__init__("local_planner")

        # ---- sources
        self.declare_parameter("global_path_topic", test_path_viz.PATH_TOPIC)
        self.declare_parameter("path_csv", "")
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("obstacles_topic", "/obstacles/array")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")

        # ---- rover dimensions. MEASURE THESE.
        self.declare_parameter("rover_length", 0.95)
        self.declare_parameter("rover_width", 0.75)
        self.declare_parameter("safety_margin", 0.25)
        self.declare_parameter("collision_margin", 0.15)
        self.declare_parameter("min_obstacle_radius", 0.0)
        self.declare_parameter("inflation_weight", 2.0)

        # ---- search
        self.declare_parameter("search_resolution", 0.20)
        self.declare_parameter("search_margin", 3.0)
        self.declare_parameter("max_search_cells", 60000)
        self.declare_parameter("cost_weight", 3.0)
        self.declare_parameter("unknown_cost", 0.35)
        self.declare_parameter("horizon", 5.0)
        self.declare_parameter("rejoin_margin", 1.0)
        self.declare_parameter("resolution", 0.10)

        # ---- field of view
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("camera_range", 4.0)
        self.declare_parameter("fov_weight", 1.5)
        self.declare_parameter("fov_mode", "soft")

        # ---- stop-before-plan
        self.declare_parameter("stopped_speed", 0.03)
        self.declare_parameter("stop_settle_s", 2.0)

        # ---- clearance guards
        self.declare_parameter("min_distance", 0.60)
        self.declare_parameter("safe_distance", 1.00)
        self.declare_parameter("backoff_enabled", False)
        self.declare_parameter("backoff_speed", 0.12)
        self.declare_parameter("backoff_cone_deg", 150.0)
        self.declare_parameter("max_backoff_m", 1.0)

        # ---- memory
        self.declare_parameter("obstacle_match_radius", 0.5)
        self.declare_parameter("forget_behind_m", 2.0)
        self.declare_parameter("confirm_range", 2.0)
        self.declare_parameter("confirm_after_s", 3.0)
        self.declare_parameter("forget_behind_deg", 120.0)

        # ---- liveness
        self.declare_parameter("obstacle_timeout_s", 1.0)
        self.declare_parameter("odom_timeout_s", 0.5)
        self.declare_parameter("require_detector", True)
        self.declare_parameter("require_costmap", False)
        self.declare_parameter("rate_hz", 10.0)

        g = self.get_parameter
        self.global_path_topic = str(g("global_path_topic").value)
        self.path_csv = str(g("path_csv").value).strip()
        self.path_frame = str(g("path_frame").value).strip()
        self.odom_topic = str(g("odom_topic").value)
        self.obstacles_topic = str(g("obstacles_topic").value)
        self.costmap_topic = str(g("costmap_topic").value)

        self.rover_length = float(g("rover_length").value)
        self.rover_width = float(g("rover_width").value)
        self.safety_margin = float(g("safety_margin").value)
        self.collision_margin = float(g("collision_margin").value)
        self.min_obstacle_radius = float(g("min_obstacle_radius").value)
        self.inflation_weight = float(g("inflation_weight").value)
        self.rover_radius = math.hypot(self.rover_length / 2.0, self.rover_width / 2.0)

        self.search_resolution = float(g("search_resolution").value)
        self.search_margin = float(g("search_margin").value)
        self.max_search_cells = int(g("max_search_cells").value)
        self.cost_weight = float(g("cost_weight").value)
        self.unknown_cost = float(g("unknown_cost").value)
        self.horizon = float(g("horizon").value)
        self.rejoin_margin = float(g("rejoin_margin").value)
        self.resolution = float(g("resolution").value)

        self.camera_half_fov = math.radians(float(g("camera_fov_deg").value)) / 2.0
        self.camera_range = float(g("camera_range").value)
        self.fov_weight = float(g("fov_weight").value)
        self.fov_mode = str(g("fov_mode").value).strip().lower()

        self.stopped_speed = float(g("stopped_speed").value)
        self.stop_settle_s = float(g("stop_settle_s").value)

        self.min_distance = float(g("min_distance").value)
        self.safe_distance = float(g("safe_distance").value)
        self.backoff_enabled = bool(g("backoff_enabled").value)
        self.backoff_speed = abs(float(g("backoff_speed").value))
        self.backoff_cone_deg = float(g("backoff_cone_deg").value)
        self.max_backoff_m = float(g("max_backoff_m").value)

        self.obstacle_match_radius = float(g("obstacle_match_radius").value)
        self.forget_behind_m = float(g("forget_behind_m").value)
        self.confirm_range = float(g("confirm_range").value)
        self.confirm_after_s = float(g("confirm_after_s").value)
        self.forget_behind_half = math.radians(float(g("forget_behind_deg").value)) / 2.0

        self.obstacle_timeout_s = float(g("obstacle_timeout_s").value)
        self.odom_timeout_s = float(g("odom_timeout_s").value)
        self.require_detector = bool(g("require_detector").value)
        self.require_costmap = bool(g("require_costmap").value)
        rate = max(1.0, float(g("rate_hz").value))

        # The margin a PLAN is held to must be at least the clearance the live
        # guard reacts at, or a detour the planner considers perfectly clear
        # trips its own stop the moment it is driven. Measured: margin 0.25
        # against min_distance 0.60 aborted every detour.
        self.plan_margin = max(self.safety_margin, self.min_distance)

        if self.safe_distance <= self.min_distance:
            raise SystemExit(
                f"safe_distance ({self.safe_distance}) must exceed min_distance "
                f"({self.min_distance}) -- otherwise a backoff can never end.")
        if self.fov_mode not in ("soft", "hard", "off"):
            raise SystemExit(f"fov_mode must be soft|hard|off, got '{self.fov_mode}'")

        # ---- state
        self.pos = None
        self.yaw = None
        self.speed = 0.0
        self.last_odom = None
        self.memory = []
        self.obstacles = []
        self.last_obs = None
        self.obs_seen = False
        self.costmap = None
        self.route = None
        self.detour = None
        self.rejoin_idx = 0
        self.state = STATE_FOLLOW
        self.last_logged = (None, None)
        self.clearance = float("inf")
        self.plan_cost = 0.0
        self.trigger_obs = None
        self.stop_since = None
        self.pending_reason = ""
        self.backoff_total = 0.0
        self.backoff_from = None
        self.prev_pos = None
        self.replans = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = test_path_viz.latched_qos()
        self.plan_pub = self.create_publisher(Path, "/local_plan", latched)
        self.hold_pub = self.create_publisher(Bool, "/local_plan/hold", 10)
        self.cmd_pub = self.create_publisher(Twist, "/local_plan/cmd", 10)
        self.state_pub = self.create_publisher(String, "/local_plan/state", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/local_plan/markers", 10)

        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(ObstacleArray, self.obstacles_topic, self._on_obstacles, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self._on_costmap, 1)
        if not self.path_csv:
            self.create_subscription(Path, self.global_path_topic, self._on_global_path, latched)
        else:
            self._load_path_csv()

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"local_planner up. rover {self.rover_length:.2f}x{self.rover_width:.2f} m "
            f"-> disc r={self.rover_radius:.2f} m. "
            f"collide = detected + {self.rover_radius:.2f} + {self.collision_margin:.2f} (hard, "
            f"triggers a detour); keepout = max(detected, {self.rover_radius:.2f}) + "
            f"{self.rover_radius:.2f} + {self.plan_margin:.2f} (preferred, costs "
            f"{self.inflation_weight}). search {self.search_resolution:.2f} m cells, "
            f"cost_weight {self.cost_weight}, fov {self.fov_mode} w={self.fov_weight} "
            f"({math.degrees(self.camera_half_fov)*2:.0f}deg / {self.camera_range:.1f} m). "
            f"costmap {self.costmap_topic}, route "
            f"{'CSV ' + os.path.basename(self.path_csv) if self.path_csv else self.global_path_topic}.")

    # ---------- inputs ----------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        # Speed, not just pose: "plan only while stopped" has to be checked
        # against the wheels actually having stopped, not against having sent a
        # zero command a moment ago.
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.last_odom = self._now()

    def _on_costmap(self, msg):
        self.costmap = {
            "res": msg.info.resolution,
            "w": msg.info.width,
            "h": msg.info.height,
            "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y,
            "frame": msg.header.frame_id or "map",
            "data": msg.data,
        }

    def _on_obstacles(self, msg):
        """Merge this frame into the running set of known obstacles.

        Spec point 6: the set persists. A cluster seen once stays a constraint
        on every later replan, because the detector only reports what is in
        frame right now and the act of driving around a rock is exactly what
        takes it out of frame."""
        now = self._now()
        for o in msg.obstacles:
            if float(o.radius) < self.min_obstacle_radius:
                continue
            # Hard: would we actually hit it, at its MEASURED size.
            collide = float(o.radius) + self.rover_radius + self.collision_margin
            # Soft: spec point 2's size floor -- the clearance we would prefer if
            # the ground allows it, on the assumption the camera undersized it.
            sized = max(float(o.radius), self.rover_radius)
            keepout = sized + self.rover_radius + self.plan_margin
            fresh = Obstacle2D(float(o.x), float(o.y), float(o.radius),
                               collide, keepout, now)
            for i, known in enumerate(self.memory):
                if known.centre_dist([o.x, o.y]) <= self.obstacle_match_radius:
                    self.memory[i] = fresh
                    break
            else:
                self.memory.append(fresh)
                self.get_logger().info(
                    f"new obstacle at ({o.x:.2f}, {o.y:.2f}) detected r={o.radius:.2f} m "
                    f"-> collide {collide:.2f} m (hard), keepout {keepout:.2f} m (preferred) "
                    f"({len(self.memory)} known)")
        self.last_obs = now
        self.obs_seen = True

    def _prune_memory(self):
        """Drop obstacles that are genuinely behind and off the route.

        Never on a timer. An obstacle stops being seen the moment the rover
        turns to avoid it, and forgetting it then is what lets a planner route
        straight back into the thing it just avoided."""
        if self.pos is None or not self.memory:
            return
        keep = []
        for o in self.memory:
            bearing = abs(wrap(math.atan2(o.y - self.pos[1], o.x - self.pos[0]) - self.yaw))
            behind = bearing > self.forget_behind_half
            far = o.centre_dist(self.pos) > o.keepout + self.forget_behind_m
            # Never drop something the detector is looking at right now. Without
            # this the prune and the next detection fight at loop rate: dropped
            # as "behind", re-added as "new", every single cycle.
            stale = (self._now() - o.last_seen) > self.obstacle_timeout_s
            on_route = self.route is not None and any(
                o.centre_dist(p) < o.keepout for p in self.route[self._route_idx_ahead():])
            # Negative evidence. If an obstacle sits inside the cone the
            # detector is looking down, within the range it can actually see,
            # and it has not been reported for confirm_after_s, then the camera
            # is looking straight at empty ground and saying so. Keeping it
            # anyway is how a phantom from an earlier session -- a bad camera, a
            # mis-tuned frame -- becomes a permanent no-go zone that no amount
            # of driving clears, because "behind and far" never becomes true for
            # something in front of a rover that is not allowed to move.
            unseen = (self._now() - o.last_seen) > self.confirm_after_s
            should_see = (o.centre_dist(self.pos) <= self.confirm_range
                          and self._in_view(self.pos, self.yaw, o))
            if unseen and should_see:
                self.get_logger().info(
                    f"obstacle at ({o.x:.2f}, {o.y:.2f}) is in view at "
                    f"{o.centre_dist(self.pos):.2f} m and has not been reported for "
                    f"{self.confirm_after_s:.1f}s -- disproved, dropped")
                continue
            if behind and far and stale and not on_route:
                self.get_logger().info(
                    f"obstacle at ({o.x:.2f}, {o.y:.2f}) is behind and off the route "
                    f"-- dropped ({len(self.memory) - 1} known)")
                continue
            keep.append(o)
        self.memory = keep
        self.obstacles = keep

    def _on_global_path(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if len(pts) < 2:
            return
        if self.route is None or len(pts) != len(self.route):
            self.get_logger().info(
                f"global route received on {self.global_path_topic}: {len(pts)} poses "
                f"(frame '{msg.header.frame_id}')")
        self.route = pts

    def _load_path_csv(self):
        import csv as _csv
        if self.path_frame not in ("map", "odom"):
            raise SystemExit(f"path_frame must be 'map' or 'odom', got '{self.path_frame}'")
        try:
            with open(self.path_csv, newline="") as f:
                self._csv_points = [[float(r["x_m"]), float(r["y_m"])]
                                    for r in _csv.DictReader(f)]
        except OSError as exc:
            raise SystemExit(f"cannot read path_csv '{self.path_csv}': {exc}")
        if len(self._csv_points) < 2:
            raise SystemExit(f"path_csv '{self.path_csv}' has fewer than 2 poses")

    def _ensure_csv_route(self):
        if self.route is not None or not self.path_csv:
            return
        if self.path_frame == "odom":
            self.route = [list(p) for p in self._csv_points]
            return
        tfm = self._lookup("odom", "map")
        if tfm is None:
            return
        tx, ty, yaw = tfm
        self.route = [transform_point(p, tx, ty, yaw) for p in self._csv_points]
        self.get_logger().info(f"global route placed in odom: {len(self.route)} poses")

    def _lookup(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), timeout=RclpyDuration(seconds=0.05))
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                quat_to_yaw(t.transform.rotation.x, t.transform.rotation.y,
                            t.transform.rotation.z, t.transform.rotation.w))

    # ---------- geometry / costs ----------

    def _route_idx_ahead(self):
        if self.route is None or self.pos is None:
            return 0
        return min(range(len(self.route)), key=lambda i: distance(self.pos, self.route[i]))

    def _point_clear(self, p):
        """Hard constraint: outside the COLLISION radius of every known obstacle.

        Hard means contact, not preference. Wanting more room than this is
        expressed as cost in _inflation_cost, so the rover can squeeze past a
        small rock when the alternative is a long way round -- and still prefers
        the long way round when the ground is free."""
        return all(o.centre_dist(p) >= o.collide for o in self.obstacles)

    def _inflation_cost(self, p):
        """Soft penalty for being inside an obstacle's preferred keepout.

        Ramps from 0 at the keepout edge to 1 at the collision edge, so the
        search gives obstacles a wide berth where it is cheap and shaves the
        margin only where it has to."""
        worst = 0.0
        for o in self.obstacles:
            if o.keepout <= o.collide:
                continue
            d = o.centre_dist(p)
            if d >= o.keepout:
                continue
            worst = max(worst, min(1.0, (o.keepout - d) / (o.keepout - o.collide)))
        return worst

    def _path_clear(self, pts, skip_from_start=0.4):
        travelled = 0.0
        for i, p in enumerate(pts):
            if i:
                travelled += distance(pts[i - 1], p)
            if travelled < skip_from_start:
                continue
            if not self._point_clear(p):
                return False
        return True

    def _cost01(self, p):
        """Costmap value at an odom point, normalised 0..1.

        No costmap, or a point off its edge, is treated as unknown rather than
        free -- an unmapped cell is not evidence of good ground."""
        cm = self.costmap
        if cm is None:
            return 0.0
        q = p
        if cm["frame"] != "odom":
            tf = self._lookup(cm["frame"], "odom")
            if tf is None:
                return self.unknown_cost
            q = transform_point(p, *tf)
        gx = int((q[0] - cm["ox"]) / cm["res"])
        gy = int((q[1] - cm["oy"]) / cm["res"])
        if not (0 <= gx < cm["w"] and 0 <= gy < cm["h"]):
            return self.unknown_cost
        v = cm["data"][gy * cm["w"] + gx]
        if v < 0:
            return self.unknown_cost
        return min(1.0, v / 100.0)

    def _in_view(self, at, heading, obs):
        """Would the rover, standing at `at` facing `heading`, still see `obs`?

        Spec 3's third constraint. The obstacle being avoided is the one thing
        that must not silently leave the frame while the rover is still beside
        it -- that is precisely when a detour needs the option to react."""
        if obs is None:
            return True
        d = obs.centre_dist(at)
        if d > self.camera_range:
            return False
        bearing = wrap(math.atan2(obs.y - at[1], obs.x - at[0]) - heading)
        # The near edge of the disc, not its centre: a wide rock is still in
        # frame when its middle is just outside the cone.
        half = math.asin(min(1.0, obs.radius / max(d, 1e-3))) if d > obs.radius else math.pi / 2
        return abs(bearing) - half <= self.camera_half_fov

    def _first_clear_after(self, route, i_block):
        """Reattachment index: past the obstacle, plus rejoin_margin, re-checked.

        The margin walk is re-checked because with two obstacles close together,
        walking clear of the first and blindly adding a metre lands the rejoin
        point inside the SECOND one's keepout, and then nothing can build a
        detour that ends there."""
        i = i_block
        while i < len(route) - 1:
            while i < len(route) - 1 and not self._point_clear(route[i]):
                i += 1
            travelled, j = 0.0, i
            while j < len(route) - 1 and travelled < self.rejoin_margin:
                travelled += distance(route[j], route[j + 1])
                j += 1
            if self._point_clear(route[j]):
                return j
            i = j
        return len(route) - 1

    def _blocked_index(self, path, start_idx=0):
        """First point on the ACTIVE path the rover would actually COLLIDE at.

        Detect everything, avoid only what is in the way: an obstacle beside the
        route that the rover clears on its current heading is remembered, costed
        and driven past, not detoured around. Only something whose collision
        disc actually overlaps the path stops the rover and triggers a replan."""
        travelled = 0.0
        for i in range(start_idx, len(path)):
            if not self._point_clear(path[i]):
                nearest = min(self.obstacles, key=lambda o: o.centre_dist(path[i]))
                return i, nearest
            if i + 1 < len(path):
                travelled += distance(path[i], path[i + 1])
                if travelled > self.horizon:
                    break
        return None, None

    # ---------- the search ----------

    def _search(self, start, goal, trigger):
        """A* over a lattice, minimising length x (1 + cost + out-of-view).

        Clearance is a hard constraint (blocked cells are simply not expanded);
        costmap cost and loss of view are terms in one objective, so the search
        can trade a slightly longer or slightly more expensive way round against
        keeping the obstacle in frame instead of ranking the two in sequence.

        Heading at a cell is taken as the bearing it was entered from, which is
        what the rover's heading will actually be there when driven -- so the
        view test is evaluated against the pose the rover will really hold."""
        res = self.search_resolution
        lo_x = min(start[0], goal[0]) - self.search_margin
        hi_x = max(start[0], goal[0]) + self.search_margin
        lo_y = min(start[1], goal[1]) - self.search_margin
        hi_y = max(start[1], goal[1]) + self.search_margin
        if trigger is not None:
            lo_x, hi_x = min(lo_x, trigger.x - self.search_margin), max(hi_x, trigger.x + self.search_margin)
            lo_y, hi_y = min(lo_y, trigger.y - self.search_margin), max(hi_y, trigger.y + self.search_margin)

        nx = int((hi_x - lo_x) / res) + 1
        ny = int((hi_y - lo_y) / res) + 1
        if nx * ny > self.max_search_cells:
            self.get_logger().warn(
                f"search window {nx}x{ny} exceeds max_search_cells "
                f"({self.max_search_cells}) -- raise search_resolution or lower "
                f"search_margin; refusing to plan rather than blocking the loop")
            return None, 0.0

        def to_cell(p):
            return (int(round((p[0] - lo_x) / res)), int(round((p[1] - lo_y) / res)))

        def to_world(c):
            return [lo_x + c[0] * res, lo_y + c[1] * res]

        s_cell, g_cell = to_cell(start), to_cell(goal)
        if not (0 <= g_cell[0] < nx and 0 <= g_cell[1] < ny):
            return None, 0.0

        # The start cell may sit inside a keepout -- the rover can already be
        # too close when the trigger fires. Expanding out of it is allowed;
        # entering one is not.
        blocked = {}

        def is_blocked(c):
            if c not in blocked:
                blocked[c] = not self._point_clear(to_world(c))
            return blocked[c]

        neigh = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        open_heap = [(0.0, s_cell)]
        came, gscore = {s_cell: None}, {s_cell: 0.0}
        goal_r = max(1, int(round(self.rejoin_margin / res / 2)))
        expanded = 0
        best = None

        while open_heap:
            _, cur = heapq.heappop(open_heap)
            expanded += 1
            if expanded > self.max_search_cells:
                break
            if abs(cur[0] - g_cell[0]) <= goal_r and abs(cur[1] - g_cell[1]) <= goal_r:
                best = cur
                break
            cx, cy = cur
            parent = came.get(cur)
            for dx, dy in neigh:
                nxt = (cx + dx, cy + dy)
                if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny):
                    continue
                if is_blocked(nxt):
                    continue
                w = to_world(nxt)
                step = res * math.hypot(dx, dy)
                heading = math.atan2(dy, dx) if parent is None else math.atan2(
                    nxt[1] - cy, nxt[0] - cx)
                pen = 0.0
                if self.fov_mode != "off" and trigger is not None:
                    if not self._in_view(w, heading, trigger):
                        if self.fov_mode == "hard":
                            continue
                        pen = self.fov_weight
                cost = step * (1.0 + self.cost_weight * self._cost01(w)
                               + self.inflation_weight * self._inflation_cost(w) + pen)
                tentative = gscore[cur] + cost
                if tentative < gscore.get(nxt, float("inf")):
                    gscore[nxt] = tentative
                    came[nxt] = cur
                    h = res * math.hypot(nxt[0] - g_cell[0], nxt[1] - g_cell[1])
                    heapq.heappush(open_heap, (tentative + h, nxt))

        if best is None:
            return None, 0.0

        cells = []
        c = best
        while c is not None:
            cells.append(to_world(c))
            c = came[c]
        cells.reverse()
        return cells, gscore[best]

    def _shortcut(self, pts):
        """String-pull the lattice path: drop any point whose neighbours can see
        each other through clear space. An 8-connected path is a staircase, and
        pure pursuit tracks a staircase by cutting every one of its corners."""
        if len(pts) < 3:
            return pts
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if self._segment_clear(pts[i], pts[j]):
                    break
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _segment_clear(self, a, b):
        n = max(2, int(distance(a, b) / (self.search_resolution / 2.0)))
        for k in range(n + 1):
            t = k / n
            if not self._point_clear([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]):
                return False
        return True

    def _densify(self, pts):
        out = []
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            n = max(1, int(distance(a, b) / self.resolution))
            for k in range(n):
                t = k / n
                out.append([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
        out.append(list(pts[-1]))
        return out

    def _plan_detour(self, trigger, i_block):
        """Full detour, current pose -> reattachment on the global route."""
        i_rejoin = self._first_clear_after(self.route, i_block)
        goal = self.route[i_rejoin]
        cells, cost = self._search(self.pos, goal, trigger)
        if cells is None:
            return None, i_rejoin, 0.0
        path = self._densify(self._shortcut(cells))
        # End exactly on the route point, so the follower reattaches rather than
        # stopping a cell short of it.
        path.append(list(goal))
        return path, i_rejoin, cost

    # ---------- outputs ----------

    def _publish(self, state, hold, plan=None, cmd=None, reason=""):
        self.state = state
        self.hold_pub.publish(Bool(data=bool(hold)))
        if plan is not None:
            self.plan_pub.publish(
                test_path_viz.path_msg(plan, self.get_clock().now().to_msg(), "odom"))
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        # null, not inf: bare `inf` is not valid JSON, so every consumer that
        # actually parses this topic threw away each sample taken while no
        # obstacle was known -- silently, because it looks like a value.
        clr = f"{self.clearance:.3f}" if self.clearance < 1e6 else "null"
        self.state_pub.publish(String(data=(
            f'{{"state":"{state}","hold":{str(bool(hold)).lower()},'
            f'"clearance_m":{clr},"known_obstacles":{len(self.obstacles)},'
            f'"plan_cost":{self.plan_cost:.2f},"replans":{self.replans},'
            f'"reason":"{reason}"}}')))
        if (state, reason) != self.last_logged:
            self.last_logged = (state, reason)
            line = state + (f" -- {reason}" if reason else "")
            if self.clearance < 1e6:
                line += f" (clearance {self.clearance:.2f} m)"
            if hold or state in (STATE_BACKOFF, STATE_STOPPING):
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
        self._publish_markers()

    def _publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, o in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id, m.header.stamp = "odom", stamp
            m.ns, m.id = "keepout", i
            m.type, m.action = Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.03
            m.color.a, m.color.r, m.color.g = 0.9, 1.0, 0.2
            if o is self.trigger_obs:
                m.color.g, m.color.b = 0.9, 0.2
            m.points = [Point(x=o.x + o.collide * math.cos(a),
                              y=o.y + o.collide * math.sin(a), z=0.0)
                        for a in [k * math.pi / 18 for k in range(37)]]
            arr.markers.append(m)
            # The preferred keepout, drawn faint: the ring the planner pays to
            # enter but is allowed to.
            k = Marker()
            k.header.frame_id, k.header.stamp = "odom", stamp
            k.ns, k.id = "preferred", i
            k.type, k.action = Marker.LINE_STRIP, Marker.ADD
            k.scale.x = 0.015
            k.color.a, k.color.r, k.color.g, k.color.b = 0.35, 1.0, 0.7, 0.2
            k.points = [Point(x=o.x + o.keepout * math.cos(a),
                              y=o.y + o.keepout * math.sin(a), z=0.0)
                        for a in [j * math.pi / 18 for j in range(37)]]
            arr.markers.append(k)
        if self.pos is not None:
            cone = Marker()
            cone.header.frame_id, cone.header.stamp = "odom", stamp
            cone.ns, cone.id = "fov", 0
            cone.type, cone.action = Marker.LINE_STRIP, Marker.ADD
            cone.scale.x = 0.02
            cone.color.a, cone.color.b, cone.color.g = 0.7, 1.0, 0.6
            pts = [(self.pos[0], self.pos[1])]
            steps = 12
            for k in range(steps + 1):
                a = self.yaw - self.camera_half_fov + (2 * self.camera_half_fov) * k / steps
                pts.append((self.pos[0] + self.camera_range * math.cos(a),
                            self.pos[1] + self.camera_range * math.sin(a)))
            pts.append((self.pos[0], self.pos[1]))
            cone.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in pts]
            arr.markers.append(cone)
        if arr.markers:
            self.markers_pub.publish(arr)

    def _live_clearance(self):
        if not self.obstacles:
            return float("inf"), None
        best, who = float("inf"), None
        for o in self.obstacles:
            c = o.surface_clearance(self.pos, self.rover_radius)
            if c < best:
                best, who = c, o
        return best, who

    def _ahead_clearance(self):
        if not self.obstacles:
            return float("inf"), None
        half = math.radians(self.backoff_cone_deg) / 2.0
        best, who = float("inf"), None
        for o in self.obstacles:
            if abs(wrap(math.atan2(o.y - self.pos[1], o.x - self.pos[0]) - self.yaw)) > half:
                continue
            c = o.surface_clearance(self.pos, self.rover_radius)
            if c < best:
                best, who = c, o
        return best, who

    def _request_replan(self, trigger, reason):
        """Spec 4/5/7: every trigger stops the rover first. Nothing is planned,
        and no new path is published, until the wheels have actually stopped."""
        self.trigger_obs = trigger
        self.pending_reason = reason
        self.stop_since = self._now()
        self.detour = None
        self._publish(STATE_STOPPING, True, reason=f"stopping to replan: {reason}")

    # ---------- cycle ----------

    def _tick(self):
        now = self._now()
        self._ensure_csv_route()

        if self.pos is None or self.last_odom is None:
            self._publish(STATE_BLOCKED, True, reason=f"no odometry on {self.odom_topic}")
            return
        if now - self.last_odom > self.odom_timeout_s:
            self._publish(STATE_BLOCKED, True, reason=f"odometry stale on {self.odom_topic}")
            return
        if self.route is None:
            src = ("map->odom for the CSV route" if self.path_csv
                   else f"global route on {self.global_path_topic}")
            self._publish(STATE_BLOCKED, True, reason=f"waiting for {src}")
            return
        if self.require_detector:
            if not self.obs_seen:
                self._publish(STATE_BLOCKED, True, reason=(
                    f"no obstacle detector on {self.obstacles_topic} "
                    f"(start the detection pipeline, or require_detector:=false)"))
                return
            if now - self.last_obs > self.obstacle_timeout_s:
                self._publish(STATE_BLOCKED, True,
                              reason=f"obstacle detector stale on {self.obstacles_topic}")
                return
        if self.require_costmap and self.costmap is None:
            self._publish(STATE_BLOCKED, True,
                          reason=f"no costmap on {self.costmap_topic}")
            return

        self._prune_memory()
        prev, self.prev_pos = self.prev_pos, list(self.pos)
        self.clearance, _ = self._live_clearance()
        ahead, nearest = self._ahead_clearance()

        # ---- STOPPING: hold until the rover has really stopped, then plan.
        if self.state == STATE_STOPPING:
            settled = self.speed <= self.stopped_speed
            timeout = (now - self.stop_since) > self.stop_settle_s
            if not settled and not timeout:
                self._publish(STATE_STOPPING, True,
                              reason=f"stopping to replan: {self.pending_reason}")
                return
            if not settled:
                self.get_logger().warn(
                    f"still moving at {self.speed:.2f} m/s after {self.stop_settle_s:.1f}s "
                    f"-- planning anyway; check the follower is honouring the hold")
            self._publish(STATE_PLANNING, True, reason=f"planning: {self.pending_reason}")
            self._do_plan()
            return

        # ---- optional last-resort reverse (off by default; spec is stop-and-replan)
        if self.state == STATE_BACKOFF:
            if prev is not None:
                self.backoff_total += distance(self.pos, prev)
            if self.backoff_total >= self.max_backoff_m and ahead < self.safe_distance:
                self._publish(STATE_BLOCKED, True, reason=(
                    f"reversed {self.backoff_total:.2f} m total without reaching "
                    f"{self.safe_distance:.2f} m -- stopping rather than backing "
                    f"further into ground no sensor has seen"))
                return
            if ahead >= self.safe_distance:
                self._request_replan(self.trigger_obs, "backed off to safe distance")
                return
            cmd = Twist()
            cmd.linear.x = -self.backoff_speed
            self._publish(STATE_BACKOFF, False, cmd=cmd,
                          reason=f"reversing to {self.safe_distance:.2f} m")
            return

        # ---- the active path: a detour if one is running, else the global route
        active = self.detour if self.detour is not None else self.route
        start_idx = 0 if self.detour is not None else self._route_idx_ahead()
        i_block, blocking = self._blocked_index(active, start_idx)

        if self.detour is not None:
            if distance(self.pos, self.detour[-1]) < self.rejoin_margin and i_block is None:
                if self._blocked_index(self.route, self.rejoin_idx)[0] is None:
                    self.get_logger().info(
                        f"reattached to the global route at index "
                        f"{self.rejoin_idx}/{len(self.route)}")
                    self.detour = None
                    self.trigger_obs = None
                    self._publish(STATE_FOLLOW, False, plan=self.route, reason="")
                    return
            if i_block is not None:
                # Spec 5: something now threatens the detour itself. Stop first.
                self._request_replan(blocking, "obstacle on the active detour")
                return
            self._publish(STATE_DETOUR, False, plan=self.detour,
                          reason="driving planned detour")
            return

        if i_block is None:
            self.trigger_obs = None
            self._publish(STATE_FOLLOW, False, plan=self.route, reason="")
            return

        if self.backoff_enabled and ahead < self.min_distance:
            self.backoff_total = 0.0
            self.trigger_obs = nearest
            self._publish(STATE_BACKOFF, False, reason=(
                f"obstacle ahead at {ahead:.2f} m, inside min_distance "
                f"{self.min_distance:.2f} m"))
            return

        self._request_replan(blocking, (
            f"obstacle at ({blocking.x:.2f}, {blocking.y:.2f}) "
            f"collide {blocking.collide:.2f} m on the global route"))

    def _do_plan(self):
        """Runs only from PLANNING, i.e. only with the rover stopped."""
        self.replans += 1
        i_block, blocking = self._blocked_index(self.route, self._route_idx_ahead())
        if i_block is None:
            self.trigger_obs = None
            self._publish(STATE_FOLLOW, False, plan=self.route,
                          reason="route clear on replan")
            return
        trigger = self.trigger_obs or blocking
        path, i_rejoin, cost = self._plan_detour(trigger, i_block)
        if path is None:
            self._publish(STATE_BLOCKED, True, reason=(
                f"no clear detour to the reattachment point past "
                f"({trigger.x:.2f}, {trigger.y:.2f}) -- {len(self.obstacles)} keepout(s) "
                f"in force"))
            return
        self.detour = path
        self.rejoin_idx = i_rejoin
        self.plan_cost = cost
        length = sum(distance(path[i], path[i + 1]) for i in range(len(path) - 1))
        self.get_logger().info(
            f"detour planned (replan #{self.replans}): {len(path)} poses, {length:.1f} m, "
            f"cost {cost:.1f}, clearing {len(self.obstacles)} keepout(s), "
            f"reattaching at index {i_rejoin}/{len(self.route)}")
        self._publish(STATE_DETOUR, False, plan=path, reason="driving planned detour")

    def stop(self):
        self.hold_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
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
