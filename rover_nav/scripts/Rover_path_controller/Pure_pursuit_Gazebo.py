#!/usr/bin/env python3
"""
Pure Pursuit Path Follower with Smooth 3D Obstacle Avoidance for Mars Rover

Subscribes to:
  - /odometry/filtered : Filtered rover state (map/odom frame)
  - /obstacles         : 3D bounding clusters from pcl_obstacle_detector

Publishes:
  - /rover_controller/cmd_vel          : TwistStamped for simulator/rover
  - /cmd_vel                           : Twist for Gazebo bridge
  - /pure_pursuit/path                 : Loaded global path
  - /pure_pursuit/local_avoidance_path : Real-time bypassed trajectory arc
  - /pure_pursuit/lookahead_marker     : Active steering target in RViz
"""

import csv
import math
import os
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from rover_nav.msg import ObstacleArray

# ═══════════════════════════════════════════════════════════════
# GLOBAL STATE
# ═══════════════════════════════════════════════════════════════

car_yaw = None
car_global_axis = None
active_obstacles = []
tf_buffer = None
tf_listener = None

# Static Occupancy Grid (Terrain & Hill/Slope Awareness)
static_map_data = None    # 2D boolean numpy array: True = occupied / hill
static_cost_data = None   # 2D float numpy array: [0.0, 1.0] continuous terrain cost
map_origin_x = -18.508737564086914
map_origin_y = -7.88718318939209
map_resolution = 0.1
map_width = 0
map_height = 0


def find_package_resource(filename, subfolder="maps"):
    """Locate package resource via ROS2 share dir or relative workspace fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory("rover_nav")
        candidate = os.path.join(pkg, subfolder, filename)
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.abspath(os.path.join(script_dir, "..", "..", subfolder, filename)),
        os.path.abspath(os.path.join(script_dir, "..", "..", "..", "marsyard", filename)),
        os.path.abspath(os.path.join(script_dir, "..", "..", "..", "rover_nav", subfolder, filename)),
        os.path.abspath(os.path.join(os.getcwd(), filename)),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0] if candidates else filename


PATH_CSV = find_package_resource("marsyard2026_tour_waypoints.csv", "maps")
path = []
current_target_idx = 0

vel_pub = None
vel_twist_pub = None
avoidance_path_pub = None
lookahead_marker_pub = None
path_pub = None
smoothed_plan_pub = None

# ── Rover Physical Kinematics & Geometry ───────────────────────────────────────
TRACK_WIDTH = 0.7095        # Geometric wheel separation / track width (m)
WHEEL_BASE = 0.5520         # Longitudinal wheelbase between front & rear axles (m)
WHEEL_RADIUS = 0.1100       # Outer tire radius (m)
MAX_WHEEL_SPEED = 1.00      # Max individual wheel linear speed limit (m/s)

HALF_TRACK_WIDTH = TRACK_WIDTH / 2.0                        # 0.3548 m
HALF_WHEEL_BASE = WHEEL_BASE / 2.0                          # 0.2760 m
ROVER_FRONT_EXTENT = HALF_WHEEL_BASE + WHEEL_RADIUS         # 0.3860 m
ROVER_REAR_EXTENT = HALF_WHEEL_BASE + WHEEL_RADIUS          # 0.3860 m
ROBOT_RADIUS = math.hypot(HALF_WHEEL_BASE, HALF_TRACK_WIDTH) # 0.450 m circumscribed radius

# ── Pure Pursuit Controller Parameters ────────────────────────────────────────
LA_GLOBAL = 0.80        # Reduced lookahead distance for tight global path tracking (m)
LA_DETOUR = 1.1       # Expanded lookahead distance for smooth detour curves (m)
BASE_VELOCITY = 0.55        # Normal cruise speed (m/s)
MIN_VELOCITY = 0.22         # Minimum forward speed during tight turns (m/s)
GOAL_TOLERANCE = 0.40      # Arrival radius to final goal (m)
MAX_CURVATURE = 1.40        # Smooth maximum curvature limit (1/m)

# ── Obstacle Avoidance Parameters ─────────────────────────────────────────────
SAFETY_MARGIN = 0.45        # Extra lateral buffer on top of robot+obstacle radii (m)
SLOWDOWN_DIST = 1.80        # Begin decelerating when obstacle is within 1.8m (m)
STOP_DIST = 0.40            # Emergency halt distance (m)
CLEAR_PAST_DISTANCE = 1.4   # Longitudinal hold distance post-obstacle (m)
AVOIDANCE_LOOKAHEAD_FACTOR = 2.2 #Scan up to LA * 2.5 = 3.0 m ahead
ENABLE_AVOIDANCE = True

# ── Dynamic Local Detour Parameters & State ───────────────────────────────────
DETOUR_SCAN_MIN_RANGE = 0.85    # Minimum obstacle detection range to trigger detour (m)
DETOUR_SCAN_MAX_RANGE = 2.30    # Maximum obstacle detection range to trigger detour (m)
DETOUR_CRUISE_VELOCITY = 0.35   # Controlled, safe forward speed during detour maneuver (m/s)

# Spawn pose parameters (for odom->map projection fallback)
SPAWN_X = 0.0
SPAWN_Y = 0.0
SPAWN_YAW = 1.5707963267948966
tf_connected = False

active_detour_path = None         # List of [x, y] waypoints for active dynamic detour
detour_target_idx = 0             # Progress waypoint index along active_detour_path
detour_rejoin_global_idx = 0      # Waypoint index on global path where detour merges back
detour_obstacle_info = None       # [gx, gy, radius, side]

# Smooth avoidance & rate limiting filters
active_lateral_offset = 0.0
bypass_side_locked = 0.0
bypassing_obs_pos = None    # [gx, gy, radius] of obstacle currently being bypassed
last_cmd_linear = 0.0
last_cmd_angular = 0.0

target_idx_initialized = False
_last_rover_pos = None
_stuck_ticks = 0
_csv_mtime = 0.0
_last_diag_log_t = 0.0


# ═══════════════════════════════════════════════════════════════
# PATH LOADING & MATH HELPERS
# ═══════════════════════════════════════════════════════════════

def load_path(csv_path):
    pts = []
    if not os.path.exists(csv_path):
        return pts
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                pts.append([float(row[0]), float(row[1])])
            except ValueError:
                continue
    return pts


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def point_global_to_local(point_global, yaw, rover_pos):
    """Transform global point into rover-centric frame (+X forward, +Y left)."""
    dx = point_global[0] - rover_pos[0]
    dy = point_global[1] - rover_pos[1]
    lx =  dx * math.cos(yaw) + dy * math.sin(yaw)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return lx, ly


def point_local_to_global(local_x, local_y, yaw, rover_pos):
    """Transform rover local coordinate into global map frame."""
    gx = rover_pos[0] + local_x * math.cos(yaw) - local_y * math.sin(yaw)
    gy = rover_pos[1] + local_x * math.sin(yaw) + local_y * math.cos(yaw)
    return gx, gy


def load_static_map(custom_yaml=None):
    global static_map_data, static_cost_data, map_origin_x, map_origin_y, map_resolution, map_width, map_height
    yaml_candidates = []
    if custom_yaml:
        yaml_candidates.append(os.path.expanduser(custom_yaml))
    default_yaml = find_package_resource("marsyard2026_occupancy.yaml", "maps")
    if default_yaml not in yaml_candidates:
        yaml_candidates.append(default_yaml)

    for ypath in yaml_candidates:
        if os.path.exists(ypath):
            try:
                import yaml
                from PIL import Image
                with open(ypath) as f:
                    meta = yaml.safe_load(f)
                img_path = os.path.join(os.path.dirname(ypath), meta["image"])
                if os.path.exists(img_path):
                    # flipud makes row 0 = origin_y (bottom-left coordinate origin, matching ROS OccupancyGrid)
                    pgm = np.flipud(np.array(Image.open(img_path), dtype=float))
                    map_height, map_width = pgm.shape
                    map_origin_x = float(meta["origin"][0])
                    map_origin_y = float(meta["origin"][1])
                    map_resolution = float(meta["resolution"])
                    # PGM: 254 is free flat ground (cost ~ 0), 0 is steep hill / obstacle (cost = 1.0)
                    static_cost_data = np.clip((255.0 - pgm) / 255.0, 0.0, 1.0)
                    static_map_data = (pgm < 89)
                    return True
            except Exception:
                pass
    return False


def get_world_cell_cost(gx, gy):
    """Returns continuous terrain elevation cost in [0.0, 1.0]. Lower is flatter/safer."""
    global static_cost_data, map_origin_x, map_origin_y, map_resolution, map_width, map_height
    if static_cost_data is None:
        return 0.0
    col = int(round((gx - map_origin_x) / map_resolution))
    row = int(round((gy - map_origin_y) / map_resolution))
    if 0 <= row < map_height and 0 <= col < map_width:
        return float(static_cost_data[row, col])
    return 1.0  # Out-of-bounds


def is_world_cell_occupied(gx, gy, inflation_margin=0.0):
    global static_map_data, map_origin_x, map_origin_y, map_resolution, map_width, map_height
    if static_map_data is None:
        return False
    col = int(round((gx - map_origin_x) / map_resolution))
    row = int(round((gy - map_origin_y) / map_resolution))
    if inflation_margin > 0.0:
        cells = int(math.ceil(inflation_margin / map_resolution))
        r_min, r_max = max(0, row - cells), min(map_height, row + cells + 1)
        c_min, c_max = max(0, col - cells), min(map_width, col + cells + 1)
        region = static_map_data[r_min:r_max, c_min:c_max]
        if region.size > 0 and region.any():
            return True
        return False
    if 0 <= row < map_height and 0 <= col < map_width:
        return bool(static_map_data[row, col])
    return True


def dist_point_to_segment(px, py, ax, ay, bx, by):
    """Calculates perpendicular distance and 2D cross product from point P to segment A->B."""
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-8:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    dist = math.hypot(px - proj_x, py - proj_y)
    cross = dx * (py - ay) - dy * (px - ax)
    return dist, cross


def get_obstacle_path_interaction(gx, gy, current_idx, max_lookahead_pts=35):
    """
    Evaluates true distance from obstacle centroid (gx, gy) to upcoming global path.
    Prevents off-path hills, mounds, and roadside obstacles from triggering avoidance.
    """
    if not path or current_idx >= len(path) - 1:
        return 999.0, 0.0, 999.0
    min_dist = 999.0
    best_cross = 0.0
    accum_dist = 0.0
    best_along_dist = 0.0
    end_idx = min(len(path) - 1, current_idx + max_lookahead_pts)
    for i in range(current_idx, end_idx):
        p1 = path[i]
        p2 = path[i + 1]
        seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        d, cross = dist_point_to_segment(gx, gy, p1[0], p1[1], p2[0], p2[1])
        if d < min_dist:
            min_dist = d
            best_cross = cross
            best_along_dist = accum_dist
        accum_dist += seg_len
    return min_dist, best_cross, best_along_dist


def get_map_clearance(gx, gy, dir_x, dir_y, max_dist=2.5, step=0.06):
    """Returns distance in meters along (dir_x, dir_y) until hitting occupied/hill terrain."""
    l = math.hypot(dir_x, dir_y)
    if l < 1e-6:
        return 0.0
    dx, dy = (dir_x / l) * step, (dir_y / l) * step
    dist = 0.0
    curr_x, curr_y = gx, gy
    while dist < max_dist:
        curr_x += dx
        curr_y += dy
        dist += step
        if is_world_cell_occupied(curr_x, curr_y):
            return max(0.0, dist - step)
    return max_dist


# ═══════════════════════════════════════════════════════════════
# CALLBACKS
# ═══════════════════════════════════════════════════════════════

def map_callback(msg: OccupancyGrid):
    global static_map_data, static_cost_data, map_origin_x, map_origin_y, map_resolution, map_width, map_height
    map_width = msg.info.width
    map_height = msg.info.height
    map_resolution = msg.info.resolution
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    raw = np.array(msg.data, dtype=float).reshape((map_height, map_width))
    static_cost_data = np.clip(raw / 100.0, 0.0, 1.0)
    # Occupied if cost > 50 (includes hills, obstacles, and inflation zones)
    static_map_data = (raw > 50)


def obstacles_callback(msg: ObstacleArray):
    global active_obstacles
    active_obstacles = msg.obstacles


def odom_callback(msg: Odometry):
    global car_yaw, car_global_axis, tf_connected
    # Always compute rover pose in map frame from odometry
    ox = msg.pose.pose.position.x
    oy = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    _, _, oyaw = r.as_euler('xyz')

    # Convert odom frame to map frame: map = spawn_pose + odom_in_spawn_frame
    cos_s = math.cos(SPAWN_YAW)
    sin_s = math.sin(SPAWN_YAW)
    gx = SPAWN_X + (ox * cos_s - oy * sin_s)
    gy = SPAWN_Y + (ox * sin_s + oy * cos_s)
    gyaw = oyaw + SPAWN_YAW

    # If TF has not yet resolved or has dropped, continuously maintain pose from odom
    if not tf_connected or car_global_axis is None or car_yaw is None:
        car_global_axis = [gx, gy]
        car_yaw = gyaw


# ═══════════════════════════════════════════════════════════════
# DYNAMIC LOCAL DETOUR PATH GENERATION & TRACKING
# ═══════════════════════════════════════════════════════════════

def cubic_bezier(p0, p1, p2, p3, n_points=25):
    """Evaluates n_points along a cubic Bezier curve."""
    pts = []
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    p3 = np.array(p3, dtype=float)
    for t in np.linspace(0.0, 1.0, n_points):
        u = 1.0 - t
        pt = (u**3) * p0 + 3.0 * (u**2) * t * p1 + 3.0 * u * (t**2) * p2 + (t**3) * p3
        pts.append([float(pt[0]), float(pt[1])])
    return pts


def generate_candidate_detour_path(node, obs, side):
    """
    Generates a smooth, kinematically-safe C1 Bezier dynamic detour path in global map frame
    starting directly from the rover's current pose, arcing past the obstacle with full chassis clearance,
    and smoothly merging back onto the global path.
    """
    global path, current_target_idx, car_global_axis, car_yaw
    if not path or car_global_axis is None or car_yaw is None:
        return None, 0

    gx, gy = point_local_to_global(obs.x, obs.y, car_yaw, car_global_axis)
    r_obs = obs.radius

    # 1. Find closest waypoint to obstacle on global path
    min_d = 999.0
    k_obs = current_target_idx
    for i in range(current_target_idx, min(len(path), current_target_idx + 80)):
        d = distance([gx, gy], path[i])
        if d < min_d:
            min_d = d
            k_obs = i

    # Global path orientation at obstacle
    if k_obs < len(path) - 1:
        p_curr = path[k_obs]
        p_next = path[k_obs + 1]
        p_dx = p_next[0] - p_curr[0]
        p_dy = p_next[1] - p_curr[1]
    elif k_obs > 0:
        p_prev = path[k_obs - 1]
        p_curr = path[k_obs]
        p_dx = p_curr[0] - p_prev[0]
        p_dy = p_curr[1] - p_prev[1]
    else:
        p_dx, p_dy = math.cos(car_yaw), math.sin(car_yaw)

    p_len = math.hypot(p_dx, p_dy)
    if p_len > 1e-4:
        tx, ty = p_dx / p_len, p_dy / p_len
    else:
        tx, ty = math.cos(car_yaw), math.sin(car_yaw)

    # Unit normal pointing towards chosen bypass side (+1 = left, -1 = right)
    nx = -ty * side
    ny =  tx * side

    # Projection of obstacle centroid onto path tangent through p_curr
    obs_dx = gx - p_curr[0]
    obs_dy = gy - p_curr[1]
    obs_along = obs_dx * tx + obs_dy * ty
    obs_lat = obs_dx * nx + obs_dy * ny  # Positive if obstacle is shifted towards bypass side

    # Required safe lateral clearance from obstacle centroid
    # Half track width (0.355) + wheel radius (0.110) + obstacle radius + compact safe margin (0.22m)
    req_clearance = HALF_TRACK_WIDTH + WHEEL_RADIUS + r_obs + 0.22
    total_lateral_offset = max(req_clearance, obs_lat + req_clearance)
    total_lateral_offset = max(0.80, min(1.05, total_lateral_offset))

    # Detour Apex placed parallel to obstacle centroid
    apex_proj_x = p_curr[0] + tx * obs_along
    apex_proj_y = p_curr[1] + ty * obs_along
    apex_x = apex_proj_x + nx * total_lateral_offset
    apex_y = apex_proj_y + ny * total_lateral_offset

    # Check terrain map clearance at apex (avoid steep mounds/hills)
    if is_world_cell_occupied(apex_x, apex_y):
        for f in [0.85, 0.70, 0.55]:
            test_x = apex_proj_x + nx * (total_lateral_offset * f)
            test_y = apex_proj_y + ny * (total_lateral_offset * f)
            if not is_world_cell_occupied(test_x, test_y):
                apex_x, apex_y = test_x, test_y
                break

    # Scan for consecutive obstacles (e.g. 2nd obstacle) along upcoming path up to 60 waypoints ahead
    furthest_k_obs = k_obs
    if active_obstacles:
        for other_obs in active_obstacles:
            ogx, ogy = point_local_to_global(other_obs.x, other_obs.y, car_yaw, car_global_axis)
            d_other, _, along_other = get_obstacle_path_interaction(ogx, ogy, current_target_idx)
            if d_other < (HALF_TRACK_WIDTH + WHEEL_RADIUS + other_obs.radius + 0.15):
                # Search waypoint index closest to other obstacle
                for k in range(k_obs, min(len(path), k_obs + 60)):
                    if distance([ogx, ogy], path[k]) < (other_obs.radius + 0.45):
                        if k > furthest_k_obs:
                            furthest_k_obs = k

    # Rejoin point: compact ~1.40m past the obstacle to quickly realign onto the hill-aware global path
    rejoin_dist = 1.40
    acc = 0.0
    k_rejoin = min(len(path) - 1, furthest_k_obs + 1)
    for i in range(furthest_k_obs, len(path) - 1):
        acc += distance(path[i], path[i + 1])
        if acc >= rejoin_dist:
            k_rejoin = i + 1
            break
    rejoin_pt = path[k_rejoin]

    # Rejoin tangent (matches global path heading seamlessly at merge point)
    if k_rejoin < len(path) - 1:
        rj_dx = path[k_rejoin + 1][0] - path[k_rejoin][0]
        rj_dy = path[k_rejoin + 1][1] - path[k_rejoin][1]
    else:
        rj_dx, rj_dy = tx, ty
    rj_len = math.hypot(rj_dx, rj_dy)
    if rj_len > 1e-4:
        rtx, rty = rj_dx / rj_len, rj_dy / rj_len
    else:
        rtx, rty = tx, ty

    # Segment 1: Rover Current Pose -> Apex (Compact S-curve entry)
    p0 = car_global_axis
    d1 = distance(p0, [apex_x, apex_y])
    p1 = [p0[0] + (tx * 0.70 + nx * 0.30) * (d1 * 0.38), p0[1] + (ty * 0.70 + ny * 0.30) * (d1 * 0.38)]
    p3 = [apex_x, apex_y]
    p2 = [p3[0] - tx * (d1 * 0.38), p3[1] - ty * (d1 * 0.38)]
    seg1 = cubic_bezier(p0, p1, p2, p3, n_points=25)

    # Segment 2: Apex -> Rejoin Point (Direct, tangent-aligned merge back onto global path)
    q0 = [apex_x, apex_y]
    d2 = distance(q0, rejoin_pt)
    q1 = [q0[0] + tx * (d2 * 0.38), q0[1] + ty * (d2 * 0.38)]
    q3 = rejoin_pt
    q2 = [q3[0] - rtx * (d2 * 0.38), q3[1] - rty * (d2 * 0.38)]
    seg2 = cubic_bezier(q0, q1, q2, q3, n_points=25)

    detour_waypoints = seg1 + seg2[1:]
    return detour_waypoints, k_rejoin


def generate_dynamic_detour_path(node, obs, side):
    return generate_candidate_detour_path(node, obs, side)


def evaluate_candidate_detour(detour_waypoints, critical_obs):
    """
    Evaluates traversability and total terrain cost for a candidate detour path.
    Returns (is_traversable, total_cost, max_cost).
    Lower cost = flatter, safer ground (away from hills/steep mounds).
    """
    if not detour_waypoints or len(detour_waypoints) < 5:
        return False, 99999.0, 1.0

    total_cost = 0.0
    max_cost = 0.0

    for pt in detour_waypoints:
        cost = get_world_cell_cost(pt[0], pt[1])
        # If cell is inside steep hill or occupied obstacle
        if cost > 0.55 or is_world_cell_occupied(pt[0], pt[1]):
            return False, 99999.0, 1.0

        # Check inflation margin
        if is_world_cell_occupied(pt[0], pt[1], inflation_margin=0.20):
            total_cost += 10.0  # Near boundary penalty

        total_cost += cost
        if cost > max_cost:
            max_cost = cost

    avg_cost = total_cost / len(detour_waypoints)
    return True, avg_cost, max_cost


def select_best_detour(node, critical_obs):
    """
    Generates candidate detours for BOTH Left (+1.0) and Right (-1.0) sides,
    evaluates continuous terrain elevation cost along both paths,
    and ALWAYS selects the candidate with lower traversable terrain cost.
    """
    cand_left_pts, k_rejoin_left = generate_candidate_detour_path(node, critical_obs, side=1.0)
    cand_right_pts, k_rejoin_right = generate_candidate_detour_path(node, critical_obs, side=-1.0)

    trav_left, cost_left, max_cost_left = evaluate_candidate_detour(cand_left_pts, critical_obs)
    trav_right, cost_right, max_cost_right = evaluate_candidate_detour(cand_right_pts, critical_obs)

    node.get_logger().info(
        f"🔍 Detour Cost Check: LEFT -> Trav={trav_left}, Cost={cost_left:.3f} | "
        f"RIGHT -> Trav={trav_right}, Cost={cost_right:.3f}"
    )

    if trav_left and not trav_right:
        return cand_left_pts, k_rejoin_left, 1.0
    elif trav_right and not trav_left:
        return cand_right_pts, k_rejoin_right, -1.0
    elif trav_left and trav_right:
        # Both sides are traversable: strictly pick the lower cost side!
        if cost_left < cost_right - 0.005:
            return cand_left_pts, k_rejoin_left, 1.0
        elif cost_right < cost_left - 0.005:
            return cand_right_pts, k_rejoin_right, -1.0
        else:
            # If costs are practically identical, pick natural geometric opposite
            chosen_side = -1.0 if critical_obs.y > 0.0 else 1.0
            if chosen_side > 0:
                return cand_left_pts, k_rejoin_left, 1.0
            else:
                return cand_right_pts, k_rejoin_right, -1.0
    else:
        # If neither is fully free, try the one with lower max cost
        if max_cost_left <= max_cost_right:
            return cand_left_pts, k_rejoin_left, 1.0
        else:
            return cand_right_pts, k_rejoin_right, -1.0


# ═══════════════════════════════════════════════════════════════
# ROBUST LOOKAHEAD SELECTION
# ═══════════════════════════════════════════════════════════════

def update_and_get_lookahead_point(lookahead_dist=None):
    """
    Finds the closest point on the global path and projects lookahead forward
    along the path arc length. Returns (target_x, target_y, norm_x, norm_y).
    """
    global current_target_idx, LA_GLOBAL
    effective_la = lookahead_dist if lookahead_dist is not None else LA_GLOBAL

    # 1. Search locally ahead for the closest path waypoint
    search_start = max(0, current_target_idx - 5)
    search_end = min(len(path), current_target_idx + 60)
    
    closest_idx = search_start
    min_dist = distance(car_global_axis, path[closest_idx])
    for i in range(search_start + 1, search_end):
        d = distance(car_global_axis, path[i])
        if d < min_dist:
            min_dist = d
            closest_idx = i

    # Advance progress along path monotonically
    current_target_idx = max(current_target_idx, closest_idx)

    # 2. Project lookahead point forward along path arc length from current_target_idx
    accumulated_dist = 0.0
    lookahead_idx = len(path) - 1
    for i in range(current_target_idx, len(path) - 1):
        accumulated_dist += distance(path[i], path[i + 1])
        if accumulated_dist >= effective_la:
            lookahead_idx = i + 1
            break

    target_pt = path[lookahead_idx]

    # Compute path unit normal (pointing to Left of path)
    idx_next = min(len(path) - 1, lookahead_idx + 1)
    if idx_next == lookahead_idx and lookahead_idx > 0:
        idx_prev = lookahead_idx - 1
        tan_x = path[lookahead_idx][0] - path[idx_prev][0]
        tan_y = path[lookahead_idx][1] - path[idx_prev][1]
    else:
        tan_x = path[idx_next][0] - path[lookahead_idx][0]
        tan_y = path[idx_next][1] - path[lookahead_idx][1]

    tan_len = math.hypot(tan_x, tan_y)
    if tan_len > 1e-5:
        norm_x = -tan_y / tan_len
        norm_y =  tan_x / tan_len
    else:
        norm_x = 0.0
        norm_y = 1.0

    return target_pt[0], target_pt[1], norm_x, norm_y


def publish_cmd_vel(node, linear, angular):
    global last_cmd_linear, last_cmd_angular

    if linear == 0.0 and angular == 0.0:
        # Immediate halt on stop/blocked command
        last_cmd_linear = 0.0
        last_cmd_angular = 0.0
        smooth_linear = 0.0
        smooth_angular = 0.0
    else:
        dt = 0.1
        max_dlinear  = 1.5 * dt  # 0.15 m/s per tick
        max_dangular = 3.5 * dt  # 0.35 rad/s per tick

        smooth_linear  = float(np.clip(linear,  last_cmd_linear  - max_dlinear,  last_cmd_linear  + max_dlinear))
        smooth_angular = float(np.clip(angular, last_cmd_angular - max_dangular, last_cmd_angular + max_dangular))

        last_cmd_linear  = smooth_linear
        last_cmd_angular = smooth_angular

    # 1. TwistStamped for controller / mock sim
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_footprint"
    msg.twist.linear.x = smooth_linear
    msg.twist.angular.z = smooth_angular
    vel_pub.publish(msg)

    # 2. Twist for Gazebo bridge
    twist_msg = Twist()
    twist_msg.linear.x = smooth_linear
    twist_msg.angular.z = smooth_angular
    vel_twist_pub.publish(twist_msg)


def publish_diagnostics(node, lookahead_gx, lookahead_gy):
    global avoidance_path_pub, lookahead_marker_pub, active_detour_path

    # 1. Publish active detour path or lookahead visualization line
    arc_msg = Path()
    arc_msg.header.frame_id = "map"
    arc_msg.header.stamp = node.get_clock().now().to_msg()

    if active_detour_path is not None and len(active_detour_path) > 1:
        for pt in active_detour_path:
            p = PoseStamped()
            p.header = arc_msg.header
            p.pose.position.x = float(pt[0])
            p.pose.position.y = float(pt[1])
            p.pose.position.z = 0.15
            p.pose.orientation.w = 1.0
            arc_msg.poses.append(p)
    elif car_global_axis is not None:
        steps = 10
        for t in np.linspace(0.0, 1.0, steps):
            p = PoseStamped()
            p.header = arc_msg.header
            p.pose.position.x = float(car_global_axis[0] * (1.0 - t) + lookahead_gx * t)
            p.pose.position.y = float(car_global_axis[1] * (1.0 - t) + lookahead_gy * t)
            p.pose.position.z = 0.15
            p.pose.orientation.w = 1.0
            arc_msg.poses.append(p)

    avoidance_path_pub.publish(arc_msg)

    # 2. Lookahead target sphere marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "pure_pursuit_target"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(lookahead_gx)
    marker.pose.position.y = float(lookahead_gy)
    marker.pose.position.z = 0.2
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    marker.color.r = 0.0
    marker.color.g = 0.85
    marker.color.b = 1.0
    marker.color.a = 0.9
    lookahead_marker_pub.publish(marker)


# ═══════════════════════════════════════════════════════════════
# MAIN CONTROL LOOP
# ═══════════════════════════════════════════════════════════════

def control_loop(node: Node):
    try:
        _control_loop_inner(node)
    except Exception as exc:
        import traceback
        node.get_logger().error(
            f"💥 control_loop EXCEPTION (will continue):\n{traceback.format_exc()}"
        )


def _control_loop_inner(node: Node):
    global car_yaw, car_global_axis, LA_GLOBAL, LA_DETOUR, current_target_idx, target_idx_initialized
    global _stuck_ticks, path, _csv_mtime, _last_rover_pos, last_cmd_linear, last_cmd_angular
    global active_detour_path, detour_target_idx, detour_rejoin_global_idx, detour_obstacle_info
    global tf_buffer, _last_diag_log_t, tf_connected

    la_val = node.get_parameter("lookahead_distance").value
    if la_val is not None:
        LA_GLOBAL = float(la_val)

    # Update rover pose in map frame from TF (map -> base_footprint / base_link)
    tf_success = False
    if tf_buffer is not None:
        for child_frame in ["base_footprint", "base_link"]:
            try:
                t = tf_buffer.lookup_transform("map", child_frame, rclpy.time.Time())
                car_global_axis = [t.transform.translation.x, t.transform.translation.y]
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                r = R.from_quat([qx, qy, qz, qw])
                _, _, car_yaw = r.as_euler('xyz')
                tf_success = True
                tf_connected = True
                break
            except Exception:
                pass
    if not tf_success:
        tf_connected = False

    # Hot-reload if CSV file changes
    if os.path.exists(PATH_CSV):
        mtime = os.path.getmtime(PATH_CSV)
        if mtime > _csv_mtime:
            new_path = load_path(PATH_CSV)
            if len(new_path) > 5 and new_path != path:
                path = new_path
                current_target_idx = 0
                target_idx_initialized = False
                _last_rover_pos = None
                _stuck_ticks = 0
                _csv_mtime = mtime
                active_detour_path = None
                detour_obstacle_info = None
                node.get_logger().info(f"🔄 Hot-reloaded plan: {len(path)} waypoints from {os.path.basename(PATH_CSV)}")
            else:
                _csv_mtime = mtime

    now_sec = node.get_clock().now().nanoseconds / 1e9
    if car_yaw is None or car_global_axis is None or not path:
        publish_cmd_vel(node, 0.0, 0.0)
        if now_sec - _last_diag_log_t > 1.5:
            _last_diag_log_t = now_sec
            node.get_logger().warn(
                f"⏳ [PurePursuit] Waiting for data: Pose={'OK' if car_global_axis else 'NONE'} "
                f"(Yaw={'OK' if car_yaw is not None else 'NONE'}) | Path points={len(path)} from {os.path.basename(PATH_CSV)}"
            )
        return

    # Snap to closest path waypoint on first tick
    if not target_idx_initialized:
        closest_idx = 0
        min_d = distance(car_global_axis, path[0])
        for i in range(1, min(len(path), 60)):
            d = distance(car_global_axis, path[i])
            if d < min_d:
                min_d = d
                closest_idx = i
        current_target_idx = closest_idx
        target_idx_initialized = True
        node.get_logger().info(
            f"📍 Target initialized at path index {closest_idx} "
            f"(rover pos [{car_global_axis[0]:.2f}, {car_global_axis[1]:.2f}], yaw {car_yaw:.3f} rad / {math.degrees(car_yaw):.1f}°)"
        )

    # Check for goal arrival (strictly prevent false positive on closed-loop paths at startup)
    if active_detour_path is None and len(path) > 0:
        traversed_enough = (current_target_idx >= max(1, int(len(path) * 0.70))) if len(path) > 10 else (current_target_idx >= len(path) - 1)
        near_end_idx = current_target_idx >= max(0, len(path) - 4)
        dist_to_goal = distance(car_global_axis, path[-1])
        if traversed_enough and near_end_idx and dist_to_goal < GOAL_TOLERANCE:
            publish_cmd_vel(node, 0.0, 0.0)
            node.get_logger().info("🏁 Goal reached! Holding position.", throttle_duration_sec=3.0)
            return

    # ── Step 1: Scan for blocking obstacles & generate dynamic detour ────────
    lookahead_gx, lookahead_gy = 0.0, 0.0
    speed_scale = 1.0

    need_detour_plan = False
    critical_obs = None
    min_front = 999.0

    if ENABLE_AVOIDANCE and active_obstacles:
        if active_detour_path is None:
            # Check for obstacles blocking upcoming global path
            for obs in active_obstacles:
                ox, oy = obs.x, obs.y
                r_obs = obs.radius
                dist_obs = math.hypot(ox, oy)
                if dist_obs < DETOUR_SCAN_MIN_RANGE or dist_obs > DETOUR_SCAN_MAX_RANGE or ox < 0.1:
                    continue

                gx, gy = point_local_to_global(ox, oy, car_yaw, car_global_axis)
                d_to_path, cross, along_dist = get_obstacle_path_interaction(gx, gy, current_target_idx)
                req_clearance = HALF_TRACK_WIDTH + WHEEL_RADIUS + r_obs + 0.20

                # Only consider obstacle if it is actually sitting on the upcoming global path ahead
                if d_to_path < req_clearance and 0.15 < along_dist < DETOUR_SCAN_MAX_RANGE and ox < min_front:
                    min_front = ox
                    critical_obs = obs
                    need_detour_plan = True
        else:
            # While executing an active detour, check if a 2nd obstacle blocks the remaining trajectory
            for obs in active_obstacles:
                ox, oy = obs.x, obs.y
                r_obs = obs.radius
                dist_obs = math.hypot(ox, oy)
                if dist_obs < DETOUR_SCAN_MIN_RANGE or dist_obs > DETOUR_SCAN_MAX_RANGE or ox < 0.1:
                    continue

                gx, gy = point_local_to_global(ox, oy, car_yaw, car_global_axis)
                # Ignore the 1st obstacle we are already bypassing
                if detour_obstacle_info is not None:
                    prev_gx, prev_gy = detour_obstacle_info[0], detour_obstacle_info[1]
                    if distance([gx, gy], [prev_gx, prev_gy]) < 0.45:
                        continue

                # Check if this new obstacle intersects remaining detour path
                min_d_detour = 999.0
                if detour_target_idx < len(active_detour_path):
                    for pt in active_detour_path[detour_target_idx:]:
                        d = distance([gx, gy], pt)
                        if d < min_d_detour:
                            min_d_detour = d

                req_clearance = HALF_TRACK_WIDTH + WHEEL_RADIUS + r_obs + 0.20
                if min_d_detour < req_clearance and ox < min_front:
                    min_front = ox
                    critical_obs = obs
                    need_detour_plan = True

        if need_detour_plan and critical_obs is not None:
            detour_pts, k_rejoin, side = select_best_detour(node, critical_obs)
            if detour_pts and len(detour_pts) > 5:
                active_detour_path = detour_pts
                detour_target_idx = 0
                detour_rejoin_global_idx = k_rejoin
                obs_gx, obs_gy = point_local_to_global(critical_obs.x, critical_obs.y, car_yaw, car_global_axis)
                detour_obstacle_info = [obs_gx, obs_gy, critical_obs.radius, side]
                node.get_logger().info(
                    f"🚀 Selected LOW-COST detour around obstacle at [{critical_obs.x:.2f}m, {critical_obs.y:.2f}m], "
                    f"side={'LEFT' if side > 0 else 'RIGHT'}, waypoints={len(active_detour_path)}"
                )

    # ── Step 2: Path Tracking (Dynamic Detour or Global Path) ─────────────────
    if active_detour_path is not None:
        # Search locally ahead for closest waypoint on detour path
        search_start = max(0, detour_target_idx - 2)
        search_end = min(len(active_detour_path), detour_target_idx + 35)
        closest_idx = search_start
        min_d = distance(car_global_axis, active_detour_path[closest_idx])
        for i in range(search_start + 1, search_end):
            d = distance(car_global_axis, active_detour_path[i])
            if d < min_d:
                min_d = d
                closest_idx = i
        detour_target_idx = max(detour_target_idx, closest_idx)

        # Lookahead along detour path (smooth, low-curvature lookahead)
        detour_LA = LA_DETOUR
        accum_dist = 0.0
        target_idx = len(active_detour_path) - 1
        for i in range(detour_target_idx, len(active_detour_path) - 1):
            accum_dist += distance(active_detour_path[i], active_detour_path[i + 1])
            if accum_dist >= detour_LA:
                target_idx = i + 1
                break

        lookahead_gx, lookahead_gy = active_detour_path[target_idx]
        adj_local_x, adj_local_y = point_global_to_local([lookahead_gx, lookahead_gy], car_yaw, car_global_axis)

        # Controlled cruise velocity during dynamic detour
        speed_scale = min(speed_scale, DETOUR_CRUISE_VELOCITY / BASE_VELOCITY)

        # Check for detour completion
        dist_to_rejoin = distance(car_global_axis, active_detour_path[-1])
        if detour_target_idx >= len(active_detour_path) - 4 or dist_to_rejoin < 0.40:
            node.get_logger().info(
                f"✅ Dynamic detour completed! Smoothly rejoining global path at index {detour_rejoin_global_idx}."
            )
            current_target_idx = detour_rejoin_global_idx
            active_detour_path = None
            detour_obstacle_info = None

    else:
        # Standard tracking on global path (reduced LA for tight, high-accuracy path tracking)
        nom_x, nom_y, norm_x, norm_y = update_and_get_lookahead_point(lookahead_dist=LA_GLOBAL)
        lookahead_gx, lookahead_gy = nom_x, nom_y
        adj_local_x, adj_local_y = point_global_to_local([lookahead_gx, lookahead_gy], car_yaw, car_global_axis)
        speed_scale = 1.0

    # ── Step 3: Trajectory-Based Collision Guard & Deceleration ─────────────
    is_blocked = False
    if active_obstacles:
        half_body_w = HALF_TRACK_WIDTH + WHEEL_RADIUS  # 0.465m
        for obs in active_obstacles:
            ox, oy = obs.x, obs.y
            r_obs = obs.radius
            gx, gy = point_local_to_global(ox, oy, car_yaw, car_global_axis)

            # 1. Immediate Physical Front Bumper Guard (hard safety stop within 15cm of chassis)
            if 0.0 < ox < (ROVER_FRONT_EXTENT + 0.15) and abs(oy) < (half_body_w + 0.05):
                is_blocked = True
                speed_scale = 0.0
                node.get_logger().warn(
                    f"🛑 Immediate bumper guard: obstacle at [{ox:.2f}m, {oy:.2f}m] front={ox - ROVER_FRONT_EXTENT:.2f}m! Pivoting away...",
                    throttle_duration_sec=0.5
                )
                break

            # 2. Trajectory Blockage Check: Is obstacle directly blocking the upcoming active path?
            if active_detour_path is not None:
                min_d_detour = 999.0
                if detour_target_idx < len(active_detour_path):
                    for pt in active_detour_path[detour_target_idx:detour_target_idx + 12]:
                        d = distance([gx, gy], pt)
                        if d < min_d_detour:
                            min_d_detour = d
                if min_d_detour < (half_body_w + r_obs + 0.05) and ox < (ROVER_FRONT_EXTENT + STOP_DIST):
                    is_blocked = True
                    speed_scale = 0.0
                    break
            else:
                # Check distance from obstacle to upcoming global path
                d_to_path, _, along_dist = get_obstacle_path_interaction(gx, gy, current_target_idx, max_lookahead_pts=25)
                if d_to_path < (half_body_w + r_obs + 0.05):
                    # Obstacle is on upcoming global path
                    if along_dist < STOP_DIST:
                        is_blocked = True
                        speed_scale = 0.0
                        node.get_logger().warn(
                            f"🛑 Path blocked ahead by obstacle at [{ox:.2f}m, {oy:.2f}m] along_dist={along_dist:.2f}m < {STOP_DIST}m!",
                            throttle_duration_sec=0.5
                        )
                        break
                    elif along_dist < SLOWDOWN_DIST:
                        ratio = (along_dist - STOP_DIST) / max(0.01, SLOWDOWN_DIST - STOP_DIST)
                        current_scale = float(np.clip(ratio, 0.35, 1.0))
                        if current_scale < speed_scale:
                            speed_scale = current_scale

            # 3. Flank Penetration Guard (alongside the chassis)
            if -ROVER_REAR_EXTENT <= ox <= ROVER_FRONT_EXTENT:
                side_clearance = abs(oy) - (half_body_w + r_obs)
                if side_clearance < -0.06:
                    is_blocked = True
                    speed_scale = 0.0
                    node.get_logger().warn(
                        f"🛑 Side collision warning: obstacle touching flank at [{ox:.2f}m, {oy:.2f}m]!",
                        throttle_duration_sec=0.5
                    )
                    break

    publish_diagnostics(node, lookahead_gx, lookahead_gy)

    # Heading angle and lookahead distance
    ld = math.hypot(adj_local_x, adj_local_y)
    angle_to_target = math.atan2(adj_local_y, adj_local_x)

    if is_blocked:
        # If path is obstructed, halt forward linear translation, BUT pivot turn in place
        # to steer away from the obstacle!
        if abs(angle_to_target) > 0.06:
            desired_linear = 0.0
            desired_angular = float(np.clip(2.0 * angle_to_target, -0.85, 0.85))
            if abs(desired_angular) < 0.20:
                desired_angular = 0.20 if angle_to_target > 0 else -0.20
        else:
            if detour_obstacle_info is not None:
                side_dir = detour_obstacle_info[3]  # +1 left, -1 right
                desired_linear = 0.0
                desired_angular = 0.45 * side_dir
            elif active_obstacles:
                nearest_obs = min(active_obstacles, key=lambda o: math.hypot(o.x, o.y))
                avoid_dir = 1.0 if nearest_obs.y < 0.0 else -1.0
                desired_linear = 0.0
                desired_angular = 0.45 * avoid_dir
            else:
                desired_linear = 0.0
                desired_angular = 0.0

        publish_cmd_vel(node, desired_linear, desired_angular)
        return

    # Proportional smooth steering without heading overshoot or zero-velocity tire scrub stall
    if abs(angle_to_target) > 0.85:  # Large heading error: sharp turning crawl
        desired_linear = float(MIN_VELOCITY * 0.85 * speed_scale)
        desired_angular = float(np.clip(1.8 * angle_to_target, -0.90, 0.90))
    elif abs(angle_to_target) > 0.40: # Medium turn: smooth forward curve
        desired_linear = float(BASE_VELOCITY * 0.60 * speed_scale)
        desired_angular = float(np.clip(1.5 * angle_to_target, -0.70, 0.70))
    else:
        # Pure pursuit continuous curvature tracking
        curv = (2.0 * math.sin(angle_to_target)) / max(0.40, ld)
        curv = float(np.clip(curv, -MAX_CURVATURE, MAX_CURVATURE))
        turn_factor = max(0.55, math.cos(angle_to_target * 0.6))
        target_vel = max(MIN_VELOCITY, BASE_VELOCITY * speed_scale * turn_factor)
        effective_v = max(target_vel, 0.30)
        desired_angular = float(np.clip(curv * effective_v, -0.65, 0.65))
        desired_linear = float(target_vel)

    # ── Anti-Stall Traction Monitor ──────────────────────────────────────────
    if _last_rover_pos is not None and (abs(desired_linear) > 0.05 or abs(desired_angular) > 0.05):
        disp = distance(car_global_axis, _last_rover_pos)
        if disp < 0.015:  # Moved less than 1.5 cm in 100ms
            _stuck_ticks += 1
        else:
            _stuck_ticks = max(0, _stuck_ticks - 2)

        if _stuck_ticks > 8:  # Hungarian/terrain stall detected (>0.8 sec without motion)
            boost = min(1.6, 1.0 + 0.08 * (_stuck_ticks - 8))
            desired_linear = float(np.clip(desired_linear * boost, MIN_VELOCITY, 0.85))
            if _stuck_ticks % 10 == 0:
                node.get_logger().warn(
                    f"⚠️ [Anti-Stall] Terrain resistance detected (stuck {_stuck_ticks*0.1:.1f}s), boosting thrust: v={desired_linear:.2f}m/s",
                    throttle_duration_sec=1.0
                )
    else:
        _stuck_ticks = 0

    _last_rover_pos = list(car_global_axis)

    # Differential drive wheel speed limit
    v_left = desired_linear - (desired_angular * TRACK_WIDTH / 2.0)
    v_right = desired_linear + (desired_angular * TRACK_WIDTH / 2.0)
    max_wheel_demand = max(abs(v_left), abs(v_right))
    if max_wheel_demand > MAX_WHEEL_SPEED:
        factor = MAX_WHEEL_SPEED / max_wheel_demand
        desired_linear *= factor
        desired_angular *= factor

    publish_diagnostics(node, lookahead_gx, lookahead_gy)
    publish_cmd_vel(node, desired_linear, desired_angular)

    # Periodic tracking status log at 1 Hz
    if now_sec - _last_diag_log_t > 1.0:
        _last_diag_log_t = now_sec
        node.get_logger().info(
            f"🚜 [PP Tracking] Pos:[{car_global_axis[0]:.2f},{car_global_axis[1]:.2f}] "
            f"Yaw:{math.degrees(car_yaw):.1f}° | Wp:{current_target_idx+1}/{len(path)} | "
            f"HeadErr:{math.degrees(angle_to_target):.1f}° | Cmd: v={desired_linear:.2f}m/s w={desired_angular:.2f}rad/s "
            f"({'DETOUR' if active_detour_path else 'GLOBAL'})"
        )


def publish_path(node):
    global path_pub, smoothed_plan_pub, path
    if not path or not rclpy.ok():
        return
    try:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        if path_pub is not None:
            path_pub.publish(msg)
        if smoothed_plan_pub is not None:
            smoothed_plan_pub.publish(msg)
    except Exception:
        pass


def goal_path_callback(msg: Path, node: Node):
    global path, current_target_idx, target_idx_initialized
    if not msg.poses:
        return
    new_pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
    if len(new_pts) >= 2:
        path = new_pts
        current_target_idx = 0
        target_idx_initialized = False
        node.get_logger().info(f"📍 Received new dynamic path with {len(path)} waypoints via topic!")
        publish_path(node)


def main(args=None):
    global vel_pub, vel_twist_pub, path, avoidance_path_pub, lookahead_marker_pub, tf_buffer, tf_listener
    global path_pub, smoothed_plan_pub, PATH_CSV
    global TRACK_WIDTH, WHEEL_BASE, WHEEL_RADIUS, MAX_WHEEL_SPEED, SPAWN_X, SPAWN_Y, SPAWN_YAW
    global LA_GLOBAL, ENABLE_AVOIDANCE, SAFETY_MARGIN, SLOWDOWN_DIST, STOP_DIST
    global HALF_TRACK_WIDTH, HALF_WHEEL_BASE, ROVER_FRONT_EXTENT, ROVER_REAR_EXTENT, ROBOT_RADIUS

    rclpy.init(args=args)
    node = rclpy.create_node("pure_pursuit")
    node.declare_parameter("lookahead_distance", LA_GLOBAL)
    node.declare_parameter("enable_obstacle_avoidance", ENABLE_AVOIDANCE)
    node.declare_parameter("safety_margin", SAFETY_MARGIN)
    node.declare_parameter("slowdown_dist", SLOWDOWN_DIST)
    node.declare_parameter("stop_dist", STOP_DIST)
    node.declare_parameter("track_width", TRACK_WIDTH)
    node.declare_parameter("wheel_base", WHEEL_BASE)
    node.declare_parameter("wheel_radius", WHEEL_RADIUS)
    node.declare_parameter("max_wheel_speed", MAX_WHEEL_SPEED)
    node.declare_parameter("path_file", PATH_CSV)
    node.declare_parameter("map_yaml", "")
    node.declare_parameter("spawn_x", SPAWN_X)
    node.declare_parameter("spawn_y", SPAWN_Y)
    node.declare_parameter("spawn_yaw", SPAWN_YAW)

    # Sync ROS2 parameters
    path_file_param = str(node.get_parameter("path_file").value)
    if path_file_param:
        PATH_CSV = os.path.expanduser(path_file_param)

    map_yaml_param = str(node.get_parameter("map_yaml").value)

    SLOWDOWN_DIST = float(node.get_parameter("slowdown_dist").value)
    STOP_DIST = float(node.get_parameter("stop_dist").value)
    TRACK_WIDTH = float(node.get_parameter("track_width").value)
    WHEEL_BASE = float(node.get_parameter("wheel_base").value)
    WHEEL_RADIUS = float(node.get_parameter("wheel_radius").value)
    MAX_WHEEL_SPEED = float(node.get_parameter("max_wheel_speed").value)
    SPAWN_X = float(node.get_parameter("spawn_x").value)
    SPAWN_Y = float(node.get_parameter("spawn_y").value)
    SPAWN_YAW = float(node.get_parameter("spawn_yaw").value)
    HALF_TRACK_WIDTH = TRACK_WIDTH / 2.0
    HALF_WHEEL_BASE = WHEEL_BASE / 2.0
    ROVER_FRONT_EXTENT = HALF_WHEEL_BASE + WHEEL_RADIUS
    ROVER_REAR_EXTENT = HALF_WHEEL_BASE + WHEEL_RADIUS
    ROBOT_RADIUS = math.hypot(HALF_WHEEL_BASE, HALF_TRACK_WIDTH)

    node.get_logger().info(
        f"🤖 Rover Kinematics: Track Width={TRACK_WIDTH:.4f}m, Wheelbase={WHEEL_BASE:.4f}m, "
        f"Radius={WHEEL_RADIUS:.3f}m, Extents=[+{ROVER_FRONT_EXTENT:.3f}m, -{ROVER_REAR_EXTENT:.3f}m]"
    )

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    try:
        path = load_path(PATH_CSV)
        node.get_logger().info(f"Loaded {len(path)} path points from {PATH_CSV}")
    except Exception as e:
        node.get_logger().error(f"Error loading path: {e}")
        path = []

    if load_static_map(map_yaml_param):
        node.get_logger().info(f"🗺️  Loaded static terrain map ({map_width}x{map_height} @ {map_resolution}m/px) for hill avoidance")
    else:
        node.get_logger().warn("⚠️  Could not load static terrain map file; relying on /map topic")

    map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/map", map_callback, map_qos)
    node.create_subscription(ObstacleArray, "/obstacles", obstacles_callback, 10)
    node.create_subscription(Odometry, "/odometry/filtered", odom_callback, 10)
    node.create_subscription(Odometry, "/odom", odom_callback, 10)
    node.create_subscription(Path, "/pure_pursuit/goal_path", lambda msg: goal_path_callback(msg, node), 10)

    vel_pub = node.create_publisher(TwistStamped, "/rover_controller/cmd_vel", 10)
    vel_twist_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    avoidance_path_pub = node.create_publisher(Path, "/pure_pursuit/local_avoidance_path", 10)
    lookahead_marker_pub = node.create_publisher(Marker, "/pure_pursuit/lookahead_marker", 10)

    path_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(Path, "/pure_pursuit/path", path_qos)
    smoothed_plan_pub = node.create_publisher(Path, "/plan_smoothed", 10)

    if path:
        publish_path(node)

    # Continuously broadcast global path at 1 Hz so RViz reliably captures it
    path_timer = node.create_timer(1.0, lambda: publish_path(node))

    timer = node.create_timer(0.1, lambda: control_loop(node))
    node.get_logger().info("🚀 Pure Pursuit Path Follower with Smooth 3D Obstacle Avoidance is running!")

    rclpy.spin(node)
    node.destroy_timer(path_timer)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
