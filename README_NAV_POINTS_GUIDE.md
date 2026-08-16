# 🧭 Autonomous Obstacle Navigation & Waypoint Feeding Guide

This guide explains how to operate the autonomous navigation stack (`obstacle_nav_gazebo.launch.py`), how to feed custom waypoints and paths into the rover, how the system architecture works, and how to tune critical parameters.

---

## 📑 Table of Contents
1. [Overview & Quick Start](#-overview--quick-start)
2. [How to Feed Waypoints to `obstacle_nav_gazebo.launch.py`](#-how-to-feed-waypoints-to-obstacle_nav_gazebolaunchpy)
   - [Method 1: Launch Argument (`path_file:=...`)](#method-1-launch-argument-path_file)
   - [Method 2: Live CSV File Hot-Reloading](#method-2-live-csv-file-hot-reloading)
   - [Method 3: Multi-Point Survey Tour Planner](#method-3-multi-point-survey-tour-planner)
   - [Method 4: ROS 2 Topic Path Injection (`/pure_pursuit/goal_path`)](#method-4-ros-2-topic-path-injection)
   - [Method 5: Interactive RViz Goal Poses](#method-5-interactive-rviz-goal-poses)
3. [Coordinate System & Waypoint Format](#-coordinate-system--waypoint-format)
4. [System Architecture & TF Tree](#-system-architecture--tf-tree)
5. [3D Perception & Obstacle Avoidance Pipeline](#-3d-perception--obstacle-avoidance-pipeline)
6. [Key Parameters & Tuning Guide](#-key-parameters--tuning-guide)
7. [ROS 2 Topic & Interface Reference](#-ros-2-topic--interface-reference)
8. [Testing with Simulated Obstacle Boulders](#-testing-with-simulated-obstacle-boulders)

---

## 🌟 Overview & Quick Start

The main entry point for the full autonomous simulation is:

```bash
ros2 launch rover_nav obstacle_nav_gazebo.launch.py
```

This single command brings up:
1. **Gazebo Sim (Harmonic)**: Mars Yard 2026 3D heightmap world, lighting, physics, and the 6-wheel rocker-bogie rover.
2. **Sensor Simulation**: Realsense D435i Depth Camera (3D Point Cloud), BNO055 IMU, wheel encoders.
3. **Perception**: Real-time 3D ground segmentation (RANSAC) and DBSCAN obstacle clustering (`pcl_obstacle_detector.py`).
4. **Nav2 Global Planning Stack**: `map_server` (Mars Yard occupancy grid), `planner_server` (SmacPlanner2D), `smoother_server`.
5. **Pure Pursuit Controller with Dynamic Avoidance**: Adaptive lookahead trajectory tracking with dynamic Bezier detour generation (`Pure_pursuit_Gazebo.py`).
6. **RViz2 Visualizer**: Pre-configured display showing 3D point clouds, bounding boxes, clearance disks, planned paths, and HUD diagnostics.

---

## 🎯 How to Feed Waypoints to `obstacle_nav_gazebo.launch.py`

You can feed waypoints into the navigation system using any of the **5 supported methods**:

---

### Method 1: Launch Argument (`path_file:=...`)

You can pass the path to any custom `.csv` file directly when launching:

```bash
ros2 launch rover_nav obstacle_nav_gazebo.launch.py path_file:=/path/to/my_custom_waypoints.csv
```

#### Launch with Custom Spawn Location:
If your custom path starts from a different location in the Mars Yard, specify the spawn coordinates:
```bash
ros2 launch rover_nav obstacle_nav_gazebo.launch.py \
    spawn_x:=-5.0 \
    spawn_y:=10.0 \
    spawn_yaw:=1.57 \
    path_file:=/path/to/my_custom_waypoints.csv
```

---

### Method 2: Live CSV File Hot-Reloading

The pure pursuit node actively monitors the waypoint CSV file (`marsyard2026_tour_waypoints.csv`). 

**You can edit and save the CSV file while the rover is running in simulation.**
- The controller instantly detects the file change timestamp (`mtime`).
- It parses the updated points, resets the lookahead tracker, and smoothly redirects the rover towards the new path without needing a restart!

Default path file location:
```bash
# In the source package
src/rover_nav/maps/marsyard2026_tour_waypoints.csv

# Or in the installed share directory
$(ros2 pkg prefix rover_nav)/share/rover_nav/maps/marsyard2026_tour_waypoints.csv
```

---

### Method 3: Multi-Point Survey Tour Planner

Use the automated path planner tool to compute a smooth, collision-free route through Mars Yard survey points or arbitrary (x, y) coordinates:

```bash
# 1. Ensure Nav2 planning stack is running (or launch obstacle_nav_gazebo.launch.py)
ros2 launch rover_nav nav2_planning.launch.py rviz:=true

# 2. In a second terminal, plan a multi-point tour:
python3 src/Mars-rover/tools/plan_multi_point_tour.py --start S1 --points W6 W5 W7 W8
```

#### Supported Options:
- `--start`: Starting landmark name (`S1`, `W1`, `L5`, etc.) or `x,y` tuple (e.g. `0.0,0.0`).
- `--points`: List of destination survey points (e.g. `W6 W5 W7 W8` or `L1 L2 L3`).
- `--no-loop`: Stop at the last point instead of looping back to `--start`.

The script:
1. Greedily sorts the points for shortest travel distance (Nearest-Neighbor TSP).
2. Queries Nav2 `SmacPlanner2D` for a continuous, terrain-aware route.
3. Smoothes the trajectory via `smoother_server`.
4. Saves the result directly to `marsyard2026_tour_waypoints.csv` and generates a preview image `marsyard2026_tour_preview.png`.

---

### Method 4: ROS 2 Topic Path Injection

You can feed paths dynamically at runtime by publishing standard `nav_msgs/msg/Path` messages to `/pure_pursuit/goal_path` or `/plan_smoothed`.

#### Python Example:
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def send_custom_path():
    rclpy.init()
    node = rclpy.create_node("path_publisher")
    pub = node.create_publisher(Path, "/pure_pursuit/goal_path", 10)

    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()

    # List of (x, y) coordinates
    waypoints = [
        (0.0, 0.0),
        (-1.5, 4.0),
        (-3.5, 12.0),
        (-4.0, 18.0),
        (0.0, 22.0)
    ]

    for x, y in waypoints:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

    pub.publish(msg)
    node.get_logger().info(f"Published {len(waypoints)} waypoints to /pure_pursuit/goal_path")
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    send_custom_path()
```

---

### Method 5: Interactive RViz Goal Poses

When `obstacle_nav_gazebo.launch.py` is active:
1. Open RViz2.
2. Select the **2D Goal Pose** tool on the top toolbar.
3. Click and drag on the Mars Yard map to set a target goal pose.
4. Nav2 computes the smoothed path, which is broadcast on `/plan_smoothed` and followed by the Pure Pursuit controller.

---

## 🗺️ Coordinate System & Waypoint Format

### CSV File Schema
Waypoints are formatted as simple comma-separated `x,y` float values in world meters:

```csv
x,y
0.000,0.000
-0.082,0.493
-0.201,1.104
-0.540,2.350
-1.200,5.600
```

### Mars Yard 2026 Reference Frame:
- **Origin `(0, 0, 0)`**: Set at surveyed starting point **`S1`**.
- **+X Axis**: Points **East** (across the yard).
- **+Y Axis**: Points **North** (forward along the main yard axis).
- **+Z Axis**: Points **Up** (elevation in meters).
- **Yaw**: `0 rad` = Facing East (+X), `+1.5708 rad` (`+90°`) = Facing North (+Y).

---

## 🏛️ System Architecture & TF Tree

```
                                  ┌─────────────────────────────┐
                                  │   Gazebo Sim (Harmonic)     │
                                  │   - Mars Yard 2026 DEM      │
                                  │   - Depth Camera & IMU      │
                                  └──────────────┬──────────────┘
                                                 │ /camera/.../points
                                                 ▼
┌─────────────────────────┐       ┌─────────────────────────────┐
│  odom_tf_broadcaster    │       │  pcl_obstacle_detector.py   │
│  - map -> odom (static) │       │  - Passthrough & SOR Filter │
│  - odom -> base_footprint│       │  - RANSAC Ground Removal    │
└────────────┬────────────┘       │  - DBSCAN 3D Clustering     │
             │                    └──────────────┬──────────────┘
             │                                   │ /obstacles
             ▼                                   ▼
┌───────────────────────────────────────────────────────────────┐
│                 Pure_pursuit_Gazebo.py                        │
│  - Adaptive Lookahead Pure Pursuit                            │
│  - Real-time Bezier Detour Generator (Left/Right Terrain Cost)│
│  - Anti-Stall Traction Monitor                                │
└────────────────────────────┬──────────────────────────────────┘
                             │ /cmd_vel & /rover_controller/cmd_vel
                             ▼
              ┌───────────────────────────────┐
              │ Gazebo Rover Wheel Actuators  │
              └───────────────────────────────┘
```

### TF Coordinate Frame Tree:
```
map  (world frame, origin at S1)
 └── odom  (odometry start frame)
      └── base_footprint  (ground projection of rover center)
           └── base_link  (chassis center of mass)
                ├── camera_link -> camera_depth_optical_frame
                ├── bno055 (IMU frame)
                └── wheel joints (fl, fr, ml, mr, rl, rr)
```

---

## 🔍 3D Perception & Obstacle Avoidance Pipeline

The perception pipeline (`pcl_obstacle_detector.py`) runs in real-time on point clouds published by Gazebo:

1. **TF Transform**: Transforms depth cloud into the `base_link` frame.
2. **PassThrough Filter**: Crops points to the rover's field of view (`X: 0.2m..4.5m`, `Y: -2.5m..+2.5m`, `Z: -0.4m..+1.2m`).
3. **Statistical Outlier Removal (SOR)**: Removes sensor speckle noise (`mean_k=15`, `std_dev_mul=1.0`).
4. **RANSAC Ground Plane Removal**: Estimates terrain surface plane and filters out ground points below `+0.07m`.
5. **DBSCAN Clustering**: Clusters remaining obstacle points into discrete 3D objects (`eps=0.22m`, `min_samples=8`).
6. **Bypass Detour Evaluation**:
   - When an obstacle intersects the path corridor, `Pure_pursuit_Gazebo.py` evaluates candidate **LEFT** and **RIGHT** cubic Bezier bypass arcs.
   - It queries the static Mars Yard elevation costmap to pick the flatter, safer side and smoothly diverts the rover around the rock!

---

## ⚙️ Key Parameters & Tuning Guide

All parameters can be tuned in `rover_nav/config/nav2_planning_params.yaml` or directly via launch arguments / node parameters:

| Parameter | Default | Description |
| :--- | :---: | :--- |
| `lookahead_distance` | `0.65 m` | Nominal lookahead distance for pure pursuit path tracking. |
| `safety_margin` | `0.20 m` | Additional clearance buffer around detected obstacles. |
| `slowdown_dist` | `1.80 m` | Distance from obstacle at which rover begins proportional deceleration. |
| `stop_dist` | `0.40 m` | Distance threshold for emergency halt / pivot avoidance. |
| `max_wheel_speed` | `1.00 m/s` | Maximum allowable linear speed per wheel. |
| `spawn_x`, `spawn_y`, `spawn_yaw` | `0, 0, 1.57` | Rover initial spawn position & orientation in the Mars Yard. |
| `enable_obstacle_avoidance`| `true` | Enables/disables reactive Bezier bypass generation. |

---

## 📡 ROS 2 Topic & Interface Reference

### Published Topics:
- `/rover_controller/cmd_vel` (`geometry_msgs/TwistStamped`): Velocity command sent to Gazebo / hardware.
- `/cmd_vel` (`geometry_msgs/Twist`): Unstamped velocity command stream.
- `/obstacles` (`rover_nav/ObstacleArray`): 3D positions, radii, and bounding boxes of detected obstacles.
- `/pure_pursuit/path` (`nav_msgs/Path`): Active global waypoint trajectory.
- `/pure_pursuit/local_avoidance_path` (`nav_msgs/Path`): Real-time Bezier detour bypass path.
- `/pure_pursuit/lookahead_marker` (`visualization_msgs/Marker`): Visual marker of the current lookahead pursuit target.

### Subscribed Topics:
- `/odometry/filtered` (`nav_msgs/Odometry`): Fused EKF odometry (position & orientation).
- `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`): Raw 3D point cloud from depth camera.
- `/map` (`nav_msgs/OccupancyGrid`): 2D Mars Yard traversability costmap.
- `/pure_pursuit/goal_path` (`nav_msgs/Path`): Input topic for dynamic path injection.

---

## 🪨 Testing with Simulated Obstacle Boulders

To test dynamic obstacle avoidance on the fly, you can spawn realistic boulder obstacles onto the rover's path in Gazebo:

```bash
# Spawn 3 boulders directly on the waypoint path:
ros2 run rover_nav spawn_obstacle_rocks.py

# Remove / clear all spawned boulders:
ros2 run rover_nav spawn_obstacle_rocks.py --ros-args -p clear_rocks:=true
```

Watch in Gazebo and RViz2 as the rover approaches each rock, detects it with the depth camera, generates a smooth Bezier detour around it, and rejoins the global route!
