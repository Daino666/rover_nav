# Mars Rover Autonomous Navigation — jazzy_ws

A full autonomous navigation stack for a Mars Rover built on **ROS 2 Jazzy** + **Gazebo Harmonic**.

## Stack Overview

| Component | Description |
|---|---|
| `rover_nav` | Nav2 planning, Pure Pursuit path follower, 3D obstacle avoidance, RViz launch |
| `rover_description` | URDF/Xacro robot model, Gazebo world, ros2_control config, Gazebo bridge |
| `rover_controllers` | Differential drive controller config, joystick teleop |
| `Mars-rover` | Tools (tour planner, occupancy grid generator), ODrive firmware interface |
| `obstacle_detection` | PCL-based 3D bounding-box obstacle detector |
| `marsyard` | Mars Yard occupancy map, tour waypoints CSV, 3D model data |

---

## 1. Prerequisites

### 1.1 Operating System
Ubuntu **24.04 LTS** (Noble Numbat) — required by ROS 2 Jazzy.

```bash
lsb_release -a   # must show Ubuntu 24.04
```

### 1.2 ROS 2 Jazzy Desktop

```bash
# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 1.3 ROS 2 Navigation & Simulation Stack

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-smoother \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-pcl-ros \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-camera-msgs \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joy \
  ros-jazzy-lifecycle \
  ros-jazzy-rclcpp-lifecycle
```

### 1.4 Python Dependencies

```bash
# System-managed Python packages (do NOT use pip on Ubuntu 24.04 for these)
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-sklearn \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Open3D for 3D point cloud processing (requires a virtual environment
# because Ubuntu 24.04 uses an externally-managed Python interpreter)
python3 -m venv --system-site-packages ~/jazzy_ws/venv_open3d
~/jazzy_ws/venv_open3d/bin/pip install open3d

# Verify open3d is available to the rover_nav scripts
~/jazzy_ws/venv_open3d/bin/python3 -c "import open3d; print(open3d.__version__)"
```

> **Note:** `rover_nav` scripts that use Open3D (`pcl_obstacle_detector.py`) are
> launched by `ros2 launch`, which uses the **system** Python. Open3D is
> installed system-wide via pip into venv and imported there. If you get
> `ModuleNotFoundError: open3d`, run:
> ```bash
> sudo ~/jazzy_ws/venv_open3d/bin/pip install open3d
> # or install directly to system site-packages:
> pip3 install open3d --break-system-packages
> ```

### 1.5 colcon Build Tool

```bash
sudo apt install -y python3-colcon-common-extensions
```

---

## 2. Workspace Setup

### 2.1 Unzip & Place the Workspace

```bash
# Unzip to your home directory — the workspace root must be ~/jazzy_ws
unzip jazzy_ws.zip -d ~/
cd ~/jazzy_ws
```

> **Important:** The workspace is designed to live at `~/jazzy_ws`. All paths
> in launch files use `os.path.expanduser('~/jazzy_ws/...')` so they resolve
> correctly regardless of your username.

### 2.2 Source ROS 2

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 Install rosdep Dependencies

```bash
cd ~/jazzy_ws
rosdep init   # only needed once per machine; skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Build the Workspace

```bash
cd ~/jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Add the workspace to your shell permanently:

```bash
echo "source ~/jazzy_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify the build:
```bash
ros2 pkg list | grep rover_nav   # should print: rover_nav
ros2 pkg list | grep rover_desc  # should print: rover_description
```

---

## 4. Launch Commands

### 4.1 Lightweight Stack — RViz only (no Gazebo, no GPU required)

Best for low-spec laptops or rapid controller tuning. Uses a kinematic mock simulator instead of Gazebo.

```bash
ros2 launch rover_nav obstacle_nav_rviz.launch.py
```

### 4.2 Full Autonomous Gazebo Stack

Launches Gazebo Harmonic with the Mars Yard 3D world, RealSense D435i depth camera, Nav2 planning, 3D obstacle perception, Pure Pursuit path follower, and RViz.

```bash
ros2 launch rover_nav obstacle_nav_gazebo.launch.py
```

### 4.3 Plan & Execute the Autonomous Multi-Point Tour

In a **second terminal** (after the stack is up and `All Nav2 lifecycle nodes ACTIVE` appears):

```bash
source /opt/ros/jazzy/setup.bash
source ~/jazzy_ws/install/setup.bash
python3 ~/jazzy_ws/src/Mars-rover/tools/plan_multi_point_tour.py
```

This plans a collision-free global path: `S1 → W7 → W6 → W5 → W8 → S1` and publishes it. The Pure Pursuit controller begins tracking immediately.

### 4.4 Teleoperation (Keyboard)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/rover_controller/cmd_vel
```

---

## 5. Key File Locations

| File | Purpose |
|---|---|
| `src/marsyard/marsyard2026_tour_waypoints.csv` | Tour path waypoints loaded by Pure Pursuit |
| `src/marsyard/marsyard2026_occupancy.yaml` | Occupancy map for Nav2 |
| `src/rover_nav/config/nav2_planning_params.yaml` | Nav2 costmap, planner, smoother parameters |
| `src/rover_nav/scripts/Rover_path_controller/Pure_pursuit_Gazebo.py` | Path follower & reactive obstacle avoidance |
| `src/rover_nav/scripts/Rover_path_controller/pcl_obstacle_detector.py` | 3D perception pipeline (RANSAC + DBSCAN) |
| `src/rover_nav/scripts/odom_tf_broadcaster.py` | TF broadcaster: `map→odom→base_footprint` |
| `src/rover_nav/rviz/obstacle_nav.rviz` | RViz2 configuration |

---

## 6. Push Status and Repository Boundaries

### Included in the `Priyam` branch

The current branch snapshot includes the rover navigation source and configuration:

- ROS 2 launch files, URDF/Xacro files, Gazebo world and bridge configuration.
- Nav2, localization, obstacle detection, Pure Pursuit, lifecycle, and mock simulation scripts.
- RViz configurations, occupancy maps, waypoint CSV files, terrain meshes, textures, and previews.
- Build and navigation documentation, dependency scripts, and project configuration.

The current local commit is `Add current rover navigation work` and contains 70 files. It includes large terrain and mesh assets, so the repository may be large even though the source code is modest.

### Excluded by `.gitignore`

These local/generated files are not included by normal `git add -A` commands:

- `build/`, `install/`, and `log/` directories generated by `colcon`.
- Python `__pycache__/` directories and compiled `*.pyc` files.

Regenerate these directories on each machine by following the build instructions above. Do not commit the workspace-level `venv_open3d/` virtual environment; install its dependencies separately.

### Files that must not be pushed

Never commit GitHub tokens, passwords, SSH private keys, `.env` files, or other credentials. A token pasted into chat, a terminal command, a remote URL, or a committed file should be revoked immediately and replaced.

Do not add temporary logs, editor settings, downloaded archives, personal datasets, or generated caches unless the project specifically requires them. Review large map and terrain assets before pushing: GitHub rejects individual files larger than 100 MB, and Git LFS is preferable for assets that change frequently or grow beyond normal Git sizes.

### Verify before pushing

```bash
git status --short
git diff --stat HEAD
git diff --check HEAD
git ls-files | grep -E '(^|/)(build|install|log|__pycache__)/|\.pyc$'
```

The final command should print nothing. Push the branch only after reviewing `git status` and confirming that no credentials or unintended local files are present:

```bash
git push -u origin Priyam
```
| `src/Mars-rover/rover_description/worlds/marsyard.sdf` | Gazebo Mars Yard world |
| `src/Mars-rover/rover_description/launch/my_robot.launch.py` | Gazebo sim launcher (robot + bridge + EKF) |

---

## 6. Tunable Parameters Quick Reference

### Pure Pursuit Controller (`Pure_pursuit_Gazebo.py`)

| Parameter | Default | Effect |
|---|---|---|
| `LA` | `1.2 m` | Lookahead distance — increase for smoother tracking, decrease for tighter corners |
| `BASE_VELOCITY` | `0.45 m/s` | Cruise speed |
| `SAFETY_MARGIN` | `0.25 m` | Extra lateral buffer around obstacles |
| `CLEAR_PAST_DISTANCE` | `0.70 m` | How far past an obstacle before returning to centerline |

### Nav2 Costmap (`nav2_planning_params.yaml`)

| Parameter | Default | Effect |
|---|---|---|
| `inflation_radius` | `0.55 m` | Obstacle halo size — larger avoids more area around rocks |
| `cost_travel_multiplier` | `3.0` | Higher = planner prefers open space more aggressively |
| `cost_scaling_factor` | `2.5` | Cost gradient steepness near obstacles |

---

## 7. Troubleshooting

### `ModuleNotFoundError: No module named 'open3d'`
```bash
pip3 install open3d --break-system-packages
```

### `Could not find package 'rover_nav'`
```bash
source ~/jazzy_ws/install/setup.bash
```

### RViz shows red robot model (TF errors)
The `odom_tf_broadcaster` node may not have started yet. Wait 5–10 s after launch for the EKF to begin publishing `/odometry/filtered`. The broadcaster has a built-in 5 s startup delay in the Gazebo launch.

### Gazebo crashes on launch (AMD/Intel GPU)
The launch file auto-detects NVIDIA vs Mesa graphics. If you see GPU/EGL errors:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch rover_nav obstacle_nav_gazebo.launch.py
```

### Path not loading in Pure Pursuit
Ensure the CSV file exists:
```bash
ls ~/jazzy_ws/src/marsyard/marsyard2026_tour_waypoints.csv
```

---

## 8. Hardware Deployment (Real Rover)

To run on the physical rover (ODrive + RealSense D435i) instead of Gazebo:

```bash
ros2 launch rover_description my_robot.launch.py use_sim:=false
```

Then in separate terminals:
```bash
# Obstacle perception from real RealSense camera
ros2 launch rover_nav obstacle_detection.launch.py

# Path follower
ros2 run rover_nav Pure_pursuit_Gazebo.py
```
