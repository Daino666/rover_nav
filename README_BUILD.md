# 🛠️ Mars Rover Workspace — Build & Setup Guide

This guide provides step-by-step instructions to unpack, install dependencies, build, and run the **Mars Rover ROS 2 Jazzy Autonomous Navigation Workspace** on a new laptop or workstation.

---

## 📋 System Requirements

| Component | Required Version / Specification |
| :--- | :--- |
| **Operating System** | **Ubuntu 24.04 LTS (Noble Numbat)** |
| **ROS 2 Distribution** | **ROS 2 Jazzy Jalisco** |
| **Gazebo Simulator** | **Gazebo Sim (Harmonic)** (`ros-jazzy-ros-gz`) |
| **Python** | **Python 3.12+** |
| **Graphics / GPU** | Dedicated GPU (NVIDIA / AMD / Intel Iris) recommended for Gazebo & point cloud perception |

---

## 📁 Workspace Package Overview

The workspace consists of the following modular ROS 2 packages inside `src/`:

```
jazzy_ws/
├── src/
│   ├── Mars-rover/
│   │   ├── rover_description/   # 3D URDF/Xacro models, meshes, Mars Yard 2026 world & Gazebo bridge
│   │   ├── rover_controllers/   # Differential drive controllers & odometry
│   │   └── tools/               # Occupancy grid generator & multi-point tour planner
│   ├── rover_nav/               # Pure pursuit controller, EKF, Nav2 planners, launch files, maps
│   ├── obstacle_detection/      # 3D Point Cloud perception (RANSAC, DBSCAN, obstacle bounding boxes)
│   ├── marsyard/                # Mars Yard 2026 DEM heightmaps, 3D meshes, survey coordinates
│   └── install_dependencies.sh  # Automated one-shot dependency installer
└── README_BUILD.md              # This build guide
```

---

## 🚀 Quick Start (Automated Setup)

If you have just extracted or unzipped the workspace to your laptop:

```bash
# 1. Open a terminal and enter the workspace root
cd ~/jazzy_ws   # (or wherever you extracted the folder)

# 2. Make the installer executable and run it
chmod +x src/install_dependencies.sh
./src/install_dependencies.sh

# 3. Source the built environment
source install/setup.bash
```

---

## 🔧 Step-by-Step Manual Setup

If you prefer to install dependencies manually or need fine-grained control:

### 1. Update System Repositories
```bash
sudo apt update
```

### 2. Install Gazebo Harmonic & ROS 2 Simulation Bridge
```bash
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-robot-localization
```

### 3. Install Nav2 Stack
```bash
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    python3-sklearn
```

### 4. Install Python Dependencies
On Ubuntu 24.04 (PEP 668 managed environment), use `--break-system-packages` for system pip:
```bash
pip3 install --break-system-packages \
    numpy \
    scipy \
    pyyaml \
    matplotlib \
    pillow \
    open3d
```

### 5. Install Package Dependencies via rosdep
```bash
cd ~/jazzy_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys zed_description
```

---

## 🏗️ Building the Workspace

Build all packages using `colcon`:

```bash
cd ~/jazzy_ws
source /opt/ros/jazzy/setup.bash

# Clean symlink-install build
colcon build --symlink-install
```

### Useful Colcon Build Flags
- **Build specific package**:
  ```bash
  colcon build --symlink-install --packages-select rover_nav
  ```
- **Clean build from scratch**:
  ```bash
  rm -rf build/ install/ log/
  colcon build --symlink-install
  ```

---

## 🌐 Environment Sourcing

To make the rover packages and nodes accessible in every new terminal:

```bash
# In your current terminal
source ~/jazzy_ws/install/setup.bash

# (Optional) Add to your ~/.bashrc automatically
echo "source ~/jazzy_ws/install/setup.bash" >> ~/.bashrc
```

---

## ✅ Verification & Testing

Verify that all packages are recognized by ROS 2:

```bash
source ~/jazzy_ws/install/setup.bash
ros2 pkg list | grep -E "rover|obstacle"
```
You should see:
- `obstacle_detection`
- `rover_controllers`
- `rover_description`
- `rover_nav`

### Test 1: Launch Gazebo Sim Alone
```bash
ros2 launch rover_description my_robot.launch.py
```
*Expected result:* Gazebo Sim opens displaying the Mars Yard 2026 terrain, starry sky, and the 6-wheel rover spawned at the S1 origin.

### Test 2: Launch Full Autonomous Obstacle Navigation Stack
```bash
ros2 launch rover_nav obstacle_nav_gazebo.launch.py
```
*Expected result:* Gazebo Sim and RViz2 launch together. The rover autonomously follows the Mars Yard tour path, detects obstacles with its 3D depth camera, dynamically calculates Bezier detours around boulders, and completes the tour!

---

## ❓ Troubleshooting & FAQs

### 1. Gazebo crashes or renders black screen (NVIDIA / Hybrid GPUs)
If your laptop has hybrid NVIDIA/Intel graphics:
```bash
# Force NVIDIA GPU rendering
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch rover_nav obstacle_nav_gazebo.launch.py

# Or if running without GPU acceleration (software fallback)
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rover_nav obstacle_nav_gazebo.launch.py
```

### 2. `rosdep install` reports missing `zed_description`
`zed_description` is deliberately skipped because the rover URDF already contains full integrated camera frames. The `--skip-keys zed_description` flag prevents rosdep from complaining.

### 3. Missing `open3d` or Python dependencies
Make sure you ran:
```bash
pip3 install --break-system-packages open3d scipy matplotlib pillow pyyaml
```

### 4. TF "Jump back in time" or odometry stall
The EKF node has `publish_tf: false` configured by default, and `odom_tf_broadcaster.py` manages the `map -> odom -> base_footprint` transformation chain with strict monotonic time-guards. If you modify EKF configs, ensure `publish_tf` remains `false`.
