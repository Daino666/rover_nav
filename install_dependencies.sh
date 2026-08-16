#!/usr/bin/env bash
# Installs everything needed to build and run this bundle on a fresh ROS2
# Jazzy machine: rover_description (Gazebo sim + Mars Yard 2026 world),
# rover_controllers, rover_nav (EKF localization, Nav2 global planner, pure
# pursuit), obstacle_detection, and the vendored ros_odrive hardware driver
# packages (built automatically even though they're unused in sim -- they're
# a dependency of rover_controllers/rover_nav's package.xml).
#
# Assumes: Ubuntu 24.04 with ROS2 Jazzy already installed
# (https://docs.ros.org/en/jazzy/Installation.html) and this repo's four
# folders (Mars-rover, rover_nav, obstacle_detection, marsyard) already
# cloned into ~/jazzy_ws/src/ -- see README.md for the exact command.
#
# Review before running -- this uses sudo for system package installs.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== Updating apt package lists ==="
sudo apt-get update

echo ""
echo "=== Gazebo Harmonic + ROS-Gazebo bridge (if not already installed) ==="
sudo apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control

echo ""
echo "=== Nav2 (global planner: map_server, planner_server, smoother_server, ==="
echo "=== lifecycle_manager, nav2_simple_commander) -- rover_nav's package.xml ==="
echo "=== doesn't declare these, so they're not picked up by rosdep below.    ==="
sudo apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

echo ""
echo "=== robot_localization (EKF) ==="
sudo apt-get install -y ros-jazzy-robot-localization

echo ""
echo "=== rosdep install for everything declared in package.xml across all ==="
echo "=== four packages (rover_description, rover_controllers, rover_nav,  ==="
echo "=== obstacle_detection, and the vendored ros_odrive packages).       ==="
echo ""
echo "zed_description is skipped: rover_description depends on it for an"
echo "unused camera xacro (already commented out in my_robot.urdf.xacro)."
echo "If you end up needing that specific xacro for the new rover's camera,"
echo "get zed_description from https://github.com/stereolabs/zed-ros2-wrapper"
echo "instead of via rosdep."
cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --skip-keys zed_description

echo ""
echo "=== Python packages for the standalone tools/ scripts (occupancy grid ==="
echo "=== generation + tour planning) -- these run outside colcon/rclpy, so ==="
echo "=== plain pip against the system interpreter is fine here. ==="
echo ""
echo "IMPORTANT: this system's Python is externally-managed (PEP 668) on a"
echo "stock Ubuntu 24.04 -- pip needs --break-system-packages or these will"
echo "fail with 'externally-managed-environment'."
pip3 install --break-system-packages \
    numpy scipy pyyaml matplotlib pillow open3d

echo ""
echo "=== obstacle_detection's own extra dependency (scikit-learn via apt, ==="
echo "=== not pip -- pip's build can fail without system BLAS/LAPACK) ==="
sudo apt-get install -y python3-sklearn

echo ""
echo "=== Building the workspace ==="
cd "$WS_DIR"
colcon build --symlink-install

echo ""
echo "Done. Source with: source $WS_DIR/install/setup.bash"
echo "Then see README.md for the test/run commands."
