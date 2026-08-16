#!/usr/bin/env bash
# Installs all dependencies for the obstacle_detection package (ROS2 Jazzy).
# Review before running -- this uses sudo for system package installs.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=== Updating apt package lists ==="
sudo apt-get update

echo "=== Installing ROS2/apt dependencies declared in package.xml (via rosdep) ==="
echo "Covers: rclpy, sensor_msgs, sensor_msgs_py, visualization_msgs,"
echo "pcl_ros, rclcpp_components, launch, launch_ros, python3-sklearn,"
echo "realsense2_camera, topic_tools."
cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y

echo ""
echo "=== Optional: GPU/Open3D experiment ==="
echo "NOT required for the active detection pipeline (CPU path is what's"
echo "actually in use -- GPU testing showed no real speedup). Only needed"
echo "if working on obstacle_filter_gpu.py."
read -p "Set up the Open3D GPU venv too? [y/N] " setup_gpu
if [[ "$setup_gpu" =~ ^[Yy]$ ]]; then
    python3 -m venv --system-site-packages "$WS_DIR/venv_open3d"
    "$WS_DIR/venv_open3d/bin/pip" install --upgrade pip
    "$WS_DIR/venv_open3d/bin/pip" install open3d
    echo "Open3D venv ready at $WS_DIR/venv_open3d"
    echo "Run GPU node scripts with $WS_DIR/venv_open3d/bin/python3 directly,"
    echo "not 'ros2 run' (which uses the system interpreter and can't see open3d)."
fi

echo ""
echo "=== Building the workspace ==="
cd "$WS_DIR"
colcon build --packages-select obstacle_detection

echo ""
echo "Done. Source with: source $WS_DIR/install/setup.bash"
