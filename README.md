# rover_sim

Self-contained ROS 2 packages for the HSM Aries Leapone rover: URDF/xacro model,
Gazebo sim, and the D435i depth-camera pipeline. Trimmed down to just what's
needed to spawn the robot and see depth data in RViz — no real-hardware
(ODrive CAN) dependencies, no unrelated packages.

## What's included

- `rover_description/` — robot model (URDF/xacro), meshes, Gazebo worlds
  (Mars Yard terrain), RViz configs, and the `ros_gz_bridge` config that
  publishes the D435i camera's color/depth/point-cloud data from Gazebo to
  ROS 2 topics.
- `rover_controllers/` — `ros2_control` diff-drive config used by the
  simulated wheels (the URDF references this package directly, so it's
  required even if you don't drive the rover).
- `rover_nav/` — EKF localization config (also hard-required: the launch
  file always starts the EKF node).

> **This branch (`mukul`) does not include the 275MB full-res terrain scan**
> (`rover_description/models/marsyard_mesh/marsyard2026_visual.obj`) — too
> large for a normal git push. Without it, the robot still spawns and the
> depth camera still works, but the depth camera won't return any range
> data over open ground (no terrain mesh for it to render against) — it'll
> only see the rover's own geometry. Ask for that file separately if you
> need real terrain depth data; drop it in that same folder and it'll be
> picked up automatically (referenced directly by
> `rover_description/worlds/marsyard.sdf`).

## 1. Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- A GPU capable of running Gazebo's Ogre2 renderer — the terrain is a
  ~1.7M-triangle textured mesh, integrated graphics will be slow.

## 2. Install required packages

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joy \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  python3-colcon-common-extensions
```

## 3. Build

```bash
mkdir -p ~/rover_ws/src
cp -r rover_description rover_controllers rover_nav ~/rover_ws/src/
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Run the simulation

```bash
ros2 launch rover_description my_robot.launch.py
```

Spawns the rover on the Mars Yard terrain (survey point S1, facing +Y) with
Gazebo's `gz_ros2_control` driving the wheels.

## 5. View depth data in RViz

In a second terminal:

```bash
source ~/rover_ws/install/setup.bash
rviz2 -d ~/rover_ws/install/rover_description/share/rover_description/rviz/urdf_config.rviz
```

This config already has RobotModel, TF, and a Camera display pointed at
`/camera/color/image_raw`. For the actual depth/point-cloud data:

- **Depth image**: `/camera/depth/image_rect_raw` (`sensor_msgs/Image`)
- **Point cloud**: `/camera/depth/points` (`sensor_msgs/PointCloud2`) — add
  a PointCloud2 display in RViz pointed at this topic to see the 3D data.
- **Camera info**: `/camera/color/camera_info`, `/camera/depth/camera_info`

The depth sensor's far clip plane is set to **3.0m** (see
`rover_description/urdf/d435i.xacro`, `camera_depth` sensor block) — points
beyond 3m from the camera aren't rendered. Adjust there if you need more
range.

## Notes

- Driving: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/rover_controller/cmd_vel`
  (the controller expects `TwistStamped`, hence `stamped:=true`).
- `rover_description/models/marsyard_mesh/marsyard2026_visual.obj` (~275MB,
  the full-res terrain scan) is what the depth camera actually renders
  against for the ground — without it the camera won't return depth data
  over open terrain, only over the rover itself.
