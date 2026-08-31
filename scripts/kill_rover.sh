#!/usr/bin/env bash
# Stop the rover and tear down the whole ROS stack.
#
# Ctrl-C on the launch terminal is often not enough: ros2 launch forwards SIGINT
# to its children, but any node that ignores or outlives it gets re-parented to
# init and keeps running -- ODrive CAN nodes in particular then spin on a socket
# whose owner is gone, logging "Bad file descriptor" thousands of times a second
# and holding the interface against the next launch. This kills the launch
# first, gives it a moment to shut down cleanly, then force-kills whatever is
# left by name.
set -u

echo "stopping the run (zero twist latched) ..."
timeout 10 ros2 service call /planner/stop std_srvs/srv/Trigger >/dev/null 2>&1

echo "asking the launch to exit ..."
pgrep -f "full_hardware.launch.py|nav2_planning.launch.py|aries_hardware.launch.py" \
  | xargs -r kill 2>/dev/null
sleep 6

echo "force-killing anything left ..."
PATTERN="full_hardware|nav2_planning|aries_hardware|cmd_vel_arbiter|cmd_vel_odrive_bridge"
PATTERN="$PATTERN|odrive_can_node|aruco_detect|aruco_pose|virtual_aruco|ekf_node|imu_yaw_zero"
PATTERN="$PATTERN|map_odom_broadcaster|realsense2_camera_node|microstrain|ros2_control_node"
PATTERN="$PATTERN|robot_state_publisher|move_group|moveit|servo_node|teleop_relay|joy_node"
PATTERN="$PATTERN|rviz2|spawner|wheel_joint|interactive_marker|map_server|planner_server"
PATTERN="$PATTERN|smoother_server|lifecycle_manager|static_transform_publisher"
pgrep -f "$PATTERN" | xargs -r kill -9 2>/dev/null
sleep 3

LEFT=$(pgrep -cf "$PATTERN" 2>/dev/null || echo 0)
echo
if [ "$LEFT" -eq 0 ]; then
  echo "  all stopped"
else
  echo "  $LEFT process(es) still alive:"
  pgrep -af "$PATTERN" | grep -v "bin/bash" | sed 's/--ros-args.*//' | cut -c1-90 | sed 's/^/    /'
fi
echo "  can0: $(ip -br link show can0 2>/dev/null | awk '{print $2}' || echo 'not found')"
