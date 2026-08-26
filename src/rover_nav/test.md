```bash
cd ~/test/src
colcon build --packages-select rover_nav
source ~/test/src/install/setup.bash

ros2 launch aries_bringup full_hardware.launch.py use_gui:=false

ros2 service call /check_rover_hardware std_srvs/srv/Trigger

ros2 launch rover_nav nav2_navigation_real.launch.py rviz:=true

ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator

ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /pcl/denoised --once
ros2 run tf2_ros tf2_echo map base_footprint

ros2 run rover_nav send_waypoints.py \
  --start S1 \
  --points W6 W5 12.5,-3.2 \
  --use-sim-time false
```
