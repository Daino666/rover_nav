cmake_minimum_required(VERSION 3.5)
project(rover_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
)

# Install Python executable nodes (scripts)
install(
  PROGRAMS
    scripts/Rover_control_Joy.py
    scripts/LeapOne_test_encoders.py
    scripts/Odom.py
    scripts/global_path_planner.py
    scripts/rover_controller_pure_pursuit.py
    scripts/publish_global_path.py
    scripts/publish_test_path.py
    scripts/plan_global_path.py
    scripts/test_path_loader.py
    scripts/test_path_viz.py
    scripts/cmd_vel_arbiter.py
    scripts/check_heading.py
    scripts/map_odom_broadcaster.py
    scripts/imu_yaw_zero.py
    scripts/imu_noise_check.py
    scripts/yaw_source_compare.py
    scripts/imu_deep_log.py
    scripts/odrive_fault_log.py
    scripts/Rover_path_controller/pcl_obstacle_detector.py
    scripts/Rover_path_controller/obstacle_detector_node.py
    scripts/Rover_path_controller/obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  scripts/Sounds
  DESTINATION share/${PROJECT_NAME}/sounds/
)

# Real-world path-tracking test courses. Installed as data (not programs) so
# test_path_loader.py finds them via the package share directory at runtime;
# regenerate with scripts/test_paths/generate_test_paths.py and rebuild.
install(DIRECTORY
  scripts/test_paths/output/
  DESTINATION share/${PROJECT_NAME}/test_paths
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    odometry_stationary
    test/test_odometry_stationary.py
    TIMEOUT 60
  )
endif()

ament_package()
