cmake_minimum_required(VERSION 3.5)
project(rover_perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoDetection.msg"
  "msg/ArucoDetectionArray.msg"
  "msg/Landmark.msg"
  "msg/LandmarkArray.msg"
  "msg/RoverPositionFix.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
