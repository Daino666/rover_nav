"""Launches ONLY the ArUco landmark detection/localization node -- does NOT
launch realsense2_camera or any camera driver. Pure consumer of
already-published camera topics, so it doesn't lock down camera settings
for anyone else who also needs the camera. Start the camera separately
(standalone for testing now; as part of a combined navigation/obstacle
avoidance/marker detection launch file later), then run this alongside it.

REQUIRES the camera to be launched with align_depth.enable:=true.

To bring up just the camera for testing this standalone, in a separate
terminal:
    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

Usage:
    ros2 launch rover_detection aruco_detection.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [get_package_share_directory('rover_detection'), 'config', 'aruco_localization_params.yaml'])

    aruco_localization = Node(
        package='rover_detection',
        executable='aruco_detect_roverpos',
        name='aruco_localization_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([aruco_localization])
