#!/usr/bin/env python3
"""Preview one real-world test path in RViz -- no rover, hardware, or
joystick needed.

  ros2 launch rover_nav view_test_path.launch.py test_path:=circle
  ros2 launch rover_nav view_test_path.launch.py test_path:=infinity rviz:=false

Shows the course and its numbered ground markers at their nominal placement
(origin at odom's origin, +x along odom's +x). Check the shape and the marker
numbering against scripts/test_paths/output/<name>.jpg here before pacing the
course out on the ground; drive it with
  ros2 launch aries_bringup full_hardware.launch.py test_path:=<name>
which republishes the same topics with the course anchored to the rover's
actual pose at run start.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'rviz', 'global_path_view.rviz'
    ])

    test_path_arg = DeclareLaunchArgument(
        'test_path', default_value='straight_line',
        choices=['straight_line', 'lane_change', 'circle', 'circle_transition', 'infinity'],
        description='Which course from scripts/test_paths/output/ to show',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Also launch RViz pre-configured to show the path + numbered markers',
    )

    publisher_node = Node(
        package='rover_nav',
        executable='publish_test_path.py',
        name='publish_test_path',
        output='screen',
        # value_type=str is load-bearing, not boilerplate: without it the
        # parameter is type-inferred, and ROS's number parser accepts
        # "infinity" as the float inf (C strtod), so test_path:=infinity
        # would come through as a double and fail the STRING declaration.
        parameters=[{
            'test_path': ParameterValue(LaunchConfiguration('test_path'), value_type=str),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        test_path_arg,
        rviz_arg,
        publisher_node,
        rviz_node,
    ])
