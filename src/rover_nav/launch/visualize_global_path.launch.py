#!/usr/bin/env python3
"""Publish the global path from global_path_planner.py's START/WAYPOINTS and
view it in RViz -- no map, hardware, or joystick needed.

  ros2 launch rover_nav visualize_global_path.launch.py
  ros2 launch rover_nav visualize_global_path.launch.py rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'rviz', 'global_path_view.rviz'
    ])

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Also launch RViz pre-configured to show the raw/smoothed path + waypoints',
    )

    publisher_node = Node(
        package='rover_nav',
        executable='publish_global_path.py',
        name='publish_global_path',
        output='screen',
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
        rviz_arg,
        publisher_node,
        rviz_node,
    ])
