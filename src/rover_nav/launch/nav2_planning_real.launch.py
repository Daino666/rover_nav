#!/usr/bin/env python3
"""
Nav2 planning stack (map_server -> global_costmap -> planner_server
(SmacPlanner2D) -> smoother_server (SimpleSmoother)) wired up for the real
rover: brings up localization.launch.py (encoder odom + BNO055 IMU + EKF,
which publishes the live odom->base_footprint transform itself --
ekf_config.yaml has publish_tf: true) plus map_odom_broadcaster.py (the
static map->odom alignment transform -- see that script's docstring for how
to set map_to_odom_yaw_deg on competition day). Together those give the
costmap a real, moving map->base_footprint chain, so unlike
nav2_planning.launch.py this needs no static_tf node.

Contrast nav2_planning_sim.launch.py, which uses odom_tf_broadcaster.py
instead because the sim's EKF has publish_tf forced False.

Does NOT bring up AMCL/controller_server/bt_navigator -- planning only, same
as nav2_planning.launch.py. Trigger it the same way: nav2_simple_commander's
BasicNavigator.getPath()/smoothPath(), or `ros2 action send_goal` on
/compute_path_to_pose and /smooth_path.

Usage:
  ros2 launch rover_nav nav2_planning_real.launch.py
  ros2 launch rover_nav nav2_planning_real.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'config', 'nav2_planning_params.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'rviz', 'nav2_path_view.rviz'
    ])
    default_map_file = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'maps', 'marsyard2026_occupancy.yaml'
    ])

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_file,
        description='Full path to the occupancy map yaml file',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Also launch RViz pre-configured to show the map + planned paths live',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_nav'), 'launch', 'localization.launch.py'
            ])
        ])
    )

    map_odom_broadcaster_node = Node(
        package='rover_nav',
        executable='map_odom_broadcaster.py',
        name='map_odom_broadcaster',
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': LaunchConfiguration('map')}],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[params_file],
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
        map_arg,
        rviz_arg,
        localization,
        map_odom_broadcaster_node,
        map_server_node,
        planner_server_node,
        smoother_server_node,
        lifecycle_manager_node,
        rviz_node,
    ])
