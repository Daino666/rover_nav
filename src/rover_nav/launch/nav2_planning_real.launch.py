#!/usr/bin/env python3
"""
Nav2 planning stack (map_server -> global_costmap -> planner_server
(SmacPlanner2D) -> smoother_server (SimpleSmoother)) wired up for the real
rover.

Does NOT start localization. It requires
`ros2 launch aries_bringup full_hardware.launch.py` (or at least
`rover_drive.launch.py`) to already be running, which brings up the real
localization stack on its own -- aries_localization/localization.launch.py:
MicroStrain 3DM-GX5-AHRS IMU + Odom.py (wheel encoders over CAN) + EKF
(base_link_frame overridden to base_footprint, publish_tf: true) -- plus
map_odom_broadcaster.py for the static map->odom alignment (see that
script's docstring for how to set map_to_odom_yaw_deg on competition day).
Together those already give a real, moving map->base_footprint chain.

This file used to bring up its own second copy of that chain (rover_nav's
own localization.launch.py, which drove a BNO055 IMU on /dev/ttyUSB0 --
not the rover's actual IMU per aries_imu/imu.launch.py, which is the
MicroStrain) plus its own map_odom_broadcaster.py. Running that alongside
full_hardware.launch.py meant two Odom.py processes both publishing /odom,
two nodes both named ekf_filter_node, and two both named
map_odom_broadcaster -- a real conflict, not a redundancy. Removed; this
launch file now only adds the planning nodes on top of an
already-running full_hardware.launch.py.

Contrast nav2_planning_sim.launch.py, which brings up its own TF chain
(odom_tf_broadcaster.py) because there is no equivalent always-on real
bringup in sim.

Does NOT bring up AMCL/controller_server/bt_navigator -- planning only, same
as nav2_planning.launch.py. Trigger it the same way: nav2_simple_commander's
BasicNavigator.getPath()/smoothPath(), or `ros2 action send_goal` on
/compute_path_to_pose and /smooth_path.

Usage:
  ros2 launch aries_bringup full_hardware.launch.py   # brings up localization
  ros2 launch rover_nav nav2_planning_real.launch.py
  ros2 launch rover_nav nav2_planning_real.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        map_server_node,
        planner_server_node,
        smoother_server_node,
        lifecycle_manager_node,
        rviz_node,
    ])
