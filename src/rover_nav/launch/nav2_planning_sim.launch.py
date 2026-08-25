#!/usr/bin/env python3
"""
Nav2 planning stack (map_server -> global_costmap -> planner_server
(SmacPlanner2D) -> smoother_server (SimpleSmoother)) wired up for
simulation: assumes the sim itself (e.g. my_robot_rover.launch.py) is
already running and publishing /odometry/filtered, and adds
odom_tf_broadcaster.py to bridge that into TF -- a static map->odom
identity plus a live odom->base_footprint broadcast from
/odometry/filtered. That workaround exists because the sim's
ekf_filter_node has publish_tf forced False (see my_robot.launch.py's
docstring for why nothing else publishes odom->base_footprint there).

Contrast nav2_planning_real.launch.py, where the real rover's EKF publishes
its own TF directly (publish_tf: true) and map_odom_broadcaster.py handles
map->odom instead.

Does NOT bring up AMCL/controller_server/bt_navigator -- planning only, same
as nav2_planning.launch.py. Trigger it the same way: nav2_simple_commander's
BasicNavigator.getPath()/smoothPath(), or `ros2 action send_goal` on
/compute_path_to_pose and /smooth_path.

Usage:
  ros2 launch rover_nav nav2_planning_sim.launch.py
  ros2 launch rover_nav nav2_planning_sim.launch.py rviz:=true
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

    odom_tf_broadcaster_node = Node(
        package='rover_nav',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
    )

    # params_file hardcodes use_sim_time: false throughout (shared with
    # nav2_planning_real.launch.py, where that's correct) -- override it true
    # here since this is the sim-only launch and Gazebo's /clock runs at a
    # documented ~0.54 real-time factor (see my_robot.launch.py). Leaving
    # these on wall-clock while TF is stamped in sim time would drift them
    # apart over any real session.
    sim_time_override = {'use_sim_time': True}

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': LaunchConfiguration('map')}, sim_time_override],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, sim_time_override],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, sim_time_override],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[params_file, sim_time_override],
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
        odom_tf_broadcaster_node,
        map_server_node,
        planner_server_node,
        smoother_server_node,
        lifecycle_manager_node,
        rviz_node,
    ])
