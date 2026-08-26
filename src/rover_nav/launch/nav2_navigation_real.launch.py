#!/usr/bin/env python3
"""Full Nav2 navigation stack for the real rover -- global planner AND local
planner, unlike nav2_planning_real.launch.py which is planning-only.

Reuses nav2_planning_real.launch.py unchanged for map_server ->
global_costmap -> planner_server (SmacPlanner2D) -> smoother_server ->
lifecycle_manager_planning (which in turn assumes full_hardware.launch.py
is already running and providing the real map->odom->base_footprint TF
chain -- see that file's docstring). Adds the local-planner half
nav2_planning_real.launch.py deliberately never had: local_costmap ->
controller_server (RegulatedPurePursuitController, publishing /cmd_vel) ->
behavior_server (spin/backup/wait recoveries) -> bt_navigator, under their
own lifecycle_manager_navigation. Together these actually DRIVE the robot
along a planned path via NavigateToPose/NavigateThroughPoses -- point
send_waypoints.py at this once it's up.

Also brings up the real front-camera perception chain feeding
local_costmap's obstacle_layer for live obstacle avoidance:
obstacle_detection.launch.py's RealSense driver (native pointcloud, unlike
sim which has no points topic bridged and needs depth_to_pointcloud.py to
synthesize one) -> PassThrough + SOR filters -> /pcl/denoised (see
nav2_local_planner_params.yaml).

nav2_local_planner_params.yaml has use_sim_time: true baked into every
node section (it was written for nav2_navigation_sim.launch.py first) --
overridden back to false here per-node, same mechanism
nav2_navigation_sim.launch.py uses in the other direction.

Usage:
  ros2 launch aries_bringup full_hardware.launch.py use_gui:=false
  ros2 launch rover_nav nav2_navigation_real.launch.py
  ros2 launch rover_nav nav2_navigation_real.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_params_file = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'config', 'nav2_planning_params.yaml'
    ])
    local_planner_params_file = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'config', 'nav2_local_planner_params.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'rviz', 'nav2_full_view.rviz'
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
        description='Also launch RViz pre-configured to show map/costmaps/plans live',
    )
    # launch_arguments set LaunchConfigurations in the shared, non-scoped launch
    # context -- planning_stack's own launch_arguments={'rviz': 'false'} below
    # (suppressing nav2_planning_real.launch.py's separate RViz) would otherwise
    # clobber this same 'rviz' key before rviz_node's condition reads it later
    # in this file. Snapshot the real value into a private key first. Same
    # trick as nav2_navigation_sim.launch.py.
    capture_rviz = SetLaunchConfiguration('_nav2_full_view_rviz', LaunchConfiguration('rviz'))

    planning_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_nav'), 'launch', 'nav2_planning_real.launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'rviz': 'false',
        }.items(),
    )

    obstacle_filter_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_nav'), 'launch', 'obstacle_detection.launch.py'
            ])
        ]),
    )

    real_time_override = {'use_sim_time': False}

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[planning_params_file, local_planner_params_file, real_time_override],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[local_planner_params_file, real_time_override],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[local_planner_params_file, real_time_override],
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[local_planner_params_file, real_time_override],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[real_time_override],
        condition=IfCondition(LaunchConfiguration('_nav2_full_view_rviz')),
    )

    return LaunchDescription([
        map_arg,
        rviz_arg,
        capture_rviz,
        planning_stack,
        obstacle_filter_chain,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_navigation_node,
        rviz_node,
    ])
