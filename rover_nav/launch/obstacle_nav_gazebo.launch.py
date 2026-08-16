#!/usr/bin/env python3
"""
Full Gazebo Simulation + Nav2 Mars Yard Planning + 3D Obstacle Avoidance Launch

Launches:
1. Gazebo Sim with Mars Yard terrain & RealSense D435i depth camera
2. ROS-Gazebo Parameter Bridge for topics & TF
3. Nav2 Planning Stack on Mars Yard Occupancy Map (with live dynamic ObstacleLayer)
4. 3D Obstacle Perception Pipeline (PassThrough -> SOR Filter -> DBSCAN Obstacle Detector)
5. Pure Pursuit Path Follower with 3D Obstacle Avoidance
6. Pre-configured RViz2 display

TF tree (same as RViz stack):
  map ──(static identity)──> odom ──(odom_tf_broadcaster)──> base_footprint
  base_footprint ──(robot_state_publisher)──> base_link ──> wheels / camera_depth_frame
  EKF publish_tf is disabled to prevent the self-consuming jump-back-in-time loop.

Usage:
  ros2 launch rover_nav obstacle_nav_gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_rover_nav = FindPackageShare('rover_nav')
    pkg_rover_desc = FindPackageShare('rover_description')

    params_file = PathJoinSubstitution([pkg_rover_nav, 'config', 'nav2_planning_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_rover_nav, 'rviz', 'obstacle_nav.rviz'])
    default_map_file = PathJoinSubstitution([pkg_rover_nav, 'maps', 'marsyard2026_occupancy.yaml'])
    default_path_file = PathJoinSubstitution([pkg_rover_nav, 'maps', 'marsyard2026_tour_waypoints.csv'])

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_file,
        description='Full path to occupancy map yaml file'
    )
    path_file_arg = DeclareLaunchArgument(
        'path_file', default_value=default_path_file,
        description='Full path to waypoints CSV file'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn Y')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.30', description='Spawn Z')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='1.5707963267948966', description='Spawn yaw (rad)')

    # 1. Gazebo Simulation with Rover & Bridge
    #    Provides: robot_state_publisher, joint_state_broadcaster (via gz_ros2_control),
    #    EKF (disabled publish_tf to prevent TF loop), Gazebo bridge, virtual_differential
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_rover_desc, 'launch', 'my_robot.launch.py'])
        ]),
        launch_arguments={
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items()
    )

    # 1b. TF broadcaster: publishes map→odom (static with spawn pose) and odom→base_footprint
    #     (dynamic, from /odometry/filtered). EKF's own publish_tf is forced False to avoid self-consuming TF loop.
    odom_tf_broadcaster_node = Node(
        package='rover_nav',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': 0.0,
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }]
    )

    # 2. Nav2 Map Server (Mars Yard Occupancy Grid)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}],
    )

    # 3. Nav2 Planner Server (SmacPlanner2D with Obstacle Layer)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # 4. Nav2 Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # 5. Standard Nav2 Lifecycle Manager for Map, Planner, and Smoother
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'planner_server', 'smoother_server'],
        }],
    )

    # 6. Obstacle Detection Sim Pipeline (Passthrough + SOR + DBSCAN Detector)
    obstacle_detection_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_rover_nav, 'launch', 'obstacle_detection_sim.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 7. Pure Pursuit Path Follower with Obstacle Avoidance
    pure_pursuit_node = Node(
        package='rover_nav',
        executable='Pure_pursuit_Gazebo.py',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'lookahead_distance': 0.65,
            'enable_obstacle_avoidance': True,
            'safety_margin': 0.20,
            'slowdown_dist': 1.8,
            'stop_dist': 0.40,
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'path_file': LaunchConfiguration('path_file'),
            'map_yaml': LaunchConfiguration('map'),
        }]
    )

    # 8. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        map_arg,
        path_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo_sim,
        # TF: delay slightly so EKF has started publishing /odometry/filtered first
        TimerAction(period=5.0, actions=[odom_tf_broadcaster_node]),
        map_server_node,
        planner_server_node,
        smoother_server_node,
        lifecycle_manager_node,
        obstacle_detection_sim,
        pure_pursuit_node,
        rviz_node
    ])
