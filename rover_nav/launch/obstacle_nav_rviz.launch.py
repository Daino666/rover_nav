#!/usr/bin/env python3
"""
Lightweight Zero-Gazebo Simulation & Visualization Launch File

Designed for low-spec laptops:
- Launches Nav2 planning on the Mars Yard occupancy grid map
- Launches robot_state_publisher with the full rover URDF
- Launches kinematic_mock_sim (0% GPU, ~0.5% CPU) for TF, odometry & synthetic camera points
- Launches pcl_obstacle_detector for 3D bounding boxes, safety clearance & HUD markers
- Launches Pure_pursuit_Gazebo with dynamic reactive 3D obstacle avoidance
- Launches RViz2 with complete multi-layer visualization

Usage:
  ros2 launch rover_nav obstacle_nav_rviz.launch.py
"""

import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_rover_nav = FindPackageShare('rover_nav')
    pkg_rover_desc = get_package_share_directory('rover_description')

    params_file = PathJoinSubstitution([pkg_rover_nav, 'config', 'nav2_planning_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_rover_nav, 'rviz', 'obstacle_nav.rviz'])
    default_map_file = PathJoinSubstitution([pkg_rover_nav, 'maps', 'marsyard2026_occupancy.yaml'])
    default_path_file = PathJoinSubstitution([pkg_rover_nav, 'maps', 'marsyard2026_tour_waypoints.csv'])

    # Process Xacro for Robot Description in RViz
    xacro_file = os.path.join(pkg_rover_desc, 'urdf', 'my_robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_file,
        description='Full path to occupancy map yaml file'
    )
    path_file_arg = DeclareLaunchArgument(
        'path_file', default_value=default_path_file,
        description='Full path to waypoints CSV file'
    )

    # 1. Robot State Publisher (Visualizes Rover Model in RViz)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': False}]
    )

    # 1b. Joint State Publisher — publishes zero positions for all 12 rover joints
    #     so robot_state_publisher can compute the full TF chain:
    #     base_footprint -> base_link -> wheels -> camera_depth_frame etc.
    #     Without this the Rover 3D Model turns red and camera_depth_frame is missing.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 2. Nav2 Map Server (Mars Yard Occupancy Grid)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False}],
    )

    # 3. Nav2 Planner Server (SmacPlanner2D with Obstacle Layer)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    # 4. Nav2 Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    # 5. Robust Lifecycle Auto-Activator
    lifecycle_activator_node = Node(
        package='rover_nav',
        executable='lifecycle_autostart.py',
        name='lifecycle_autostart',
        output='screen',
        parameters=[{'use_sim_time': False, 'node_names': ['map_server', 'planner_server', 'smoother_server']}],
    )

    # 6. Lightweight Kinematic & Camera Simulator
    mock_sim_node = Node(
        package='rover_nav',
        executable='kinematic_mock_sim.py',
        name='kinematic_mock_sim',
        output='screen',
        parameters=[{'use_sim_time': False, 'initial_x': 0.0, 'initial_y': 0.0, 'initial_yaw': 1.5707963267948966}]
    )

    # 7. 3D Obstacle Detector (Downsampling, Ground Removal, DBSCAN, Markers, ObstacleArray)
    # Remap /pcl/denoised to /camera/depth/points for direct perception
    obstacle_detector_node = Node(
        package='rover_nav',
        executable='pcl_obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
        remappings=[('/pcl/denoised', '/camera/depth/points')],
        parameters=[{'use_sim_time': False}]
    )

    # 8. Pure Pursuit Path Follower with 3D Obstacle Avoidance
    pure_pursuit_node = Node(
        package='rover_nav',
        executable='Pure_pursuit_Gazebo.py',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'lookahead_distance': 1.0,
            'enable_obstacle_avoidance': True,
            'safety_margin': 0.25,
            'slowdown_dist': 1.8,
            'stop_dist': 0.55,
            'path_file': LaunchConfiguration('path_file'),
            'map_yaml': LaunchConfiguration('map'),
        }]
    )

    # 9. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        map_arg,
        path_file_arg,
        rsp_node,
        joint_state_publisher_node,
        map_server_node,
        planner_server_node,
        smoother_server_node,
        lifecycle_activator_node,
        mock_sim_node,
        obstacle_detector_node,
        pure_pursuit_node,
        rviz_node
    ])
