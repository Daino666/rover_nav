#!/usr/bin/env python3
"""Single launch: Gazebo sim (rover only, no MoveIt/arm) + the full Nav2
navigation stack (global planner AND local planner) + RViz.

Contrast full_sim.launch.py, which keeps the arm/gripper/MoveIt stack up and
only brings up nav2_planning_sim.launch.py's planning-only half (no
controller_server/bt_navigator -- paths get computed but nothing drives
them). This launch is for actually navigating: spawns the rover with
use_moveit:=false (no arm control, no MoveIt RViz -- see aries's
my_robot.launch.py), then nav2_navigation_sim.launch.py (map_server,
global_costmap, planner_server, smoother_server, local_costmap,
controller_server, behavior_server, bt_navigator), then RViz pre-configured
with map + global/local costmap + global/local plan displays
(nav2_full_view.rviz) -- no MoveIt panels since there's no arm to plan for.

Once this is up and settled, send it goals with send_waypoints.py.

Usage:
  ros2 launch rover_nav full_nav_sim.launch.py
  ros2 launch rover_nav full_nav_sim.launch.py world:=soil_world.sdf nav_delay:=25.0

Raise nav_delay if you still see TF/costmap errors right after Gazebo comes
up -- there's no clean signal from aries's launch file to hook an event
handler off from the outside. With use_moveit:=false this settles faster
than full_sim.launch.py since there's no arm/gripper/MoveIt bring-up, but
controller_manager + joint_state_broadcaster still take a few seconds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map_file = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'maps', 'marsyard2026_occupancy.yaml'
    ])

    world_arg = DeclareLaunchArgument(
        'world', default_value='marsyard2026.sdf',
        choices=['sandbox_world.sdf', 'soil_world.sdf', 'marsyard2026.sdf', 'maintenance_world.sdf'],
        description='World file in aries/worlds -- defaults to the world matching this '
                    "package's default map",
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_file,
        description='Full path to the occupancy map yaml file',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz pre-configured with map/costmaps/plans (nav2_full_view.rviz)',
    )
    nav_delay_arg = DeclareLaunchArgument(
        'nav_delay', default_value='15.0',
        description='Seconds to wait after starting Gazebo before bringing up the Nav2 '
                    'stack, so the sim EKF/controller_manager have time to settle before '
                    'global_costmap and local_costmap go looking for TF.',
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('aries'), 'launch', 'my_robot.launch.py'])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_moveit': 'false',
            'use_joystick': 'false',
        }.items(),
    )

    nav2_navigation_sim = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rover_nav'), 'launch', 'nav2_navigation_sim.launch.py'
                    ])
                ]),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'rviz': LaunchConfiguration('rviz'),
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        rviz_arg,
        nav_delay_arg,
        gazebo_sim,
        nav2_navigation_sim,
    ])
