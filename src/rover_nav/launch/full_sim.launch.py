#!/usr/bin/env python3
"""Single launch for Gazebo sim + Nav2 planning-only stack together.

Brings up, in order:
  1. aries's Gazebo sim directly (robot model, ros2_control, sim EKF) --
     not through aries_bringup's my_robot.launch.py wrapper, since that
     wrapper only forwards joystick args and silently drops `world`.
  2. rover_nav's nav2_planning_sim.launch.py (map_server -> planner_server
     -> smoother_server -> lifecycle_manager, plus odom_tf_broadcaster.py
     bridging /odometry/filtered into TF), held back by `nav_delay`
     seconds so it doesn't start probing for map -> base_footprint before
     Gazebo has spawned the robot and the sim EKF has started publishing
     /odometry/filtered. Racing that produces canTransform "frame does
     not exist" spam from global_costmap -- see NAV2_PLANNING_NOTES.md.

RViz: aries's my_robot.launch.py already starts one (gated by `use_gui`,
default true) with moveit.rviz -- Map + the three nav2 path topics
(/unsmoothed_plan, /plan_smoothed, /pure_pursuit/path) are now baked into
that same config, so this launch does NOT start a second RViz instance via
nav2_planning_sim.launch.py's own `rviz` arg (always passed false). One
window shows both the arm/MoveIt view and the nav2 map/paths.

Usage:
  ros2 launch rover_nav full_sim.launch.py
  ros2 launch rover_nav full_sim.launch.py use_gui:=false   # no RViz at all
  ros2 launch rover_nav full_sim.launch.py world:=soil_world.sdf nav_delay:=12.0

If nav2 still races the sim on your machine (e.g. base_footprint TF errors
right after Gazebo comes up), raise nav_delay -- there's no clean signal
from aries's launch file to hook an event handler off from the outside.
Note this robot's sim bring-up includes arm + gripper + MoveIt on top of
the rover base, so it's slower to settle than a bare-rover sim would be.
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
    nav_delay_arg = DeclareLaunchArgument(
        'nav_delay', default_value='20.0',
        description='Seconds to wait after starting Gazebo before bringing up the nav2 '
                    'planning stack, so the sim EKF has time to spawn and start publishing '
                    '/odometry/filtered before global_costmap goes looking for TF. This robot '
                    'brings up arm + gripper + MoveIt + all ros2_control controllers, not just '
                    'the rover base, so controller activation alone can take ~18s -- measured '
                    'empirically, raise it further if you still see "unconnected trees" TF '
                    'errors right after nav2 starts.',
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('aries'), 'launch', 'my_robot.launch.py'])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    nav2_planning_sim = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rover_nav'), 'launch', 'nav2_planning_sim.launch.py'
                    ])
                ]),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'rviz': 'false',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        nav_delay_arg,
        gazebo_sim,
        nav2_planning_sim,
    ])
