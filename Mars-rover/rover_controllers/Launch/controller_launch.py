#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, LogInfo


def generate_launch_description():

    # gz_ros2_control Gazebo plugin auto-loads, configures, and activates all
    # controllers from the YAML when the robot spawns in my_robot.launch.py.
    # No spawner nodes are needed here.

    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[launch.substitutions.PathJoinSubstitution([
            launch_ros.substitutions.FindPackageShare('rover_controllers'),
            'config', 'joyConfig.yaml'
        ])]
    )

    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[launch.substitutions.PathJoinSubstitution([
            launch_ros.substitutions.FindPackageShare('rover_controllers'),
            'config', 'joyTeleop.yaml'
        ])],
        remappings=[('/cmd_vel', '/rover_controller/cmd_vel')]
    )

    return launch.LaunchDescription([

        joy_node,
        RegisterEventHandler(OnProcessStart(
            target_action=joy_node,
            on_start=[LogInfo(msg='[STARTED] joy_node is up')]
        )),

        teleop_node,
        RegisterEventHandler(OnProcessStart(
            target_action=teleop_node,
            on_start=[LogInfo(msg='[STARTED] teleop_twist_joy is up')]
        )),

    ])