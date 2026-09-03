#!/usr/bin/env python3
"""Compatibility wrapper around the modular rover bringup.

The full arm/gripper launch already owns robot_state_publisher and RViz, so this
wrapper starts only rover hardware, localization, teleop, and encoder-derived
wheel joint states.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aries_bringup"),
                    "launch",
                    "rover_drive.launch.py",
                ]
            )
        ),
        launch_arguments={
            "rover_hardware_protocol": LaunchConfiguration(
                "rover_hardware_protocol"
            ),
            "can_interface": LaunchConfiguration("can_interface"),
            "can_bitrate": LaunchConfiguration("can_bitrate"),
            "setup_can": LaunchConfiguration("setup_can"),
            "use_joystick": LaunchConfiguration("use_joystick"),
            "use_joy_node": LaunchConfiguration("use_joy_node"),
            "joy_driver": LaunchConfiguration("joy_driver"),
            "joy_layout": LaunchConfiguration("joy_layout"),
            "joy_dev": LaunchConfiguration("joy_dev"),
            "use_cmd_vel_bridge": "true",
            "drive_auto_arm": LaunchConfiguration("drive_auto_arm"),
            "use_cmd_vel_relay": "true",
            "use_robot_description": "false",
            "use_wheel_joint_publisher": "true",
            "use_rviz": "false",
            "start_checker": "false",
            "start_imu_driver": LaunchConfiguration("start_imu_driver"),
            "use_imu": LaunchConfiguration("use_imu"),
            "imu_port": LaunchConfiguration("imu_port"),
            "imu_baudrate": LaunchConfiguration("imu_baudrate"),
            "imu_frame": LaunchConfiguration("imu_frame"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "map_to_odom_x": LaunchConfiguration("map_to_odom_x"),
            "map_to_odom_y": LaunchConfiguration("map_to_odom_y"),
            "map_to_odom_yaw_deg": LaunchConfiguration("map_to_odom_yaw_deg"),
            "drive_command_timeout_s": LaunchConfiguration("drive_command_timeout_s"),
            "drive_max_linear_mps": LaunchConfiguration("drive_max_linear_mps"),
            "drive_max_angular_rps": LaunchConfiguration("drive_max_angular_rps"),
            "drive_max_wheel_rps": LaunchConfiguration("drive_max_wheel_rps"),
            "drive_wheel_accel_rps2": LaunchConfiguration("drive_wheel_accel_rps2"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_to_odom_x", default_value=""),
            DeclareLaunchArgument("map_to_odom_y", default_value=""),
            DeclareLaunchArgument("map_to_odom_yaw_deg", default_value=""),
            DeclareLaunchArgument(
                "rover_hardware_protocol",
                default_value="auto",
                choices=["auto", "odrive", "mock_hardware"],
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("can_bitrate", default_value="250000"),
            DeclareLaunchArgument("setup_can", default_value="true"),
            DeclareLaunchArgument("drive_auto_arm", default_value="false"),
            DeclareLaunchArgument("use_joystick", default_value="true"),
            DeclareLaunchArgument("use_joy_node", default_value="false"),
            DeclareLaunchArgument(
                "joy_driver",
                default_value="game_controller_node",
                choices=["game_controller_node", "joy_node"],
            ),
            DeclareLaunchArgument(
                "joy_layout",
                default_value="auto",
                choices=[
                    "auto",
                    "dongle",
                    "bluetooth",
                    "game_controller",
                    "passthrough",
                ],
            ),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument(
                "start_imu_driver",
                default_value="auto",
                choices=["auto", "true", "false"],
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="auto",
                choices=[
                    "auto",
                    "true",
                    "false",
                    "microstrain",
                ],
            ),
            DeclareLaunchArgument("imu_port", default_value="/dev/microstrain_main"),
            DeclareLaunchArgument("imu_baudrate", default_value="115200"),
            DeclareLaunchArgument("imu_frame", default_value="imu_frame"),
            DeclareLaunchArgument(
                "imu_topic", default_value="/microstrain/ekf/imu/data"
            ),
            DeclareLaunchArgument("drive_command_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("drive_max_linear_mps", default_value="0.45"),
            DeclareLaunchArgument("drive_max_angular_rps", default_value="2.10"),
            DeclareLaunchArgument("drive_max_wheel_rps", default_value="1.50"),
            DeclareLaunchArgument(
                "drive_wheel_accel_rps2", default_value="3.0",
                description=(
                    "ROS-side ramp limit (rev/s^2) on commanded wheel speed -- NOT an "
                    "ODrive firmware setting. Lower = gentler/smoother acceleration "
                    "(less likely to break wheels loose on loose terrain), higher = "
                    "snappier response. Applied in cmd_vel_odrive_bridge.py's ramp(), "
                    "independently per side."
                ),
            ),
            rover,
        ]
    )
