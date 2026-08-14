#!/usr/bin/env python3
"""
real_robot.launch.py

Brings up the full real-rover stack:
  - robot_state_publisher   (my_robot.urdf.xacro, mirrors display.launch.xml)
  - ros2_control_node        (ODriveHardwareInterface on can0 via rover_control.xacro)
  - joint_state_broadcaster  (spawned after controller_manager is ready)
  - rover_controller         (diff_drive_controller, odom TF enabled for RViz)
  - joy_node                 (gamepad input)
  - teleop_twist_joy         (joy → /rover_controller/cmd_vel)
  - ekf_node                 (robot_localization, rover_nav/config/ekf_config.yaml)

CAN interface is can0 (hardcoded in rover_control.xacro).

Usage:
  ros2 launch rover_description real_robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------

    urdf_path = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'urdf', 'my_robot.urdf.xacro'
    ])

    diff_controller_yaml = PathJoinSubstitution([
        FindPackageShare('rover_controllers'), 'config', 'diff_controller.yaml'
    ])

    joy_config_yaml = PathJoinSubstitution([
        FindPackageShare('rover_controllers'), 'config', 'joyConfig.yaml'
    ])

    joy_teleop_yaml = PathJoinSubstitution([
        FindPackageShare('rover_controllers'), 'config', 'joyTeleop.yaml'
    ])

    ekf_config_yaml = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'config', 'ekf_config.yaml'
    ])

    # ------------------------------------------------------------------
    # Robot descriptions
    #
    # display_description  – mirrors display.launch.xml exactly (no args,
    #                         use_sim defaults to true).  Used by RSP and
    #                         RViz so the robot renders identically to the
    #                         stand-alone display launch.
    #
    # hw_description       – use_sim:=false so rover_control.xacro selects
    #                         the ODriveHardwareInterface block.  Passed
    #                         only to ros2_control_node.
    # ------------------------------------------------------------------

    display_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    hw_description = ParameterValue(
        Command(['xacro ', urdf_path, ' use_sim:=false']),
        value_type=str
    )

    # ------------------------------------------------------------------
    # Robot State Publisher  (mirrors display.launch.xml)
    # ------------------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': display_description,
            'use_sim_time': False,
        }]
    )

    # Passive joints (rockers, boggies, aux rockers) have no ODrive encoder —
    # joint_state_broadcaster only publishes the 6 wheel joints.  Without this
    # node RSP defaults those revolute joints to 0.0 silently, which makes the
    # suspension geometry appear collapsed/wrong in RViz.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': display_description,
            'source_list': ['/joint_states'],
            'use_sim_time': False,
        }]
    )

    # ------------------------------------------------------------------
    # ros2_control node  — loads ODriveHardwareInterface on CAN bus
    # enable_odom_tf is set in diff_controller.yaml so the
    # diff_drive_controller broadcasts the odom → base_footprint TF.
    # ------------------------------------------------------------------

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': hw_description},
            diff_controller_yaml,
        ],
    )

    # ------------------------------------------------------------------
    # Controller spawners (deferred until controller_manager is ready)
    # ------------------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    rover_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rover_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Separate handler so rover_controller_spawner is an explicit top-level
    # action in the LaunchDescription — nested actions inside on_start/on_exit
    # lists are unreliable in some Humble builds and can silently get dropped.
    # Fires once joint_state_broadcaster has actually finished loading/
    # activating (not a fixed wall-clock guess), so it's no longer a race
    # against variable ODrive/CAN handshake time.
    delay_rover_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rover_controller_spawner],
        )
    )

    # ------------------------------------------------------------------
    # Joystick input
    # ------------------------------------------------------------------

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_config_yaml],
    )

    # teleop_twist_joy publishes TwistStamped (publish_stamped_twist: true in
    # joyTeleop.yaml) directly on /rover_controller/cmd_vel which the
    # diff_drive_controller subscribes to.
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[joy_teleop_yaml],
        remappings=[('/cmd_vel', '/rover_controller/cmd_vel')],
    )

    # ------------------------------------------------------------------
    # IMU restamp — the MPU6050/micro-ROS firmware stamps /imu with its
    # own boot-relative clock, not synced ROS time. ekf_node's predict
    # step uses message timestamps to compute dt, so mixing that with
    # /rover_controller/odom's real-time stamps produces a multi-billion
    # -second dt and blows up /odometry/filtered's x/y to huge garbage
    # values even at rest. This re-stamps /imu with the host clock
    # before EKF sees it (see rover_description/scripts/imu_restamp.py).
    # ------------------------------------------------------------------

    imu_restamp_node = Node(
        package='rover_description',
        executable='imu_restamp.py',
        name='imu_restamp',
        output='screen',
    )

    # ------------------------------------------------------------------
    # EKF — fuses wheel odometry + IMU into a filtered odom estimate
    # ------------------------------------------------------------------

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_yaml],
    )

    # ------------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------------

    return LaunchDescription([
        # Core robot stack
        robot_state_publisher,
        joint_state_publisher_node,
        ros2_control_node,
        delay_jsb,
        delay_rover_ctrl,

        # Joystick control
        joy_node,
        teleop_node,

        # State estimation
        imu_restamp_node,
        ekf_node,
    ])
