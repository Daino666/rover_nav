#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_joystick", default_value="true"),
        DeclareLaunchArgument("joy_driver", default_value="game_controller_node", choices=["game_controller_node", "joy_node"]),
        DeclareLaunchArgument("joy_layout", default_value="auto", choices=["auto", "dongle", "bluetooth", "game_controller", "passthrough"]),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joystick_control_mode", default_value="servo", choices=["move_group", "servo"]),

        DeclareLaunchArgument("gripper_type", default_value="v2", choices=["old", "new", "v2"]),
        DeclareLaunchArgument("finger_type", default_value="bucket", choices=["bucket", "maintenance", "probe"]),
        DeclareLaunchArgument("hardware_protocol", default_value="auto", choices=["auto", "rebel", "mock_hardware", "gazebo"]),
        DeclareLaunchArgument("arm_hardware_protocol", default_value="auto", choices=["auto", "rebel", "mock_hardware", "gazebo"]),
        DeclareLaunchArgument("gripper_hardware_protocol", default_value="auto", choices=["auto", "rebel", "mock_hardware", "gazebo"]),
        DeclareLaunchArgument("serial_port", default_value="/dev/serial/by-id/usb-Teensyduino_USB_Serial_16739090-if00"),
        DeclareLaunchArgument("suppress_rebel_logs", default_value="true"),
        DeclareLaunchArgument("suppress_moveit_execution_logs", default_value="true"),
        # Cameras default OFF so the single D435i stays free for the standalone
        # obstacle-detection pipeline (rover_nav/obstacle_detection.launch.py),
        # which starts its own RealSense driver. Two drivers cannot share the
        # device. To hand the camera back to this launch, pass
        #   enable_depth_sensor:=auto enable_front_camera:=auto
        # or restore both default_value="auto" here and in aries_hardware.launch.py.
        DeclareLaunchArgument("enable_depth_sensor", default_value="false", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("gripper_camera_serial", default_value=""),
        DeclareLaunchArgument("enable_front_camera", default_value="false", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("front_camera_serial", default_value=""),
        DeclareLaunchArgument(
            "use_static_wheel_joint_publisher",
            default_value="false",
            description=(
                "Publish zero-valued wheel joints from the arm stack. Keep "
                "false when the rover encoder-backed publisher is active."
            ),
        ),

        DeclareLaunchArgument("start_rover", default_value="true"),
        DeclareLaunchArgument("rover_hardware_protocol", default_value="auto", choices=["auto", "odrive", "mock_hardware"]),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("setup_rover_can", default_value="true"),
        DeclareLaunchArgument("drive_auto_arm", default_value="true"),
        DeclareLaunchArgument(
            "use_rover_imu",
            default_value="auto",
            choices=[
                "auto",
                "true",
                "false",
                "microstrain",
            ],
        ),
        DeclareLaunchArgument(
            "rover_imu_port", default_value="/dev/microstrain_main"
        ),
        DeclareLaunchArgument("rover_imu_baudrate", default_value="115200"),
        DeclareLaunchArgument("rover_imu_frame", default_value="imu_frame"),
        DeclareLaunchArgument(
            "rover_imu_topic", default_value="/microstrain/ekf/imu/data"
        ),
        DeclareLaunchArgument("use_rover_joy_node", default_value="false"),

        DeclareLaunchArgument(
            "start_pure_pursuit",
            default_value="true",
            description=(
                "Start cmd_vel_arbiter (rover_nav), which drives the waypoints in "
                "global_path_planner.py's WAYPOINTS by publishing /cmd_vel -- the same "
                "way LB-gated teleop does, so cmd_vel_teleop_relay yields to it "
                "automatically. Only takes effect when start_rover is also true."
            ),
        ),
        DeclareLaunchArgument(
            "pure_pursuit_autostart",
            default_value="false",
            description=(
                "Start driving the waypoints immediately once armed and localized, "
                "without waiting for /planner/start. Keep false for physical testing; "
                "call `ros2 service call /planner/start std_srvs/srv/Trigger` when ready."
            ),
        ),
        DeclareLaunchArgument(
            "test_path",
            default_value="",
            choices=["", "straight_line", "lane_change", "circle", "circle_transition", "infinity"],
            description=(
                "Drive one of rover_nav's real-world test courses instead of "
                "global_path_planner.py's WAYPOINTS -- a dense, already-shaped path "
                "driven continuously (no per-waypoint stops) to measure path-tracking "
                "accuracy on the physical rover. Empty (default) keeps the waypoint "
                "behaviour. Preview a course first with "
                "`ros2 launch rover_nav view_test_path.launch.py test_path:=<name>`; see "
                "rover_nav/README.md's \"Real-world path-tracking tests\" for the field procedure."
            ),
        ),
        DeclareLaunchArgument(
            "map_to_odom_x", default_value="",
            description="map->odom x offset (m). Empty keeps MAP_TO_ODOM_X from "
                        "global_path_planner.py.",
        ),
        DeclareLaunchArgument(
            "map_to_odom_y", default_value="",
            description="map->odom y offset (m). Empty keeps MAP_TO_ODOM_Y.",
        ),
        DeclareLaunchArgument(
            "map_to_odom_yaw_deg", default_value="",
            description=(
                "How far the rover's odom frame is rotated inside the competition "
                "map frame, in degrees. This is what aligns a map-frame path_csv to "
                "the ground: if the rover's forward axis points along the map's +Y, "
                "this is 90. It depends on how the rover was parked at boot (odom's "
                "yaw is zeroed there by imu_yaw_zero.py), so it is a per-run value. "
                "Empty keeps MAP_TO_ODOM_YAW_DEG from global_path_planner.py."
            ),
        ),
        DeclareLaunchArgument(
            "path_csv",
            default_value="",
            description=(
                "Drive a dense global path CSV from rover_nav's plan_global_path.py, "
                "crossing every waypoint without stopping. Empty (default) keeps the "
                "WAYPOINTS stop-and-go behaviour. Mutually exclusive with test_path."
            ),
        ),
        DeclareLaunchArgument(
            "waypoints_csv",
            default_value="",
            description=(
                "The <name>_waypoints.csv written alongside path_csv. Optional; supplies "
                "the per-waypoint crossing-distance report at the end of the run."
            ),
        ),
        DeclareLaunchArgument(
            "path_frame",
            default_value="map",
            choices=["map", "odom"],
            description=(
                "Frame path_csv is expressed in. 'map' applies the map->odom correction "
                "from map_odom_broadcaster.py (correct for plan_global_path.py output); "
                "'odom' drives the coordinates verbatim."
            ),
        ),
        DeclareLaunchArgument(
            "test_path_anchor",
            default_value="start_pose",
            choices=["start_pose", "odom_origin"],
            description=(
                "Where a test_path course is placed. 'start_pose' anchors its origin and "
                "heading to the rover's live pose when /planner/start is called (park the "
                "rover on the marked origin, then start). 'odom_origin' drives the CSV "
                "coordinates verbatim in odom, valid only if the rover has not moved since "
                "localization came up."
            ),
        ),

        DeclareLaunchArgument("start_checker", default_value="true"),
        # Preserve the top-level choice before rover_drive_auto.launch.py sets
        # its own nested start_checker argument to false. Included launch
        # configurations share context and would otherwise disable this checker.
        SetLaunchConfiguration(
            "_start_full_hardware_checker",
            LaunchConfiguration("start_checker"),
        ),
        DeclareLaunchArgument("checker_interval", default_value="4.0"),

        # Arm + gripper + MoveIt/RViz + shared joy_node.
        # Existing aries_hardware.launch.py handles real-or-mock arm/gripper auto behavior.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("aries_bringup"),
                    "launch",
                    "aries_hardware.launch.py",
                ])
            ),
            launch_arguments={
                "use_gui": LaunchConfiguration("use_gui"),
                "use_joystick": LaunchConfiguration("use_joystick"),
                "joy_driver": LaunchConfiguration("joy_driver"),
                "joy_layout": LaunchConfiguration("joy_layout"),
                "joy_dev": LaunchConfiguration("joy_dev"),
                "joystick_control_mode": LaunchConfiguration("joystick_control_mode"),
                "gripper_type": LaunchConfiguration("gripper_type"),
                "finger_type": LaunchConfiguration("finger_type"),
                "hardware_protocol": LaunchConfiguration("hardware_protocol"),
                "arm_hardware_protocol": LaunchConfiguration("arm_hardware_protocol"),
                "gripper_hardware_protocol": LaunchConfiguration("gripper_hardware_protocol"),
                "serial_port": LaunchConfiguration("serial_port"),
                "suppress_rebel_logs": LaunchConfiguration("suppress_rebel_logs"),
                "suppress_moveit_execution_logs": LaunchConfiguration("suppress_moveit_execution_logs"),
                "enable_depth_sensor": LaunchConfiguration("enable_depth_sensor"),
                "gripper_camera_serial": LaunchConfiguration("gripper_camera_serial"),
                "enable_front_camera": LaunchConfiguration("enable_front_camera"),
                "front_camera_serial": LaunchConfiguration("front_camera_serial"),
                "use_wheel_joint_publisher": LaunchConfiguration(
                    "use_static_wheel_joint_publisher"
                ),
            }.items(),
        ),

        # Rover real-or-mock backend.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("aries_bringup"),
                    "launch",
                    "rover_drive_auto.launch.py",
                ])
            ),
            condition=IfCondition(LaunchConfiguration("start_rover")),
            launch_arguments={
                "rover_hardware_protocol": LaunchConfiguration("rover_hardware_protocol"),
                "can_interface": LaunchConfiguration("can_interface"),
                "setup_can": LaunchConfiguration("setup_rover_can"),
                "drive_auto_arm": LaunchConfiguration("drive_auto_arm"),
                "use_joystick": LaunchConfiguration("use_joystick"),
                # joy_node is already started by aries_hardware when use_joystick:=true.
                "use_joy_node": LaunchConfiguration("use_rover_joy_node"),
                "joy_driver": LaunchConfiguration("joy_driver"),
                "joy_layout": LaunchConfiguration("joy_layout"),
                "joy_dev": LaunchConfiguration("joy_dev"),
                "use_imu": LaunchConfiguration("use_rover_imu"),
                "imu_port": LaunchConfiguration("rover_imu_port"),
                "imu_baudrate": LaunchConfiguration("rover_imu_baudrate"),
                "imu_frame": LaunchConfiguration("rover_imu_frame"),
                "imu_topic": LaunchConfiguration("rover_imu_topic"),
                "map_to_odom_x": LaunchConfiguration("map_to_odom_x"),
                "map_to_odom_y": LaunchConfiguration("map_to_odom_y"),
                "map_to_odom_yaw_deg": LaunchConfiguration("map_to_odom_yaw_deg"),
            }.items(),
        ),

        # Waypoint follower: publishes /cmd_vel directly, same as LB-gated teleop
        # would -- cmd_vel_teleop_relay (started above by rover_drive_auto) yields
        # to it automatically once it sees this node's /cmd_vel publisher. Motion
        # still requires the drive armed (/aries_drive/enable) and either
        # pure_pursuit_autostart:=true or a /planner/start call.
        Node(
            condition=IfCondition(LaunchConfiguration("start_pure_pursuit")),
            package="rover_nav",
            executable="cmd_vel_arbiter.py",
            name="cmd_vel_arbiter",
            output="screen",
            parameters=[{
                "autostart": LaunchConfiguration("pure_pursuit_autostart"),
                # value_type=str is load-bearing, not boilerplate: without it
                # the value is type-inferred, and ROS's number parser accepts
                # "infinity" as the float inf (C strtod), so
                # test_path:=infinity would arrive as a double and fail the
                # node's STRING parameter declaration.
                "test_path": ParameterValue(
                    LaunchConfiguration("test_path"), value_type=str),
                "test_path_anchor": ParameterValue(
                    LaunchConfiguration("test_path_anchor"), value_type=str),
                "path_csv": ParameterValue(
                    LaunchConfiguration("path_csv"), value_type=str),
                "waypoints_csv": ParameterValue(
                    LaunchConfiguration("waypoints_csv"), value_type=str),
                "path_frame": ParameterValue(
                    LaunchConfiguration("path_frame"), value_type=str),
            }],
        ),

        # Separate checker.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("aries_bringup"),
                    "launch",
                    "full_hardware_checker.launch.py",
                ])
            ),
            condition=IfCondition(
                LaunchConfiguration("_start_full_hardware_checker")
            ),
            launch_arguments={
                "checker_interval": LaunchConfiguration("checker_interval"),
                "serial_port": LaunchConfiguration("serial_port"),
                "can_interface": LaunchConfiguration("can_interface"),
                "use_imu": LaunchConfiguration("use_rover_imu"),
                "imu_port": LaunchConfiguration("rover_imu_port"),
                "imu_frame": LaunchConfiguration("rover_imu_frame"),
                "imu_topic": LaunchConfiguration("rover_imu_topic"),
            }.items(),
        ),
    ])
