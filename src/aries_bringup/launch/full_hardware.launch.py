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
        DeclareLaunchArgument("drive_command_timeout_s", default_value="0.25"),
        DeclareLaunchArgument("drive_max_linear_mps", default_value="0.45"),
        DeclareLaunchArgument("drive_max_angular_rps", default_value="2.10"),
        DeclareLaunchArgument("drive_max_wheel_rps", default_value="1.50"),
        DeclareLaunchArgument(
            "drive_wheel_accel_rps2", default_value="3.0",
            description=(
                "ROS-side ramp limit (rev/s^2) on commanded wheel speed -- NOT an "
                "ODrive firmware setting (that lives on each ODrive's own flash, "
                "current_soft_max/current_hard_max, not touched here). Lower = "
                "gentler/smoother acceleration, less likely to break wheels loose on "
                "loose terrain; higher = snappier response. This was declared several "
                "launch files down (aries_drive/drive.launch.py) but never reachable "
                "from here until 2026-09-03 -- previously the only way to change it "
                "was editing that file's default directly."
            ),
        ),
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
            "lookahead_distance",
            default_value="0.5",
            description=(
                "Pure-pursuit lookahead (m), used as the starting value before "
                "lookahead_dynamic (on by default, see below) takes over every tick. "
                "cmd_vel_arbiter reads this ONCE at construction, so `ros2 param set` on "
                "the running node reports success and changes nothing -- it has to be "
                "given here. Only matters on its own if lookahead_dynamic:=false."
            ),
        ),
        DeclareLaunchArgument(
            "lookahead_dynamic",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Widen the lookahead automatically instead of driving on a fixed "
                "lookahead_distance -- short on straights for tight tracking, wider "
                "when cross-track error or upcoming curvature demands it. This is what "
                "absorbs an ArUco pose snap: a fixed 0.5 m lookahead pinned curvature "
                "to the 2.0 clamp on a 1.1 m snap and the rover circled instead of "
                "rejoining the path. lookahead_min/lookahead_max below are tuned "
                "around the validated 0-0.7 m ArUco correction range -- see "
                "cmd_vel_arbiter.py's own declare_parameter comments for the full "
                "derivation. "
                "Back ON (2026-09-03): the finalized W1/W9/W3/W5 competition route "
                "is tight and pivot-heavy (p90 curvature 1.43 1/m, p95 2.00 1/m -- "
                "right at the planner's clamp), where a fixed lookahead has no way to "
                "ease off before a corner. lookahead_curvature_gain below was retuned "
                "for this route's real curvature at the same time."
            ),
        ),
        DeclareLaunchArgument("lookahead_min", default_value="0.4"),
        DeclareLaunchArgument("lookahead_max", default_value="0.7"),
        DeclareLaunchArgument("lookahead_error_gain", default_value="1.0"),
        DeclareLaunchArgument(
            "lookahead_curvature_gain", default_value="0.8",
            description=(
                "Retuned 2026-09-03 from 0.267 (calibrated against the old "
                "MIN_TURNING_RADIUS=1.5m / 0.667 1/m test-course curvature) to 0.8 "
                "(lookahead_min * new MIN_TURNING_RADIUS=0.5m's 2.0 1/m), matching the "
                "much tighter curvature the current competition route actually drives. "
                "See cmd_vel_arbiter.py's declare_parameter comment for the full "
                "derivation and the retune formula if MIN_TURNING_RADIUS changes again."
            ),
        ),
        DeclareLaunchArgument(
            "local_planner",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Add the local planner layer (rover_nav/local_planner.py). It watches "
                "the global route cmd_vel_arbiter is chasing and, when an obstacle from "
                "/obstacles/array sits on it, publishes a detour on /local_plan for the "
                "arbiter to drive instead, handing the global route back once past. "
                "Replaces the /obstacle_detected stop-gate with /local_plan/hold, which "
                "fails closed the same way. Needs the detection pipeline running "
                "(rover_nav/obstacle_detection.launch.py) -- without it the planner "
                "holds rather than assuming the way is clear."
            ),
        ),
        DeclareLaunchArgument(
            "rover_length", default_value="0.95",
            description=(
                "Rover footprint length (m), local_planner only. With rover_width it "
                "sets the footprint disc, hypot(L/2, W/2), that every detour clearance "
                "is derived from. MEASURE the real rover before a real run."
            ),
        ),
        DeclareLaunchArgument("rover_width", default_value="0.75"),
        DeclareLaunchArgument(
            "min_distance", default_value="0.6",
            description=(
                "Live clearance (m) from the footprint to an obstacle's surface at which "
                "the local planner abandons the detour, stops and reverses to "
                "safe_distance before replanning on the other side."
            ),
        ),
        DeclareLaunchArgument("safe_distance", default_value="1.0"),
        DeclareLaunchArgument(
            "obstacle_stop_enabled",
            default_value="true",
            description=(
                "Gate motion on /obstacle_detected. Fails CLOSED: with this true and "
                "nothing publishing that topic, cmd_vel_arbiter holds forever rather "
                "than driving blind -- which is correct, but looks exactly like a stuck "
                "rover if you are not reading its log. Set false for a controlled test "
                "with no obstacle pipeline running (the node then warns loudly that "
                "forward obstacles will not stop it), or start "
                "rover_nav/obstacle_detection.launch.py alongside."
            ),
        ),
        DeclareLaunchArgument(
            "start_aruco",
            default_value="false",
            description=(
                "Start the ArUco landmark localization pipeline: the detector "
                "(rover_detection/aruco_detect_roverpos) and the pose corrector "
                "(aruco_pose_reset). REQUIRES the front camera -- pass "
                "enable_front_camera:=true front_camera_serial:=<serial>, or the "
                "detector sits waiting on topics nothing publishes. The wrist "
                "camera is NOT interchangeable here: camera_offset_* in "
                "aruco_localization_params.yaml describes the front mount."
            ),
        ),
        DeclareLaunchArgument(
            "aruco_snap",
            default_value="false",
            description=(
                "Let aruco_pose_reset actually call /set_pose on a landmark sighting, "
                "overwriting the EKF's accumulated drift with the landmark-derived "
                "position. Default false runs it in observe-only mode: it logs every "
                "correction it WOULD make and touches nothing, which is how to check "
                "the fixes are sane before letting them move the rover."
            ),
        ),
        DeclareLaunchArgument(
            "aruco_max_correction_m",
            default_value="0.8",
            description=(
                "Sanity gate on aruco_pose_reset: refuse any single correction bigger "
                "than this rather than snapping the rover to it. Default 0.8 m is the "
                "field-validated 0-0.7 m correction range plus a little headroom -- "
                "raise it deliberately if genuine drift ever exceeds that, and re-check "
                "lookahead_max (above), which is tuned assuming corrections stay under "
                "0.7 m."
            ),
        ),
        DeclareLaunchArgument(
            "aruco_min_correction_m",
            default_value="0.05",
            description="Ignore corrections smaller than this -- below the pipeline's own noise floor.",
        ),
        DeclareLaunchArgument(
            "aruco_min_interval_s",
            default_value="2.0",
            description="Minimum seconds between accepted snaps, so the corrector doesn't fight the path follower.",
        ),
        DeclareLaunchArgument(
            "aruco_start_point",
            default_value="S1",
            description=(
                "Which survey start line the rover is on. Published once at startup as "
                "a seed (landmark_id=-1) so consumers have something before the first "
                "real sighting; never repeated, and ignored by aruco_pose_reset unless "
                "accept_startup_seed is set."
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
            "pivots_csv",
            default_value="",
            description=(
                "The <name>_pivots.csv written alongside path_csv, ONLY when "
                "plan_global_path.py needed an in-place pivot to connect two waypoints "
                "(no arrival heading served both getting there and getting to the next "
                "point). Optional -- most routes have none, and the file is only written "
                "when at least one pivot is needed. When given, cmd_vel_arbiter stops and "
                "physically rotates in place at each recorded joint instead of curving "
                "through it. This is new (2026-09-03), untested on real hardware -- verify "
                "at low speed on the bench before trusting it on a competition run."
            ),
        ),
        DeclareLaunchArgument(
            "pivot_max_angular_rps", default_value="0.4",
            description="Angular rate (rad/s) used while executing an in-place pivot.",
        ),
        DeclareLaunchArgument(
            "pivot_tolerance_deg", default_value="3.0",
            description="Heading error within which a pivot is considered done.",
        ),
        DeclareLaunchArgument(
            "stop_at_waypoints",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Stop at each waypoint in waypoints_csv (except the start and the "
                "final/loop-closing point) and wait for a human to confirm before "
                "continuing, instead of path_csv's normal drive-straight-through. Run "
                "`ros2 run rover_nav wait_and_continue.py` in a separate terminal and "
                "press Enter each time to send it on -- or call "
                "`ros2 service call /planner/continue std_srvs/srv/Trigger` directly. "
                "Stops at the CLOSEST APPROACH actually reached to each point (not a "
                "fixed radius), so it lands as close as the real tracking error allows."
            ),
        ),
        DeclareLaunchArgument(
            "waypoint_stop_margin_m", default_value="0.05",
            description=(
                "How far past a waypoint's closest approach (stop_at_waypoints only) "
                "before treating it as passed and stopping."
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
                "drive_command_timeout_s": LaunchConfiguration("drive_command_timeout_s"),
                "drive_max_linear_mps": LaunchConfiguration("drive_max_linear_mps"),
                "drive_max_angular_rps": LaunchConfiguration("drive_max_angular_rps"),
                "drive_max_wheel_rps": LaunchConfiguration("drive_max_wheel_rps"),
                "drive_wheel_accel_rps2": LaunchConfiguration("drive_wheel_accel_rps2"),
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
                "pivots_csv": ParameterValue(
                    LaunchConfiguration("pivots_csv"), value_type=str),
                "pivot_max_angular_rps": ParameterValue(
                    LaunchConfiguration("pivot_max_angular_rps"), value_type=float),
                "pivot_tolerance_deg": ParameterValue(
                    LaunchConfiguration("pivot_tolerance_deg"), value_type=float),
                "stop_at_waypoints": ParameterValue(
                    LaunchConfiguration("stop_at_waypoints"), value_type=bool),
                "waypoint_stop_margin_m": ParameterValue(
                    LaunchConfiguration("waypoint_stop_margin_m"), value_type=float),
                "path_frame": ParameterValue(
                    LaunchConfiguration("path_frame"), value_type=str),
                "obstacle_stop_enabled": ParameterValue(
                    LaunchConfiguration("obstacle_stop_enabled"), value_type=bool),
                "local_plan_enabled": ParameterValue(
                    LaunchConfiguration("local_planner"), value_type=bool),
                "lookahead_distance": ParameterValue(
                    LaunchConfiguration("lookahead_distance"), value_type=float),
                "lookahead_dynamic": ParameterValue(
                    LaunchConfiguration("lookahead_dynamic"), value_type=bool),
                "lookahead_min": ParameterValue(
                    LaunchConfiguration("lookahead_min"), value_type=float),
                "lookahead_max": ParameterValue(
                    LaunchConfiguration("lookahead_max"), value_type=float),
                "lookahead_error_gain": ParameterValue(
                    LaunchConfiguration("lookahead_error_gain"), value_type=float),
                "lookahead_curvature_gain": ParameterValue(
                    LaunchConfiguration("lookahead_curvature_gain"), value_type=float),
            }],
        ),

        # The local planner layer. Deliberately a separate node that never
        # publishes /cmd_vel: cmd_vel_arbiter refuses to drive if a second
        # publisher appears there, which is what keeps exactly one owner of the
        # motors. This one publishes geometry (/local_plan) and intent
        # (/local_plan/hold), and the arbiter above executes it.
        Node(
            condition=IfCondition(LaunchConfiguration("local_planner")),
            package="rover_nav",
            executable="local_planner.py",
            name="local_planner",
            output="screen",
            parameters=[{
                "rover_length": ParameterValue(
                    LaunchConfiguration("rover_length"), value_type=float),
                "rover_width": ParameterValue(
                    LaunchConfiguration("rover_width"), value_type=float),
                "min_distance": ParameterValue(
                    LaunchConfiguration("min_distance"), value_type=float),
                "safe_distance": ParameterValue(
                    LaunchConfiguration("safe_distance"), value_type=float),
            }],
        ),

        # ArUco landmark localization. Two nodes, deliberately separate:
        #   aruco_detect_roverpos  sees a marker -> publishes a map-frame position
        #   aruco_pose_reset       takes that position -> /set_pose on the EKF
        # Splitting them keeps "where am I" measurable on its own (run the
        # detector alone and watch the fixes) from "act on it", which is the
        # part that moves the rover's estimate and wants to be trusted first.
        #
        # The corrector SNAPS rather than fuses: wheel odometry drifts without
        # bound, a landmark sighting does not, so the sighting overwrites the
        # estimate instead of being averaged into it. Fusing would need a
        # covariance nobody has measured for this pipeline yet.
        Node(
            condition=IfCondition(LaunchConfiguration("start_aruco")),
            package="rover_detection",
            executable="aruco_detect_roverpos",
            name="aruco_localization_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("rover_detection"),
                    "config",
                    "aruco_localization_params.yaml",
                ]),
                {"start_point": ParameterValue(
                    LaunchConfiguration("aruco_start_point"), value_type=str)},
            ],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("start_aruco")),
            package="rover_detection",
            executable="aruco_pose_reset",
            name="aruco_pose_reset",
            output="screen",
            parameters=[{
                "enabled": ParameterValue(
                    LaunchConfiguration("aruco_snap"), value_type=bool),
                "max_correction_m": ParameterValue(
                    LaunchConfiguration("aruco_max_correction_m"), value_type=float),
                "min_correction_m": ParameterValue(
                    LaunchConfiguration("aruco_min_correction_m"), value_type=float),
                "min_interval_s": ParameterValue(
                    LaunchConfiguration("aruco_min_interval_s"), value_type=float),
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
