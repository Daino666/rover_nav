import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)

from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError


def generate_launch_description():

    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------

    rover_description_share = get_package_share_directory('rover_description')
    try:
        gz_ros2_control_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    except PackageNotFoundError:
        gz_ros2_control_lib = None

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------

    urdf_path = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'urdf', 'my_robot.urdf.xacro'
    ])

    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'config', 'gazebo_bridge.yaml'
    ])

    virtual_diff_config_path = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'config', 'virtual_differential.yaml'
    ])

    gz_gui_config_path = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'config', 'gz_gui_config.config'
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'worlds', 'marsyard.sdf'
    ])

    ekf_config_path = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'config', 'ekf_config.yaml'
    ])

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='true = Gazebo simulation  |  false = real ODrive hardware'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock (set false for real robot wall clock)'
    )

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn Y')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.30', description='Spawn Z')
    # S1 is the coordinate-system origin (0,0) and the Mars Yard's local Y axis
    # is "forward into the yard" (S1->S2 calibration direction); yaw the spawn
    # 90 deg so the rover's body +X (forward) faces +Y instead of +X.
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='1.5707963267948966', description='Spawn yaw (rad)')

    # ------------------------------------------------------------------
    # Launch configurations
    # ------------------------------------------------------------------

    use_sim      = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # ------------------------------------------------------------------
    # Gazebo resource path
    # ------------------------------------------------------------------

    gz_resource_path = EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')

    try:
        zed_description_share = get_package_share_directory('zed_description')
    except PackageNotFoundError:
        zed_description_share = None

    gz_sim_resource_path_value = [
        os.path.dirname(rover_description_share), ':',
        rover_description_share, '/models:',
        rover_description_share, '/worlds:',
        os.path.expanduser('~/.gz/fuel'), ':',
        gz_resource_path
    ]
    if zed_description_share is not None:
        gz_sim_resource_path_value.insert(0, ':')
        gz_sim_resource_path_value.insert(0, os.path.dirname(zed_description_share))

    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_sim_resource_path_value
    )

    # ------------------------------------------------------------------
    # NVIDIA GPU — PRIME render offload for hybrid-graphics laptops
    # ------------------------------------------------------------------

    gz_sim_system_plugin_path_value = [EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')]
    if gz_ros2_control_lib is not None:
        gz_sim_system_plugin_path_value.insert(0, ':')
        gz_sim_system_plugin_path_value.insert(0, gz_ros2_control_lib)

    set_gz_sim_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gz_sim_system_plugin_path_value
    )

    has_nvidia = os.path.exists('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    nvidia_env_actions = []
    if has_nvidia:
        nvidia_env_actions = [
            SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
            SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
            SetEnvironmentVariable(name='__EGL_VENDOR_LIBRARY_FILENAMES', value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        ]

    # ------------------------------------------------------------------
    # Robot description from xacro
    # use_sim  → selects simulation vs real hardware in rover_control.xacro
    # ------------------------------------------------------------------

    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_sim:=', use_sim
        ]),
        value_type=str
    )

    # ------------------------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------------------------

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # ------------------------------------------------------------------
    # Gazebo launch  (sim only)
    # ------------------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [world_path, ' -r --gui-config ', gz_gui_config_path]
        }.items(),
        condition=IfCondition(use_sim)
    )

    # ------------------------------------------------------------------
    # Spawn robot in Gazebo  (sim only)
    # ------------------------------------------------------------------

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-name', 'rover_description',
            '-topic', 'robot_description',
            '-allow_renaming', 'true',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            '-timeout', '30.0',
        ],
        condition=IfCondition(use_sim)
    )

    # ------------------------------------------------------------------
    # Gazebo ↔ ROS2 bridge  (sim only)
    # ------------------------------------------------------------------

    # NOTE: Gazebo's /clock topic transiently jumps backward in time during world
    # loading and physics reset. Delaying the bridge by 6 s lets the simulation
    # clock settle before any use_sim_time node (EKF, costmap, PCL filters, RViz)
    # receives its first clock tick. Without this delay, every node detects
    # "jump back in time" during the first second of startup and clears its TF
    # buffer, resetting localization and the costmap.
    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': gazebo_config_path,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_sim)
    )
    delayed_bridge = TimerAction(
        period=6.0,
        actions=[parameter_bridge_node],
        condition=IfCondition(use_sim)
    )

    # ------------------------------------------------------------------
    # ros2_control node  (real robot only — loads ODrive hardware interface)
    # ------------------------------------------------------------------

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            PathJoinSubstitution([
                FindPackageShare('rover_controllers'), 'config', 'diff_controller.yaml'
            ])
        ],
        remappings=[
            ('/rover_controller/odom', '/odom_wheel'),
        ],
        condition=UnlessCondition(use_sim)
    )

    # ------------------------------------------------------------------
    # Controller spawners  (shared between sim and real)
    # ------------------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    rover_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ------------------------------------------------------------------
    # Controller spawn sequencing
    #   sim  : joint_state_broadcaster after spawn_robot exits (gz_ros2_control is then ready)
    #   real : joint_state_broadcaster after ros2_control_node starts (ODrive hardware interface ready)
    #   both : rover_controller only after joint_state_broadcaster has actually
    #          finished loading/activating (event-driven, no fixed wall-clock guess)
    # ------------------------------------------------------------------

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner]
        ),
        condition=IfCondition(use_sim)
    )

    delay_controllers_after_ros2_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner]
        ),
        condition=UnlessCondition(use_sim)
    )

    rover_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rover_controller_spawner]
        )
    )


    # ------------------------------------------------------------------
    # Virtual differential  (sim only)
    # ------------------------------------------------------------------

    virtual_differential_node = Node(
        package='rover_description',
        executable='virtual_differential.py',
        name='virtual_differential',
        output='screen',
        parameters=[
            virtual_diff_config_path,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_sim)
    )

    # ------------------------------------------------------------------
    # IMU restamp (sim only) — the Gazebo IMU sensor bridges in as raw
    # "/imu" with all-zero covariance, which makes robot_localization's
    # EKF lock up after a few updates (see imu_restamp.py docstring).
    # Republishes fixed-covariance data on "/imu/synced" for the EKF.
    # ------------------------------------------------------------------

    imu_restamp_node = Node(
        package='rover_description',
        executable='imu_restamp.py',
        name='imu_restamp',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim)
    )

    # ------------------------------------------------------------------
    # Joystick + teleop
    # ------------------------------------------------------------------

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('rover_controllers'), 'config', 'joyConfig.yaml'
        ])]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('rover_controllers'), 'config', 'joyTeleop.yaml'
        ])],
        remappings=[('/cmd_vel', '/rover_controller/cmd_vel')]
    )

    joy_started_handler = RegisterEventHandler(
        OnProcessStart(target_action=joy_node, on_start=[LogInfo(msg='[STARTED] joy_node is up')])
    )

    teleop_started_handler = RegisterEventHandler(
        OnProcessStart(target_action=teleop_node, on_start=[LogInfo(msg='[STARTED] teleop_twist_joy is up')])
    )

    # ------------------------------------------------------------------
    # Visual odometry  (ZED2i RGB-D, sim only)  — disabled
    # ------------------------------------------------------------------

    # zed_rgbd_odometry_node = Node(
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     name='zed_rgbd_odometry',
    #     output='screen',
    #     parameters=[{
    #         'frame_id': 'base_footprint',
    #         'odom_frame_id': 'zed_odom',
    #         'publish_tf': False,
    #         'approx_sync': True,
    #         'use_sim_time': use_sim_time,
    #     }],
    #     remappings=[
    #         ('rgb/image', '/zed/left/image_rect_color'),
    #         ('rgb/camera_info', '/zed/left/camera_info'),
    #         ('depth/image', '/zed/depth/depth_registered'),
    #         ('odom', '/zed/odom'),
    #     ],
    #     arguments=['--uerror', '--ros-args', '--log-level', 'warn'],
    #     condition=IfCondition(use_sim)
    # )

    # ekf_config.yaml's odom0/imu0 defaults (/odom, /microstrain/imu/data) are
    # real-hardware topic names that nothing publishes in sim. This pipeline
    # actually publishes wheel odom on /rover_controller/odom and IMU on
    # /imu/synced (see imu_restamp.py) -- remap EKF's inputs to match instead
    # of duplicating/forking ekf_config.yaml.
    # publish_tf is forced False here: odom_tf_broadcaster (launched from
    # obstacle_nav_gazebo.launch.py) handles map→odom and odom→base_footprint.
    # Letting EKF publish_tf=True creates a self-consuming TF listener loop that
    # causes "jump back in time" errors and stalls position integration.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time, 'publish_tf': False}],
        remappings=[
            ('/odom', '/rover_controller/odom'),
            ('/microstrain/imu/data', '/imu/synced'),
        ],
    )

    # ------------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------------

    return LaunchDescription([

        # Arguments
        use_sim_arg,
        use_sim_time_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,

        # Environment
        set_gz_sim_plugin_path,
        set_gz_sim_resource_path,
        *nvidia_env_actions,

        # Robot description + TF
        robot_state_publisher_node,

        # --- Simulation only ---
        gazebo_launch,
        TimerAction(period=10.0, actions=[spawn_robot_node], condition=IfCondition(use_sim)),
        delayed_bridge,
        delay_controllers_after_spawn,
        virtual_differential_node,
        imu_restamp_node,

        # --- Real robot only ---
        ros2_control_node,
        delay_controllers_after_ros2_control,

        # --- Controller spawn sequencing (shared) ---
        rover_controller_after_joint_state_broadcaster,

        # --- Joystick + teleop ---
        joy_node,
        teleop_node,
        joy_started_handler,
        teleop_started_handler,

        # --- State estimation ---
        # zed_rgbd_odometry_node,  # Visual odometry disabled
        ekf_node,
    ])
