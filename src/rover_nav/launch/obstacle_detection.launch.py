from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # 1. RealSense camera
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'pointcloud.enable':       'true',
            'depth_module.profile':    '1280x720x30',
            'rgb_camera.profile':      '1280x720x30',
            # The aries URDF already carries every camera_* frame, and
            # robot_state_publisher publishes them. Letting the driver publish
            # its own copies puts two publishers on the same TF edges, which
            # corrupts the tree for the nav/moveit stack as well as for these
            # markers. Same reason aries_hardware.launch.py sets this false.
            'publish_tf':              'false',
        }.items()
    )

    # 2. PassThrough filter — crop to 0.1–2.0 m on Z axis
    passthrough = Node(
        package='pcl_ros',
        executable='filter_passthrough_node',
        name='passthrough_filter',
        remappings=[
            ('input',  '/camera/camera/depth/color/points'),
            ('output', '/pcl/front'),
        ],
        parameters=[{
            'filter_field_name': 'z',
            'filter_limit_min':  0.1,
            # Must exceed the node's max_range (2.5 m) or this crop, not the
            # node, becomes the real detection limit. Small headroom so a
            # cluster straddling 2.5 m still has its far side.
            'filter_limit_max':  2.8,
        }]
    )

    # 3. Statistical Outlier Removal — denoise
    sor = Node(
        package='pcl_ros',
        executable='filter_statistical_outlier_removal_node',
        name='sor_filter',
        remappings=[
            ('input',  '/pcl/front'),
            ('output', '/pcl/denoised'),
        ],
        parameters=[{
            'mean_k': 50,
            'stddev': 1.0,
        }]
    )

    # 4. Obstacle detector
    detector = Node(
        package='rover_nav',
        executable='pcl_obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
    )

    return LaunchDescription([
        realsense,
        passthrough,
        sor,
        detector,
    ])
