from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1. RealSense camera
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            # Pin the ROVER camera. Without a serial, rs_launch.py opens
            # whichever RealSense enumerates first -- and on this rover that is
            # often the GRIPPER camera, which is mounted on the arm and points
            # wherever the arm happens to be. Its clusters then arrive as
            # obstacles at positions that have nothing to do with the driving
            # direction, and the local planner reverses away from things that
            # are not in front of the rover. The leading underscore is the
            # realsense driver's own convention for serial_no.
            'serial_no':               LaunchConfiguration('serial_no'),
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
            'filter_limit_max':  2.0,
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
    #
    # `params:=<file.yaml>` loads a tune saved by
    #   ros2 run rover_nav tune_obstacle_detection.py   ->  yaml <file>
    # so a tuning session can be made permanent without editing this file. The
    # default is the empty list: the node's own declared defaults apply.
    def detector(context, *_args, **_kwargs):
        # Resolved here rather than passed straight through: launch_ros reads a
        # plain string as a path to a yaml file, and the empty default is not a
        # path. Build the list only when a file was actually given.
        path = LaunchConfiguration('params').perform(context)
        return [Node(
            package='rover_nav',
            executable='pcl_obstacle_detector.py',
            name='obstacle_detector',
            output='screen',
            parameters=[path] if path else [],
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no', default_value='_207522077539',
            description=(
                'RealSense serial to open. Default is the ROVER camera. The '
                'gripper camera is _216322070216 -- it is on the arm and is the '
                'wrong device for driving. Empty opens whichever enumerates '
                'first, which is what this argument exists to stop.'
            ),
        ),
        DeclareLaunchArgument(
            'params', default_value='',
            description='YAML parameter file for obstacle_detector '
                        '(from the tuner\'s `yaml <file>` command)'),
        realsense,
        passthrough,
        sor,
        OpaqueFunction(function=detector),
    ])
