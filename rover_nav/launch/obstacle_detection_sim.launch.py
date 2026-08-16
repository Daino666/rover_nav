from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. PassThrough filter — crop to 0.2–5.5 m on X axis (forward depth range in camera_depth_frame)
    passthrough = Node(
        package='pcl_ros',
        executable='filter_passthrough_node',
        name='passthrough_filter',
        remappings=[
            ('input',  '/camera/depth/points'),
            ('output', '/pcl/front'),
        ],
        parameters=[{
            'filter_field_name': 'x',
            'filter_limit_min':  0.2,
            'filter_limit_max':  3.2,
            'use_sim_time': use_sim_time,
        }]
    )

    # 2. Statistical Outlier Removal — denoise
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
            'use_sim_time': use_sim_time,
        }]
    )

    # 3. Obstacle detector
    detector = Node(
        package='rover_nav',
        executable='pcl_obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        passthrough,
        sor,
        detector,
    ])
