from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1. PassThrough filter — crop to 0.1–2.0 m on Z axis
    passthrough = Node(
        package='pcl_ros',
        executable='filter_passthrough_node',
        name='passthrough_filter',
        remappings=[
            ('input',  '/camera/depth/points'),
            ('output', '/pcl/front'),
        ],
        parameters=[{
            'filter_field_name': 'z',
            'filter_limit_min':  0.1,
            'filter_limit_max':  2.0,
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
        }]
    )

    # 3. Obstacle detector
    detector = Node(
        package='rover_nav',
        executable='pcl_obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
    )

    return LaunchDescription([
        passthrough,
        sor,
        detector,
    ])
