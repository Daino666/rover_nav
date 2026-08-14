import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_description',
            executable='trajectory.py',
            name='path_projection',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(
                    get_package_share_directory('rover_description'),
                    'rviz',
                    'display_config.rviz',
                ),
            ],
        ),
    ])
