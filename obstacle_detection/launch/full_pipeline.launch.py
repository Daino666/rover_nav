import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CAMERA_TOPIC = '/camera/camera/depth/color/points'
FILTERED_TOPIC = '/obstacles/filtered'

# depth_module.visual_preset isn't in realsense2_camera's accepted launch-arg
# list, so passing it as a launch_argument below is silently ignored -- has
# to be applied with a live `ros2 param set` after the node is up instead.
VISUAL_PRESET_HIGH_ACCURACY = '3'
PRESET_APPLY_DELAY_SEC = 6.0


def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'accelerate_gpu_with_glsl': 'true',
            'decimation_filter.enable': 'true',
            'decimation_filter.filter_magnitude': '2',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'pointcloud.pointcloud_qos': 'SENSOR_DATA',
        }.items(),
    )

    apply_visual_preset = TimerAction(
        period=PRESET_APPLY_DELAY_SEC,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'param', 'set', '/camera/camera',
                    'depth_module.visual_preset', VISUAL_PRESET_HIGH_ACCURACY,
                ],
                output='screen',
            )
        ],
    )

    relay = Node(
        package='topic_tools',
        executable='relay',
        name='obstacles_filtered_relay',
        arguments=[CAMERA_TOPIC, FILTERED_TOPIC],
        output='screen',
    )

    clustering = Node(
        package='obstacle_detection',
        executable='obstacle_clustering',
        name='obstacle_clustering',
        output='screen',
    )

    return LaunchDescription([
        camera_launch,
        apply_visual_preset,
        relay,
        clustering,
    ])
