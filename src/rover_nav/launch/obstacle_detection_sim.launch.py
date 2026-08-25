"""PassThrough (range-crop) + Statistical Outlier Removal filter chain for
the front depth camera, feeding local_costmap's obstacle_layer -- see
scripts/depth_to_pointcloud.py (synthesizes /camera/depth/points, since the
sim only bridges the depth image, not a native points topic) and
nav2_local_planner_params.yaml (the actual costmap wiring, consuming this
file's output topic /pcl/denoised).

The third node this file used to include (an "obstacle detector" producing
a custom ObstacleArray) is dropped: its script was never installed by
CMakeLists.txt (rosidl/PROGRAMS list has no pcl_obstacle_detector.py entry),
so including it here would abort the whole launch with "executable not
found". Nothing in this codebase currently consumes ObstacleArray anyway
(see NAV2_PLANNING_NOTES.md) -- costmap-based avoidance below doesn't need
it.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1. PassThrough filter — crop to 0.1–2.0 m on Z axis (camera-frame Z is
    #    depth/forward-distance, not world height -- this is a detection-range
    #    crop, not a ground-plane filter. World-frame height filtering happens
    #    downstream in local_costmap's obstacle_layer via min/max_obstacle_height.)
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

    return LaunchDescription([
        passthrough,
        sor,
    ])
