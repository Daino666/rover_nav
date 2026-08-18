from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

INPUT_TOPIC = '/camera/camera/depth/color/points'
PASSTHROUGH_TOPIC = '/obstacles/passthrough'
VOXEL_TOPIC = '/obstacles/downsampled'
OUTPUT_TOPIC = '/obstacles/filtered'

Z_MIN = 0.2
Z_MAX = 4.0
VOXEL_LEAF_SIZE = 0.02


def generate_launch_description():
    container = ComposableNodeContainer(
        name='obstacle_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::PassThrough',
                name='passthrough_z',
                remappings=[
                    ('input', INPUT_TOPIC),
                    ('output', PASSTHROUGH_TOPIC),
                ],
                parameters=[{
                    'filter_field_name': 'z',
                    'filter_limit_min': Z_MIN,
                    'filter_limit_max': Z_MAX,
                    'filter_limit_negative': False,
                }],
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::VoxelGrid',
                name='voxel_grid',
                remappings=[
                    ('input', PASSTHROUGH_TOPIC),
                    ('output', VOXEL_TOPIC),
                ],
                parameters=[{
                    'leaf_size': VOXEL_LEAF_SIZE,
                }],
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::StatisticalOutlierRemoval',
                name='sor',
                remappings=[
                    ('input', VOXEL_TOPIC),
                    ('output', OUTPUT_TOPIC),
                ],
                parameters=[{
                    # Lower mean_k and looser stddev than before: point density
                    # falls off with range (fixed angular resolution covers more
                    # area farther out), so a strict/high mean_k tuned for
                    # close-range density was rejecting legitimate sparse far
                    # points as "outliers".
                    'mean_k': 10,
                    'stddev': 1.5,
                }],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
