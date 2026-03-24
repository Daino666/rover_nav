from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the odrive launch file
    odrive_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('odrive_can'), 
                        'launch',
                        'example_launch.yaml')
        )
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    # Your rover control node
    rover_control = Node(
        package='rover_nav',
        executable='Rover_control_Joy.py',
        name='rover_control',
        output='screen'
    )
    
    return LaunchDescription([
        odrive_launch,
        joy_node,
        rover_control
    ])