from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ODrive CAN nodes - one per axis (must start before rover_control_joy)
    odrive_nodes = [
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            name=f'can_node_{i}',
            namespace=f'odrive_axis{i}',
            output='screen',
            parameters=[{
                'node_id': i,
                'interface': 'can0'
            }]
        )
        for i in range(6)
    ]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    rover_control = Node(
        package='rover_nav',
        executable='Rover_control_Joy.py',
        name='rover_control',
        output='screen'
    )

    return LaunchDescription(odrive_nodes + [joy_node, rover_control])