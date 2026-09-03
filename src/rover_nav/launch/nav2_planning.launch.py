#!/usr/bin/env python3
"""
Minimal Nav2 stack for path planning against the 2026 Mars Yard occupancy
grid: map_server (loads marsyard2026_occupancy.yaml) -> global_costmap
(static layer + inflation layer, so the planner is biased away from
obstacles the same way the inflation radius/cost_scaling_factor in
nav2_planning_params.yaml define) -> planner_server (SmacPlanner2D) ->
smoother_server (SimpleSmoother, collision-checked against the costmap so it
can't smooth a path into an obstacle).

This does NOT bring up the full Nav2 navigation stack (no AMCL, no
controller_server, no bt_navigator) -- it's planning only: given a start and
goal pose, produce a smooth, obstacle-clear path. Trigger it with
nav2_simple_commander's BasicNavigator.getPath()/smoothPath() (see
tools/plan_path.py, tools/plan_multi_point_tour.py in Mars-rover), or via
`ros2 action send_goal` on /compute_path_to_pose and /smooth_path directly.

planner_server publishes the raw path on /unsmoothed_plan and
smoother_server publishes the smoothed result on /plan_smoothed every time
either is triggered -- pass `rviz:=true` to also launch RViz pre-configured
to show both live on top of the map, no manual display setup needed.

static_tf defaults to true (a fixed map->base_footprint transform, just
enough for the costmap to initialize when running this planning stack on
its own). Pass `static_tf:=false` when running alongside the actual sim +
odom_tf_broadcaster.py, which provides a real, moving map->odom->base_link
chain instead -- running both at once puts two different parents on
base_link's TF chain and they'll fight each other.

Usage:
  ros2 launch rover_nav nav2_planning.launch.py
  ros2 launch rover_nav nav2_planning.launch.py rviz:=true
  ros2 launch rover_nav nav2_planning.launch.py rviz:=true static_tf:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'config', 'nav2_planning_params.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('rover_nav'), 'rviz', 'nav2_path_view.rviz'
    ])

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Also launch RViz pre-configured to show the map + planned paths live',
    )
    static_tf_arg = DeclareLaunchArgument(
        'static_tf', default_value='true',
        description='Publish a fixed map->base_footprint transform (only for standalone planning; '
                    'set false when running alongside the sim + odom_tf_broadcaster.py)',
    )
    # Portable stand-in for nav2_planning_params.yaml's static yaml_filename,
    # which is necessarily one hardcoded machine's path. marsyard/ sits as a
    # top-level sibling of src/ (see "Fold Mars-rover, obstacle_detection,
    # and marsyard data into this repo"), same on every rover laptop as long
    # as the workspace is checked out to ~/jazzy_ws.
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution(
            [EnvironmentVariable('HOME'), 'jazzy_ws', 'marsyard', 'marsyard2026_occupancy.yaml']),
        description='Occupancy map YAML to load. Override to point at a different map.',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': LaunchConfiguration('map_yaml')}],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[params_file],
    )

    # global_costmap requires a valid map->base_footprint transform to
    # initialize, even for pure planning-between-explicit-poses (no live
    # robot tracked here, rolling_window is off). A static identity-ish
    # transform is enough to satisfy that -- but only when nothing else is
    # providing a real one (see static_tf_arg above).
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_footprint_static',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
        condition=IfCondition(LaunchConfiguration('static_tf')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        static_tf_arg,
        map_yaml_arg,
        static_tf_node,
        map_server_node,
        planner_server_node,
        smoother_server_node,
        lifecycle_manager_node,
        rviz_node,
    ])
