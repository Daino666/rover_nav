"""Plan the global route and watch the local planner detour around obstacles, in RViz.

One command for the whole picture: Nav2 plans the marsyard route, the follower
drives it, and local_planner.py detours around whatever is in the way -- with the
global route, the live local plan, the keepout rings and the camera cone all
drawn on the occupancy map.

    ros2 launch rover_nav global_local_view.launch.py
    ros2 service call /planner/start std_srvs/srv/Trigger      # nothing moves until this

By default this is the DESK version: sim:=true supplies a fake rover (it
integrates /cmd_vel into /odometry/filtered) and fake rocks placed at chosen
arclengths ALONG the planned route, so the whole thing runs with no rover, no
camera and no ODrives. sim:=false expects the real stack to be providing
/odometry/filtered and /obstacles/array instead.

ORDERING, which is the whole reason this is a launch file and not a shell script:
plan_global_path.py needs planner_server up before it can plan, and
cmd_vel_arbiter reads its path CSV once at startup -- so the followers cannot
start until the plan has been WRITTEN. Nav2 comes up, a timer gives it
plan_delay seconds to activate, the planner runs to completion, and only on its
exit do the sim, the follower and the local planner start.

Nothing drives until /planner/start. Until then this is a static view of the
route, the obstacles and the detour the planner intends to take.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _stage(context, *args, **kwargs):
    """Build the actions that depend on argument VALUES rather than substitutions.

    --points takes several names, and the output CSV path is <out_dir>/<name>.csv;
    both need the resolved strings, which is what OpaqueFunction gives us."""
    lc = LaunchConfiguration
    start = lc("start").perform(context)
    points = lc("points").perform(context).split()
    name = lc("name").perform(context)
    out_dir = os.path.expanduser(lc("out_dir").perform(context))
    loop = lc("loop").perform(context).lower() == "true"
    sim = lc("sim").perform(context).lower() == "true"
    rocks = lc("rocks").perform(context)
    start_yaw = lc("start_yaw").perform(context)
    plan_delay = float(lc("plan_delay").perform(context))

    path_csv = os.path.join(out_dir, f"{name}.csv")
    waypoints_csv = os.path.join(out_dir, f"{name}_waypoints.csv")

    plan_cmd = ["ros2", "run", "rover_nav", "plan_global_path.py",
                "--start", start, "--points", *points,
                "--name", name, "--out-dir", out_dir]
    if loop:
        plan_cmd.append("--loop")
    # Deliberately NOT --publish. cmd_vel_arbiter publishes the route itself, in
    # odom, once it has loaded the CSV; letting the planner latch its own copy in
    # map as well puts two publishers on /pure_pursuit/path showing the same line
    # in two frames.

    plan = ExecuteProcess(cmd=plan_cmd, output="screen", name="plan_global_path")

    followers = [
        Node(package="rover_nav", executable="cmd_vel_arbiter.py",
             name="cmd_vel_arbiter", output="screen",
             parameters=[{
                 "path_csv": path_csv,
                 "waypoints_csv": waypoints_csv,
                 # The route is placed in odom by the arbiter. With sim there is
                 # no map->odom correction to apply (the static transform below
                 # is identity), so the map coordinates are driven verbatim.
                 "path_frame": "odom" if sim else "map",
                 "local_plan_enabled": True,
             }]),
        Node(package="rover_nav", executable="local_planner.py",
             name="local_planner", output="screen",
             parameters=[{"require_detector": True}]),
    ]
    if sim:
        followers.insert(0, Node(
            package="rover_nav", executable="sim_local_planner.py",
            name="sim_local_planner", output="screen",
            parameters=[{"rocks": [float(v) for v in rocks.strip("[]").split(",") if v.strip()],
                         "start_yaw": float(start_yaw)}]))

    return [
        TimerAction(period=plan_delay, actions=[plan]),
        RegisterEventHandler(OnProcessExit(target_action=plan, on_exit=followers)),
    ]


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rover_nav"), "rviz", "global_local_view.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument("start", default_value="S1"),
        DeclareLaunchArgument("points", default_value="W7 W8 W9 W4",
                              description="Space-separated survey names or 'x,y' pairs"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("name", default_value="global_local_view",
                              description=("Output basename. NOT global_path_hybrid by "
                                           "default, so a visualisation run cannot "
                                           "overwrite the route you plan to drive.")),
        DeclareLaunchArgument("out_dir", default_value="~/jazzy_ws/marsyard"),
        DeclareLaunchArgument("sim", default_value="true",
                              description=("Fake rover + fake rocks, no hardware. "
                                           "false expects real odometry and detections.")),
        DeclareLaunchArgument("rocks", default_value="[5.0, 14.0]",
                              description="sim only: arclengths (m) along the route"),
        DeclareLaunchArgument("start_yaw", default_value="1.8326",
                              description="sim only: initial heading, matches the CSV"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("plan_delay", default_value="14.0",
                              description="Seconds to let planner_server activate first"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare("rover_nav"), "launch", "nav2_planning.launch.py"])]),
            launch_arguments={"rviz": "false"}.items(),
        ),

        # Identity map->odom, sim only. The paths are published in odom and the
        # occupancy map in map; without this they are two unconnected trees and
        # RViz draws one or the other. On hardware the real correction comes from
        # map_odom_broadcaster and this must stay off.
        Node(condition=IfCondition(LaunchConfiguration("sim")),
             package="tf2_ros", executable="static_transform_publisher",
             name="map_to_odom_identity",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),

        Node(condition=IfCondition(LaunchConfiguration("rviz")),
             package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz_config], output="log"),

        OpaqueFunction(function=_stage),
    ])
