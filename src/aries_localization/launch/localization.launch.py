#!/usr/bin/env python3
"""
Wheel odometry + EKF.

This package fuses; it does not drive sensors. The IMU node itself is started by
aries_imu, and this file only needs to know which source won, because that
decides which EKF config is loaded:

  microstrain  3DM-GX5-AHRS on its serial port -> ekf_config.yaml
  none         no IMU, wheel odometry only     -> ekf_odom_only.yaml

It resolves the source with the same aries_common probe aries_imu uses, so both
packages reach the same answer from the same arguments without having to agree
out of band.

Skipped entirely on the mock drive backend, where mock_rover_drive already owns
/odom and the odom -> base_footprint transform.

Odom.py and the ekf_*.yaml configs live in rover_nav.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from aries_common.detect import resolve_imu_source, resolve_rover_backend

ODOM_ONLY_CONFIG = [
    False, False, False,  # x, y, z
    False, False, False,  # roll, pitch, yaw
    True, False, False,   # vx, vy, vz
    False, False, True,   # vroll, vpitch, vyaw
    False, False, False,  # accelerations
]


def _ekf_config(name):
    return os.path.join(get_package_share_directory("rover_nav"), "config", name)


def _sim_ekf_config():
    return os.path.join(
        get_package_share_directory("aries_localization"),
        "config",
        "ekf_sim.yaml",
    )


def _enabled(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _start_localization(context, *args, **kwargs):
    use_sim_ekf = _enabled(LaunchConfiguration("use_sim_ekf").perform(context))
    if use_sim_ekf:
        odom_topic = LaunchConfiguration("sim_odom_topic")
        imu_topic = LaunchConfiguration("sim_imu_topic")
        filtered_topic = LaunchConfiguration("filtered_odom_topic")
        return [
            LogInfo(msg=(
                "[rover localization] simulation EKF enabled: "
                "/ground_truth/odom + /imu yaw rate -> /odometry/filtered; "
                "EKF owns corrected odom->base_footprint TF"
            )),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    _sim_ekf_config(),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "odom0": odom_topic,
                        "imu0": imu_topic,
                        "publish_tf": True,
                    },
                ],
                remappings=[("odometry/filtered", filtered_topic)],
            ),
        ]

    backend = resolve_rover_backend(
        LaunchConfiguration("rover_hardware_protocol").perform(context),
        LaunchConfiguration("can_interface").perform(context),
    )
    use_imu = LaunchConfiguration("use_imu").perform(context)
    imu_port = LaunchConfiguration("imu_port").perform(context)
    imu_frame = LaunchConfiguration("imu_frame").perform(context)
    imu_topic = LaunchConfiguration("imu_topic").perform(context)

    imu_source, imu_present = resolve_imu_source(use_imu, imu_port)

    # On the mock backend with no IMU there is nothing to fuse and no encoders
    # to read, so mock_rover_drive keeps /odom and the transform to itself.
    if backend != "odrive" and imu_source != "microstrain":
        return [LogInfo(msg=(
            "[rover localization] mock backend and no IMU — skipping Odom.py/EKF; "
            "mock_rover_drive owns /odom and the odom->base_footprint TF."
        ))]

    ekf_overrides = {"base_link_frame": "base_footprint"}
    zeroed_imu_topic = imu_topic + "_yaw_zeroed"

    if imu_source == "microstrain":
        # imu_yaw_zero.py (started below) relays imu_topic with yaw
        # pre-zeroed at its first message -- ekf_config.yaml's own
        # imu0_relative: true was confirmed on hardware to not reliably
        # zero fused yaw (constant ~90 degree offset reproduced across two
        # different boot orientations), so this fuses the pre-zeroed topic
        # as a plain absolute-yaw measurement instead of relying on
        # robot_localization's own relative-mode bookkeeping at all.
        #
        # TRIED (2026-08-19/20) two follow-up relays here to address
        # motor-current EMI on the magnetometer, both reverted -- see
        # imu_yaw_drift_handoff.md for the full investigation:
        #
        # 1. imu_yaw_trust_gate.py: swung the AHRS topic's own yaw covariance
        #    between "trust" (stationary) and "ignore" (moving), based on
        #    /odom. Made things WORSE across three variants (instant snap at
        #    40Hz/20Hz EKF frequency, then a 2s ramp) -- introduced an
        #    x-axis error (1.8-2.2m) the static baseline never had (only y
        #    drifted, up to 1.7m).
        # 2. imu_yaw_source_switch.py: switched which TOPIC's orientation got
        #    fused entirely -- the raw, non-AHRS topic (magnetometer-immune
        #    by construction) while moving, AHRS while stationary. Far
        #    worse still (5m x, 13m y) -- passing raw's own offset-tracked
        #    yaw through as a normally-trusted measurement let its unbounded
        #    gyro bias drift get fully believed by the EKF for the whole
        #    moving duration of every leg, instead of the EKF falling back
        #    on its own more conservative internal dead-reckoning the way
        #    attempt 1's covariance-starving did.
        #
        # Do not re-attempt either category (swinging trust in one topic, or
        # switching which topic is fused) without new evidence explaining
        # why the previous attempts regressed first. Static magnetometer
        # trust (current state) is the best-known baseline.
        ekf_overrides["imu0"] = zeroed_imu_topic
        ekf_overrides["imu0_relative"] = False
        ekf_config = _ekf_config("ekf_config.yaml")
    else:
        ekf_overrides["odom0_config"] = ODOM_ONLY_CONFIG
        ekf_config = _ekf_config("ekf_odom_only.yaml")

    actions = [
        LogInfo(msg=(
            "[rover localization] "
            f"backend={backend} use_imu={use_imu} selected={imu_source} "
            f"microstrain_available={imu_present} "
            f"imu_frame={imu_frame} imu_topic={imu_topic}"
        ))
    ]

    if backend == "odrive":
        # Wheel odometry off the ODrive encoders over CAN.
        actions.append(
            Node(
                package="rover_nav",
                executable="Odom.py",
                name="odom_node",
                output="screen",
            )
        )
    else:
        # Odom.py would find no CAN bus here. mock_rover_drive already
        # publishes /odom from the joystick command, so the EKF fuses that for
        # forward speed and the real IMU for heading. drive.launch.py starts
        # the mock with publish_tf false so this EKF is the only owner of
        # odom -> base_footprint.
        actions.append(
            LogInfo(msg=(
                "[rover localization] mock backend with IMU — fusing "
                "mock_rover_drive /odom with the real IMU; EKF owns "
                "odom->base_footprint"
            ))
        )

    if imu_source == "microstrain":
        actions.append(
            Node(
                package="rover_nav",
                executable="imu_yaw_zero.py",
                name="imu_yaw_zero",
                output="screen",
                parameters=[{
                    "input_topic": imu_topic,
                    "output_topic": zeroed_imu_topic,
                }],
            )
        )

    actions.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, ekf_overrides],
        )
    )
    return actions


def _start_map_odom_broadcaster(context, *args, **kwargs):
    # map -> odom alignment (see rover_nav/scripts/map_odom_broadcaster.py).
    # Frame setup, independent of which drive backend/IMU source got
    # selected above -- always start it. Launch args default to "" (unset)
    # rather than a numeric default so they can be left out of the
    # parameters dict entirely when not overridden, and the node falls back
    # to its own script-level default (global_path_planner.py's
    # MAP_TO_ODOM_*) -- passing an empty string against a float-typed
    # declared parameter would otherwise be a type mismatch.
    overrides = {}
    for name in ("map_to_odom_x", "map_to_odom_y", "map_to_odom_yaw_deg"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)

    return [
        Node(
            package="rover_nav",
            executable="map_odom_broadcaster.py",
            name="map_odom_broadcaster",
            output="screen",
            parameters=[overrides] if overrides else [],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_ekf",
            default_value="false",
            description="Fuse simulation ground truth and IMU yaw rate.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("sim_odom_topic", default_value="/ground_truth/odom"),
        DeclareLaunchArgument(
            "sim_imu_topic",
            default_value="/imu",
            description="Simulated IMU topic fused by the localization EKF.",
        ),
        DeclareLaunchArgument("filtered_odom_topic", default_value="/odometry/filtered"),

        DeclareLaunchArgument("rover_hardware_protocol", default_value="auto",
                              choices=["auto", "odrive", "mock_hardware"]),
        DeclareLaunchArgument("can_interface", default_value="can0"),

        DeclareLaunchArgument("use_imu", default_value="auto",
                              choices=["auto", "true", "false", "microstrain"]),
        DeclareLaunchArgument("imu_port", default_value="/dev/microstrain_main"),
        DeclareLaunchArgument("imu_frame", default_value="imu_frame"),
        DeclareLaunchArgument("imu_topic", default_value="/microstrain/ekf/imu/data"),

        DeclareLaunchArgument(
            "map_to_odom_x", default_value="",
            description="Override map_odom_broadcaster's x (metres); "
                        "empty = use global_path_planner.py's MAP_TO_ODOM_X.",
        ),
        DeclareLaunchArgument(
            "map_to_odom_y", default_value="",
            description="Override map_odom_broadcaster's y (metres); "
                        "empty = use global_path_planner.py's MAP_TO_ODOM_Y.",
        ),
        DeclareLaunchArgument(
            "map_to_odom_yaw_deg", default_value="",
            description="Override map_odom_broadcaster's yaw (degrees); "
                        "empty = use global_path_planner.py's MAP_TO_ODOM_YAW_DEG.",
        ),

        OpaqueFunction(function=_start_localization),
        OpaqueFunction(function=_start_map_odom_broadcaster),
    ])
