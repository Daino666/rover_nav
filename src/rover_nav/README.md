# rover_nav

ROS 2 navigation and drive-control package for the rover.

> **New machine setup and bringup instructions are in the [`rover_bringup`](../rover_bringup/README.md) package.**

---

## Package structure

```
rover_nav/
├── config/
│   └── ekf_config.yaml                  # EKF localisation config
├── launch/
│   └── LeapOne_Safety_launch.py
├── msg/                                 # Custom ROS 2 message definitions
├── scripts/
│   ├── Rover_control_Joy.py             # Joystick teleop control node
│   ├── Odom.py                          # Odometry node
│   ├── rover_controller_pure_pursuit.py # Pure-pursuit path controller
│   └── setup_can_sudo.sh                # One-time passwordless CAN sudoers setup
└── package.xml
```

---

## Dependencies

- ODrive ROS 2 driver
- `rover_bringup` — provides `joystick_config.yaml` read by `Rover_control_Joy.py`

---

## Diagnosing an IMU/heading mounting offset

The 3DM-GX5-AHRS reports true absolute heading — confirmed empirically:
repeatable across power cycles for the same physical orientation, and it
correctly tracks manual rotation even while unpowered. That absolute reading
includes the IMU's own fixed mechanical mounting rotation relative to the
chassis, so raw yaw at boot is whatever that combination happens to be (e.g.
25°), not necessarily 0.

`config/ekf_config.yaml`'s `imu0_relative: true` (under `imu0`) fixes this
at the fusion level rather than needing a precise mechanical-offset
measurement: the yaw fused when the EKF starts becomes the zero reference,
and every later reading is reported relative to it. So "forward" at launch
is always yaw=0, and driving straight forward shows up as motion cleanly
along `x` in `/odometry/filtered` — regardless of the IMU's mounting angle,
which gets absorbed into the zero-point automatically along with whatever
direction the rover happened to be facing. No need to touch `imu_joint` in
`aries_base.xacro` for this.

**The real implication**: "forward" is defined *per launch*, relative to
wherever the rover was physically pointed when localization started — not a
fixed compass direction. For the same `WAYPOINTS` coordinates
(`global_path_planner.py`) to correspond to the same real-world locations
across different runs, **start the rover in the same physical orientation
every time** you bring localization up.

Verify with:

```
ros2 run rover_nav check_heading.py
```

Live-prints fused yaw and position from `/odometry/filtered`. Yaw should
read ~0 immediately at launch regardless of which way the rover is actually
facing; driving straight forward a couple meters should then move `x`
cleanly with `y` staying near 0.

