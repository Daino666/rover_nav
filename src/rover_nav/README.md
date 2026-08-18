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

`config/ekf_config.yaml`'s `imu0_relative: true` (under `imu0`) was *meant*
to fix this at the fusion level rather than needing a precise
mechanical-offset measurement: the yaw fused when the EKF starts becomes the
zero reference, and every later reading is reported relative to it. In
practice this was confirmed on hardware to not do that reliably — fused
yaw stayed off from the IMU's own absolute heading by a constant ~90
degrees, reproduced across two different boot orientations, instead of
settling near 0. Externally correcting the EKF's state after the fact (via
its `/set_pose` service) was also tried and confirmed not to hold: ongoing
IMU fusion keeps measuring yaw relative to robot_localization's own internal
reference and pulls the corrected state back away from 0 within a few
updates.

`scripts/imu_yaw_zero.py` (started automatically by
`aries_localization/launch/localization.launch.py`, upstream of the EKF)
sidesteps the whole mechanism instead: it relays the IMU topic, capturing
the yaw of the very first message it sees and republishing every message
(including that first one) with yaw replaced by `raw_yaw - reference`, a
plain, self-computed relative heading. `localization.launch.py` points the
EKF's `imu0` at this relayed topic and forces `imu0_relative: false`, so the
EKF fuses it as a plain absolute-yaw measurement that already reads 0 at the
first sample — robot_localization's own relative-mode bookkeeping isn't
involved at all. So "forward" at launch is always yaw=0, and driving
straight forward shows up as motion cleanly along `x` in
`/odometry/filtered` — regardless of the IMU's mounting angle, which gets
absorbed into the zero-point automatically along with whatever direction
the rover happened to be facing. No need to touch `imu_joint` in
`aries_base.xacro` for this.

**The real implication**: `imu0_relative` zeros yaw *within* the rover's own
`odom` frame, but `odom` is still just "wherever the rover happened to be
pointed when localization started" — it has no relationship to the
competition's own axes (defined by the two reference points the organizers
give). Requiring the rover to physically boot facing the same way every
time was a fragile workaround for that, not a real fix. `scripts/map_odom_broadcaster.py`
(started automatically as part of `aries_localization/launch/localization.launch.py`)
replaces it: it publishes a static `map -> odom` transform holding the
alignment correction between the two, so `WAYPOINTS` in
`scripts/global_path_planner.py` are given in `map` frame (the competition's
axes) and `cmd_vel_arbiter.py` transforms them into `odom` once at startup
before driving — the rover's actual boot orientation no longer matters for
`WAYPOINTS` to mean the same real-world locations. The correction itself
(`MAP_TO_ODOM_X/Y/YAW_DEG` in `global_path_planner.py`, defaulting to
identity) is still a manually-set number for now — there's no
absolute-position sensor on this rover to compute it automatically, and the
actual competition-day procedure for determining it is still undecided.

Verify with:

```
ros2 run rover_nav check_heading.py
```

Live-prints fused yaw and position from `/odometry/filtered`. Yaw should
read ~0 immediately at launch regardless of which way the rover is actually
facing; driving straight forward a couple meters should then move `x`
cleanly with `y` staying near 0.

