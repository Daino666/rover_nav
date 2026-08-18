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

If the rover's fused heading/position doesn't line up with reality (e.g.
driving physically forward shows up as motion along the wrong axis in
`/odometry/filtered`), don't guess at a correction — a wrong rotation makes
it worse, not better. Check it empirically:

1. **Bring up localization only** (IMU driver + EKF), no need for the full
   drive stack.
2. **Run the checker**, in a separate terminal:
   ```
   ros2 run rover_nav check_heading.py
   ```
   It live-prints fused yaw (degrees) and position (x, y) from
   `/odometry/filtered`.
3. **Note the starting x/y** (should read ~0, 0).
4. **Drive the rover straight forward** a couple meters (manual/joystick is
   fine).
5. **Read which axis actually changed:**
   - `x` grows, `y` ~0 → forward is correctly +X, no offset.
   - `y` shrinks (goes negative), `x` ~0 → forward is actually -Y → **90°**
     offset.
   - `x` shrinks (goes negative), `y` ~0 → forward is actually -X → **180°**
     offset.
   - Anything else → that reading *is* the exact correction needed, not a
     guess.

   Yaw alone is not a reliable signal for this: it typically reads ~0 at
   launch regardless of the rover's true physical orientation, since there's
   no absolute compass reference — 0 just means "wherever it was facing at
   boot." Position drift under known forward motion is the real tell.

Once you know the actual offset, the fix belongs in the `imu_joint`'s `rpy`
in `aries_base.xacro` (`base_link -> imu_frame` fixed transform) — **not**
in `ekf_config.yaml` or the path planner. `robot_state_publisher` publishes
that transform over TF, and `robot_localization`'s EKF uses it to rotate the
IMU's raw orientation into `base_link` automatically before fusing, so
correcting the mounting transform there fixes it everywhere downstream in
one place instead of needing per-consumer workarounds.

