# integration/

Reference copies of files from other repos/packages that call into
rover_nav, kept here so the full picture of an integration is visible from
this repo alone. Nothing in this directory is built or installed by
rover_nav's own `CMakeLists.txt` -- these are snapshots, not part of the
ROS package.

## aries_bringup_full_hardware.launch.py

Source of truth: `src/aries_bringup/launch/full_hardware.launch.py` in the
`aries` monorepo (the real rover's top-level bringup, run as
`ros2 launch aries_bringup full_hardware.launch.py`).

This is the file that actually starts `cmd_vel_arbiter.py`
(`scripts/cmd_vel_arbiter.py` in this repo) alongside the rest of the
hardware stack -- the `start_pure_pursuit` and `pure_pursuit_autostart`
launch arguments and the `cmd_vel_arbiter` `Node(...)` action near the
bottom of the file are the relevant addition. Everything else in it
(arm/gripper, joystick, hardware checker) is aries_bringup's own concern
and unrelated to rover_nav.

If aries_bringup's copy changes, re-sync this one by hand -- it's a
snapshot for reference, not a build dependency.
