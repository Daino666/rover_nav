# Session Handoff — 2026-08-19

Repo root (this checkout): `~/test/src` (packages live one directory deeper,
`~/test/src/src/<package>/`). Not a git repo in this checkout — no commits
to carry over, just file state. Copy the whole `~/test/src` tree (or at
least the files listed below) to the other laptop.

## What this session did, in order

1. **`colcon build`** — full workspace build, 23 packages, all succeeded.
   One pre-existing warning: `odrive_can` overrides a package already built
   in an underlay workspace at `~/Mars-rover/install/odrive_can` — not
   necessarily a problem, just worth knowing if behavior ever looks off.

2. **Read `src/imu_yaw_drift_handoff.md`** (pre-existing doc) for context on
   an ongoing investigation: real driving tests show `x` stays accurate but
   `y` drifts up to 1.7m. Root cause ruled down to: fused yaw swings away
   from true *while driving* (motor-current EMI on the magnetometer,
   correlating with turning), then recovers once stopped — but position is
   an integral of yaw, so the transient error is permanently baked into y.
   That doc's own "what's built but not yet run" step was `imu_deep_log.py`
   — see below, it now exists and has grown further this session.

3. **Applied 3 previously-unpushed changes** from that handoff doc:
   - `src/aries_imu/config/microstrain.yaml` — added `imu_mag_data_rate: 100`
     (publishes raw magnetometer vector for observability; doesn't affect
     fusion).
   - `src/rover_nav/CMakeLists.txt` — added `scripts/imu_deep_log.py` to the
     `install(PROGRAMS ...)` list.
   - `src/rover_nav/scripts/imu_deep_log.py` — created (see full section
     below, it's been extended twice since).
   - Rebuilt `aries_imu` + `rover_nav`, confirmed clean.

4. **Documented the full `full_hardware.launch.py` CLI** (all args + a
   `--show-args` tip) — see cheat-sheet below.

5. **Edited `TEB_Pure_Pursuit_Planner.py`** (repo-root standalone script,
   *not* installed into any package — run directly with `python3`, not
   `ros2 run`) through several rounds:
   - Changed the default fallback waypoint list to a single point `(-5.0, -1.0)`.
   - **Removed the obstacle-avoidance subsystem** (`ObstacleEntity`,
     `LocalDetourPlanner`, obstacle topics/markers, `STATE_AVOID_OBSTACLE`) —
     it was dead code per the file's own comment: no LiDAR on the rover, so
     it never had a data source.
   - **Changed `autostart` default to `True`** — the planner now drives as
     soon as armed+localized without needing an explicit
     `ros2 service call /planner/start std_srvs/srv/Trigger`. Old behavior
     is available via `--no-autostart`.
   - **Diagnosed "robot is not moving"** twice — root cause both times was
     `cmd_vel_arbiter.py` (started by `full_hardware.launch.py`'s default
     `start_pure_pursuit:=true`) detecting this script as a second
     `/cmd_vel` publisher and refusing to drive (its own documented
     fail-closed behavior). **Fix: always launch with
     `start_pure_pursuit:=false`** when running this script manually.
   - **Full simplification pass** ("simple waypoint navigator, just TEB +
     pure pursuit"): removed `--sim`/Gazebo profile, arena bounding box,
     `--closed-loop`, geofence radius, and the elaborate per-waypoint RViz
     decoration (bullseye rings, crosshairs, accuracy-pointer arrows, HUD
     text) — replaced with one sphere + text label per waypoint. Kept TEB
     spline/curvature-velocity-profile trajectory generation, lookahead
     pure pursuit with align-in-place, and the full hardware safety layer
     (arm check, odometry-staleness hard stop, teleop override,
     acceleration ramping).
   - **Fixed a real bug found along the way**: `HW_DRIVE_ENABLE_SERVICE` had
     gotten corrupted into two garbage lines (`HW_DRIVE_ENAn` /
     `BLE_SERVICE = ...`) — syntactically valid Python (so `py_compile`
     didn't catch it) but would have crashed with `NameError` the moment it
     actually ran. Not something this session caused; just found and fixed.
   - **Fixed CLI parsing of negative waypoints** — `argparse`'s `nargs='+'`
     was misreading `-5.0,-1.0` as a flag (it only special-cases pure
     negative numbers, not `X,Y` pairs). Replaced with manual pre-scanning
     of `argv` for `--waypoints` tokens; any number of negative-coordinate
     waypoints now work directly, no `=` or quoting tricks needed.
   - **"Script stopped" after reaching goal** — this was expected behavior
     (measured landing error ≈0.30m was inside the 0.35m tolerance, so it
     had already reached the goal and gone quiet by design). Added a
     throttled `"🎯 parked at goal, holding station"` log every 5s at idle
     so it doesn't look hung.
   - **Tightened `WAYPOINT_GOAL_TOLERANCE`** from 0.35m → 0.10m (in response
     to "make accuracy 100%" — true zero error isn't achievable or even
     safe to target, since it'd never register "reached" and the rover
     would hunt indefinitely; 0.10m is about as tight as makes sense given
     the drivetrain's 0.10 m/s minimum-move deadband).

6. **Extended `imu_deep_log.py`** with ground-truth checkpoint marking — a
   `/imu_deep_log/mark_checkpoint` (`std_srvs/Trigger`) service you call
   each time you physically stop the rover at a pre-known point. It logs
   the EKF's reported position, the known-true position, and the error, as
   a specially-flagged row in the same CSV as the continuous IMU/mag data
   — no manual timestamp correlation needed afterward.

7. **Discussed whether raising IMU publish rate helps accuracy — it
   doesn't, here.** Two independent reasons: (a) `robot_localization` runs
   at `frequency: 20.0` in `ekf_config.yaml` while the IMU already publishes
   at 100Hz (`imu_data_rate`/`filter_imu_data_rate` in `microstrain.yaml`)
   — already 5x oversampled relative to what the EKF actually consumes, so
   raising it further changes nothing reaching the filter; (b) rate and
   accuracy are different axes — a systematic EMI disturbance during
   turning isn't high-frequency noise that faster sampling averages away,
   it's a real, correlated change in the magnetic field. Moved on to `csv_path`
   default relocation (see below) instead.

8. **Moved `imu_deep_log.py`'s default `csv_path`** from `/tmp` to
   `~/test/src/imu_deep_log_<timestamp>.csv`, matching where other test
   CSVs in this repo already live (`straight_test1.csv`, `yaw_compare1.csv`).

## Files changed this session

- `src/aries_imu/config/microstrain.yaml` — added `imu_mag_data_rate: 100`
- `src/rover_nav/CMakeLists.txt` — added `imu_deep_log.py` to install list
- `src/rover_nav/scripts/imu_deep_log.py` — created, then extended with
  checkpoint marking
- `TEB_Pure_Pursuit_Planner.py` (repo root) — multiple rounds, see #5 above

## CLI cheat-sheet

**Launch the hardware stack** (always disable the built-in arbiter if
you're going to run `TEB_Pure_Pursuit_Planner.py` manually):
```bash
ros2 launch aries_bringup full_hardware.launch.py start_pure_pursuit:=false
```

**Run the simple waypoint navigator** (multiple waypoints, negative
coordinates work fine now):
```bash
python3 TEB_Pure_Pursuit_Planner.py --waypoints -5.0,-1.0 3.0,2.0 -2.0,4.0
```
Useful flags: `--relative-coords` (offsets from current pose instead of
absolute odom coords), `--speed`, `--halt-time`, `--dry-run`,
`--no-autostart`, `--arm`, `--ignore-drive-state`, `--no-teleop-override`,
`--odom-timeout`. Full list: `python3 TEB_Pure_Pursuit_Planner.py -h`.

**Run the IMU deep-log diagnostic** (plain, no ground truth):
```bash
ros2 run rover_nav imu_deep_log.py --ros-args -p duration_s:=90.0
```
With ground-truth checkpoints:
```bash
ros2 run rover_nav imu_deep_log.py --ros-args \
  -p duration_s:=120.0 \
  -p ground_truth_waypoints:="-5.0,-1.0;3.0,2.0;-2.0,4.0"

# each time you physically stop the rover at the next point in that list:
ros2 service call /imu_deep_log/mark_checkpoint std_srvs/srv/Trigger
```
Verdict logic once it finishes (auto-printed): mag-magnitude range large →
real EMI; AHRS vyaw range >> raw vyaw range → disturbance enters through the
device's internal AHRS fusion, not the raw gyro (would explain why loosening
`imu_orientation_cov` alone had no effect); both raw and AHRS disturbed
similarly → points at vibration coupling into gyro bias instead.

**Diagnose duplicate `/cmd_vel` publishers** (the "robot not moving" bug):
```bash
ros2 topic info /cmd_vel --verbose
```
More than one publisher listed = `cmd_vel_arbiter.py` and your manually-run
script fighting each other; restart with `start_pure_pursuit:=false`.

## Open threads (unfinished, carry these over)

- The original `imu_yaw_drift_handoff.md` investigation is still open —
  `imu_deep_log.py` (now with checkpoint marking) is built and ready but
  hasn't actually been run on a real driving test yet as of this session.
  That's still the next concrete step to resolve the y-drift root cause.
- No new ground-truth driving test has been run with the checkpoint feature
  yet — first real use of it will validate the mechanism itself too.

## Session norms carried over from the original handoff doc (still apply)

- Verify config changes actually landed on the live driver (`ros2 param
  get ...`) rather than trusting a restart alone.
- Confirm before applying EKF/IMU config or drive-behavior changes — this
  is safety/accuracy-critical on real hardware.
- No git repo in this checkout, so nothing to push — just copy files.
