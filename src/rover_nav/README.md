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
│   ├── cmd_vel_arbiter.py               # Waypoint / test-path follower (publishes /cmd_vel)
│   ├── test_path_loader.py              # Loads + anchors the real-world test courses
│   ├── publish_test_path.py             # Previews one course in RViz
│   ├── test_paths/
│   │   ├── generate_test_paths.py       # Regenerates the courses
│   │   └── output/                      # The course CSVs + JPEGs (installed to share/)
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

### Full bring-up for bench testing (IMU only, no wheel hardware)

`aries_localization/launch/localization.launch.py` only starts the fusion
side (`imu_yaw_zero.py` → `ekf_node` → `map_odom_broadcaster.py`) — its own
docstring says so explicitly: *"This package fuses; it does not drive
sensors."* Launching it alone leaves `/odometry/filtered` ticking but never
actually updating (confirmed on hardware: `robot_localization` silently
discards every IMU measurement whose `frame_id` doesn't resolve via TF, with
no error at default log level). To exercise the real pipeline on a bench
with just the IMU connected, three more things need to be running:

```
ros2 launch aries_imu imu.launch.py                 # starts the IMU driver
ros2 run aries_bringup mock_rover_drive.py           # stands in for absent wheel hardware
```

plus a `robot_state_publisher` publishing the URDF (needed for the static
`base_link -> imu_frame` TF that `imu0`'s fusion depends on) — the full
`aries_bringup`/`full_hardware.launch.py` chain provides this already; for a
minimal manual bring-up without that chain, launch `robot_state_publisher`
directly with the xacro'd `robot_description` param (see
`aries`/`display.launch.xml` for the `Command(...)` + `ParameterValue(...,
value_type=str)` pattern — passing xacro output as a raw CLI param string
breaks on multi-line content and on `: ` inside xacro comments).


---

## Real-world path-tracking tests

Five reference courses for measuring how well the rover actually tracks a
path on the ground. They are ported unchanged from rover_nav's `omar`
branch, where the same five validate its nav2 global planner in simulation —
the CSVs here are byte-identical to that branch's, so a tracking result
measured on this rover is directly comparable to the simulated one.

| Course | What it tests | Real-world size |
|---|---|---|
| `straight_line` | Straight-line tracking | 12 m long |
| `lane_change` | Lateral step | 3.2 m straight → 4.8 m shift of 1.6 m → 3.2 m straight (11.2 m total) |
| `circle` | Constant curvature | 5.0 m radius loop (~31 m around) |
| `circle_transition` | Curvature step | One 3.0 m loop into one 6.0 m loop, tangent at the origin |
| `infinity` | Figure-8 / direction reversal | Lemniscate with 5.0 m lobes, ~8 m × 8 m, self-crossing at the origin |

Each course ships as four files in `scripts/test_paths/output/` (installed to
`share/rover_nav/test_paths/`):

| File | Contents |
|---|---|
| `<name>.csv` | Dense waypoints (`x_m, y_m, yaw_rad`, 0.1 m apart) — what the rover drives |
| `<name>_markers.csv` | Sparse points ~2 m apart (`marker_id, x_m, y_m, yaw_rad`) — pace these out with cones/stakes |
| `<name>.jpg` | The path plotted with the markers numbered, so a cone in the field matches a CSV row |
| `overview.jpg` | All five on one sheet, for a sense of scale |

List what is available, and print one course's markers to take into the field:

```
ros2 run rover_nav test_path_loader.py --list
ros2 run rover_nav test_path_loader.py --info circle
```

### Frame

Every course is in a **rover-start-relative** frame: origin `(0, 0)` is the
rover's launch point, `+x` its initial heading, `+y` to its left. That is
exactly what `odom` is on this branch — `imu_yaw_zero.py` zeroes fused yaw at
the first IMU sample, so the EKF's `odom` frame starts at the rover's own
launch pose facing `+x`. **These courses therefore need no `map->odom`
alignment**, unlike `WAYPOINTS` in `global_path_planner.py`, which are in the
competition's map frame and wait on `map_odom_broadcaster.py`.

### Field procedure

1. **Pick the origin.** Choose a launch point and heading in the field and
   mark both on the ground. That is local `(0, 0)`, yaw `0`.
2. **Pace out the markers.** Open `<name>_markers.csv` (or
   `test_path_loader.py --info <name>`) and put a cone at each `(x_m, y_m)` —
   x forward along the launch heading, y to its left. Cross-check against
   `<name>.jpg`; the numbers on the plot are the CSV's `marker_id`.
3. **Preview it in RViz** before driving, to confirm shape and numbering:
   ```
   ros2 launch rover_nav view_test_path.launch.py test_path:=circle
   ```
4. **Park the rover on the marked origin, facing the marked heading**, and
   bring up the stack with the course selected:
   ```
   ros2 launch aries_bringup full_hardware.launch.py test_path:=circle
   ```
5. **Arm the drive**, then start the run when clear:
   ```
   ros2 service call /planner/start std_srvs/srv/Trigger
   ```
6. **Watch it against the cones.** The node logs live tracking error and
   prints a summary on completion:
   ```
   test path 'circle' COMPLETE -- tracking error vs reference:
   mean 0.041 m, max 0.118 m over 1523 samples.
   ```

`/planner/start` can be called again to repeat the course; each run
re-anchors to wherever the rover is standing at that moment and re-zeroes the
error statistics.

### How a course is driven

Unlike the `WAYPOINTS` route, a test path is driven as **one continuous run
with no per-waypoint stops** — the whole point of these courses is to measure
uninterrupted curvature tracking, which stopping every couple of metres would
hide. Teleop override still works mid-course (LB-gated, same as always) and
deliberately does *not* re-anchor: the course is a fixed piece of ground being
measured against, so after a nudge the rover resumes chasing the same course
rather than restarting it under its new pose.

Relevant parameters on `cmd_vel_arbiter`:

| Parameter | Default | What it does |
|---|---|---|
| `test_path` | `""` | Course name, or empty to drive `WAYPOINTS` instead |
| `test_path_anchor` | `start_pose` | `start_pose` puts the course origin/heading under the rover at `/planner/start`. `odom_origin` drives the CSV coordinates verbatim in `odom` — only valid if the rover has not moved since localization came up |
| `test_path_goal_tolerance` | `0.25` | How close to the final point counts as done. Deliberately tighter than `goal_tolerance` (0.5), which is sized for stopping *at* waypoints — half a metre of slop there costs nothing, but on a fixed-length course it would leave the last stretch, and on the closed loops the loop closure itself, never driven |
| `base_velocity` | `0.2` | Drive speed (m/s). A 31 m circle at 0.2 m/s takes ~2.5 minutes |
| `lookahead_distance` | `0.5` | Pure-pursuit lookahead. Raise it if the rover weaves on the tighter loops; lower it if it cuts corners |

### Regenerating or rescaling the courses

Every dimension in `generate_test_paths.py`'s `CONFIG` block is expressed as a
multiple of the rover's own footprint (derived in the module docstring from
`src/aries/urdf/`), so the whole course set rescales together:

```
python3 src/rover_nav/scripts/test_paths/generate_test_paths.py
colcon build --packages-select rover_nav     # CSVs are installed to share/
```

Note that editing `CONFIG` breaks the byte-identical parity with the `omar`
branch, and with it the direct comparability of results between the two.

### One gotcha: `test_path:=infinity`

ROS's parameter parsers run values through C `strtod`, which accepts
`"infinity"` as the float `inf` — so that one course name arrives as a
`DOUBLE` rather than a string, on both the `ros2 run -p` and `ros2 launch`
paths. `test_path_loader.coerce_name()` maps it back, so `test_path:=infinity`
just works; the parameter is declared with `dynamic_typing` for that reason,
and it is not safe to "tidy" that back to a plain string declaration.

---

## Global planner: Hybrid-A* with a bounded turning radius

`config/nav2_planning_params.yaml` plans with `SmacPlannerHybrid`, not
`SmacPlanner2D`. The 2D planner searches an 8-connected grid, so its output is
built from 45/90-degree corners and it has no notion of curvature at all — a
downstream smoother can round those corners but cannot bound how tight they
get. Measured over the same S1→S3→S4→S5→S1 tour, that planner emitted turns
down to a **0.06 m radius**, with 19.6% of poses tighter than the rover's
2.0 m low-slip limit.

Hybrid-A* searches over kinematically feasible motion primitives instead, so
`minimum_turning_radius` is enforced *during* the search:

| Planner | Length | Max curvature | Tightest radius | Poses over the 2 m limit |
|---|---|---|---|---|
| `SmacPlanner2D` | 68.5 m | 17.31 1/m | 0.06 m | 121/617 (19.6%) |
| Hybrid, `smooth_path: true` | 82.1 m | 0.756 1/m | 1.32 m | 2/486 (0.4%) |
| Hybrid, `smooth_path: false` | 82.8 m | **0.500 1/m** | **2.00 m** | **0/486 (0%)** |

Respecting the radius costs roughly 20% extra path length. `smooth_path` is
off because Hybrid's own smoother has no curvature term and pulls the path
locally tighter than the search guaranteed; raw Dubins output is arcs and
straight segments, already smooth, and holds the bound exactly.

Key settings and why:

| Parameter | Value | Rationale |
|---|---|---|
| `minimum_turning_radius` | `2.0` | `MIN_SAFE_RADIUS` from `scripts/test_paths/generate_test_paths.py` — 2.5x the rover's 0.8 m footprint, below which the rocker-bogie scrubs its wheels against each other's arcs |
| `motion_model_for_search` | `DUBIN` | Forward-only. `REEDS_SHEPP` plans reversing manoeuvres that `cmd_vel_arbiter.py` (constant positive `base_velocity`, no reverse) cannot execute |
| `non_straight_penalty` | `1.35` | Spend curvature only where it genuinely shortens the route |
| `change_penalty` | `0.15` | Discourage switching turn direction — removes S-wiggles down open corridors |
| `robot_radius` (costmap) | `0.30` | Was undeclared, so nav2 defaulted to a **0.1 m** robot. 2D got away with that; Hybrid must not, since its value is that returned poses are drivable. 0.10 m baked into the PGM + 0.30 here ≈ the rover's real 0.41 m half-width |

### Generating a path

```
ros2 launch rover_nav nav2_planning.launch.py rviz:=true
ros2 run rover_nav plan_global_path.py --start S1 --points W2 W3 W4 --loop --publish
```

`scripts/plan_global_path.py` exists rather than omar's
`Mars-rover/tools/plan_multi_point_tour.py` because that tool sets every goal
orientation to identity (yaw = 0). `SmacPlanner2D` ignores goal orientation, so
that was harmless there; `SmacPlannerHybrid` **enforces** it, and this nav2
build has no `goal_heading_mode` to relax the constraint — an identity goal yaw
makes the planner add a loop at every waypoint just to arrive facing +X.

Two things that tool does as a result:

- **Heading candidates.** Each waypoint is offered the bisector of its incoming
  and outgoing bearings first, then fallbacks. A badly chosen yaw can make an
  otherwise open leg unplannable: demanding a 168-degree reversal at a point
  with 2 m of clearance needs a turning loop that will not fit inside a 2 m
  radius.
- **One-step lookahead.** A leg's start heading is fixed by however the previous
  leg ended, so a heading that plans fine *into* a waypoint can still leave the
  rover unable to get *out* of it. Measured: S4→S5 plans from a 0 or 45 degree
  start heading but not from 75 or 90. The tool only commits to a heading the
  next leg can actually depart on.

### Driving a global path, crossing every waypoint

The goal is for the rover to physically cross each waypoint — its centre
passing over the marker — without stopping. `cmd_vel_arbiter`'s `path_csv`
mode does that: it drives a dense path from `plan_global_path.py` straight
through, with no per-waypoint halt.

```
ros2 run rover_nav plan_global_path.py --start S1 --points W5 W4 W3 W2 --in-order
ros2 launch aries_bringup full_hardware.launch.py \
  path_csv:=$HOME/jazzy_ws/marsyard/global_path_hybrid.csv \
  waypoints_csv:=$HOME/jazzy_ws/marsyard/global_path_hybrid_waypoints.csv
ros2 service call /planner/start std_srvs/srv/Trigger
```

This is **not** the `WAYPOINTS` sequencer. That one drives to within
`goal_tolerance` (0.5 m) of each point, halts for `halt_time`, then replans —
the opposite of crossing through. `path_csv` mode reuses the continuous
tracking built for the test courses instead.

### Adding ArUco landmark correction on top

`full_hardware.launch.py` is the one launch for competition day: it always
has the global-path follower above available through `path_csv`, and can
optionally layer ArUco landmark correction on top of it with `start_aruco`
and `aruco_snap`. Both default to `false`, so the plain command above already
*is* "global planner only, no ArUco" — nothing extra to pass for that case.

```
ros2 launch aries_bringup full_hardware.launch.py \
  path_csv:=$HOME/jazzy_ws/marsyard/global_path_hybrid.csv \
  waypoints_csv:=$HOME/jazzy_ws/marsyard/global_path_hybrid_waypoints.csv \
  enable_front_camera:=true front_camera_serial:=<serial> \
  start_aruco:=true aruco_snap:=true \
  aruco_start_point:=S1 \
  map_to_odom_yaw_deg:=90
ros2 service call /planner/start std_srvs/srv/Trigger
```

`start_aruco:=true aruco_snap:=false` runs the detector and corrector in
observe-only mode — every correction it *would* make is logged but nothing
moves the rover, which is how to sanity-check the fixes on a new course
before trusting them. See `rover_detection`'s `aruco_pose_reset` node
docstring for why this snaps the EKF pose rather than fusing the landmark
fix, and `aries_bringup/launch/full_hardware.launch.py`'s own
`start_aruco`/`aruco_snap` argument descriptions for the camera requirements.

### Competition day: what to actually tune

Everything below is already set to validated defaults — this is a map of
where each competition-day knob lives, not a to-do list. Re-tune only what
the day's course or hardware actually demands.

| Knob | Where | Default | Tune when |
|---|---|---|---|
| `map_to_odom_x/y/yaw_deg` | `full_hardware.launch.py` args | `0/0/90` (from `global_path_planner.py`) | **Every run.** Depends on exactly how the rover is parked at boot — see the tuning note in `scripts/global_path_planner.py`. 1 degree of yaw error is 35 cm at a waypoint 20 m out |
| `aruco_start_point` | `full_hardware.launch.py` arg | `S1` | Whichever start line the rover is actually placed on |
| `lookahead_min` / `lookahead_max` | `full_hardware.launch.py` args → `cmd_vel_arbiter.py` | `0.4` / `0.7` | Path-tracking tightness vs. recovery margin. `lookahead_max=0.7` is sized to the validated 0-0.7 m ArUco correction range — don't lower it if `start_aruco` is on, or a snap can pin curvature to the clamp and the rover circles instead of rejoining the path (see `cmd_vel_arbiter.py`'s `lookahead_dynamic` comments) |
| `aruco_max_correction_m` | `full_hardware.launch.py` arg → `aruco_pose_reset` | `0.8` | Sanity gate on a single snap. Matches the validated 0-0.7 m range plus headroom; raise together with `lookahead_max` if genuine drift ever exceeds it, never alone |
| `inflation_radius` / `cost_scaling_factor` | `config/nav2_planning_params.yaml`, `global_costmap.inflation_layer` | `1.0` / `3.0` | How far the planner stays from obstacles. Lower `cost_scaling_factor` (cost decays slower) or raise `inflation_radius` if planned paths run too close to rocks; the reverse if paths look unnecessarily indirect |
| `minimum_turning_radius` | `config/nav2_planning_params.yaml`, `planner_server.GridBased` | `1.5` | The rocker-bogie wheel-scrub curvature bound for *this* yard's terrain — see the "Global planner" table above before changing; a wider value can make routes unplannable on tight legs (e.g. W6) |
| `map_yaml` | `nav2_planning.launch.py` arg | `~/jazzy_ws/marsyard/marsyard2026_occupancy.yaml` | Only if the map moves off the standard path |

One-command reference for the day, once `map_to_odom_yaw_deg` is measured and
a route is planned:

```
ros2 launch aries_bringup full_hardware.launch.py \
  path_csv:=<planned route csv> waypoints_csv:=<planned route waypoints csv> \
  start_aruco:=true aruco_snap:=true aruco_start_point:=<S1..S9> \
  map_to_odom_yaw_deg:=<measured>
```

Drop `start_aruco:=true aruco_snap:=true` (or leave them at their `false`
default) to run the global planner alone, with no ArUco correction.

Two differences from test-path mode:

- **It uses `map->odom`.** A global path is in the competition's map frame,
  not the rover's start-relative frame, so unlike the test courses it needs
  the alignment correction from `map_odom_broadcaster.py`. It holds until
  that transform exists rather than assuming identity.
- **It measures crossings continuously.** The rover never arrives anywhere,
  so there is no arrival event to measure at — distance to every waypoint is
  sampled each tick and the minimum kept. On completion it prints the closest
  approach per waypoint.

| Parameter | Default | What it does |
|---|---|---|
| `path_csv` | `""` | Dense path CSV to drive. Empty = waypoint mode |
| `waypoints_csv` | `""` | The `<name>_waypoints.csv` written alongside it, for the crossing report |
| `path_frame` | `map` | `map` applies the map->odom correction; `odom` drives the CSV verbatim |
| `path_goal_tolerance` | `0.05` | How close to the final pose ends the run |

`path_goal_tolerance` is deliberately far tighter than
`test_path_goal_tolerance` (0.25). A test course is judged on the shape
driven, so ending a handspan short costs nothing — but a global path's final
pose *is* a waypoint that has to be crossed. Measured on the
S1→W5→W4→W3→W2 tour, 0.25 left the final waypoint missed by 24.7 cm while
every waypoint before it was crossed within 4.3 cm; at 0.05 the worst case
across all four is 4.7 cm.

**Measured accuracy** (kinematic simulation, perfect odometry):

| | Plan misses by | Rover centre came within |
|---|---|---|
| W5 | 6.3 cm | 4.3 cm |
| W4 | 0.0 cm | 1.3 cm |
| W3 | 5.9 cm | 2.8 cm |
| W2 | 0.0 cm | 4.7 cm |

Path-tracking error against the plan was 4.5 cm mean / 9.1 cm max. The rover
often passes *closer* than the plan's own miss distance, because it moves
continuously between discrete poses.

The residual is dominated by the costmap's 0.1 m grid — the planner cannot
place a pose closer than half a cell to a waypoint, which is exactly the
6.3 cm and 5.9 cm above at the two tight-clearance points. Regenerating the
occupancy grid at 0.05 m resolution would halve that floor if it matters.

**These numbers are a floor, not a forecast.** They come from a perfect
unicycle model; on hardware, EKF drift, wheel slip on regolith, and
`map->odom` alignment error all add on top, and the last of those is
currently an unmeasured hand-set constant (see `MAP_TO_ODOM_*` above).
