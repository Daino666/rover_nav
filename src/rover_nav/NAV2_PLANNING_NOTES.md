# Nav2 planning stack — setup notes

Handoff notes from getting `rover_nav`'s Nav2 planning-only stack running,
first in isolation and then against the real rover. Read this before
touching `nav2_planning*.launch.py`, `nav2_planning_params.yaml`, or
`maps/`.

---

## 1. Multiple workspaces on this machine — know which one you're building

There are (at least) four separate trees containing a `rover_nav` package,
and there is no git tying them together, so they silently drift:

| Path | Role |
|---|---|
| `~/Mars-rover` | Auto-sourced by `.bashrc` (`source ~/Mars-rover/install/setup.bash`) — this is the default underlay in every new shell. |
| `~/test` (outer) | A colcon workspace whose `src/` is `~/test/src` (itself a workspace root — see next row). Building here installs to `~/test/install`, which nothing sources by default. |
| `~/test/src` (nested, the one that matters) | A second, nested colcon workspace: source tree is `~/test/src/src/*` (yes, `src/src`), installs to `~/test/src/install`. **This is the one actually sourced in the working shell** (`source ~/test/src/install/setup.bash`) and where all the file paths below live. |
| `~/jazzy_ws` | A teammate's ("hsm") separate copy of the same packages. Has diverged from `~/test/src/src/rover_nav` — it has obstacle-avoidance launch files/scripts ours doesn't, and ours has `cmd_vel_arbiter.py` / `global_path_planner.py` / `map_odom_broadcaster.py` it doesn't. Never reconciled — do that before it drifts further. |

**Build/source loop, every time you touch a launch/config/script file:**

```bash
cd ~/test/src
colcon build --packages-select rover_nav
source ~/test/src/install/setup.bash
```

If `ros2 launch` says a file "was not found in the share directory," 9/10
it's this: you built into the wrong workspace, or forgot to re-source after
a rebuild (sourcing isn't automatic — it's not in `.bashrc`).

---

## 2. The planning stack is planning-only — no driving, no live obstacle avoidance

`nav2_planning*.launch.py` brings up exactly: `map_server` → `global_costmap`
(static + inflation layers) → `planner_server` (SmacPlanner2D) →
`smoother_server` (SimpleSmoother) → `lifecycle_manager`. Confirmed by
`nav2_planning_params.yaml`'s top-level keys — nothing else is in there.

**What this means in practice:**

- No `controller_server`, no local costmap, no `bt_navigator`. It plans a
  path between two poses when asked; it does not follow one, and it does
  not react to obstacles that show up live. `obstacle_detector_node.py` /
  `pcl_obstacle_detector.py` (under `scripts/Rover_path_controller/`)
  publish an `ObstacleArray` but nothing in this codebase currently
  subscribes to it — that perception path is disconnected from both this
  planner and `cmd_vel_arbiter.py`.
- RViz's "2D Nav Goal" click tool does **nothing** here — there's no
  `bt_navigator`/`NavigateToPose` action server listening for it. Nothing
  auto-triggers a plan on launch either. You have to explicitly ask
  `planner_server` for a path:
  - one-off: `ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{goal: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0}}}}"`
  - multi-point tour: `tools/plan_multi_point_tour.py` (see §5).
- Separately, `full_hardware.launch.py` already auto-starts
  `cmd_vel_arbiter.py` (pure-pursuit, `start_pure_pursuit:=true` by default)
  — that's a **pre-existing, completely separate** navigation system that
  drives to hardcoded `WAYPOINTS` in `global_path_planner.py`. Nothing
  currently bridges `planner_server`'s computed path into an actual
  controller that drives the rover — bringing up both at once is two
  independent nav concepts side by side, not one pipeline.

---

## 3. Sim vs. real: two different launch files, because the TF source differs

Split into `nav2_planning_sim.launch.py` / `nav2_planning_real.launch.py`
(the original `nav2_planning.launch.py` still exists, untouched, for
standalone testing with a manual `static_tf` flag). The split exists
because `global_costmap` needs a real `map -> base_footprint` chain, and
sim and real provide it via genuinely different nodes:

- **Real** (`nav2_planning_real.launch.py`): includes `localization.launch.py`
  (encoder odom + BNO055 IMU + EKF) plus `map_odom_broadcaster.py`. The
  real EKF (`ekf_config.yaml`) has `publish_tf: true`, so it publishes
  `odom -> base_footprint` itself; `map_odom_broadcaster.py` adds the
  static `map -> odom` alignment on top.
- **Sim** (`nav2_planning_sim.launch.py`): includes `odom_tf_broadcaster.py`
  instead. The *sim's* EKF (a different instance, from `my_robot.launch.py`
  in `aries_bringup`) has `publish_tf` forced `False` — see that script's
  own docstring for why — so `odom_tf_broadcaster.py` bridges
  `/odometry/filtered` into `odom -> base_footprint` plus a static
  `map -> odom` identity.
- `odom_tf_broadcaster.py` existed in `scripts/` but was never in
  `CMakeLists.txt`'s `install(PROGRAMS ...)` — added it, otherwise
  `nav2_planning_sim.launch.py` fails at runtime with "executable not
  found."

Both new launch files declare a `map` argument (see §4) and `rviz`
(default `false`) — no other required args.

Full real-robot bring-up, three separate commands:
```bash
ros2 launch aries_bringup full_hardware.launch.py         # robot model, real drive (add start_pure_pursuit:=false if you don't want the waypoint follower armed)
ros2 launch rover_nav nav2_planning_real.launch.py rviz:=true
# then trigger a path request — see §2 / §5
```
`full_hardware.launch.py` requires real hardware: BNO055 IMU on
`/dev/ttyUSB0` (via `localization.launch.py`), Teensy on
`/dev/serial/by-id/usb-Teensyduino_USB_Serial_...`, CAN interface `can0`.
If those aren't present/powered, expect hangs or errors from those specific
nodes rather than a clean launch failure.

---

## 4. Map: package-relative now, not a teammate's absolute path

`nav2_planning_params.yaml`'s `map_server.yaml_filename` used to be
hardcoded to `/home/hsm/jazzy_ws/src/marsyard/marsyard2026_occupancy.yaml`
— fails on any machine that isn't hsm's. Fixed:

- Map files (`marsyard2026_occupancy.yaml`/`.pgm`) and the survey
  coordinates file (`2026_MarsYard_3D_Model.../Coordinates_MarsYard2026.txt`,
  needed by `tools/plan_multi_point_tour.py`) now live in `rover_nav/maps/`,
  copied from `~/jazzy_ws`. `CMakeLists.txt` installs that directory.
- `nav2_planning_params.yaml` just has the bare filename now, with a
  comment noting it's overridden at launch.
- All three `nav2_planning*.launch.py` files declare a `map` argument
  (default: `maps/marsyard2026_occupancy.yaml` inside the package) and
  merge it into `map_server`'s parameters:
  `parameters=[params_file, {'yaml_filename': LaunchConfiguration('map')}]`.
  Swap maps with `map:=/path/to/other.yaml` — no file edits needed.

---

## 5. Triggering paths: `tools/plan_multi_point_tour.py`

New file, `~/test/src/src/tools/plan_multi_point_tour.py` — adapted copy of
`~/jazzy_ws/src/Mars-rover/tools/plan_multi_point_tour.py`. Same logic
(nearest-neighbor ordering, then one `getPathThroughPoses` +
`smoothPath` call against the already-running `planner_server`/
`smoother_server`, writes a waypoints CSV + preview PNG into
`rover_nav/maps/`), adapted:

- Paths point at `rover_nav/maps/` directly (this workspace has no sibling
  `marsyard/` directory the way `jazzy_ws` does).
- `--start` accepts a raw `x,y` coordinate directly (e.g. `--start 3,2`),
  not just a named survey point — same as `--points` already did.

```bash
ros2 launch rover_nav nav2_planning_real.launch.py    # bring the planner up first
python3 ~/test/src/src/tools/plan_multi_point_tour.py \
  --start 3,2 --points W6 W5 W7 W8 --use-sim-time false
```

`--use-sim-time` defaults `true` (matching the original tool) — always pass
`false` against the real robot, since `nav2_planning_params.yaml` runs with
`use_sim_time: false`; a mismatch here can make the action call hang
silently rather than error.

---

## 6. Open items, not yet done

- `~/test/src/src/rover_nav` vs `~/jazzy_ws/src/rover_nav` divergence
  (§1) — needs a real merge/reconciliation with hsm, ideally after finally
  putting this under git.
- No `controller_server`/local costmap/`bt_navigator` — planned paths
  don't drive the robot yet, and there's no live obstacle avoidance (see
  §2). If/when that's wired up, it should consume `planner_server`'s
  output, not duplicate `cmd_vel_arbiter.py`'s separate pure-pursuit path.
- `map_odom_broadcaster.py`'s `map_to_odom_yaw_deg` alignment is still a
  manually-set placeholder (identity) in `global_path_planner.py` — the
  actual competition-day alignment procedure is still undecided (also
  noted in `README.md`).
