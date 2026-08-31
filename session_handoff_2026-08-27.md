# Session Handoff — 2026-08-27

**Topic:** Bringing up the rover camera + obstacle detection, ahead of any
nav2 integration. Repo root for this checkout: `~/jazzy_ws/src` (packages
live one level deeper, `~/jazzy_ws/src/src/<package>/`). Not a git repo —
file state only.

The session started as "CLI to use the rover camera and load my local
planner" and turned into a survey of how the nav / camera / detection
pieces actually fit together. Most of the value below is in the
**Findings** section: several things that look wired up are not.

---

## 1. Stack topology (as verified, not as assumed)

```
full_hardware.launch.py
├── aries_hardware.launch.py        arm + gripper + MoveIt/RViz + joy + cameras
├── rover_drive_auto.launch.py
│   └── rover_drive.launch.py       robot_state_publisher (rover.urdf.xacro)
│       ├── aries_localization/localization.launch.py   Odom.py + EKF
│       ├── aries_drive/drive.launch.py                 ODrive CAN bridge
│       └── aries_teleop/joystick.launch.py
├── cmd_vel_arbiter.py              (start_pure_pursuit:=true)  <-- "local planner"
└── full_hardware_checker.launch.py
```

EKF (`/odometry/filtered`, `odom -> base_footprint`) comes free through that
chain. `map -> odom` does **not** — see Finding 5.

### The three detection pipelines

There are three, with three different sets of constants:

| Launch | Chain | Notes |
|---|---|---|
| `rover_nav/obstacle_detection.launch.py` | camera → passthrough(z) → SOR → `pcl_obstacle_detector.py` | The one we used. Publishes `/obstacles/markers` + `/obstacle_detected` |
| `obstacle_detection/full_pipeline.launch.py` | camera → `topic_tools relay` → `obstacle_clustering.py` | **Relay does no filtering.** No z-crop, no voxel, no SOR |
| `obstacle_detection/obstacle_filter.launch.py` | composable pcl_ros passthrough → voxel → SOR | Best filtering of the three; **no launch file includes it** |

---

## 2. Commands

Always: `source /opt/ros/jazzy/setup.bash && source ~/jazzy_ws/install/setup.bash`

### Camera only (bench)

```bash
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  align_depth.enable:=true \
  enable_gyro:=false enable_accel:=false \
  enable_infra1:=false enable_infra2:=false \
  publish_tf:=true
```

`publish_tf:=true` only when nothing else runs `robot_state_publisher`.
Set `false` alongside any bringup, or two publishers fight over the same
TF edges.

### Obstacle-detection tuning rig (recommended — no arm, no MoveIt)

```bash
# terminal 1 — rover hardware + TF + RViz, starts NO camera driver
ros2 launch aries_bringup rover_drive.launch.py use_rviz:=true

# terminal 2 — camera + detection
ros2 launch rover_nav obstacle_detection.launch.py
```

`rover_drive.launch.py` publishes `camera_link` / `camera_depth_optical_frame`
from `rover.urdf.xacro` (which includes `d435i.xacro` unconditionally), so
the detection launch's `publish_tf:=false` is correct and the single D435i
stays free. It also deliberately omits `cmd_vel_arbiter`.

### Full hardware + separate detection pipeline

```bash
# terminal 1  (both camera flags stay at their false defaults)
ros2 launch aries_bringup full_hardware.launch.py \
  start_pure_pursuit:=true pure_pursuit_autostart:=false

# terminal 2
ros2 launch rover_nav obstacle_detection.launch.py
```

### Full hardware owning the camera (requires the edit in Finding 2)

```bash
ros2 launch aries_bringup full_hardware.launch.py \
  enable_depth_sensor:=false \
  enable_front_camera:=true \
  start_pure_pursuit:=true \
  pure_pursuit_autostart:=false
```

Then detection **nodes only** (no second driver), noting the single-`camera`
namespace:

```bash
ros2 run pcl_ros filter_passthrough_node --ros-args \
  -r input:=/camera/depth/color/points -r output:=/pcl/front \
  -p "filter_field_name:='z'" -p filter_limit_min:=0.1 -p filter_limit_max:=2.0 &
ros2 run pcl_ros filter_statistical_outlier_removal_node --ros-args \
  -r input:=/pcl/front -r output:=/pcl/denoised -p mean_k:=10 -p stddev:=1.5 &
ros2 run rover_nav pcl_obstacle_detector.py
```

### nav2 (planning only)

```bash
ros2 launch rover_nav nav2_planning.launch.py rviz:=true static_tf:=false
ros2 run rover_nav map_odom_broadcaster.py        # nothing else starts this
```

`static_tf:=false` is mandatory alongside real hardware.

### Local planner run control

```bash
ros2 service call /aries_drive/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /planner/start std_srvs/srv/Trigger
```

---

## 3. Findings

### 1. nav2 and the local planner are not connected

`nav2_planning.launch.py` is **planning only** — map_server, planner_server,
smoother_server, lifecycle_manager. `controller_server` and `bt_navigator`
appear nowhere in the workspace outside that file's own docstring saying it
omits them. `nav2_planning_params.yaml` has only those four sections.

Separately, `cmd_vel_arbiter.py` has **no `Path` subscription** — its only
`Path` is a *publisher* for RViz viz (line ~225). Waypoints come from
`WAYPOINTS` imported out of `global_path_planner.py`, or a `test_path:=` CSV.

Net: nav2 publishes `/unsmoothed_plan` and `/plan_smoothed`, and nothing
consumes them. The two stacks run side by side without talking.

**To close the loop:** add a `nav_msgs/Path` subscription on `/plan_smoothed`
to `cmd_vel_arbiter` that replaces its waypoint list. Contained change to
its sequencer. **Not done.**

### 2. `full_hardware`'s camera driver publishes no point cloud

`_realsense_driver()` in `aries_hardware.launch.py` (~line 100-125) has no
`pointcloud.enable` in its launch-argument dict — color and aligned depth
only. It also sets `camera_namespace: ""`, so topics are `/camera/depth/...`
and `/gripper_camera/depth/...`, **not** `/camera/camera/depth/...` which
every detection node hardcodes.

Fix, if you want full_hardware to feed detection:

```bash
sed -i '/"align_depth.enable": "true",/a\        "pointcloud.enable": "true",' \
  ~/jazzy_ws/src/src/aries_bringup/launch/aries_hardware.launch.py
colcon build --packages-select aries_bringup --symlink-install
```

**Not applied this session.**

### 3. One camera: detection and arm-vision are mutually exclusive

> **Stale as of 2026-08-28: there are now two D435i on USB** — 216322070216
> (gripper) and 207522077539 (rover front), so they are no longer mutually
> exclusive. The serial-assignment trap below still applies, and gets worse:
> unclaimed serials go gripper-first *lowest serial first*, which for this
> pair assigns them backwards. Pin both explicitly.

Two `realsense2_camera` drivers cannot open the same D435i; the loser exits
*after* locking the device.

- Arm stack (`vision_grasp_node.py`, `soil_sample_node.py`,
  `yolo_detection_node.py`) subscribes to `/gripper_camera/...` only, with
  no topic override.
- Detection wants `/camera/camera/...`.

With one device, unclaimed serials are assigned **gripper-first**
(`aries_hardware.launch.py:157-161`), so `front_camera_serial` stays empty
and `enable_front_camera:=auto` evaluates `"" in detected_serials` → False.
**`auto`/`auto` silently gives you the gripper camera only.** Use
`enable_depth_sensor:=false enable_front_camera:=true` to force the rover
camera.

### 4. Ignored resolution args in `obstacle_detection.launch.py`

Lines 18-19 pass `depth_module.profile` and `rgb_camera.profile`. Neither is
a real `rs_launch.py` argument — the correct names are
`depth_module.depth_profile` and `rgb_camera.color_profile`. Confirmed live:
driver came up at **Depth 848x480x30, Color 1280x720x30**, i.e. driver
defaults, ~400k points at 30 Hz into a Python open3d + sklearn node.

```bash
sed -i "s/'depth_module.profile'/'depth_module.depth_profile'/; \
        s/'rgb_camera.profile'/'rgb_camera.color_profile'/" \
  ~/jazzy_ws/src/src/rover_nav/launch/obstacle_detection.launch.py
```
then set both to `'640x480x15'`. **Not applied this session.**

### 5. `map_odom_broadcaster.py` is started by no launch file

Both nav2's `global_costmap` (needs `map -> base_footprint`) and
`cmd_vel_arbiter` (transforms map-frame `WAYPOINTS` into odom) depend on it.
Run it manually. Defaults to identity; override with
`--ros-args -p map_to_odom_yaw_deg:=<deg>`.

### 6. The detector works entirely in the optical frame

`pcl_obstacle_detector.py` never transforms anything. `_detect()` runs on raw
points and `_publish()` re-stamps them with `msg.header.frame_id`. So
everything is in `camera_depth_optical_frame`, where **X right, Y down,
Z forward**. Consequences:

- The passthrough's `filter_limit_min/max` on `z` is a **range gate**, not a
  height slice.
- `MAX_CENTROID_Z = 1.5` is a **second, tighter range gate** overriding the
  passthrough's 2.0 m. Real detection horizon is 1.5 m.
- **There is no height filtering anywhere.**

### 7. Ground removal is unreliable; use a Y cut instead

> **Superseded 2026-08-28 — do not add the Y passthrough.** The detector now
> removes ground by height (`camera_height` / `ground_margin`) and still fits
> a plane, but only within `ground_band` and only if its normal is within
> `normal_tol_deg` of vertical. That fixes the actual bug diagnosed here (a
> wall winning the fit) without a second passthrough node.

`pcd.segment_plane()` removes exactly **one** plane — the dominant one. In an
indoor corridor cropped to 0.1–2.0 m the dominant plane can be a *wall*, so
the floor survives into DBSCAN. Observed live: floor patches present in
`/pcl/denoised`.

`camera_link` is mounted `rpy="0 0 0"` on `base_link`, so the camera is
**level**, so the floor sits at roughly constant **Y** regardless of range.
A fixed Y cut removes it deterministically:

```bash
ros2 run pcl_ros filter_passthrough_node --ros-args \
  -r input:=/pcl/front -r output:=/pcl/noground \
  -p "filter_field_name:='y'" \
  -p filter_limit_min:=-2.0 -p filter_limit_max:=0.15
```

Then repoint the SOR at `/pcl/noground`. Tune `filter_limit_max` while
watching the floor vanish from `/pcl/denoised`; lower = more aggressive.
Once this works the RANSAC block is redundant and should be deleted — it
costs a plane fit per frame and can remove the wrong surface.

### 8. `DBSCAN_EPS = 0.80` merges everything into one box

> **Fixed 2026-08-28.** Defaults are now 0.20 / 12. Reproduced live first:
> one box spanning a person and a wall section ~0.5 m to their side.

At `VOXEL_SIZE = 0.05` that tolerates a 16-voxel gap, so residual floor
points chain into the real obstacle's cluster. Observed live as one giant
bbox spanning a person plus floor patches. Drop to `0.20` (4-voxel gap) and
raise `DBSCAN_MIN_PTS` from 5 to ~12 — at 5, five stray noise points form a
valid cluster seed.

### 9. YAML 1.1 boolean trap in `ros2` CLI params

`-p filter_field_name:=y` fails with `InvalidParameterTypeException` — YAML
1.1 treats `y`, `n`, `yes`, `no`, `on`, `off` as booleans. Quote it:

```bash
-p "filter_field_name:='y'"                        # ros2 run
ros2 param set /passthrough_filter filter_field_name "'y'"
```

Same parser class as the `value_type=str` comment at
`full_hardware.launch.py:210-215` (which exists because `infinity` parses as
a float).

### 10. RViz gotcha

The MarkerArray display in the saved config defaults to
`/global_path/waypoints`. It shows **Status: Ok** on that topic because the
topic exists and is empty — no error, no boxes. Point it at
`/obstacles/markers`.

### 11. `TEB_Pure_Pursuit_Planner.py` is not installed

Lives at `~/jazzy_ws/src/TEB_Pure_Pursuit_Planner.py`, not in any package's
`install(PROGRAMS ...)`. Its own docstring says `ros2 run rover_nav
TEB_Pure_Pursuit_Planner.py`, which does not work. Run with `python3`.
It also has no obstacle subscription of any kind.

### 12. Camera mount ambiguity — **unresolved**

- `camera_link` → `base_link` at `xyz="0.2756 0.0025 0.2535"`, fixed, level.
- `gripper_camera_link` → `arm_gripper_base_link` at `rpy="0 -1.57 -1.57"`,
  moves with the arm.

Obstacle detection is only geometrically correct with the camera on the
**body mount**. On the gripper mount, boxes are displaced by the arm's full
offset and swing with arm motion — silently, because the frame exists and
TF resolves fine. **Confirm which mount the single D435i is physically on.**

---

## 4. Tuning parameters

**Live-tunable** (`ros2 param set`, no restart):

| Node | Param | Default | Effect |
|---|---|---|---|
| `/passthrough_filter` | `filter_limit_min` / `max` | 0.1 / 2.0 | Z = forward range gate |
| `/sor_filter` | `mean_k` / `stddev` | 50 / 1.0 | Denoise; 50/1.0 eats sparse far returns |

> **Updated 2026-08-28.** The detector was reworked since this was written.
> Every constant below is now a **declared ROS parameter re-read each frame**,
> so nothing here needs an edit or a restart. Ground removal is height-based
> (`camera_height` / `ground_margin`), not the Y-passthrough of Finding 7.

**Live-tunable** on `/obstacle_detector` — use the tuner, which types values off
each node's descriptors and so dodges the Finding 9 YAML trap:

```bash
ros2 run rover_nav tune_obstacle_detection.py     # presets: tight lowrock fast open
```

| Param | Default | Notes |
|---|---|---|
| `voxel_size` | 0.03 | Biggest speed lever |
| `dbscan_eps` | 0.20 | Was 0.80; see Finding 8 |
| `dbscan_min_pts` | 12 | Was 5 |
| `min_cluster_pts` | 15 | Coupled to `voxel_size` |
| `max_range` | 2.0 | Gates on the cluster's **nearest** point, not its centroid |
| `min_obstacle_height` | 0.15 | Height above ground — a real height filter now |
| `camera_height` | 0.489 | Optical centre above ground; measured, not taped |
| `ground_margin` | 0.04 | Keep points this far above the ground surface |
| `ground_band` | 0.15 | Points below this are candidates for the plane fit |
| `fit_ground_plane` | True | Fit within the band vs. assume flat |
| `plane_dist_thresh` / `normal_tol_deg` | 0.05 / 25.0 | Fit tolerance; reject non-horizontal fits |

`/obstacles/stats` publishes `in_pts → voxel_pts → kept_pts → raw_clusters` per
frame, so "my rock disappeared" names the stage that dropped it.

The `voxel_size` ↔ `min_cluster_pts` coupling is the one that bites: coarsen
the voxel without lowering the floor and small rocks vanish silently.

Other pipeline, `obstacle_clustering.py:12-24`: `DBSCAN_EPS` 0.20,
`DBSCAN_MIN_SAMPLES` 20, `MAX_CENTROID_Z` 3.0, `VOXEL_SIZE` 0.03.

`rover_nav` is currently installed as **file copies, not symlinks**, so a code
edit needs a rebuild before `ros2 run` sees it. Parameter tuning never does.
One-time fix:

```bash
rm -rf ~/jazzy_ws/build/rover_nav ~/jazzy_ws/install/rover_nav
colcon build --packages-select rover_nav --symlink-install
```

---

## 5. Test method

The detector is **stateless per frame** — same cloud in, same clusters out —
so bag replay gives deterministic input for parameter sweeps. Live camera
tuning does not.

### Record scenario bags (15-20 s each; the cloud runs up to ~150 MB/s)

```bash
ros2 bag record -o rock_1m --compression-mode file --compression-format zstd \
  /camera/camera/depth/color/points /tf_static
```

| Bag | Tests |
|---|---|
| `rock_1m` / `2m` / `3m` | Range accuracy; real cutoff |
| `two_rocks_60cm` | Whether `DBSCAN_EPS` merges them |
| `low_rock` | Whether `MIN_CLUSTER_PTS` survives downsampling |
| `empty_ground` | **False positives** — most important |
| `sloped_ground` | RANSAC picks the dominant plane |
| `direct_sunlight` | D435i IR washout outdoors — the Mars-yard failure mode |

### Replay loop

```bash
ros2 bag play --loop ~/obst_takes/rock_2m     # terminal A
# terminal B: the pcl_ros + detector nodes, no camera driver
```

In RViz set **Fixed Frame to `camera_depth_optical_frame`** when no
`robot_state_publisher` is running — all published data is already in that
frame, so no TF tree is needed.

### Add per-frame numbers (the node currently logs nothing)

`pcl_obstacle_detector.py` publishes only bbox `LINE_LIST` markers and a
Bool — no count, no centroid, no timing, so `topic echo` gives 24 line
endpoints per obstacle and nothing comparable to a tape measure. Add at the
end of `_detect()`:

```python
self.get_logger().info(
    f'{len(clusters)} clusters @ ' +
    ', '.join(f'{c[:, 2].mean():.2f}m ({len(c)}pts)' for c in clusters))
```

### Acceptance criteria before integrating

- `empty_ground` → `/obstacle_detected` False for the whole bag, no flicker
- `rock_2m` → reported range within ~5 cm of tape, stable across frames
- `/obstacles/markers` rate ≥ cloud rate (else CPU-bound, flag lags reality)
- `direct_sunlight` → degrades to *no detections*, never false detections

### Tuning order

1. Y passthrough until `/pcl/denoised` shows only the obstacle, no floor
2. Then `DBSCAN_EPS` until the box hugs the obstacle
3. Then `MIN_CLUSTER_PTS` / `DBSCAN_MIN_PTS` against `empty_ground`

Do not do 2 before 1 — with floor still in the cloud you will drive `EPS`
far lower than it should be and then miss real multi-part obstacles outdoors.

Corridor testing is misleading: walls at 1-1.5 m are *correctly* obstacles
there but absent on the yard. Validate against an outdoor `empty_ground` bag.

---

## 6. Open items

Done since (2026-08-28):

- [x] `DBSCAN_EPS` 0.80 → 0.20, `DBSCAN_MIN_PTS` 5 → 12 — confirmed live: at
      0.80 a single box spanned a person and a wall section ~0.5 m to their
      side; 0.20 / 12 splits them
- [x] Lift the detector constants into declared ROS parameters — all of them,
      re-read per frame
- [x] Add per-frame numbers — `/obstacles/stats` publishes
      `in_pts / voxel_pts / kept_pts / raw_clusters`
- [x] Y passthrough / RANSAC — **superseded**, not done as written. The
      detector now removes ground by height (`camera_height` /
      `ground_margin`) and the plane fit is kept but fitted only within
      `ground_band` and rejected if tilted past `normal_tol_deg`, which was
      the actual failure in Finding 7

Still open:

- [ ] Confirm which mount each D435i is on (Finding 12) — blocks trusting any
      detection geometry
- [ ] Apply the resolution-arg fix (Finding 4) — still unfixed;
      `obstacle_detection.launch.py` passes `depth_module.profile` /
      `rgb_camera.profile`, neither of which is a real `rs_launch.py` arg, so
      the driver silently comes up at its 848x480x30 default
- [ ] Pin a serial in `obstacle_detection.launch.py`. **Finding 3 is out of
      date: there are now two D435i on USB** — 216322070216 (gripper, per the
      comment at `aries_hardware.launch.py:24-29`) and 207522077539 (rover
      front). The launch pins neither, so it binds whichever librealsense
      enumerates first. Note auto-assignment takes the *lowest* serial for the
      gripper, which for this pair is backwards from the wiring
- [ ] Rebuild `rover_nav` with `--symlink-install` (currently file copies)
- [ ] Record the scenario bags, especially `empty_ground` and
      `direct_sunlight` outdoors
- [ ] Add a `use_camera:=false` arg to `obstacle_detection.launch.py` so bag
      replay is one command
- [ ] Wire `/obstacle_detected` into `cmd_vel_arbiter` as a stop-gate
- [ ] Wire `/plan_smoothed` into `cmd_vel_arbiter` to connect nav2 (Finding 1)
