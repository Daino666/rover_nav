# Mars Rover Sim — Setup & Test Guide

ERC Mars Yard 2026 Gazebo simulation, EKF localization, Nav2 global
planner, and pure pursuit path follower. This repo root mirrors
`~/jazzy_ws/src/` exactly — `Mars-rover/`, `rover_nav/`,
`obstacle_detection/`, and `marsyard/` sit as siblings, so it drops
straight into a workspace with no reshuffling.

## 1. Get the code

Inside `~/jazzy_ws/src/`, run:
```bash
git clone -b omar https://github.com/Daino666/rover_nav.git ~/jazzy_ws/src
```

One file doesn't come with the clone: `marsyard2026_visual.obj` (276MB,
over GitHub's 100MB file limit). See
`Mars-rover/rover_description/models/marsyard_mesh/MISSING_VISUAL_MESH.md`
for where to get it and where it goes.

## 2. Install dependencies + build

```bash
cd ~/jazzy_ws/src
chmod +x install_dependencies.sh
./install_dependencies.sh
```

---

## Commands to test

### Representing and testing the path

Planning is standalone — no need for the Gazebo sim to be running yet.
This should bring up **only RViz, showing the map and costmap** (no
robot, nothing driving).

```bash
cd ~/jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rover_nav nav2_planning.launch.py rviz:=true
```

**To choose which points the rover should visit**, edit the parameters
block near the top of `Mars-rover/tools/plan_multi_point_tour.py`:
```bash
cd ~/jazzy_ws/src/Mars-rover/tools
nano plan_multi_point_tour.py
```
```python
START = "S1"                       # starting point (survey name or (x, y) tuple)
POINTS = ["W6", "W5", "W7", "W8"]  # points to visit, any order -- auto-sorted nearest-first
RETURN_TO_START = True             # loop back to START at the end?
```

Then generate the path:
```bash
python3 plan_multi_point_tour.py
```

With the RViz from the step above still open, you should now see the
planned path drawn on the map (gray = raw, green = smoothed).

This also writes a preview image with every point plotted on the map —
`~/jazzy_ws/src/marsyard/marsyard2026_tour_preview.png` — open it to
confirm the route and point order look right before driving it.

Close this planning-only stack before moving on (`pkill -f nav2_planning.launch.py`).

### Running the rover

**1. Launch the simulation (spawns the rover):**
```bash
cd ~/jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rover_description my_robot.launch.py
```
Wait ~20s for full spawn.

**2. Fix up localization for RViz** (the EKF's own TF publishing is
disabled to avoid a self-referential TF loop — this node supplies
`map->odom` and `odom->base_footprint` from the EKF's output instead):
```bash
# new terminal, same sourcing
ros2 run rover_nav odom_tf_broadcaster.py
```

**3. RViz + costmap, this time live against the running sim:**
```bash
# new terminal, same sourcing
ros2 launch rover_nav nav2_planning.launch.py rviz:=true static_tf:=false
```

**4. Path planning** — if you already generated a path in the Testing
section above, the CSV is already saved and this step can be skipped.
To generate a fresh one instead, repeat the "choosing points" steps
above (`plan_multi_point_tour.py`) with this stack running.

**5. Pure pursuit — starts the rover driving the generated path:**
```bash
# new terminal, same sourcing
ros2 run rover_nav Pure_pursuit_Gazebo.py --ros-args -r __node:=pure_pursuit
```

### What you should see

The rover moving along the path **in RViz and in the Gazebo simulation
window at the same time** — same position, same orientation, same
motion, in both. If that holds, localization and the planner/pursuit
pipeline are wired up the same way they were originally, and the setup
is confirmed working end-to-end.

---

## Additional: generating new costmaps

The occupancy grid (costmap) is generated once from the terrain heightmap
and cached as a PGM+YAML pair — regenerate it if you change the terrain,
or want to retune how aggressively it marks terrain as blocked.

```bash
cd ~/jazzy_ws/src/Mars-rover/tools
nano generate_occupancy_grid.py   # edit the PARAMETERS block near the top
python3 generate_occupancy_grid.py
```

**Parameters you can change** (all in the `PARAMETERS` block, lines ~49–83):

| Parameter | Default | What it does |
|---|---|---|
| `SLOPE_THRESH_DEG` | `25.0` | Max climbable slope in degrees. A cell is marked occupied if terrain tilt exceeds this. |
| `ROUGHNESS_THRESH_M` | `0.06` | Max local elevation jitter (meters) tolerated before a cell is occupied — catches rock clusters/steps that a smooth slope reading would miss. |
| `ROUGHNESS_WINDOW_CELLS` | `3` | Neighborhood size (in 10cm cells) used to compute roughness. Smaller = more sensitive to individual small rocks. |
| `INFLATE_RADIUS_M` | `0.20` | How far occupied cells get padded outward (~half the rover's track width), so planned paths keep real clearance. `0` disables inflation. |
| `SLOPE_METHOD` | `"mesh"` | `"mesh"` (recommended): slope fit directly from raw mesh vertices, most accurate. `"grid"`: faster finite-difference on the heightmap raster, fallback if the mesh file isn't available. |
| `SLOPE_MESH_RADIUS_M` | `0.15` | (`SLOPE_METHOD="mesh"` only) Radius of mesh vertices considered per grid cell when fitting local slope. Larger = smoother but blurs small features. |
| `SLOPE_SMOOTH_M` | `0.3` | (`SLOPE_METHOD="grid"` only) Gaussian smoothing applied before computing slope, to remove scan noise. |

A cell ends up occupied if `slope > SLOPE_THRESH_DEG` **OR**
`roughness > ROUGHNESS_THRESH_M` — then gets grown outward by
`INFLATE_RADIUS_M`. Every run also prints each survey point's
clear/BLOCKED status to the console, so you can see the effect of a
parameter change immediately.

All of the same parameters can also be set as command-line flags instead
of edited in the file, e.g. `python3 generate_occupancy_grid.py --slope-thresh 30`
— run `--help` for the full list.

**Where the files go** (all under `~/jazzy_ws/src/marsyard/`):
- `marsyard2026_occupancy.pgm` + `marsyard2026_occupancy.yaml` — the actual costmap `nav2_planning.launch.py` loads (`map_server`'s `yaml_filename` points here)
- `marsyard2026_occupancy_preview.png` — every survey point plotted on the map, color-coded clear/BLOCKED, so you can check the new parameters before planning a path against them
- `mesh_slope_cache.npy` — cached mesh-vertex slope computation (only with `SLOPE_METHOD="mesh"`); delete it or pass `--no-slope-cache` to force a recompute if the mesh source changed

Restart `nav2_planning.launch.py` after regenerating (`map_server` loads
the PGM/YAML once at startup, it won't pick up a change live).

---

## Additional: real-world global-planner test paths

The nav2 global planner above was tuned and validated in simulation.
`rover_nav/scripts/test_paths/generate_test_paths.py` generates five
reference paths for repeating that validation on the physical rover, sized
to the same rover model used in simulation (`Mars-rover/rover_description`,
~0.8m x 0.8m footprint, ~0.71m track width).

```bash
cd ~/jazzy_ws/src/rover_nav/scripts/test_paths
python3 generate_test_paths.py
```

Output goes to `rover_nav/scripts/test_paths/output/`:

| File | Contents |
|---|---|
| `<name>.csv` | Full-resolution waypoints (`x_m, y_m, yaw_rad`, 0.1m apart) — drive these directly, e.g. as the `path` list in `rover_nav/scripts/rover_controller_pure_pursuit.py`, or convert rows to `PoseStamped`s for nav2 |
| `<name>_markers.csv` | Sparse points (~2m apart along the path, `marker_id, x_m, y_m, yaw_rad`) — pace these out with cones/stakes/GPS to check the rover actually passes through them |
| `<name>.jpg` | Plotted path with the same markers numbered, so you can match a cone in the field to a row in the markers CSV |
| `overview.jpg` | All five paths on one sheet, for a quick side-by-side glimpse of scale |

All coordinates are in a **rover-start-relative frame**: origin (0, 0) is the
rover's launch point, +x is its initial heading (yaw = 0), +y is to its
left. Mark that origin and heading on the ground first, then everything
else is relative to it.

**Path sizes** (real-world footprint, so you can picture each one before driving it):

| Path | What it tests | Real-world size |
|---|---|---|
| `straight_line` | Straight-line tracking | 12m long |
| `lane_change` | Straight, then a lane change, then straight | 3.2m straight → 4.8m lane-change maneuver (1.6m lateral shift) → 3.2m straight (11.2m total). The maneuvering region is shaded/labeled on the JPEG: x = 3.2–8.0m |
| `circle` | Constant-curvature tracking | 5.0m radius loop (~10m diameter, ~31m circumference) |
| `circle_transition` | Curvature-step handling | One loop of a 3.0m-radius circle tangent into one loop of a 6.0m-radius circle, both touching at the same point/heading (marked on the JPEG) — ~12m x 12m overall footprint |
| `infinity` | Figure-8 / direction-reversal tracking | Lemniscate with 5.0m half-width lobes, ~8m x 8m overall footprint, self-crossing marked at the origin |

Re-run the script after editing the `CONFIG` block near the top of the file
to rescale — every dimension is expressed as a multiple of the rover's own
footprint/turning radius, so the whole course scales together.

**Marking the paths on the ground:**
1. Pick a launch point and heading in the field; that's local (0, 0), yaw = 0.
2. For each path you're testing, open `<name>_markers.csv` and place a
   cone/stake at each `(x_m, y_m)` — x measured forward along the launch
   heading, y measured to its left.
3. Cross-check placement against `<name>.jpg`: marker numbers on the plot
   match `marker_id` in the CSV.
4. Drive the rover through nav2 (or the CSV path directly) and watch
   whether it passes close to each marker in order — that's the pass/fail
   signal for the real-world test.

Note: on `circle_transition`, several markers sit right on top of each
other near the shared tangent point (where both loops touch) — use the CSV
for exact coordinates there rather than the JPEG labels.

---

## Additional: obstacle detection (RealSense camera)

```bash
cd ~/jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rover_nav obstacle_detection.launch.py
```

Brings up the full RealSense-based obstacle detection pipeline:
1. Launches the RealSense camera driver (`realsense2_camera`) with the depth+RGB pointcloud enabled at 1280x720@30fps.
2. Crops the pointcloud to 0.1–2.0m on the Z axis (front-facing range of interest) with a `pcl_ros` PassThrough filter.
3. Denoises the cropped cloud with a Statistical Outlier Removal filter.
4. Runs the `obstacle_detector` node, which voxel-downsamples the cloud, removes the ground plane via RANSAC, clusters the remaining points with DBSCAN, and discards clusters that are too small or too high. It publishes:
   - `/obstacles/markers` — bounding-box markers per detected obstacle (viewable in RViz)
   - `/obstacle_detected` — a `Bool` flag, true if any obstacle is currently detected
