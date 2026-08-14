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

## Commands for generating new costmaps

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
