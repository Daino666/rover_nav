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
