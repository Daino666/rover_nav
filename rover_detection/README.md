# rover_detection

ArUco landmark detection for the ERC 2026 Mars Yard: detects the competition's
L1–L15 landmark markers on a RealSense camera feed, and publishes the rover
chassis center's position in the competition's global survey frame
(`rover_perception_msgs/RoverPositionFix`: landmark ID + X/Y, X=right,
Y=front, meters) whenever a real landmark fix is available.

This package is a pure ROS2 **consumer** of camera topics — it does not
launch or own the RealSense camera itself, so it can run alongside whatever
else (obstacle avoidance, navigation) also needs the same physical camera.

## 1. What to install

Assumes ROS2 Jazzy already installed and sourced (`/opt/ros/jazzy/setup.bash`).

**ROS2 package dependencies** (installable via `rosdep`, see below):
`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`,
`message_filters`, plus the sibling `rover_perception_msgs` package (already
in this repo — no separate install needed, just build it alongside).

**System Python dependencies** (not ROS packages, install directly):
```bash
# opencv with the aruco contrib module -- almost certainly already present
# if any ROS2 desktop/perception install is on this machine already.
python3 -c "import cv2.aruco" || sudo apt install python3-opencv

# Only needed for the standalone tuning script (tools/realsense_aruco_test_v3.py),
# NOT for the ROS node itself -- the node subscribes to ROS topics instead.
pip install --break-system-packages pyrealsense2
```

You'll also need the RealSense camera driver installed separately (it's
launched independently of this package, see below):
```bash
sudo apt install ros-jazzy-realsense2-camera
```

## 2. How to install / build

From the repo root (the colcon workspace root — `src/` sits directly under it):

```bash
cd ~/HSM_Aries   # or wherever this repo is checked out
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select rover_perception_msgs rover_detection
source install/setup.bash
```

(`colcon build` with no `--packages-select` also works and builds everything
else in the repo too — the two-package select above is just faster if you
only care about this piece.)

## 3. How to launch

Camera and detection node run as two separate steps, in two terminals — this
package deliberately does **not** start the camera itself (see the top of
`launch/aruco_detection.launch.py` for why: it keeps the camera shareable
with whatever else needs it).

**Terminal 1 — camera** (`align_depth.enable:=true` is required, not optional
— see §6):
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

**Terminal 2 — detection node:**
```bash
cd ~/HSM_Aries
source install/setup.bash
ros2 launch rover_detection aruco_detection.launch.py
```

**Terminal 3 — verify it's working:**
```bash
source install/setup.bash   # custom message types need this sourced in THIS terminal too
ros2 topic echo /aruco_localization/rover_position_fix
```
Point the camera at a landmark — you should see messages with `landmark_id`
(51–65) and `position.x`/`position.y`. Nothing is published on frames where
no landmark is visible — that's intentional (see the docstring in
`rover_detection/aruco_detect_roverpos.py`), not a bug.

**To see the camera feed with detections drawn on it:**
```bash
ros2 run rqt_image_view rqt_image_view /aruco_localization/debug_image
```

## 4. Every terminal needs its own `source`

`source install/setup.bash` only affects the terminal it's run in — it does
not carry over to other terminals, and running it once doesn't mean it stays
sourced forever if you close and reopen a shell. If `ros2 topic echo` says a
message type is "invalid," or `ros2 launch`/`ros2 run` can't find this
package, the fix is almost always: `cd` to the workspace root and `source
install/setup.bash` in *that specific terminal*.

## 5. Configuration — `config/aruco_localization_params.yaml`

Every tunable value the node uses lives in this one file, loaded
automatically by `aruco_detection.launch.py`. After editing it, rebuild
before relaunching:
```bash
colcon build --packages-select rover_detection
```

### Starting position — update this before every run

```yaml
start_point: S1
```

Set this to whichever start line (`S1`–`S9`) you're told 30 minutes before
your run. On launch, the node publishes **one** `RoverPositionFix` using that
point's known coordinates, tagged `landmark_id: -1` (an ID outside the real
51–65 range, so it's clearly not a real detection) — a starting estimate for
whatever's consuming this topic, published exactly once and never repeated.
Every message after that is a real landmark fix or nothing at all.

### Marker size — measure your actual printout

```yaml
marker_length_m: 0.150
```

The ERC spec's nominal size is 150mm, but if a printed sheet was scaled to
fit a page, the real printed square can be smaller. **Measure the printed
marker itself with a ruler/calipers** and put the real number here — every
reported distance and position scales linearly with this value, so getting
it wrong doesn't cause an error, it just makes every position confidently
wrong by that scale factor.

## 6. Tuning the camera offset (do this after mounting the camera on the rover)

`camera_offset_x/y/z` in the same config file describe where the camera sits
relative to the rover chassis's own reference point (rover body frame:
x=forward, y=left, z=up, meters):

```yaml
camera_offset_x: 0.2608
camera_offset_y: 0.00
camera_offset_z: 0.2535
```

**Step 0 — agree on what "rover center" means.** Before measuring anything,
confirm with the navigation team which physical point on the chassis all
position outputs are supposed to represent (usually the URDF `base_link`
origin, or the geometric center of the wheelbase). Mark it physically on the
rover. If this doesn't match what the nav team's localization considers the
rover's origin, no amount of camera calibration will fix that mismatch.

**Step 1 — rough baseline (tape measure).** With the rover on level ground,
measure from the marked center point to the camera's front glass: forward
distance, left/right offset (usually 0 if centered), height. Good enough to
get running; off by a few cm because the camera's actual sensor sits inside
the housing, not at the visible glass.

**Step 2 — precision calibration (recommended).** This sidesteps needing to
know exactly where the internal optical sensor is:

1. Use `tools/realsense_aruco_test_v3.py` (standalone, no ROS needed) —
   temporarily set `CAMERA_OFFSET_X_FWD/Y_LEFT/Z_UP = 0, 0, 0` at the top of
   that file.
2. Place one ArUco marker in front of the stationary rover. Tape-measure its
   **true** position relative to your marked rover-center point (forward
   distance, left/right offset) — a simple flat measurement, much easier to
   get right than locating a sensor inside a camera housing.
3. Run the script (`python3 tools/realsense_aruco_test_v3.py`) and read off
   what it reports for that marker: `(measured_fwd, measured_left,
   measured_up)`.
4. Compute:
   ```
   camera_offset_x = true_fwd  - measured_fwd
   camera_offset_y = true_left - measured_left
   camera_offset_z = true_up   - measured_up
   ```
5. Put those numbers in `config/aruco_localization_params.yaml`
   (`camera_offset_x/y/z`) and rebuild.
6. **Validate**: move the marker to a new, freshly-measured position and
   confirm the computed output now matches. If it doesn't, something beyond
   the offset is wrong (marker size, depth accuracy) — recalibrating the
   offset again won't fix that.

**What this won't catch: mount tilt.** This calibration is a pure
translation (XYZ shift) — it assumes the camera is mounted level and facing
exactly forward. If calibration error gets worse at longer range or depends
on test direction, that's tilt/rotation in the mount, not a translation
error, and needs the physical mount adjusted, not another offset tweak.

## 7. Known limitations (by design, not oversights)

- **No heading/yaw tracking.** The rover's body frame is assumed aligned
  with the global frame (forward = global +Y) at the moment of each
  sighting. If the rover is facing some other direction when it sights a
  landmark, the reported X/Y is off by that heading error. Fixing this
  requires the rover's actual heading (e.g. from the nav team's EKF/IMU)
  composed into the math — this package doesn't attempt it, and computing
  heading from a landmark's own visual orientation genuinely does not work
  for these markers (see the module docstring history in
  `aruco_detect_roverpos.py` — ERC landmarks are 4-sided poles with an
  identical graphic on every face, so a single sighting's orientation is
  ambiguous by an unknown multiple of 90°).
- **No covariance/uncertainty estimate.** Don't fabricate one to force this
  into `PoseWithCovarianceStamped` for an EKF without first measuring this
  pipeline's actual real-world error at typical operating ranges.
- **~3m reliable detection range.** This is the RealSense D435i's own rated
  "ideal accuracy" depth range (Intel's spec: <2% error at 2m, degrading
  past it), not a limitation of this code — see `min_valid_depth_samples`/
  `max_depth_std_m` in the config for the rejection thresholds this uses to
  avoid trusting noisy depth beyond that range.
- **Requires `align_depth.enable:=true`** on the camera launch. Depth must
  line up pixel-for-pixel with the color image; raw unaligned depth has a
  different resolution/FOV and would silently produce wrong positions, not
  an error.
- **Exclusive camera access still applies at the physical device level**:
  this package doesn't launch the camera itself specifically so it *can*
  share a running camera with other consumers, but only one process can
  ever launch `realsense2_camera` against one physical device at a time.

## 8. Package layout

```
rover_detection/
├── rover_detection/
│   ├── aruco_math.py             # core detection/pose math, unit-tested (test/)
│   └── aruco_detect_roverpos.py  # the ROS node
├── launch/aruco_detection.launch.py
├── config/aruco_localization_params.yaml
├── test/test_aruco_math.py       # run with: python3 -m pytest test/
└── tools/realsense_aruco_test_v3.py   # standalone (no ROS) bench-test/tuning tool
```
