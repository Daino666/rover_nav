# Handoff: Aries rover IMU/EKF yaw drift investigation

Repo: `~/jazzy_ws/src` (git, branch `Aries_real_world`, remote `Daino666/rover_nav.git`).
Packages live one directory deeper than the repo root: `~/jazzy_ws/src/src/<package>/`.

## Hardware / stack

- Rover: real 6-wheel skid-steer, ODrive CAN motor controllers + wheel encoders.
- IMU: MicroStrain 3DM-GX5-AHRS (no GNSS — magnetometer is the *only* absolute
  heading reference the device has).
- Localization: `robot_localization` EKF (`aries_localization/launch/localization.launch.py`),
  fusing wheel odometry (`rover_nav/scripts/Odom.py` → `/odom`) and the IMU
  (`/microstrain/ekf/imu/data`, relayed through `imu_yaw_zero.py` to zero yaw
  at boot before the EKF ever sees it).
- Config: `rover_nav/config/ekf_config.yaml` (EKF fusion mask/process noise),
  `aries_imu/config/microstrain.yaml` (IMU driver params, measurement covariance).

## The problem

Real hall driving tests (~18-22m straight/loop runs) consistently show **x
stays accurate, y drifts** — anywhere from 0.5m to 1.7m depending on how
aggressively the rover turned during the run.

## What's been ruled out / confirmed (don't re-investigate these)

1. **Not a startup-convergence issue.** Error scales with distance/turning,
   not concentrated near t=0. A pure rotation-in-place control test (no
   translation) stayed near zero (x:0.03, y:0.07 — placement-precision
   noise only), ruling out "turning mechanically corrupts odometry" as a
   standalone cause.

2. **Not the wheel odometry.** `Odom.py`'s `robust_side_displacement()`
   already uses `statistics.median()` of the 3 per-side encoder values as
   the actual displacement estimate (not just for outlier flagging) —
   already immune to any single bad wheel per side. Confirmed sound design;
   only fails if *both other* wheels on a side also slip together
   (uncorrectable by any per-side statistic, not a tuning problem).

3. **Not the vy=0 fusion.** `odom0_config` fuses `vy` as a non-holonomic
   (zero side-slip) constraint — `Odom.py` already publishes
   `twist.twist.linear.y = 0.0` with tuned covariance every message, this
   just lets the EKF use it. Tested A/B directly: removing it roughly
   **tripled** single-loop y error (unconstrained vy random-walks over the
   long straight legs via process noise alone). Keep this enabled.

4. **Not `robot_localization`'s EKF fusion math.** Built `yaw_source_compare.py`
   to log the IMU's own raw AHRS yaw (`/microstrain/ekf/imu/data`) alongside
   the EKF's fused yaw (`/odometry/filtered`) for the same window. Result:
   **swings matched to within 0.3° over a 60s run** (fused swing 10.66°,
   AHRS swing 10.61° in one test; another run: 18.82° vs 18.91°). The EKF is
   just faithfully relaying whatever the AHRS reports — it isn't
   introducing or amplifying anything.

5. **Real physical ground-truth check.** User physically measured the
   rover's actual end position after a run: **x matched the reported
   value, y did not** — the rover did NOT actually end up as far off to the
   side as the software reported. So the y drift is a real *estimation*
   error, not real physical veering. Combined with #4, this means: fused
   yaw swings away from true *while driving*, then settles back close to
   true once stopped (that's why a stationary heading check "looks fine").
   But position is an *integral* of yaw over the whole run — once a
   transient yaw error is baked into x/y via that integral, it doesn't
   undo itself just because yaw later recovers. That's the whole mechanism:
   x barely moves (`cos(small angle) ≈ 1`), y absorbs it (`sin(small angle)`),
   permanently.

6. **Root cause: motor-current EMI on the magnetometer**, most likely.
   Correlates with aggressive turning/differential steering (more motor
   current = bigger yaw swings, observed directly by the user). This was
   already anticipated in `microstrain.yaml`'s own pre-existing comments
   before this investigation started.

7. **Tried loosening `imu_orientation_cov`'s yaw term** (0.0025 → 0.08 rad²,
   in `microstrain.yaml`) to make `robot_localization` trust each
   individual absolute-yaw sample less. Confirmed the new value **was**
   live on the running driver (`ros2 param get
   /microstrain/microstrain_inertial_driver imu_orientation_cov` showed
   ...0.08 in the last slot) — but it had **zero measurable effect**: the
   fused/AHRS swing ratio was still ~1.00 in the next test. This means the
   disturbance likely isn't entering *purely* through the absolute-yaw
   channel.

## Current leading hypothesis (untested — this is the open thread)

`/microstrain/ekf/imu/data`'s `angular_velocity` (the `vyaw` that
`robot_localization` actually fuses at high trust, via `imu0_config`, with
`imu_angular_cov` never touched) is **itself AHRS/Kalman-filtered by the
device**, not a raw gyro reading. If the device's internal filter blends
magnetometer-driven correction into more than just the orientation it
reports — i.e. into the angular *rate* too — then de-weighting only the
absolute-yaw covariance (step 7 above) would never help, because the real
corrupted signal is arriving through the `vyaw` channel, which is still
being trusted fully.

There's a separate, distinct topic — `/microstrain/imu/data` — which is the
device's calibrated sensor output *without* AHRS/Kalman fusion (confirmed
via `/opt/ros/jazzy/share/microstrain_inertial_driver/microstrain_inertial_driver_common/config/params.yml`).
Comparing this "candidate clean" gyro rate against the AHRS-fused one is
the next diagnostic step — see below.

## What's built but NOT YET RUN: `imu_deep_log.py`

This is the tool to resolve the open thread above. It logs, all paired to
the same timestamps and saved to CSV:

- `/microstrain/imu/data` angular_velocity.z — candidate clean (non-AHRS) gyro rate
- `/microstrain/ekf/imu/data` angular_velocity.z — the AHRS-fused rate actually being fused as `vyaw`
- `/microstrain/ekf/imu/data` orientation → yaw — the AHRS-fused yaw
- `/microstrain/imu/mag` magnetic field vector → magnitude — direct EMI evidence (Earth's field magnitude should stay ~constant; a real disturbance changes magnitude, not just direction)
- `/odometry/filtered` → yaw, x, y — final EKF output, for correlation

**Verdict logic once run:**
- If raw `vyaw` stays flat while AHRS-fused `vyaw` swings → the disturbance
  is in the device's internal AHRS fusion, not the raw gyro. Next step:
  either switch `imu0`'s angular-velocity source to the raw topic, or
  disable `filter_heading_source` (magnetometer) entirely, or gate
  magnetometer trust to stationary-only (see "Options considered" below).
- If magnetometer magnitude swings noticeably during driving vs. staying
  flat when stationary → direct confirmation of real EMI at the sensor.
- If raw gyro is *also* disturbed while driving → points at vibration
  coupling into gyro bias instead of (or in addition to) magnetometer EMI;
  disabling the magnetometer alone wouldn't fully fix it.

## This machine has 1 unpushed commit — recreate it manually

The other machine needs these three changes (the last local commit here,
`af6cc7b`, was never pushed). Apply by hand:

### 1. `src/aries_imu/config/microstrain.yaml` — add after the `imu_data_rate: 100` line:

```yaml
    # imu/mag carries the raw magnetometer vector (off by default). Enabled
    # purely for observability -- lets imu_deep_log.py see the actual field
    # magnitude directly, the most direct evidence of real EMI (a genuine
    # disturbance changes the field's magnitude, not just its direction).
    # Doesn't affect fusion; nothing consumes this topic.
    imu_mag_data_rate: 100
```

(This is in addition to the already-pushed `imu_orientation_cov` yaw term
of `0.08` — that part should already be present via git.)

### 2. `src/rover_nav/CMakeLists.txt` — add to the `install(PROGRAMS ...)` list
(alongside the other already-present scripts):

```
    scripts/imu_deep_log.py
```

### 3. `src/rover_nav/scripts/imu_deep_log.py` — new file, full content:

```python
#!/usr/bin/env python3
"""Deep IMU diagnostic logger: raw (non-AHRS) gyro rate, AHRS-fused gyro
rate, AHRS yaw, raw magnetometer vector, and the final EKF-fused output, all
in one CSV, timestamped together.

Built because loosening imu_orientation_cov's yaw term (microstrain.yaml)
had zero measurable effect on how closely /odometry/filtered's yaw tracks
/microstrain/ekf/imu/data's raw AHRS yaw -- confirmed the new covariance
value was actually live on the running driver, so the lack of effect is
real, not a deployment miss. That means the disturbance may not be entering
purely through the absolute-yaw channel: /microstrain/ekf/imu/data's
angular_velocity (the vyaw robot_localization actually fuses at high trust,
imu_angular_cov unchanged) is itself AHRS/Kalman-filtered by the device, not
a raw gyro reading -- if the device's internal filter blends magnetometer
disturbance into more than just the orientation it reports, de-weighting
only orientation wouldn't touch it.

This logs /microstrain/imu/data (calibrated, NOT AHRS-fused -- a candidate
"clean" gyro rate) alongside /microstrain/ekf/imu/data (AHRS-fused, what's
actually being fused as imu0) so the two can be compared directly: if raw
gyro stays flat while the AHRS-fused rate swings, the disturbance is
entering through the device's internal fusion, not the raw sensor itself.
The magnetometer vector's magnitude is also logged -- Earth's field
magnitude should stay roughly constant regardless of orientation, so a
change in magnitude (not just direction) is direct evidence of real EMI
rather than just normal heading change.

Requires imu_mag_data_rate: 100 in microstrain.yaml (already added) for the
magnetometer topic to actually publish.

  ros2 run rover_nav imu_deep_log.py --ros-args -p duration_s:=90.0 -p csv_path:=/tmp/deep_log1.csv
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuDeepLog(Node):
    def __init__(self):
        super().__init__("imu_deep_log")
        self.declare_parameter("duration_s", 90.0)
        self.declare_parameter("raw_imu_topic", "/microstrain/imu/data")
        self.declare_parameter("ahrs_imu_topic", "/microstrain/ekf/imu/data")
        self.declare_parameter("mag_topic", "/microstrain/imu/mag")
        self.declare_parameter(
            "csv_path", f"/tmp/imu_deep_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.start_time = None
        self.rows = []

        self._raw_vyaw = None
        self._ahrs_yaw = None
        self._ahrs_vyaw = None
        self._mag = None  # (x, y, z)

        self.create_subscription(
            Imu, str(self.get_parameter("raw_imu_topic").value), self._raw_cb, 10
        )
        self.create_subscription(
            Imu, str(self.get_parameter("ahrs_imu_topic").value), self._ahrs_cb, 10
        )
        self.create_subscription(
            MagneticField, str(self.get_parameter("mag_topic").value), self._mag_cb, 10
        )
        self.create_subscription(Odometry, "/odometry/filtered", self._fused_cb, 10)
        self.timer = self.create_timer(0.5, self._check_done)
        self.get_logger().info(
            f"Deep-logging raw/AHRS gyro, magnetometer, and fused yaw for "
            f"{self.duration_s:.0f}s -> {self.csv_path}"
        )

    def _raw_cb(self, msg):
        self._raw_vyaw = msg.angular_velocity.z

    def _ahrs_cb(self, msg):
        q = msg.orientation
        self._ahrs_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._ahrs_vyaw = msg.angular_velocity.z

    def _mag_cb(self, msg):
        self._mag = (msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    def _fused_cb(self, msg):
        # Pace rows off the fused output (~20Hz) -- wait until every other
        # source has produced at least one sample to pair with.
        if self._raw_vyaw is None or self._ahrs_yaw is None or self._mag is None:
            return
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t = (now - self.start_time).nanoseconds / 1e9
        q = msg.pose.pose.orientation
        fused_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        mag_mag = math.sqrt(sum(c * c for c in self._mag))
        self.rows.append((
            t, fused_yaw, self._ahrs_yaw, self._raw_vyaw, self._ahrs_vyaw,
            self._mag[0], self._mag[1], self._mag[2], mag_mag,
            msg.pose.pose.position.x, msg.pose.pose.position.y,
        ))

    def _check_done(self):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration_s:
            remaining = self.duration_s - elapsed
            self.get_logger().info(
                f"...{remaining:.0f}s left, {len(self.rows)} samples so far",
                throttle_duration_sec=5.0,
            )
            return
        self._save_and_report()
        self.timer.cancel()
        rclpy.shutdown()

    def _save_and_report(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t_s", "fused_yaw_deg", "ahrs_yaw_deg",
                "raw_vyaw_deg_s", "ahrs_vyaw_deg_s",
                "mag_x", "mag_y", "mag_z", "mag_magnitude",
                "x_m", "y_m",
            ])
            for (t, fused_yaw, ahrs_yaw, raw_vyaw, ahrs_vyaw,
                 mx, my, mz, mmag, x, y) in self.rows:
                writer.writerow([
                    f"{t:.3f}", f"{math.degrees(fused_yaw):.4f}", f"{math.degrees(ahrs_yaw):.4f}",
                    f"{math.degrees(raw_vyaw):.4f}", f"{math.degrees(ahrs_vyaw):.4f}",
                    f"{mx:.6f}", f"{my:.6f}", f"{mz:.6f}", f"{mmag:.6f}",
                    f"{x:.4f}", f"{y:.4f}",
                ])
        n = len(self.rows)
        if n == 0:
            self.get_logger().error("No paired samples recorded -- check all 4 topics are publishing.")
            return
        mags = [r[8] for r in self.rows]
        raw_vyaws = [math.degrees(r[3]) for r in self.rows]
        ahrs_vyaws = [math.degrees(r[4]) for r in self.rows]
        self.get_logger().info(
            "\n"
            f"=== {n} samples -> {self.csv_path} ===\n"
            f"mag magnitude: min={min(mags):.4f} max={max(mags):.4f} "
            f"range={max(mags)-min(mags):.4f} (stable magnitude = no real EMI; "
            f"large range = something is disturbing the field, not just heading)\n"
            f"raw  vyaw range: {max(raw_vyaws)-min(raw_vyaws):.2f} deg/s\n"
            f"AHRS vyaw range: {max(ahrs_vyaws)-min(ahrs_vyaws):.2f} deg/s "
            "(if AHRS range is much bigger than raw, the device's own internal "
            "filter is where the disturbance is entering, not the raw gyro)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuDeepLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### After creating these three, rebuild and run:

```bash
colcon build --packages-select aries_imu rover_nav
# restart the full stack (needed regardless, to pick up imu_mag_data_rate)
ros2 run rover_nav imu_deep_log.py --ros-args -p duration_s:=90.0 -p csv_path:=/tmp/deep_log1.csv
```

Drive the same kind of straight-ish/loop test as before, then read the
printed verdict and/or analyze the CSV directly.

## Current EKF fusion config (for reference — this part IS already pushed)

`rover_nav/config/ekf_config.yaml`:
- `odom0` (wheel odometry, `/odom`): fuses **vx, vy** only. Not x/y/z, not
  roll/pitch/yaw (encoder yaw is never trusted — vulnerable to differential
  slip in ways the median rejection can't always catch), not angular
  velocities/accelerations.
- `imu0` (IMU): fuses **yaw, vyaw** only. Not x/y/z, not vx/vy/vz, not
  accelerations.
- `two_d_mode: true`, `frequency: 20.0`, `world_frame: odom`.
- Process noise: yaw `0.004`, vyaw `0.0015` (tightened from `0.06`/`0.02`
  based on measured stationary IMU noise).

`aries_imu/config/microstrain.yaml`:
- `filter_heading_source: 1` (magnetometer — the only absolute heading
  reference available, no GNSS on this unit).
- `mag_low_pass_filter_enable/auto: true` (pre-existing, GX5-25 may not
  fully support this).
- `imu_orientation_cov` yaw term: `0.08` rad² (loosened from `0.0025`,
  confirmed live, confirmed to have had no effect on the swing-tracking
  problem — see above).
- `imu_angular_cov`: unchanged, `0.00004` — **candidate next thing to
  loosen if `imu_deep_log.py` shows the AHRS-fused vyaw is where the
  disturbance actually enters.**

## Options considered for the actual fix (not yet decided — pending the deep-log data)

1. **Disable magnetometer entirely** (`filter_heading_source: 0`, pure gyro
   integration). Gains: no more EMI-driven swings while driving. Costs: no
   absolute heading reference at all — pure gyro drift will accumulate
   over a long run with nothing correcting it. Risk depends on whether the
   raw gyro itself is clean (see deep-log verdict above).
2. **Gate magnetometer trust to stationary-only** — dynamically swing
   `imu0`'s yaw measurement covariance in `robot_localization` based on
   whether the rover is moving (loose while driving, tight while
   stopped). Matches the directly-confirmed pattern that heading "recovers
   close to truth" once stopped. Doable at the ROS/EKF level without
   touching device firmware — more reversible than option 1. Probably the
   better option if raw gyro turns out clean.
3. **Physical/hardware fix** (the actual root-cause fix, not a software
   workaround): investigate whether the IMU is mounted near/bundled with
   motor power wiring; re-route, twist, or shield if so. Not something
   fixable from software; needs hands-on-hardware time.

## Session norms worth carrying over

- User wants concrete numeric evidence, not guesses — always verify with
  real data (`ros2 param get` to confirm a config change actually landed,
  direct CSV analysis rather than trusting a verbal summary) before
  concluding anything.
- Confirm before applying EKF/IMU config changes (AskUserQuestion-style)
  given this is safety/accuracy-critical on real hardware — this session's
  pattern was propose → confirm → apply → rebuild → verify live → re-test.
- User sets git identity locally; commits are fine to create, but pushing
  requires the user's own credentials (no git push access from an agent
  session) — always hand back the exact push command rather than
  attempting it.
