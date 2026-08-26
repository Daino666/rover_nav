#!/usr/bin/env python3
"""
Generates reference test paths for real-world validation of the pure-pursuit
waypoint follower (scripts/cmd_vel_arbiter.py) and the global path shaping in
scripts/global_path_planner.py, both of which were tuned against the Gazebo
sim rather than the physical rover.

Ported from rover_nav's `omar` branch (where the same five paths validate its
nav2 global planner), with the CONFIG block left numerically identical on
purpose: the point is to drive the *same* course here that was driven there,
so a tracking result on this rover is directly comparable.

Each path is produced in a local, rover-start-relative frame:
  - origin (0, 0)   = rover starting position
  - +x              = rover's initial heading (yaw = 0, forward)
  - +y              = left of the initial heading

That frame is exactly what `/odometry/filtered` reports on this branch:
`scripts/imu_yaw_zero.py` (started by aries_localization/launch/localization.launch.py)
zeroes fused yaw at the first IMU sample, so the EKF's `odom` frame starts at
the rover's own launch pose facing +x. So these CSVs can be driven directly in
`odom`, with no map->odom alignment needed -- see `test_path_loader.py` and
`cmd_vel_arbiter.py`'s `test_path` parameter.

Path sizes are scaled to this rover, from the neutral (zero rocker/bogie
angle) pose in src/aries/urdf/{aries_base,left_link,right_link}.xacro:
  - wheel-center wheelbase   ~0.552 m (front/rear wheel x-extent:
    L_1 at x=-0.323 m through L_3 at x=+0.230 m)
  - wheel-center track width  0.759 m (rocker joints at y=+-0.2842 m plus the
    wheels' own +-0.0953 m offset; matches <wheel_separation>0.759</wheel_separation>
    in aries_gazebo.xacro. aries_drive's track_width_m=0.566 is the smaller
    *effective* skid-steer value the ODrive bridge uses for its differential
    mixing, not a geometric span)
  - wheel radius              0.110 m, thickness 0.070 m (common_properties.xacro)
giving an overall ground footprint of roughly 0.77 m x 0.83 m (wheelbase/track
plus wheel radius/thickness overhang), which the 0.8 m x 0.8 m nominal below
rounds. All path dimensions are expressed as multiples of that footprint so
the course scales if the rover does.

Outputs (to ./output/):
  <name>.csv          - x_m, y_m, yaw_rad waypoints, spaced CONFIG["spacing"] apart
  <name>_markers.csv   - sparse ground-truth points (~MARKER_SPACING apart) to
                         physically mark on the field (cones/stakes/GPS) and
                         check the rover passes near each one
  <name>.jpg          - plotted path with the marker points numbered
  overview.jpg        - all five paths on one sheet
"""

import csv
import os

import matplotlib.pyplot as plt
import numpy as np

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

# Rover ground footprint (see module docstring for derivation).
ROVER_LENGTH = 0.8  # [m] bounding length (wheelbase + wheel radius overhang)
ROVER_WIDTH = 0.8  # [m] bounding width (track width + wheel thickness)

# Skid-steer rocker-bogie rigs can spin in place, so no radius is
# mechanically forbidden -- but tracking a very tight arc with a
# pure-pursuit controller scrubs the rocker-bogie wheels hard against each
# other's arcs. Keep working radii at/above this multiple of the wheelbase
# for clean, low-slip tracking.
MIN_SAFE_RADIUS = 2.5 * ROVER_LENGTH  # [m] -> 2.0 m

MARKER_SPACING = 2.0  # [m] arc-length spacing between physical ground-marker points

CONFIG = {
    "spacing": 0.1,  # [m] waypoint spacing along each path
    "straight": {
        "length": 15 * ROVER_LENGTH,  # [m] -> 12.0 m
    },
    "lane_change": {
        "lead_in": 4 * ROVER_LENGTH,  # [m] -> 3.2 m
        "transition_length": 6 * ROVER_LENGTH,  # [m] -> 4.8 m
        "lane_offset": 2 * ROVER_WIDTH,  # [m] -> 1.6 m, clears the 0.8 m body with margin
        "lead_out": 4 * ROVER_LENGTH,  # [m] -> 3.2 m
    },
    "circle": {
        "radius": 2.5 * MIN_SAFE_RADIUS,  # [m] -> 5.0 m
    },
    "circle_transition": {
        "r1": 1.5 * MIN_SAFE_RADIUS,  # [m] -> 3.0 m
        "r2": 3 * MIN_SAFE_RADIUS,  # [m] -> 6.0 m
    },
    "infinity": {
        # Lemniscate "a" parameter (half-width of each loop). Min radius of
        # curvature at the loop tips is 2a/3, so a >= 1.5*MIN_SAFE_RADIUS
        # keeps the tightest point of the figure-8 at/above MIN_SAFE_RADIUS.
        "scale": 2.5 * MIN_SAFE_RADIUS,  # [m] -> 5.0 m
    },
}


def _yaw_from_xy(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    return np.arctan2(dy, dx)


def _smootherstep(t):
    # Quintic (minimum-jerk-ish) smoothstep: zero velocity & acceleration at
    # both ends, so the lane-change has no sharp curvature discontinuity.
    t = np.clip(t, 0.0, 1.0)
    return 6 * t**5 - 15 * t**4 + 10 * t**3


def make_straight(cfg, spacing):
    length = cfg["length"]
    x = np.arange(0.0, length + spacing / 2, spacing)
    y = np.zeros_like(x)
    return x, y, {}


def make_lane_change(cfg, spacing):
    lead_in = cfg["lead_in"]
    trans_len = cfg["transition_length"]
    offset = cfg["lane_offset"]
    lead_out = cfg["lead_out"]
    total = lead_in + trans_len + lead_out

    x = np.arange(0.0, total + spacing / 2, spacing)
    y = np.zeros_like(x)

    x1, x2 = lead_in, lead_in + trans_len
    in_transition = (x >= x1) & (x <= x2)
    y[in_transition] = offset * _smootherstep((x[in_transition] - x1) / trans_len)
    y[x > x2] = offset

    meta = {
        "maneuvering_region_x_m": (x1, x2),
        "lane_offset_m": offset,
    }
    return x, y, meta


def make_circle(cfg, spacing):
    r = cfg["radius"]
    n = int(round(2 * np.pi * r / spacing))
    theta = np.linspace(0.0, 2 * np.pi, n, endpoint=True)
    # Start at origin heading +x, curving left (CCW), center at (0, r).
    x = r * np.sin(theta)
    y = r * (1 - np.cos(theta))
    return x, y, {"center_m": (0.0, r)}


def make_circle_transition(cfg, spacing):
    r1, r2 = cfg["r1"], cfg["r2"]

    n1 = int(round(2 * np.pi * r1 / spacing))
    theta1 = np.linspace(0.0, 2 * np.pi, n1, endpoint=True)
    x1 = r1 * np.sin(theta1)
    y1 = r1 * (1 - np.cos(theta1))

    n2 = int(round(2 * np.pi * r2 / spacing))
    theta2 = np.linspace(0.0, 2 * np.pi, n2, endpoint=True)
    x2 = r2 * np.sin(theta2)
    y2 = r2 * (1 - np.cos(theta2))

    # Both loops start/end at the origin, tangent to the x-axis, same
    # heading (+x) and same turn direction (CCW) -> continuous position and
    # heading at the handoff, curvature jumps there. That jump *is* the
    # "transition point" being tested.
    x = np.concatenate([x1, x2[1:]])
    y = np.concatenate([y1, y2[1:]])

    meta = {"transition_point_m": (0.0, 0.0), "r1_m": r1, "r2_m": r2}
    return x, y, meta


def make_infinity(cfg, spacing):
    a = cfg["scale"]
    t = np.linspace(0.0, 2 * np.pi, 4000, endpoint=False)
    denom = 1 + np.sin(t) ** 2
    xr = a * np.cos(t) / denom
    yr = a * np.sin(t) * np.cos(t) / denom

    # Re-parameterize by arc length for even spacing.
    ds = np.hypot(np.diff(xr), np.diff(yr))
    s = np.concatenate([[0.0], np.cumsum(ds)])
    total_len = s[-1]
    n = int(round(total_len / spacing))
    s_uniform = np.linspace(0.0, total_len, n, endpoint=True)
    xr = np.interp(s_uniform, s, xr)
    yr = np.interp(s_uniform, s, yr)

    # The curve crosses the origin twice, at s=0 and s=total_len/2 -- roll
    # so it starts at a crossing, then translate/rotate so it starts at the
    # local-frame origin heading +x.
    start_idx = int(round(len(xr) / 4))  # one of the two crossings
    xr = np.roll(xr, -start_idx)
    yr = np.roll(yr, -start_idx)
    xr = np.concatenate([xr, xr[:1]])
    yr = np.concatenate([yr, yr[:1]])

    xr -= xr[0]
    yr -= yr[0]
    heading0 = np.arctan2(yr[1] - yr[0], xr[1] - xr[0])
    c, s_ = np.cos(-heading0), np.sin(-heading0)
    x = xr * c - yr * s_
    y = xr * s_ + yr * c

    return x, y, {"crossing_point_m": (0.0, 0.0)}


def resample_markers(x, y, yaw, spacing):
    ds = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(ds)])
    total_len = s[-1]
    n = max(int(np.floor(total_len / spacing)), 1)
    s_markers = np.linspace(0.0, n * spacing, n + 1)
    s_markers = s_markers[s_markers <= total_len]
    if s_markers[-1] < total_len - 1e-6:
        s_markers = np.concatenate([s_markers, [total_len]])  # always mark the end
    mx = np.interp(s_markers, s, x)
    my = np.interp(s_markers, s, y)
    myaw = np.interp(s_markers, s, np.unwrap(yaw))
    myaw = (myaw + np.pi) % (2 * np.pi) - np.pi
    return mx, my, myaw


PATHS = {
    "straight_line": ("Straight line", make_straight, CONFIG["straight"]),
    "lane_change": ("Straight - lane change - straight", make_lane_change, CONFIG["lane_change"]),
    "circle": ("Circular path", make_circle, CONFIG["circle"]),
    "circle_transition": ("Circle -> bigger circle transition", make_circle_transition, CONFIG["circle_transition"]),
    "infinity": ("Infinity (figure-8) path", make_infinity, CONFIG["infinity"]),
}


def save_csv(name, x, y, yaw):
    path = os.path.join(OUTPUT_DIR, f"{name}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_m", "y_m", "yaw_rad"])
        for xi, yi, yawi in zip(x, y, yaw):
            w.writerow([f"{xi:.4f}", f"{yi:.4f}", f"{yawi:.4f}"])
    return path


def save_marker_csv(name, mx, my, myaw):
    path = os.path.join(OUTPUT_DIR, f"{name}_markers.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["marker_id", "x_m", "y_m", "yaw_rad"])
        for i, (xi, yi, yawi) in enumerate(zip(mx, my, myaw)):
            w.writerow([i, f"{xi:.3f}", f"{yi:.3f}", f"{yawi:.3f}"])
    return path


def plot_path(name, title, x, y, meta, mx, my):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(x, y, "-", color="#1f5fd1", linewidth=2, label="Path")
    ax.plot(x[0], y[0], "o", color="green", markersize=10, label="Start")
    ax.plot(x[-1], y[-1], "s", color="red", markersize=9, label="End")

    ax.plot(mx, my, "o", color="black", markersize=5, zorder=6, label="Ground marker")
    for i, (mxi, myi) in enumerate(zip(mx, my)):
        ax.annotate(str(i), xy=(mxi, myi), xytext=(3, 3), textcoords="offset points",
                    fontsize=7, color="black")

    ax.annotate(
        "", xy=(x[1], y[1]), xytext=(x[0], y[0]),
        arrowprops=dict(arrowstyle="-|>", color="green", lw=2, mutation_scale=20),
    )

    if "maneuvering_region_x_m" in meta:
        x1, x2 = meta["maneuvering_region_x_m"]
        ax.axvspan(x1, x2, color="orange", alpha=0.15)
        ax.axvline(x1, color="orange", linestyle="--", linewidth=1)
        ax.axvline(x2, color="orange", linestyle="--", linewidth=1)
        ymid = meta["lane_offset_m"] / 2
        ax.annotate(
            f"maneuvering region\nx: {x1:.1f}-{x2:.1f} m ({x2 - x1:.1f} m long)\n"
            f"lateral shift: {meta['lane_offset_m']:.1f} m",
            xy=((x1 + x2) / 2, ymid), xytext=((x1 + x2) / 2, ymid + 1.2),
            ha="center", fontsize=9,
            bbox=dict(boxstyle="round", fc="lightyellow", ec="orange"),
            arrowprops=dict(arrowstyle="->", color="orange"),
        )

    if "transition_point_m" in meta:
        tx, ty = meta["transition_point_m"]
        ax.plot(tx, ty, "*", color="orange", markersize=16, zorder=5)
        ax.annotate(
            f"transition point\nr1={meta['r1_m']:.1f} m -> r2={meta['r2_m']:.1f} m\n"
            "(curvature step here)",
            xy=(tx, ty), xytext=(tx + 1.5, ty - 2.0),
            fontsize=9, bbox=dict(boxstyle="round", fc="lightyellow", ec="orange"),
            arrowprops=dict(arrowstyle="->", color="orange"),
        )

    if "crossing_point_m" in meta:
        cx, cy = meta["crossing_point_m"]
        ax.plot(cx, cy, "*", color="orange", markersize=16, zorder=5)
        ax.annotate("self-crossing point", xy=(cx, cy), xytext=(cx + 1.5, cy + 1.5),
                    fontsize=9, bbox=dict(boxstyle="round", fc="lightyellow", ec="orange"),
                    arrowprops=dict(arrowstyle="->", color="orange"))

    ax.set_title(title)
    ax.set_xlabel("x (m) - forward from rover start")
    ax.set_ylabel("y (m) - left of rover start heading")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best")
    fig.tight_layout()

    out_path = os.path.join(OUTPUT_DIR, f"{name}.jpg")
    fig.savefig(out_path, format="jpg", dpi=200)
    plt.close(fig)
    return out_path


def plot_overview(all_results):
    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    axes = axes.ravel()
    for ax, (name, (title, x, y, meta)) in zip(axes, all_results.items()):
        ax.plot(x, y, "-", color="#1f5fd1", linewidth=2)
        ax.plot(x[0], y[0], "o", color="green", markersize=7)
        ax.plot(x[-1], y[-1], "s", color="red", markersize=6)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, linestyle=":", alpha=0.6)
    for ax in axes[len(all_results):]:
        ax.axis("off")
    fig.suptitle("Real-world nav2 global-planner test paths (rover-start-relative frame)", fontsize=13)
    fig.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "overview.jpg")
    fig.savefig(out_path, format="jpg", dpi=200)
    plt.close(fig)
    return out_path


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    spacing = CONFIG["spacing"]

    results = {}
    print(f"Generating test paths (waypoint spacing = {spacing} m, "
          f"ground markers every {MARKER_SPACING} m)\n")
    for name, (title, gen_fn, cfg) in PATHS.items():
        x, y, meta = gen_fn(cfg, spacing)
        yaw = _yaw_from_xy(x, y)
        mx, my, myaw = resample_markers(x, y, yaw, MARKER_SPACING)

        csv_path = save_csv(name, x, y, yaw)
        markers_path = save_marker_csv(name, mx, my, myaw)
        jpg_path = plot_path(name, title, x, y, meta, mx, my)
        results[name] = (title, x, y, meta)

        xmin, xmax = x.min(), x.max()
        ymin, ymax = y.min(), y.max()
        print(f"[{name}] {title}")
        print(f"  waypoints: {len(x)}  length: ~{np.sum(np.hypot(np.diff(x), np.diff(y))):.2f} m")
        print(f"  bounding box: x [{xmin:.2f}, {xmax:.2f}] m (span {xmax - xmin:.2f} m)  "
              f"y [{ymin:.2f}, {ymax:.2f}] m (span {ymax - ymin:.2f} m)")
        for k, v in meta.items():
            print(f"  {k}: {v}")
        print(f"  ground markers ({len(mx)}):")
        for i, (mxi, myi) in enumerate(zip(mx, my)):
            print(f"    #{i}: x={mxi:.2f} m, y={myi:.2f} m")
        print(f"  -> {csv_path}")
        print(f"  -> {markers_path}")
        print(f"  -> {jpg_path}\n")

    overview_path = plot_overview(results)
    print(f"[overview] -> {overview_path}")


if __name__ == "__main__":
    main()
