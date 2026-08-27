#!/usr/bin/env python3
"""
Generate a 2D occupancy grid (ROS map_server / Nav2 PGM+YAML format) from the
2026 Mars Yard heightmap, for use with A* or other grid-based planners.

The heightmap itself (rover_description/models/dem/marsyard2026_terrain_hm.tif)
is fixed -- it's the ground-control-corrected elevation scan, 10cm/cell, in the
same S1-origin world frame as everything else in the sim. This script only
tunes how that elevation data gets turned into occupied/free cells:

  - slope:      local terrain tilt, fit from raw mesh vertices (deg)
  - roughness:  local elevation std-dev in a small window (m) -- catches rock
                clusters/steps that don't show up as a steep edge-to-edge slope
  - inflation:  grows occupied cells outward so planned paths keep clearance

A cell is occupied if slope > SLOPE_THRESH_DEG OR roughness > ROUGHNESS_THRESH_M.

HOW TO USE: edit the PARAMETERS block right below, then run with no arguments:
  python3 generate_occupancy_grid.py

Every run prints each survey point's traversability (clear/BLOCKED) to the
console, and always writes a preview PNG plotting every waypoint on the map
color-coded by that same clear/BLOCKED status -- so you can see the effect of
each parameter change immediately, both as a map and as a plain-text summary.

All parameters below can still be overridden from the command line too, e.g.
`--slope-thresh 30`, for scripting/one-off tests -- run --help for the full
list. But for normal tuning, just edit the values below and re-run.
"""

import argparse
import os

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import (
    binary_dilation,
    gaussian_filter,
    generate_binary_structure,
    iterate_structure,
    uniform_filter,
)

# ============================================================================
# PARAMETERS -- edit these, then just run the script with no arguments.
# ============================================================================

SLOPE_THRESH_DEG = 25.0
# Max climbable slope, in degrees (not percent grade). A cell is occupied if
# the terrain tilt here exceeds this angle.

ROUGHNESS_THRESH_M = 0.06
# Max local elevation "jitter" tolerated, in meters (std-dev of height within
# ROUGHNESS_WINDOW_CELLS). Catches rock clusters/steps that aren't captured by
# slope alone. 0.05 = 50mm, matching the rover's stated obstacle-height spec.

ROUGHNESS_WINDOW_CELLS = 3
# Size (in 10cm cells) of the neighborhood used to compute roughness. Smaller
# = more sensitive to individual small rocks. 3 cells = ~30cm.

INFLATE_RADIUS_M = 0.20
# How far to pad every occupied cell outward (~half the rover's track width),
# so planned paths keep real clearance instead of grazing an obstacle edge.
# Set to 0 to disable inflation entirely.

BOUNDARY = "hull"
# Restrict the drivable area to the competition arena. The terrain model covers
# a larger rectangle than the yard itself, and the ground outside is flat, so
# slope/roughness alone mark it traversable -- 20% of all free space sat outside
# the survey area, touching the map border, and the planner routed through it.
# "hull" marks everything outside the convex hull of the survey points (grown by
# BOUNDARY_MARGIN_M) as occupied. "none" restores the old behaviour.

BOUNDARY_MARGIN_M = 3.0
# How far past the outermost survey markers still counts as in-bounds. The
# markers sit inside the arena, not on its fence, so some margin is needed or
# points near the edge lose the room to manoeuvre around them.

SLOPE_METHOD = "mesh"
# "mesh" (recommended, accurate): slope fit directly from raw mesh vertices,
#   no rasterize/smoothing information loss. Verified against the "grid"
#   method below -- "grid" was found to both under- and over-estimate slope
#   at various points on this terrain (up to several degrees off).
# "grid": fast finite-difference on the heightmap raster instead of the mesh.
#   Kept only as a quick fallback if the mesh file isn't available.

SLOPE_MESH_RADIUS_M = 0.15
# (SLOPE_METHOD="mesh" only) Radius of mesh vertices considered per grid cell
# when fitting the local slope plane. Larger = smoother/less noise-sensitive
# but blurs out small real features; smaller = more locally precise but
# noisier if it gets too close to individual-point scan noise.

SLOPE_SMOOTH_M = 0.3
# (SLOPE_METHOD="grid" only) Gaussian smoothing applied before the gradient
# calc, to remove scan noise. Not used by the default "mesh" method.

# ============================================================================
# End of parameters. Nothing below this line needs editing for normal use.
# ============================================================================

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_HEIGHTMAP = os.path.join(
    REPO_ROOT, "rover_description", "models", "dem", "marsyard2026_terrain_hm.tif"
)
DEFAULT_OUT_DIR = os.path.expanduser("~/jazzy_ws/src/marsyard")
DEFAULT_COORDS = os.path.join(
    DEFAULT_OUT_DIR, "2026_MarsYard_3D_Model-20260812T165935Z-1-001",
    "2026_MarsYard_3D_Model", "Coordinates_MarsYard2026.txt",
)
DEFAULT_MESH = os.path.join(
    DEFAULT_OUT_DIR, "2026_MarsYard_3D_Model-20260812T165935Z-1-001",
    "2026_MarsYard_3D_Model", "Model3D_mesh1.ply",
)
SLOPE_CACHE = os.path.join(DEFAULT_OUT_DIR, "mesh_slope_cache.npy")

# Ground-control-point Z correction (mesh has a slight tilt vs. surveyed
# heights -- fit once against all 34 survey points, see project history).
# delta_z = GCP_A*X + GCP_B*Y + GCP_C, subtracted from raw mesh Z.
GCP_A, GCP_B, GCP_C = -0.01022, -0.02276, 0.05351

GROUP_COLOR = {
    "S": "#e6194B",   # red    - Starting Locations
    "L": "#3b82f6",   # blue   - Landmarks (ArUco)
    "W": "#f58231",   # orange - Nav-Traverse Waypoints
    "P": "#a020f0",   # purple - Deep Sampling Location
}
GROUP_NAME = {
    "S": "Starting Locations", "L": "Landmarks", "W": "Nav Waypoints", "P": "Deep Sampling",
}

# World bounds of the heightmap raster (S1-origin frame, same as
# Coordinates_MarsYard2026.txt / the rest of the sim). Fixed by how the
# heightmap itself was generated -- not a tuning parameter.
XMIN, XMAX = -18.508737564086914, 18.889720916748047
YMIN, YMAX = -7.88718318939209, 35.92618179321289
RESOLUTION = 0.1  # m/cell, baked into the heightmap raster


def parse_args():
    p = argparse.ArgumentParser(
        description="Tune slope/roughness/inflation to generate the occupancy grid.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--heightmap", default=DEFAULT_HEIGHTMAP, help="Source elevation TIFF")
    p.add_argument("--slope-thresh", type=float, default=SLOPE_THRESH_DEG, help="Max climbable slope (deg)")
    p.add_argument(
        "--slope-thresh-pct", type=float, default=None,
        help="Max climbable slope as percent grade (rise/run x100), e.g. 20 for 20%%. "
             "Overrides --slope-thresh if given -- this is how rover climbing specs are usually quoted.",
    )
    p.add_argument(
        "--slope-method", choices=["mesh", "grid"], default=SLOPE_METHOD,
        help="'mesh' (default, accurate): slope from raw mesh triangle normals, no smoothing "
             "information-loss. 'grid': fast finite-difference on the heightmap raster (needs "
             "--slope-smooth to avoid scan-noise false positives, but can underestimate real "
             "slope elsewhere -- verified against 'mesh' method, kept only as a fast fallback).",
    )
    p.add_argument("--mesh", default=DEFAULT_MESH, help="Source mesh PLY (used by --slope-method mesh)")
    p.add_argument(
        "--slope-mesh-radius", type=float, default=SLOPE_MESH_RADIUS_M,
        help="Radius (m) of mesh triangles considered per grid cell for --slope-method mesh",
    )
    p.add_argument("--no-slope-cache", action="store_true", help="Recompute mesh slope instead of using cache")
    p.add_argument(
        "--slope-smooth", type=float, default=SLOPE_SMOOTH_M,
        help="Gaussian smoothing (m) applied before slope calc, to remove scan noise "
             "so slope reflects real terrain grade rather than photogrammetry jitter. "
             "Only used by --slope-method grid. 0 disables.",
    )
    p.add_argument("--roughness-thresh", type=float, default=ROUGHNESS_THRESH_M, help="Max local elevation std-dev (m)")
    p.add_argument("--roughness-window", type=int, default=ROUGHNESS_WINDOW_CELLS, help="Roughness window size (cells)")
    p.add_argument("--inflate-radius", type=float, default=INFLATE_RADIUS_M, help="Obstacle inflation radius (m); 0 disables")
    p.add_argument("--boundary", choices=["none", "hull"], default=BOUNDARY,
                   help="Mark everything outside the operating area as occupied. "
                        "'hull' = convex hull of the survey points, grown by "
                        "--boundary-margin. The terrain model extends well past the "
                        "arena and that outside ground is flat, so without this the "
                        "planner will happily route around the outside of the yard")
    p.add_argument("--boundary-margin", type=float, default=BOUNDARY_MARGIN_M,
                   help="How far outside the survey-point hull still counts as in-bounds (m)")
    p.add_argument("--coords", default=DEFAULT_COORDS, help="Coordinates_MarsYard2026.txt path")
    p.add_argument("--no-waypoints", action="store_true", help="Skip overlaying survey points on the preview")
    p.add_argument("--out-pgm", default=os.path.join(DEFAULT_OUT_DIR, "marsyard2026_occupancy.pgm"))
    p.add_argument("--out-yaml", default=os.path.join(DEFAULT_OUT_DIR, "marsyard2026_occupancy.yaml"))
    p.add_argument("--out-viz", default=os.path.join(DEFAULT_OUT_DIR, "marsyard2026_occupancy_preview.png"))
    p.add_argument("--no-save-map", action="store_true", help="Only write the preview PNG, skip PGM/YAML")
    return p.parse_args()


def load_heightmap(path):
    h = np.array(Image.open(path)).astype(np.float64)
    return h


def load_survey_points(path):
    """Parse Coordinates_MarsYard2026.txt. File columns are (Name, Y, X, H)."""
    points = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split("\t") if p.strip() != ""]
            if len(parts) < 4:
                continue
            name = parts[0]
            if name.lower().startswith("point"):
                continue
            try:
                y, x, h = (float(v.replace(",", ".")) for v in parts[1:4])
            except ValueError:
                continue
            points.append((name, x, y, h))
    return points


def world_to_cell(x, y, W, H):
    col = int(round((x - XMIN) / (XMAX - XMIN) * (W - 1)))
    row = int(round((YMAX - y) / (YMAX - YMIN) * (H - 1)))
    return row, col


def check_overlaps(points, occupied):
    H, W = occupied.shape
    conflicts = []
    for name, x, y, h in points:
        row, col = world_to_cell(x, y, W, H)
        in_bounds = 0 <= row < H and 0 <= col < W
        blocked = in_bounds and occupied[row, col]
        if blocked:
            conflicts.append(name)
        status = "OUT OF GRID" if not in_bounds else ("BLOCKED" if blocked else "clear")
        print(f"  {name:<5} world=({x:6.2f},{y:6.2f})  {status}")
    return conflicts


def compute_slope_deg_from_grid(h, resolution, smooth_m=0.3):
    """Fast but lossy: finite-difference gradient on the rasterized heightmap.
    Needs smoothing to avoid scan-noise false positives, but that same
    smoothing can blur out and underestimate real slope elsewhere (verified:
    some cells read off by several degrees vs. the mesh-based method below).
    Kept as a fast fallback via --slope-method grid; not the default."""
    if smooth_m > 0:
        sigma_cells = smooth_m / resolution
        h = gaussian_filter(h, sigma=sigma_cells)
    gy, gx = np.gradient(h, resolution)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def compute_slope_deg_from_mesh(shape, mesh_path, radius=0.15, cache_path=None, use_cache=True,
                                 min_points=6):
    """Accurate: for each grid cell, fit a least-squares plane through all raw
    mesh VERTICES within `radius`, then take that plane's tilt as the slope.

    Deliberately NOT "median of nearby triangle normals" -- verified that
    approach is vulnerable to dense clusters of millimeter-scale reconstruction
    noise (near-vertical micro-facets a few mm-cm across, smaller than the
    rover's own roughness tolerance, that are not real obstacles but can
    dominate a per-triangle median when many cluster in one spot). A
    least-squares plane fit over many points naturally averages out that kind
    of noise instead of being skewed by whichever facet happens to be
    steepest -- this is the properly-robust version of what heightmap
    smoothing was crudely approximating.

    Expensive (KD-tree query over ~840k vertices x ~164k cells), so cached to
    disk -- only depends on the mesh/grid/radius, not on tunable thresholds,
    so the cache is reused across parameter-tuning runs.
    """
    H, W = shape
    if use_cache and cache_path and os.path.exists(cache_path):
        cached = np.load(cache_path, allow_pickle=True).item()
        if (cached.get("shape") == shape and cached.get("radius") == radius
                and cached.get("mesh") == mesh_path and cached.get("method") == "plane_fit"):
            print(f"  (using cached mesh slope: {cache_path})")
            return cached["slope_deg"]

    import open3d as o3d
    from scipy.spatial import cKDTree

    print("  loading mesh vertices...")
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    verts = np.asarray(mesh.vertices).copy()
    verts[:, 2] -= GCP_A * verts[:, 0] + GCP_B * verts[:, 1] + GCP_C

    tree = cKDTree(verts[:, :2])
    xs = np.linspace(XMIN, XMAX, W)
    ys = np.linspace(YMAX, YMIN, H)
    gx, gy = np.meshgrid(xs, ys)
    query_pts = np.stack([gx.ravel(), gy.ravel()], axis=-1)

    print(f"  fitting local planes for {len(query_pts)} grid cells against {len(verts)} vertices...")
    neighbor_lists = tree.query_ball_point(query_pts, r=radius)
    slope_out = np.full(len(query_pts), np.nan)
    for i, idxs in enumerate(neighbor_lists):
        if len(idxs) >= min_points:
            pts = verts[idxs]
            A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
            coef, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
            a, b, _ = coef
            slope_out[i] = np.degrees(np.arctan(np.hypot(a, b)))
    slope_out = slope_out.reshape(H, W)

    nan_mask = np.isnan(slope_out)
    if nan_mask.any():
        from scipy.ndimage import distance_transform_edt
        idx = distance_transform_edt(nan_mask, return_distances=False, return_indices=True)
        slope_out = slope_out[tuple(idx)]

    if cache_path:
        np.save(cache_path, {
            "shape": shape, "radius": radius, "mesh": mesh_path,
            "method": "plane_fit", "slope_deg": slope_out,
        })
        print(f"  cached: {cache_path}")

    return slope_out


def compute_roughness(h, window):
    mean = uniform_filter(h, size=window)
    mean_sq = uniform_filter(h * h, size=window)
    return np.sqrt(np.clip(mean_sq - mean * mean, 0, None))


def out_of_bounds_mask(shape, points, margin_m, resolution):
    """Cells outside the convex hull of `points`, grown by `margin_m`.

    Returns None if the hull cannot be built (needs 3+ non-collinear points)."""
    import numpy as np
    from matplotlib.path import Path as MplPath
    try:
        from scipy.spatial import ConvexHull
    except ImportError:
        return None

    H, W = shape
    xy = np.array([(x, y) for _, x, y, _ in points])
    if len(xy) < 3:
        return None
    try:
        hull = ConvexHull(xy)
    except Exception:
        return None

    poly = MplPath(xy[hull.vertices])
    cols, rows = np.meshgrid(np.arange(W), np.arange(H))
    wx = XMIN + cols / (W - 1) * (XMAX - XMIN)
    wy = YMAX - rows / (H - 1) * (YMAX - YMIN)
    inside = poly.contains_points(np.column_stack([wx.ravel(), wy.ravel()])).reshape(H, W)

    if margin_m > 0:
        cells = max(1, int(round(margin_m / resolution)))
        struct = iterate_structure(generate_binary_structure(2, 1), cells)
        inside = binary_dilation(inside, structure=struct)

    # Seal the outermost ring. Where the hull plus its margin reaches the edge
    # of the terrain model, the grown region otherwise leaves a few free cells
    # sitting on the map border -- measured at exactly the rover's collision
    # radius, i.e. just passable. A costmap the rover can drive off the edge of
    # is not a bounded arena.
    inside[0, :] = inside[-1, :] = False
    inside[:, 0] = inside[:, -1] = False
    return ~inside


def classify(h, slope_deg, roughness, slope_thresh, roughness_thresh, inflate_radius, resolution):
    occupied = (slope_deg > slope_thresh) | (roughness > roughness_thresh)
    raw_pct = 100 * occupied.mean()

    if inflate_radius > 0:
        inflate_cells = max(1, int(round(inflate_radius / resolution)))
        struct = iterate_structure(generate_binary_structure(2, 1), inflate_cells)
        occupied = binary_dilation(occupied, structure=struct)
    inflated_pct = 100 * occupied.mean()

    return occupied, raw_pct, inflated_pct


def save_map(occupied, out_pgm, out_yaml, resolution):
    # row0 of the heightmap = Ymax (top/north), which matches ROS map_server's
    # convention directly: image row0=top, origin=(xmin,ymin) is the
    # bottom-left corner (last image row).
    pgm_data = np.where(occupied, 0, 254).astype(np.uint8)
    Image.fromarray(pgm_data, mode="L").save(out_pgm)

    yaml_content = {
        "image": os.path.basename(out_pgm),
        "resolution": resolution,
        "origin": [float(XMIN), float(YMIN), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(out_yaml, "w") as f:
        yaml.dump(yaml_content, f, default_flow_style=False)


def save_preview(h, slope_deg, occupied, out_viz, slope_thresh, roughness_thresh, inflate_radius,
                  points=None, conflicts=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    conflicts = conflicts or set()
    extent = [XMIN, XMAX, YMIN, YMAX]
    fig, axes = plt.subplots(1, 3, figsize=(22, 8))

    axes[0].imshow(h, cmap="terrain", extent=extent, origin="upper")
    axes[0].set_title("Elevation (m)")

    axes[1].imshow(slope_deg, cmap="hot", extent=extent, origin="upper", vmax=45)
    axes[1].set_title("Slope (deg)")

    axes[2].imshow(h, cmap="gray", extent=extent, origin="upper", alpha=0.6)
    axes[2].imshow(
        np.where(occupied, 1, np.nan), cmap="Reds", extent=extent,
        origin="upper", vmin=0, vmax=1, alpha=0.85,
    )
    axes[2].set_title(
        f"Occupied (red): slope>{slope_thresh:.1f}deg or rough>{roughness_thresh}m, inflate {inflate_radius}m"
    )

    if points:
        # Traversability marker on every panel: filled circle = clear,
        # black X = BLOCKED -- same convention everywhere so it reads at a
        # glance whether a point is drivable under the current parameters.
        seen_groups = set()
        for name, x, y, hgt in points:
            prefix = name[0]
            color = GROUP_COLOR.get(prefix, "#333333")
            is_conflict = name in conflicts
            label = f"{prefix} = {GROUP_NAME.get(prefix, prefix)}" if prefix not in seen_groups else None
            seen_groups.add(prefix)
            for ax in (axes[0], axes[2]):
                ax.scatter(
                    [x], [y],
                    c=("black" if is_conflict else color),
                    s=100 if is_conflict else 50,
                    marker="x" if is_conflict else "o",
                    linewidths=2.5 if is_conflict else 0.8,
                    edgecolors="black", zorder=5,
                    label=label if ax is axes[2] else None,
                )
                ax.annotate(name, (x, y), fontsize=6, xytext=(2, 2), textcoords="offset points", zorder=6)
        axes[2].legend(loc="upper left", fontsize=7, framealpha=0.9,
                        title="● clear   ✕ BLOCKED", title_fontsize=7)

        # Per-group traversability summary box, printed right on the figure.
        by_group = {}
        for name, x, y, hgt in points:
            by_group.setdefault(name[0], []).append(name)
        lines = ["TRAVERSABILITY SUMMARY"]
        for g in sorted(by_group):
            names = by_group[g]
            blocked = [n for n in names if n in conflicts]
            clear_n = len(names) - len(blocked)
            lines.append(f"{GROUP_NAME.get(g, g):<18} {clear_n}/{len(names)} clear"
                         + (f"   BLOCKED: {', '.join(sorted(blocked))}" if blocked else ""))
        total_blocked = len(conflicts)
        lines.append(f"{'TOTAL':<18} {len(points)-total_blocked}/{len(points)} clear")
        summary_text = "\n".join(lines)
        fig.text(
            0.5, -0.02, summary_text, ha="center", va="top", fontsize=9.5,
            family="monospace",
            bbox=dict(boxstyle="round", facecolor="#f5f5f5", edgecolor="#999999", pad=0.6),
        )

    for ax in axes:
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
    plt.tight_layout()
    plt.savefig(out_viz, dpi=130, bbox_inches="tight")
    plt.close(fig)


def main():
    args = parse_args()

    slope_thresh = args.slope_thresh
    if args.slope_thresh_pct is not None:
        slope_thresh = np.degrees(np.arctan(args.slope_thresh_pct / 100.0))
        print(f"slope threshold: {args.slope_thresh_pct}% grade -> {slope_thresh:.2f} deg")

    h = load_heightmap(args.heightmap)
    H, W = h.shape
    print(f"heightmap: {W}x{H} cells @ {RESOLUTION}m  elevation {h.min():.3f}..{h.max():.3f}m")

    if args.slope_method == "mesh":
        print("computing slope from mesh triangle normals (accurate)...")
        slope_deg = compute_slope_deg_from_mesh(
            (H, W), args.mesh, radius=args.slope_mesh_radius,
            cache_path=SLOPE_CACHE, use_cache=not args.no_slope_cache,
        )
    else:
        print("computing slope from heightmap gradient (fast, less accurate)...")
        slope_deg = compute_slope_deg_from_grid(h, RESOLUTION, smooth_m=args.slope_smooth)

    roughness = compute_roughness(h, args.roughness_window)

    occupied, raw_pct, inflated_pct = classify(
        h, slope_deg, roughness,
        slope_thresh, args.roughness_thresh, args.inflate_radius, RESOLUTION,
    )
    print(f"occupied: {raw_pct:.1f}% raw -> {inflated_pct:.1f}% after {args.inflate_radius}m inflation")

    points, conflicts = None, set()
    if not args.no_waypoints:
        points = load_survey_points(args.coords)

    if args.boundary == "hull":
        # Applied AFTER inflation on purpose: inflation grows obstacles
        # outward, and dilating the arena edge the same way would eat margin
        # off the inside of the boundary, shrinking the drivable area rather
        # than just closing it off.
        pts = points if points is not None else load_survey_points(args.coords)
        oob = out_of_bounds_mask(occupied.shape, pts, args.boundary_margin, RESOLUTION)
        if oob is None:
            print("boundary: could not build survey-point hull -- left unbounded")
        else:
            before = occupied.mean()
            occupied = occupied | oob
            print(f"boundary: hull of {len(pts)} survey points + {args.boundary_margin}m margin "
                  f"-> occupied {100*before:.1f}% -> {100*occupied.mean():.1f}% "
                  f"({100*oob.mean():.1f}% of the map is outside the arena)")
        print(f"\nchecking {len(points)} survey points against occupancy grid:")
        conflicts = set(check_overlaps(points, occupied))
        if conflicts:
            print(f"\n  *** {len(conflicts)} point(s) BLOCKED by an obstacle cell: {sorted(conflicts)} ***")
        else:
            print("\n  all points clear of obstacles")

    save_preview(
        h, slope_deg, occupied, args.out_viz, slope_thresh, args.roughness_thresh, args.inflate_radius,
        points=points, conflicts=conflicts,
    )
    print("\npreview:", args.out_viz)

    if not args.no_save_map:
        save_map(occupied, args.out_pgm, args.out_yaml, RESOLUTION)
        print("map:", args.out_pgm, "+", args.out_yaml)
    else:
        print("(--no-save-map: PGM/YAML not written)")


if __name__ == "__main__":
    main()
