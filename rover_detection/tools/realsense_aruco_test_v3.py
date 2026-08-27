
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'rover_detection'))
import aruco_math  # noqa: E402

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not importable. Install with `pip install --break-system-packages "
          "pyrealsense2`, or check that librealsense2's Python bindings are on your PYTHONPATH if "
          "you built it from source (same librealsense2 the ROS realsense2_camera node uses).")
    sys.exit(1)

# ============================================================================
# EDIT THESE to match your physical setup before trusting the output.
# ============================================================================

# Camera mounting offset from the rover chassis BASE CENTER, in the rover
# body frame (x=forward, y=left, z=up), meters.
CAMERA_OFFSET_X_FWD = -0.25 ## 0.2608
CAMERA_OFFSET_Y_LEFT = 0
CAMERA_OFFSET_Z_UP = 0 ##0.2535

# Which named start line the rover is actually sitting on today (looked up
# in START_POINTS_XY below). Set to None and use --start-x/--start-y instead
# for an arbitrary position not in the table.
START_POINT = 'S1'

# ============================================================================
# ERC 2026 Mars Yard survey data, global frame (X=right, Y=front), meters.
# Transcribed directly from the organizer-supplied coordinate table -- if
# Update Report #3 ever revises these, update here to match.
# ============================================================================

# Starting locations (S1-S9): (X, Y)
START_POINTS_XY = {
    'S1': (0.000, 0.000),
    'S2': (0.000, 26.427),
    'S3': (10.709, 6.918),
    'S4': (11.210, 17.881),
    'S5': (13.223, 25.658),
    'S6': (-9.428, 15.824),
    'S7': (-8.970, 22.831),
    'S8': (-15.676, 4.991),
    'S9': (7.160, -4.762),
}

# Landmarks (L1-L15) keyed by their ENCODED ArUco ID (= 50 + printed sign
# number, per tools/generate_aruco_markers.py / the ERC landmark spec), not
# by the printed L-number: (X, Y)
LANDMARKS_XY = {
    51: (3.183, 8.012),     # L1
    52: (7.269, 9.482),     # L2
    53: (7.878, 17.583),    # L3
    54: (9.225, 22.389),    # L4
    55: (3.518, 23.990),    # L5
    56: (0.882, 16.870),    # L6
    57: (-3.944, 21.415),   # L7
    58: (-5.491, 16.334),   # L8
    59: (-7.695, 13.528),   # L9
    60: (-1.610, 12.602),   # L10
    61: (-7.715, 9.721),    # L11
    62: (-4.311, 4.442),    # L12
    63: (-5.720, 28.118),   # L13
    64: (-11.438, 5.230),   # L14
    65: (6.483, 1.102),     # L15
}
LANDMARK_NAMES = {51 + i: f'L{i + 1}' for i in range(15)}

WARMUP_FRAMES = 30


def camera_to_rover_frame(point_cam: np.ndarray) -> np.ndarray:
    """Camera optical frame (x right, y down, z forward) -> rover body frame
    (x forward, y left, z up), for a camera mounted level and facing
    straight forward. Same as V2 -- see that file for the derivation."""
    cam_x, cam_y, cam_z = point_cam
    return np.array([cam_z, -cam_x, -cam_y], dtype=np.float64)


def compute_rover_global_position(landmark_xy, v_fwd, v_left):
    """landmark_xy: (X, Y) known surveyed position of the marker, global
    frame. v_fwd, v_left: vector from the ROVER CENTER to the marker
    (camera_offset + camera-to-marker), rover-frame axes. Returns the
    rover chassis center's (X, Y) in the global frame -- see module
    docstring for the derivation and its heading-alignment assumption."""
    x_m, y_m = landmark_xy
    return x_m + v_left, y_m - v_fwd


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--marker-length', type=float, default=0.150,
                    help='marker square size in meters (default 0.150 -- measure your actual printout)')
    p.add_argument('--dictionary', default='DICT_5X5_100')
    p.add_argument('--min-valid-depth-samples', type=int, default=3)
    p.add_argument('--max-depth-std-m', type=float, default=0.05)

    p.add_argument('--start-point', default=START_POINT, choices=list(START_POINTS_XY.keys()),
                    help=f'named start line to assume when no landmark is in view (default {START_POINT})')
    p.add_argument('--start-x', type=float, default=None,
                    help='explicit starting global X (meters) -- overrides --start-point if given')
    p.add_argument('--start-y', type=float, default=None,
                    help='explicit starting global Y (meters) -- overrides --start-point if given')

    p.add_argument('--camera-offset-x', type=float, default=CAMERA_OFFSET_X_FWD,
                    help=f'camera offset forward of rover center, meters (default {CAMERA_OFFSET_X_FWD})')
    p.add_argument('--camera-offset-y', type=float, default=CAMERA_OFFSET_Y_LEFT,
                    help=f'camera offset left of rover center, meters (default {CAMERA_OFFSET_Y_LEFT})')
    p.add_argument('--camera-offset-z', type=float, default=CAMERA_OFFSET_Z_UP,
                    help=f'camera offset above rover center, meters (default {CAMERA_OFFSET_Z_UP})')
    return p.parse_args()


def main():
    args = parse_args()

    if args.start_x is not None and args.start_y is not None:
        start_xy = (args.start_x, args.start_y)
        start_label = f'({args.start_x:.3f}, {args.start_y:.3f}) [explicit --start-x/--start-y]'
    else:
        start_xy = START_POINTS_XY[args.start_point]
        start_label = f'{args.start_point} = ({start_xy[0]:.3f}, {start_xy[1]:.3f})'
    camera_offset = (args.camera_offset_x, args.camera_offset_y, args.camera_offset_z)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    try:
        profile = pipeline.start(config)
    except RuntimeError as ex:
        print(f'ERROR: could not start RealSense pipeline at {args.width}x{args.height}@{args.fps}: {ex}')
        sys.exit(1)

    align = rs.align(rs.stream.color)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    camera_matrix = np.array([
        [intr.fx, 0.0, intr.ppx],
        [0.0, intr.fy, intr.ppy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    print(f'RealSense pipeline started: {args.width}x{args.height}@{args.fps}, depth_scale={depth_scale}')
    print(f'start position (used when no landmark in view): {start_label}')
    print(f'camera offset from rover center (fwd, left, up): {camera_offset}')
    print('WARNING: no heading/yaw tracking -- assumes rover forward is aligned with global +Y '
          'at the moment of each sighting. See module docstring.\n')

    dictionary = aruco_math.get_dictionary(args.dictionary)
    detector = aruco_math.make_detector(dictionary)

    print(f'warming up ({WARMUP_FRAMES} frames)...')
    for _ in range(WARMUP_FRAMES):
        pipeline.wait_for_frames()
    print("warmup done -- press 'q' or Esc in the window to quit\n")

    window_name = 'RealSense ArUco test v3 (global rover position)'
    frame_count = 0
    fps_ema = None
    last_t = time.perf_counter()
    last_log_t = last_t

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                print('WARNING: frame grab failed (missing color or depth), retrying...')
                continue
            frame_count += 1

            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps_ema = inst_fps if fps_ema is None else (0.9 * fps_ema + 0.1 * inst_fps)

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

            corners, ids = aruco_math.detect_markers(detector, gray)

            display = color.copy()
            fixes = []  # (marker_id, name_or_None, x_rover, y_rover, dist_m)

            if ids is not None:
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    marker_id = int(marker_id)
                    c2d = marker_corners.reshape(4, 2).astype(np.float64)
                    cv2.aruco.drawDetectedMarkers(display, [marker_corners], None)

                    translation_cam, n_valid = aruco_math.estimate_translation_from_depth(
                        c2d, depth, camera_matrix, depth_scale=depth_scale,
                        min_valid_samples=args.min_valid_depth_samples,
                        max_std_m=args.max_depth_std_m)
                    center = c2d.mean(axis=0)

                    if translation_cam is None:
                        cv2.putText(display, f'id={marker_id} depth rejected ({n_valid}/5)',
                                    (int(center[0]) - 90, int(center[1]) - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
                        continue

                    cam_fwd, cam_left, cam_up = camera_to_rover_frame(translation_cam)
                    dist_m = float(np.linalg.norm(translation_cam))

                    if marker_id in LANDMARKS_XY:
                        v_fwd = camera_offset[0] + cam_fwd
                        v_left = camera_offset[1] + cam_left
                        x_rover, y_rover = compute_rover_global_position(LANDMARKS_XY[marker_id], v_fwd, v_left)
                        name = LANDMARK_NAMES[marker_id]
                        fixes.append((marker_id, name, x_rover, y_rover, dist_m))
                        label = f'{name}(id{marker_id}) d={dist_m:.2f}m -> rover=({x_rover:.2f},{y_rover:.2f})'
                        color_ok = (0, 255, 0)
                    else:
                        label = f'id={marker_id} d={dist_m:.2f}m (not a known landmark ID)'
                        color_ok = (0, 165, 255)

                    cv2.putText(display, label, (int(center[0]) - 100, int(center[1]) - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_ok, 2)

            # Headline reading: closest landmark fix this frame (closest =
            # generally most accurate depth), falling back to the
            # configured start position if no landmark is currently visible.
            if fixes:
                marker_id, name, x_rover, y_rover, dist_m = min(fixes, key=lambda f: f[4])
                headline = f'ROVER POSITION (from {name}, id={marker_id}): X={x_rover:.3f} Y={y_rover:.3f}'
                headline_color = (0, 255, 0)
            else:
                x_rover, y_rover = start_xy
                headline = f'rover position (assumed start, no landmark in view): X={x_rover:.3f} Y={y_rover:.3f}'
                headline_color = (0, 0, 255)

            cv2.putText(display, headline, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, headline_color, 2)
            fps_text = f'{fps_ema:.1f} FPS' if fps_ema is not None else '-- FPS'
            cv2.putText(display, fps_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if len(fixes) > 1:
                agreement = ', '.join(f'{n}=({x:.2f},{y:.2f})' for _, n, x, y, _ in fixes)
                cv2.putText(display, f'cross-check: {agreement}', (10, 85),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            cv2.imshow(window_name, display)

            if now - last_log_t > 2.0:
                print(f'frame {frame_count}: {fps_text}, {headline}')
                if len(fixes) > 1:
                    print(f'  cross-check (all landmarks in view): {agreement}')
                last_log_t = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
