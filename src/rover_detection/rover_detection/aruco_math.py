"""Pure-Python ArUco detection/pose math -- no ROS dependencies, so it can be
unit-tested standalone (see test/test_aruco_math.py) before ever touching
real hardware.

Pose strategy: PnP gives a reliable marker *orientation* but its
*translation* is sensitive to the flatness/pattern-quality assumption and
gets noisy at range. RealSense depth gives a much more reliable range
estimate directly. So: use PnP's rvec for orientation, and use the median of
valid RealSense depth samples at the marker's corners + center for
translation -- see estimate_orientation() and estimate_translation_from_depth().
"""

import cv2
import cv2.aruco as aruco
import numpy as np

# True on OpenCV < 4.7, where cv2.aruco.ArucoDetector doesn't exist yet and
# detection goes through the free function cv2.aruco.detectMarkers() instead
# of a detector object. Support both since we don't control what's installed
# on the robot.
_LEGACY_API = not hasattr(aruco, 'ArucoDetector')


def get_dictionary(name: str):
    """name e.g. 'DICT_5X5_100' (see cv2.aruco.DICT_* constants). Works on
    both pre- and post-4.7 OpenCV."""
    dict_id = getattr(aruco, name)
    if hasattr(aruco, 'getPredefinedDictionary'):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)


def make_detector(dictionary, corner_refinement: bool = True):
    """Returns either an ArucoDetector instance (new API) or a
    (dictionary, params) tuple (legacy API) -- pass straight into
    detect_markers().

    Prefers DetectorParameters_create() over the bare DetectorParameters()
    constructor whenever it exists: at least on OpenCV 4.6.0 (python3-opencv
    from Ubuntu 24.04 apt), the two are NOT equivalent -- the bare
    constructor's object segfaults the interpreter on the very next
    attribute set (e.g. cornerRefinementMethod), while the factory function
    produces a working object. This was caught by test_aruco_math.py, not
    by inspection -- don't "simplify" this back to DetectorParameters()
    without re-running that test on the actual target OpenCV build.
    """
    if hasattr(aruco, 'DetectorParameters_create'):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    if corner_refinement:
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    if _LEGACY_API:
        return dictionary, params
    return aruco.ArucoDetector(dictionary, params)


def detect_markers(detector_or_dict_params, gray_image: np.ndarray):
    """Returns (corners, ids). corners: list of (1,4,2) or (4,2) float32
    arrays in OpenCV's standard order (top-left, top-right, bottom-right,
    bottom-left); ids: (N,1) int array, or None if nothing detected."""
    if _LEGACY_API:
        dictionary, params = detector_or_dict_params
        corners, ids, _rejected = aruco.detectMarkers(gray_image, dictionary, parameters=params)
    else:
        corners, ids, _rejected = detector_or_dict_params.detectMarkers(gray_image)
    return corners, ids


def marker_object_points(marker_length_m: float) -> np.ndarray:
    """4x3 object points for solvePnP, in the marker's own frame (marker
    plane = XY, center = origin), ordered to match cv2.aruco's corner order
    (top-left, top-right, bottom-right, bottom-left)."""
    h = marker_length_m / 2.0
    return np.array([
        [-h, h, 0],
        [h, h, 0],
        [h, -h, 0],
        [-h, -h, 0],
    ], dtype=np.float64)


def estimate_orientation(corners_2d: np.ndarray, marker_length_m: float,
                          camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
    """solvePnP for orientation + reprojection RMS error in pixels (quality
    metric). Returns (quat_xyzw, reprojection_error_px, rvec, tvec), or None
    if PnP fails. tvec is returned for drawFrameAxes() convenience only --
    callers should use estimate_translation_from_depth() for the actual
    reported position, not this tvec.
    """
    obj_pts = marker_object_points(marker_length_m)
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, corners_2d, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None

    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
    reproj_err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners_2d) ** 2, axis=1))))
    quat = rotation_vector_to_quaternion(rvec)
    return quat, reproj_err, rvec, tvec


def rotation_vector_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    """Returns (x, y, z, w). Camera optical frame convention (REP-103: x
    right, y down, z forward) matches OpenCV's convention directly, so no
    extra axis remap is needed here."""
    R, _ = cv2.Rodrigues(rvec)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def deproject_pixel(u: float, v: float, depth_m: float, camera_matrix: np.ndarray) -> np.ndarray:
    """Pinhole deprojection of one pixel + depth into a 3D point in the
    camera optical frame (x right, y down, z forward)."""
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=np.float64)


def estimate_translation_from_depth(corners_2d: np.ndarray, depth_image: np.ndarray,
                                     camera_matrix: np.ndarray, depth_scale: float = 0.001,
                                     min_valid_samples: int = 3, max_std_m: float = 0.05):
    """Samples depth at the 4 corners + center pixel, deprojects each valid
    one, and returns the median 3D point -- robust to a single bad corner
    (occlusion, specular highlight, or straddling a depth edge) without
    needing a full plane fit.

    Returns (point_or_None, num_valid_samples). point is None if too few
    samples were valid, or if they disagree too much (usually means the
    marker is partially occluded or the corner box is straddling a depth
    discontinuity -- e.g. the marker's edge against the sky/background).

    depth_scale converts the raw depth image's stored units to meters --
    0.001 for RealSense's default uint16-millimeters aligned depth image.
    """
    h, w = depth_image.shape[:2]
    center = corners_2d.mean(axis=0)
    sample_px = np.vstack([corners_2d, center[None, :]])

    points = []
    for u, v in sample_px:
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            continue
        raw = depth_image[vi, ui]
        if raw == 0 or not np.isfinite(raw):
            continue
        depth_m = float(raw) * depth_scale
        points.append(deproject_pixel(u, v, depth_m, camera_matrix))

    if len(points) < min_valid_samples:
        return None, len(points)

    points = np.array(points)
    if points[:, 2].std() > max_std_m:
        return None, len(points)

    return np.median(points, axis=0), len(points)
