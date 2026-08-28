"""Round-trip sanity tests for aruco_math.py, with no ROS dependency at all --
runnable directly with `python3 -m pytest test/test_aruco_math.py` or even
`python3 test/test_aruco_math.py`, against just opencv-contrib + numpy.

Strategy: render a synthetic camera image of a known ArUco marker at a known
pose (rvec_true/tvec_true), run it back through detection + our pose
estimation, and check we recover something close to the pose we started
with. This is the cheapest way to catch a corner-ordering / sign / axis-
convention bug before it ever meets real hardware.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'rover_detection'))
import aruco_math  # noqa: E402

MARKER_ID = 51
MARKER_LENGTH_M = 0.150
IMAGE_W, IMAGE_H = 640, 480
CAMERA_MATRIX = np.array([
    [600.0, 0.0, 320.0],
    [0.0, 600.0, 240.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
DIST_COEFFS = np.zeros(5, dtype=np.float64)

# A marker's own +Z axis points out of its printed face. For the camera to
# see that face (not its blank back), the marker must be rotated ~180 deg
# about X relative to the camera before any additional tilt is applied --
# rvec=[0,0,0] alone puts the camera behind the marker looking at nothing,
# which (found by this test) renders as a mirrored, undecodable image, not
# just a wrong-but-detectable pose. R_FRONTAL is that base "facing the
# camera, upright" orientation; TILT is a small additional rotation applied
# in the marker's own frame on top of it, so the test still exercises
# solvePnP on a non-degenerate pose.
_R_FRONTAL = np.diag([1.0, -1.0, -1.0])
# A near-zero tilt here lands in the well-documented planar-pose ambiguity
# zone for square markers viewed near head-on (IPPE/homography-based PnP can
# flip between two similar-reprojection-error solutions under tiny pixel
# noise) -- that's a real property of square-marker pose estimation, not a
# bug, but it makes a head-on synthetic test flaky. A non-trivial tilt, more
# representative of an actual Mars-Yard viewing angle anyway, avoids it.
_TILT_RVEC = np.array([0.35, -0.25, 0.05], dtype=np.float64)
_R_TILT, _ = cv2.Rodrigues(_TILT_RVEC)
RVEC_TRUE, _ = cv2.Rodrigues(_R_FRONTAL @ _R_TILT)
RVEC_TRUE = RVEC_TRUE.flatten()
TVEC_TRUE = np.array([0.03, -0.01, 1.0], dtype=np.float64)


def _render_synthetic_frame():
    """Renders MARKER_ID at (RVEC_TRUE, TVEC_TRUE) into a IMAGE_H x IMAGE_W
    grayscale frame via a perspective warp, and returns the frame plus the
    ground-truth camera-frame XYZ of each of the 4 marker corners (for
    building a matching synthetic depth image)."""
    dictionary = aruco_math.get_dictionary('DICT_5X5_100')
    marker_px = 300
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker_img = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, marker_px)
    else:
        marker_img = cv2.aruco.drawMarker(dictionary, MARKER_ID, marker_px)

    obj_pts = aruco_math.marker_object_points(MARKER_LENGTH_M)
    image_pts_true, _ = cv2.projectPoints(obj_pts, RVEC_TRUE, TVEC_TRUE, CAMERA_MATRIX, DIST_COEFFS)
    image_pts_true = image_pts_true.reshape(4, 2).astype(np.float32)

    src_corners = np.array([[0, 0], [marker_px - 1, 0],
                             [marker_px - 1, marker_px - 1], [0, marker_px - 1]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(src_corners, image_pts_true)
    # Warping straight to full canvas size: pixels outside the source square
    # map from outside [0,marker_px)^2, so they come out as borderValue --
    # white -- giving a clean background with no separate masking step.
    frame = cv2.warpPerspective(marker_img, homography, (IMAGE_W, IMAGE_H),
                                 flags=cv2.INTER_LINEAR, borderValue=255)

    R_true, _ = cv2.Rodrigues(RVEC_TRUE)
    true_corner_cam_pts = (R_true @ obj_pts.T).T + TVEC_TRUE  # (4,3)

    return frame, true_corner_cam_pts


def test_detection_and_orientation_roundtrip():
    frame, _ = _render_synthetic_frame()

    dictionary = aruco_math.get_dictionary('DICT_5X5_100')
    detector = aruco_math.make_detector(dictionary)
    corners, ids = aruco_math.detect_markers(detector, frame)

    assert ids is not None, 'marker was not detected at all'
    ids_flat = ids.flatten().tolist()
    assert MARKER_ID in ids_flat, f'expected id {MARKER_ID}, detected {ids_flat}'

    idx = ids_flat.index(MARKER_ID)
    detected_corners = corners[idx].reshape(4, 2).astype(np.float64)

    result = aruco_math.estimate_orientation(detected_corners, MARKER_LENGTH_M, CAMERA_MATRIX, DIST_COEFFS)
    assert result is not None, 'solvePnP failed'
    quat, reproj_err, rvec_est, tvec_est = result

    assert reproj_err < 2.0, f'reprojection error too high: {reproj_err:.2f}px'

    # Compare recovered vs. true rotation as an angle between the two
    # rotation matrices, not a raw rvec/quat diff (avoids sign-convention
    # false failures for an equivalent rotation).
    R_true, _ = cv2.Rodrigues(RVEC_TRUE)
    R_est, _ = cv2.Rodrigues(rvec_est)
    R_diff = R_est @ R_true.T
    angle_err_rad = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
    assert angle_err_rad < 0.05, f'orientation error too high: {np.degrees(angle_err_rad):.2f} deg'

    # quat should be unit-norm regardless of the specific pose.
    assert abs(np.linalg.norm(quat) - 1.0) < 1e-6

    # tvec from PnP itself (not the production path, which uses depth
    # instead -- see estimate_translation_from_depth -- but it should still
    # be in the right ballpark as a sanity check on the PnP call itself).
    assert np.linalg.norm(tvec_est.flatten() - TVEC_TRUE) < 0.05


def test_translation_from_depth():
    frame, true_corner_cam_pts = _render_synthetic_frame()

    dictionary = aruco_math.get_dictionary('DICT_5X5_100')
    detector = aruco_math.make_detector(dictionary)
    corners, ids = aruco_math.detect_markers(detector, frame)
    idx = ids.flatten().tolist().index(MARKER_ID)
    detected_corners = corners[idx].reshape(4, 2).astype(np.float64)

    # Build a synthetic depth image: at each of the 4 detected corner pixels
    # (rounded, same as production sampling), store that corner's true
    # camera-frame Z in mm (RealSense aligned-depth convention). Center
    # pixel gets the mean of the 4 corners' true Z, matching what a real
    # near-planar marker would give.
    depth_image = np.zeros((IMAGE_H, IMAGE_W), dtype=np.uint16)
    for (u, v), true_pt in zip(detected_corners, true_corner_cam_pts):
        ui, vi = int(round(u)), int(round(v))
        depth_image[vi, ui] = int(round(true_pt[2] * 1000.0))
    center_px = detected_corners.mean(axis=0)
    center_z_mm = int(round(true_corner_cam_pts[:, 2].mean() * 1000.0))
    depth_image[int(round(center_px[1])), int(round(center_px[0]))] = center_z_mm

    point, n_valid = aruco_math.estimate_translation_from_depth(
        detected_corners, depth_image, CAMERA_MATRIX)

    assert point is not None, f'depth estimate rejected (n_valid={n_valid})'
    assert n_valid == 5
    err = np.linalg.norm(point - TVEC_TRUE)
    assert err < 0.02, f'depth-based translation off by {err * 1000:.1f}mm: got {point}, expected {TVEC_TRUE}'


def test_rejects_too_few_depth_samples():
    depth_image = np.zeros((IMAGE_H, IMAGE_W), dtype=np.uint16)
    depth_image[240, 320] = 1000  # only one valid sample
    corners = np.array([[300, 220], [340, 220], [340, 260], [300, 260]], dtype=np.float64)
    point, n_valid = aruco_math.estimate_translation_from_depth(corners, depth_image, CAMERA_MATRIX)
    assert point is None
    assert n_valid <= 1


def test_dictionary_is_nested_5x5():
    """Confirms the ERC spec's nesting claim: a given ID renders identically
    regardless of which 5x5_* dictionary size it's drawn from -- this is
    what lets tools/generate_aruco_markers.py use DICT_5X5_100 for IDs up to
    65 interchangeably with DICT_5X5_250/1000."""
    d100 = aruco_math.get_dictionary('DICT_5X5_100')
    d250 = aruco_math.get_dictionary('DICT_5X5_250')
    draw = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker
    img100 = draw(d100, MARKER_ID, 100)
    img250 = draw(d250, MARKER_ID, 100)
    assert np.array_equal(img100, img250)


if __name__ == '__main__':
    test_detection_and_orientation_roundtrip()
    print('test_detection_and_orientation_roundtrip: PASSED')
    test_translation_from_depth()
    print('test_translation_from_depth: PASSED')
    test_rejects_too_few_depth_samples()
    print('test_rejects_too_few_depth_samples: PASSED')
    test_dictionary_is_nested_5x5()
    print('test_dictionary_is_nested_5x5: PASSED')
    print('\nall aruco_math tests passed')
