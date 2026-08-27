"""ROS2 node: detects ERC landmark markers on a SHARED RealSense camera feed
-- subscribed via the ROS realsense2_camera driver's published topics, NOT a
directly-owned pyrealsense2 pipeline. Publishes
rover_perception_msgs/RoverPositionFix (landmark ID + the rover chassis
center's position in the ERC 2026 Mars Yard global survey frame, X=right,
Y=front) whenever a fix is available.

Does NOT launch or own the camera -- this node is a pure topic consumer, so
it can be included alongside however the camera actually gets started
(standalone for testing, or as part of a combined navigation/obstacle
avoidance/marker detection launch file later) without fighting anything
else for exclusive device access.

REQUIRES the camera to be launched with align_depth.enable:=true --
depth values must line up pixel-for-pixel with the color image, not raw
unaligned depth (different resolution/FOV, would silently give wrong
positions rather than erroring).

DELIBERATELY DOES NOT PUBLISH A FALLBACK/ASSUMED POSITION: nothing is
published on frames with no landmark in view. A consumer (EKF, localization
node) should simply not receive an update that cycle -- publishing a
guessed position as if it were a real measurement would be actively
misleading to whatever's fusing it.

NO HEADING/YAW, NO COVARIANCE: the rover's body frame is assumed aligned
with the global frame (forward = global +Y) at the moment of each sighting.
There's no real uncertainty estimate behind this fix either; don't
fabricate one to force this into PoseWithCovarianceStamped for an EKF
without first measuring this pipeline's actual error at typical operating
ranges.

Usage:
    ros2 run rover_detection aruco_detect_roverpos --ros-args \\
        --params-file install/rover_detection/share/rover_detection/config/aruco_localization_params.yaml
(or ros2 launch rover_detection aruco_detection.launch.py)
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rover_perception_msgs.msg import RoverPositionFix
from sensor_msgs.msg import CameraInfo, Image

from rover_detection import aruco_math

# RealSense's aligned-depth image is uint16 millimeters by default.
DEPTH_SCALE_MM_TO_M = 0.001

# ERC 2026 Mars Yard landmarks (L1-L15), keyed by ENCODED ArUco ID
# (= 50 + printed sign number). Global frame: X=right, Y=front, meters.
LANDMARKS_XY = {
    51: (3.183, 8.012), 52: (7.269, 9.482), 53: (7.878, 17.583), 54: (9.225, 22.389),
    55: (3.518, 23.990), 56: (0.882, 16.870), 57: (-3.944, 21.415), 58: (-5.491, 16.334),
    59: (-7.695, 13.528), 60: (-1.610, 12.602), 61: (-7.715, 9.721), 62: (-4.311, 4.442),
    63: (-5.720, 28.118), 64: (-11.438, 5.230), 65: (6.483, 1.102),
}
LANDMARK_NAMES = {51 + i: f'L{i + 1}' for i in range(15)}

# ERC 2026 Mars Yard starting locations (S1-S9), global frame, meters --
# same table as realsense_aruco_test_v3.py. Used ONLY for the one-time
# startup seed below, never as an ongoing fallback -- see _publish_start_seed().
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
# ArUco IDs are 51-65; any value outside that range unambiguously marks a
# RoverPositionFix as NOT a real landmark detection. Used for the one-time
# startup seed's landmark_id, so consumers can tell it apart from a real fix.
START_SEED_LANDMARK_ID = -1


def camera_to_rover_frame(point_cam: np.ndarray) -> np.ndarray:
    """Camera optical frame (x right, y down, z forward) -> rover body frame
    (x forward, y left, z up), for a camera mounted level and facing
    straight forward."""
    cam_x, cam_y, cam_z = point_cam
    return np.array([cam_z, -cam_x, -cam_y], dtype=np.float64)


def compute_rover_global_position(landmark_xy, v_fwd, v_left):
    """Assumes rover forward is aligned with global +Y at the moment of the
    sighting -- see module docstring."""
    x_m, y_m = landmark_xy
    return x_m + v_left, y_m - v_fwd


class ArucoLocalizationNode(Node):

    def __init__(self):
        super().__init__('aruco_localization_node')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_length_m', 0.150)
        self.declare_parameter('dictionary_name', 'DICT_5X5_100')
        # [-1] sentinel, not []: rclpy can't infer an array parameter's type
        # from an empty list default -- ParameterUninitializedException,
        # confirmed on hardware. Don't "simplify" this back to [].
        self.declare_parameter(
            'id_whitelist', [-1],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.declare_parameter('min_valid_depth_samples', 3)
        self.declare_parameter('max_depth_std_m', 0.05)
        # Camera mounting offset from the rover chassis base center, rover
        # body frame (fwd, left, up), meters -- measured on the physical rover.
        self.declare_parameter('camera_offset_x', 0.2608)
        self.declare_parameter('camera_offset_y', 0.00)
        self.declare_parameter('camera_offset_z', 0.2535)
        self.declare_parameter('position_topic', '/aruco_localization/rover_position_fix')
        self.declare_parameter('position_frame_id', 'map')
        self.declare_parameter('publish_debug_image', True)
        # Which start line the rover is actually sitting on for this run --
        # THE thing to update right before a competition attempt (see
        # config/aruco_localization_params.yaml). Used once at startup only,
        # see _publish_start_seed().
        self.declare_parameter('start_point', 'S1')

        whitelist = [w for w in self.get_parameter('id_whitelist').value if w >= 0]
        self.id_whitelist = set(whitelist) if whitelist else None
        self.marker_length_m = float(self.get_parameter('marker_length_m').value)
        self.min_valid_depth_samples = int(self.get_parameter('min_valid_depth_samples').value)
        self.max_depth_std_m = float(self.get_parameter('max_depth_std_m').value)
        self.camera_offset = (
            float(self.get_parameter('camera_offset_x').value),
            float(self.get_parameter('camera_offset_y').value),
            float(self.get_parameter('camera_offset_z').value),
        )
        self.position_frame_id = self.get_parameter('position_frame_id').value
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        dictionary = aruco_math.get_dictionary(self.get_parameter('dictionary_name').value)
        self.detector = aruco_math.make_detector(dictionary)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.color_sub = Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.image_callback)

        topic = self.get_parameter('position_topic').value
        self.position_pub = self.create_publisher(RoverPositionFix, topic, 10)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, '/aruco_localization/debug_image', 1)

        self._frame_count = 0
        self.get_logger().info(
            f'aruco_localization_node up: color={color_topic}, depth={depth_topic}, '
            f'publishing RoverPositionFix on "{topic}" (frame_id={self.position_frame_id}) '
            f'only when a landmark fix exists. camera_offset(fwd,left,up)={self.camera_offset}, '
            f'id_whitelist={sorted(self.id_whitelist) if self.id_whitelist else "ALL"}'
        )
        # One-time startup position seed (see _publish_start_seed docstring
        # for why this is a timer, not a publish call right here in
        # __init__). Purely a courtesy for whatever's consuming this topic
        # to have SOMETHING to initialize from before the first real
        # landmark sighting -- it is published exactly once, then never
        # repeated. This does NOT change the "no fallback" behavior of
        # image_callback() below; that logic is untouched.
        self._start_seed_timer = self.create_timer(2.0, self._publish_start_seed)

        self.get_logger().warn(
            'No heading/yaw tracking -- assumes rover forward is aligned with the global '
            '+Y survey axis at the moment of each sighting.'
        )

    def _publish_start_seed(self):
        """Fires once, ~2s after startup, then cancels itself -- the delay
        gives subscribers time to connect before the message goes out,
        since a publish call made directly in __init__() can be lost if no
        one has subscribed yet (a standard ROS2 race condition, not
        specific to this node). landmark_id is set to
        START_SEED_LANDMARK_ID (-1, outside the real 51-65 range) so
        anything consuming this topic can tell a seed apart from an actual
        detection. This is the ONLY place a non-detection position ever
        gets published -- image_callback() still never publishes a
        fallback, exactly as before."""
        self._start_seed_timer.cancel()

        start_point = self.get_parameter('start_point').value
        if start_point not in START_POINTS_XY:
            self.get_logger().error(
                f'start_point parameter "{start_point}" is not a known survey point '
                f'({sorted(START_POINTS_XY)}) -- not publishing a startup seed. Fix the '
                f'start_point parameter (see config/aruco_localization_params.yaml) and restart.'
            )
            return

        x, y = START_POINTS_XY[start_point]
        msg = RoverPositionFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.position_frame_id
        msg.landmark_id = START_SEED_LANDMARK_ID
        msg.position = Point(x=x, y=y, z=0.0)
        self.position_pub.publish(msg)
        self.get_logger().info(
            f'published ONE-TIME startup seed from start_point={start_point}: '
            f'X={x:.3f} Y={y:.3f} (landmark_id={START_SEED_LANDMARK_ID} marks this as a '
            f'seed, not a real detection -- this will not repeat)'
        )

    def image_callback(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        if self.camera_matrix is None:
            self.camera_matrix = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(info_msg.d, dtype=np.float64)
            self.get_logger().info(
                f'camera intrinsics received: fx={self.camera_matrix[0, 0]:.1f} '
                f'fy={self.camera_matrix[1, 1]:.1f}'
            )

        corners, ids = aruco_math.detect_markers(self.detector, gray)

        display = color.copy() if self.debug_pub is not None else None
        fixes = []  # (marker_id, name, x_rover, y_rover, dist_m)

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                marker_id = int(marker_id)
                if display is not None:
                    cv2.aruco.drawDetectedMarkers(display, [marker_corners], None)

                if self.id_whitelist is not None and marker_id not in self.id_whitelist:
                    continue
                if marker_id not in LANDMARKS_XY:
                    continue

                c2d = marker_corners.reshape(4, 2).astype(np.float64)
                translation_cam, n_valid = aruco_math.estimate_translation_from_depth(
                    c2d, depth, self.camera_matrix, depth_scale=DEPTH_SCALE_MM_TO_M,
                    min_valid_samples=self.min_valid_depth_samples,
                    max_std_m=self.max_depth_std_m)
                if translation_cam is None:
                    continue

                cam_fwd, cam_left, _cam_up = camera_to_rover_frame(translation_cam)
                v_fwd = self.camera_offset[0] + cam_fwd
                v_left = self.camera_offset[1] + cam_left
                x_rover, y_rover = compute_rover_global_position(LANDMARKS_XY[marker_id], v_fwd, v_left)
                dist_m = float(np.linalg.norm(translation_cam))
                fixes.append((marker_id, LANDMARK_NAMES[marker_id], x_rover, y_rover, dist_m))

        if fixes:
            # Closest landmark wins -- generally the most accurate depth reading.
            marker_id, name, x_rover, y_rover, dist_m = min(fixes, key=lambda f: f[4])
            msg = RoverPositionFix()
            msg.header.stamp = color_msg.header.stamp
            msg.header.frame_id = self.position_frame_id
            msg.landmark_id = marker_id
            msg.position = Point(x=x_rover, y=y_rover, z=0.0)
            self.position_pub.publish(msg)
            status = f'FIX via {name}(id={marker_id}) d={dist_m:.2f}m: X={x_rover:.3f} Y={y_rover:.3f}'
        else:
            status = 'no landmark in view -- not publishing'

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(f'frame {self._frame_count}: {status}')

        if display is not None:
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if fixes else (0, 0, 255), 2)
            debug_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            debug_msg.header = color_msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
