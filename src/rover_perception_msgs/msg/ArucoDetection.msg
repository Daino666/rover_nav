# One ArUco marker detection, in the camera's optical frame (pose.header.frame_id).
# Orientation comes from solvePnP; position comes from RealSense depth at the
# marker corners (more reliable range than PnP's translation at typical
# Mars-Yard detection distances) -- see aruco_math.py for why.

int32 id
geometry_msgs/PoseStamped pose
float32 distance                   # straight-line camera-to-marker-center, meters
float32 reprojection_error         # PnP reprojection RMS error, pixels -- quality metric
int32 num_valid_depth_samples      # out of 5 (4 corners + center); low = shaky depth estimate
geometry_msgs/Point32[4] corners_2d  # detected pixel corners (x,y), z unused, TL/TR/BR/BL order
