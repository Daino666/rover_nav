# One rover position fix from ArUco landmark localization.
# header.frame_id is the global survey frame (e.g. "map"); header.stamp is
# the detection timestamp. position.z is unused (0) -- no reliable height
# estimate. No orientation/heading and no real uncertainty estimate --
# don't treat this as a full pose, and don't fabricate a covariance for it.
# `distance` is provided so a consumer can apply its own trust policy, but
# the publishing node already enforces a hard range gate (max_reliable_range_m,
# default 3.0m -- see config/aruco_localization_params.yaml) and never
# publishes a fix beyond it, so this will never exceed that gate's value.

std_msgs/Header header
int32 landmark_id          # ArUco ID (51-65) of the landmark that produced this fix
geometry_msgs/Point position    # rover chassis center, global frame, meters
float32 distance                # straight-line camera-to-marker distance, meters
