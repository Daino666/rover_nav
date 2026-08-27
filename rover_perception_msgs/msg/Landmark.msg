# A fused, running position estimate for one marker ID, in marker_geolocation_node's
# target_frame (map, by default). Built from possibly many ArucoDetections over time.

int32 id
geometry_msgs/Point position
int32 num_observations
builtin_interfaces/Time last_seen
float32 position_stddev
