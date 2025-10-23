std_msgs/Header header

# Tracking state
int8 TRACKING_SYSTEM_NOT_READY = -1
int8 TRACKING_NO_IMAGES_YET = 0
int8 TRACKING_NOT_INITIALIZED = 1
int8 TRACKING_OK = 2
int8 TRACKING_RECENTLY_LOST = 3
int8 TRACKING_LOST = 4
int8 TRACKING_OK_KLT = 5

int8 tracking_state

# Map change, e.g., loop closure or bundle adjustment
bool map_changed

# Current map id
uint32 map_id

# Latest pose
geometry_msgs/Pose pose

# Tracked map points
sensor_msgs/PointCloud2 tracked_points