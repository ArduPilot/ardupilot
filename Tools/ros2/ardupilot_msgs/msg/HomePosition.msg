std_msgs/Header header

# Latitude in degrees (WGS84)
float64 latitude

# Longitude in degrees (WGS84)
float64 longitude

# Altitude in meters (AMSL)
float32 altitude

# Local ENU position relative to EKF origin in meters
geometry_msgs/Point position

# Surface orientation quaternion (NaN if unavailable)
geometry_msgs/Quaternion orientation

# Approach vector endpoint in local ENU frame (zero if unavailable)
geometry_msgs/Point approach
