# Image capture feedback
std_msgs/Header header

uint64 timestamp_us      # Image timestamp (microseconds since epoch)
uint8 camera_id          # Camera ID
uint16 image_index       # Image index

# Vehicle position
float64 latitude         # Latitude in degrees
float64 longitude        # Longitude in degrees
float32 altitude_msl     # Altitude MSL in meters
float32 altitude_rel     # Altitude relative to home in meters

# Vehicle attitude
float32 roll_deg         # Roll angle in degrees
float32 pitch_deg        # Pitch angle in degrees
float32 yaw_deg          # Yaw angle in degrees

# Camera parameters
float32 focal_length_mm  # Focal length in mm
uint8 flags              # Feedback flags
uint16 completed_captures # Completed image captures
