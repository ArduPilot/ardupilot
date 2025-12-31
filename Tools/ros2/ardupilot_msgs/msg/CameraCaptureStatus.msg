# Camera capture status information
# Based on MAVLink CAMERA_CAPTURE_STATUS message
std_msgs/Header header

uint32 time_boot_ms          # Timestamp (time since system boot) in ms
uint8 image_status           # Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
uint8 video_status           # Current status of video capturing (0: idle, 1: capture in progress)
float32 image_interval       # Image capture interval in seconds
uint32 recording_time_ms     # Elapsed time since recording started (0: Not supported/available)
float32 available_capacity   # Available storage capacity in MiB
int32 image_count           # Total number of images captured ('forever', or until reset)
