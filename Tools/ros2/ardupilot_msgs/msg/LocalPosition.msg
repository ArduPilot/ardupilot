# Experimental REP-147 Goal Interface - Modifiend for trajectory setpoints
# https://ros.org/reps/rep-0147.html#goal-interface
# Coordinate frame is determined by header.frame_id:
# - "map" or "odom": Earth-fixed frame (ENU convention: x=East, y=North, z=Up)
# - "base_link": Body-fixed frame (FLU convention: x=Forward, y=Left, z=Up)

std_msgs/Header header

uint16 type_mask
uint16 IGNORE_X = 1 # Position ignore flags
uint16 IGNORE_Y = 2
uint16 IGNORE_Z = 4
uint16 IGNORE_VX = 8  # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

float32 x
float32 y
float32 z # in meters, in ROS ENU convention (positive up) for map/odom frames
geometry_msgs/Twist velocity
geometry_msgs/Twist acceleration_or_force
float32 yaw # in radians
float32 yaw_rate # in radians/s

