# Experimental REP-147 Goal Interface - Modified for trajectory setpoints
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
uint16 IGNORE_AFX = 64  # Acceleration ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 IGNORE_YAW = 512       # Orientation ignore
uint16 IGNORE_YAW_RATE = 1024  # Angular velocity ignore

geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel
