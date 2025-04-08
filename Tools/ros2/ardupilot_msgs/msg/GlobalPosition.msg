# Experimental REP-147 Goal Interface
# https://ros.org/reps/rep-0147.html#goal-interface

std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_GLOBAL_INT = 5
uint8 FRAME_GLOBAL_REL_ALT = 6
uint8 FRAME_GLOBAL_TERRAIN_ALT = 11

uint16 type_mask
uint16 IGNORE_LATITUDE = 1 # Position ignore flags
uint16 IGNORE_LONGITUDE = 2
uint16 IGNORE_ALTITUDE = 4
uint16 IGNORE_VX = 8  # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

float64 latitude
float64 longitude
float32 altitude # in meters, AMSL or above terrain
geometry_msgs/Twist velocity
geometry_msgs/Twist acceleration_or_force
float32 yaw
