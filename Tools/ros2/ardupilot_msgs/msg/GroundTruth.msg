std_msgs/Header header

# Geographical position
float64 latitude
float64 longitude
float64 altitude


# Local ENU position (meters)
geometry_msgs/Vector3 position


# Orientation (ENU)
geometry_msgs/Quaternion orientation

# Linear velocity (ENU, m/s)
geometry_msgs/Vector3 linear_velocity

# Angular velocity ( body frame, rad/s)
geometry_msgs/Vector3 angular_velocity
