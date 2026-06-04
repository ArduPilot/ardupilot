std_msgs/Header header

# True Airspeed vector in ROS REP103 axis orientation
# x: forward; y: left; z: up
geometry_msgs/Vector3 true_airspeed

# Equivalent to True airspeed conversion factor.
float32 eas_2_tas
