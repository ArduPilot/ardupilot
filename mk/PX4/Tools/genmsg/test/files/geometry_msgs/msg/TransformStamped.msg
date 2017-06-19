# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://www.ros.org/wiki/tf">tf</a> package. 
# See it's documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform
