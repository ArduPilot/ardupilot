#pragma once

#if AP_DDS_ENABLED
#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "geometry_msgs/msg/TwistStamped.h"
#include "ardupilot_msgs/msg/AngularVelandAccn.h"

#include <AP_Common/Location.h>

class AP_DDS_External_Control
{
public:
    // REP-147 Goal Interface Global Position Control
    // https://ros.org/reps/rep-0147.html#goal-interface
    static bool handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos);
    static bool handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel);
    static bool handle_angular_control(custom_msgs_msg_AngularVelandAccn& cmd_angular_goals);
private:
    static bool convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out);
};
#endif // AP_DDS_ENABLED
