#pragma once
#include "AP_DDS_config.h"

#if AP_DDS_ENABLED
#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "geometry_msgs/msg/TwistStamped.h"
#if AP_DDS_GLIDING_SERVER_ENABLED
#include "ardupilot_msgs/srv/SetGliding.h"
#endif // AP_DDS_GLIDING_SERVER_ENABLED
#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>

class AP_DDS_External_Control
{
public:
    // REP-147 Goal Interface Global Position Control
    // https://ros.org/reps/rep-0147.html#goal-interface
    static bool handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos);
    static bool handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel);
    static bool arm(AP_Arming::Method method, bool do_arming_checks);
    static bool disarm(AP_Arming::Method method, bool do_disarm_checks);
#if AP_DDS_GLIDING_SERVER_ENABLED
    static bool request_gliding(const ardupilot_msgs_srv_SetGliding_Request& req);
#endif // AP_DDS_GLIDING_SERVER_ENABLED

private:
    static bool convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out);
};
#endif // AP_DDS_ENABLED
