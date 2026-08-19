#pragma once

#if AP_DDS_ENABLED
#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "geometry_msgs/msg/TwistStamped.h"
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
    static bool start_image_capture(uint8_t camera_id, uint32_t interval_msec, uint16_t total_images);
    static bool stop_image_capture(uint8_t camera_id);
    static uint16_t image_current_seq(uint8_t camera_id);
    static uint16_t total_image_cap(uint8_t camera_id);

private:
    static bool convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out);
};
#endif // AP_DDS_ENABLED
