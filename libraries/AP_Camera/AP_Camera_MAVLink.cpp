#include "AP_Camera_MAVLink.h"

#if AP_CAMERA_MAVLINK_ENABLED
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// entry point to actually take a picture.  returns true on success
bool AP_Camera_MAVLink::trigger_pic()
{
    // tell all of our components to take a picture:
    mavlink_command_long_t cmd_msg {};
    cmd_msg.command = MAV_CMD_DO_DIGICAM_CONTROL;
    cmd_msg.param5 = 1;

    // forward to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
    return true;
}

// configure camera
void AP_Camera_MAVLink::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    // convert to mavlink message and send to all components
    mavlink_command_long_t mav_cmd_long = {};

    // convert mission command to mavlink command_long
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONFIGURE;
    mav_cmd_long.param1 = shooting_mode;
    mav_cmd_long.param2 = shutter_speed;
    mav_cmd_long.param3 = aperture;
    mav_cmd_long.param4 = ISO;
    mav_cmd_long.param5 = exposure_type;
    mav_cmd_long.param6 = cmd_id;
    mav_cmd_long.param7 = engine_cutoff_time;

    // send to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&mav_cmd_long, sizeof(mav_cmd_long));
}

// handle camera control message
void AP_Camera_MAVLink::control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    // take picture and ignore other arguments
    if (is_equal(shooting_cmd, 1.0f)) {
        take_picture();
        return;
    }

    // convert command to mavlink command long
    mavlink_command_long_t mav_cmd_long = {};
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONTROL;
    mav_cmd_long.param1 = session;
    mav_cmd_long.param2 = zoom_pos;
    mav_cmd_long.param3 = zoom_step;
    mav_cmd_long.param4 = focus_lock;
    mav_cmd_long.param5 = shooting_cmd;
    mav_cmd_long.param6 = cmd_id;

    // send to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&mav_cmd_long, sizeof(mav_cmd_long));
}

#endif // AP_CAMERA_MAVLINK_ENABLED
