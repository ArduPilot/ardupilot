#include "AP_Camera_SoloGimbal.h"

#if AP_CAMERA_SOLOGIMBAL_ENABLED

#include <GCS_MAVLink/GCS.h>

// Toggle the shutter on the GoPro
// This is so ArduPilot can toggle the shutter directly, either for mission/GCS commands, or when the
// Solo's gimbal is installed on a vehicle other than a Solo.  The usual GoPro controls thorugh the 
// Solo app and Solo controller do not use this, as it is done offboard on the companion computer.
// entry point to actually take a picture.  returns true on success
bool AP_Camera_SoloGimbal::trigger_pic()
{
    if (gopro_status != GOPRO_HEARTBEAT_STATUS_CONNECTED) {
        gcs().send_text(MAV_SEVERITY_ERROR, "GoPro Not Available");
        return false;
    }

    const uint8_t gopro_shutter_start[4] = { 1, 0, 0, 0};
    const uint8_t gopro_shutter_stop[4] = { 0, 0, 0, 0};

    if (gopro_capture_mode == GOPRO_CAPTURE_MODE_PHOTO) {
        // Trigger shutter start to take a photo
        gcs().send_text(MAV_SEVERITY_INFO, "GoPro Photo Trigger");
        mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_SHUTTER,gopro_shutter_start);

    } else if (gopro_capture_mode == GOPRO_CAPTURE_MODE_VIDEO) {
        if (gopro_is_recording) {
            // GoPro is recording, so stop recording
            gcs().send_text(MAV_SEVERITY_INFO, "GoPro Recording Stop");
            mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_SHUTTER,gopro_shutter_stop);
        } else {
            // GoPro is not recording, so start recording
            gcs().send_text(MAV_SEVERITY_INFO, "GoPro Recording Start");
            mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_SHUTTER,gopro_shutter_start);
        }
    } else {
        gcs().send_text(MAV_SEVERITY_ERROR, "GoPro Unsupported Capture Mode");
        return false;
    }

    return true;
}

// Cycle the GoPro capture mode
// This is so ArduPilot can cycle through the capture modes of the GoPro directly, probably with an RC Aux function.
// This is primarily for Solo's gimbal being installed on a vehicle other than a Solo. The usual GoPro controls 
// through the Solo app and Solo controller do not use this, as it is done offboard on the companion computer.
// momentary switch to change camera between picture and video modes
void AP_Camera_SoloGimbal::cam_mode_toggle()
{
    uint8_t gopro_capture_mode_values[4] = { };

    if (gopro_status != GOPRO_HEARTBEAT_STATUS_CONNECTED) {
        gcs().send_text(MAV_SEVERITY_ERROR, "GoPro Not Available");
        return;
    }

    switch(gopro_capture_mode) {
        case GOPRO_CAPTURE_MODE_VIDEO:
            if (gopro_is_recording) {
                // GoPro is recording, cannot change modes
                gcs().send_text(MAV_SEVERITY_INFO, "GoPro recording, can't change modes");
            } else {
                // Change to camera mode
                gopro_capture_mode_values[0] = GOPRO_CAPTURE_MODE_PHOTO;
                mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_CAPTURE_MODE,gopro_capture_mode_values);
                gcs().send_text(MAV_SEVERITY_INFO, "GoPro changing to mode photo");
            }
            break;

        case GOPRO_CAPTURE_MODE_PHOTO:
        default:
            // Change to video mode
            gopro_capture_mode_values[0] =  GOPRO_CAPTURE_MODE_VIDEO;
            mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_CAPTURE_MODE,gopro_capture_mode_values);
            gcs().send_text(MAV_SEVERITY_INFO, "GoPro changing to mode video");
            break;
    }
}

// handle incoming heartbeat from the Solo gimbal GoPro
void AP_Camera_SoloGimbal::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_gopro_heartbeat_t report_msg;
    mavlink_msg_gopro_heartbeat_decode(&msg, &report_msg);
    gopro_is_recording = report_msg.flags & GOPRO_FLAG_RECORDING;
    heartbeat_channel = chan;

    switch((GOPRO_HEARTBEAT_STATUS)report_msg.status) {
        case GOPRO_HEARTBEAT_STATUS_DISCONNECTED:
        case GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE:
        case GOPRO_HEARTBEAT_STATUS_ERROR:
        case GOPRO_HEARTBEAT_STATUS_CONNECTED:
            gopro_status = (GOPRO_HEARTBEAT_STATUS)report_msg.status;
            break;
        case GOPRO_HEARTBEAT_STATUS_ENUM_END:
            break;
    }

    switch((GOPRO_CAPTURE_MODE)report_msg.capture_mode){
        case GOPRO_CAPTURE_MODE_VIDEO:
        case GOPRO_CAPTURE_MODE_PHOTO:
        case GOPRO_CAPTURE_MODE_BURST:
        case GOPRO_CAPTURE_MODE_TIME_LAPSE:
        case GOPRO_CAPTURE_MODE_MULTI_SHOT:
        case GOPRO_CAPTURE_MODE_PLAYBACK:
        case GOPRO_CAPTURE_MODE_SETUP:
        case GOPRO_CAPTURE_MODE_UNKNOWN:
            gopro_capture_mode = (GOPRO_CAPTURE_MODE)report_msg.capture_mode;
            break;
        case GOPRO_CAPTURE_MODE_ENUM_END:
            break;
    }
}

#endif // AP_CAMERA_SOLOGIMBAL_ENABLED
