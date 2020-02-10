#include "AP_Camera_SoloGimbal.h"

#include <GCS_MAVLink/GCS.h>

uint8_t AP_Camera_SoloGimbal::gopro_capture_mode;
uint8_t AP_Camera_SoloGimbal::gopro_status;
bool AP_Camera_SoloGimbal::gopro_is_recording;
mavlink_channel_t AP_Camera_SoloGimbal::heartbeat_channel;

// Toggle the shutter on the GoPro
// This is so ArduPilot can toggle the shutter directly, either fo mission/GCS commands, or when the
// Solo's gimbal is installed on a vehicle other than a Solo.  The usual GoPro controls thorugh the 
// Solo app and Solo controller do not use this, as it is done offboard on the companion computer.
void AP_Camera_SoloGimbal::gopro_shutter_toggle()
{
    if (gopro_status != STATUS::GOPRO_CONNECTED) {
        gcs().send_text(MAV_SEVERITY_INFO, "GoPro Not Available");
        return;
    }

    const uint8_t gopro_shutter_start[4] = { 1, 0, 0, 0};
    const uint8_t gopro_shutter_stop[4] = { 0, 0, 0, 0};

    if (gopro_capture_mode == CAPTURE::MODE_PHOTO) {
        // Trigger shutter start to take a photo
        gcs().send_text(MAV_SEVERITY_INFO, "GoPro Photo Trigger");
        mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_SHUTTER,gopro_shutter_start);

    } else if (gopro_capture_mode == CAPTURE::MODE_VIDEO) {
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
        gcs().send_text(MAV_SEVERITY_INFO, "Unsupported GoPro Capture Mode");    
    }
}

// Toggle the GoPro capture mode
// This is so ArduPilot can toggle the capture mode of the GoPro directly, probably with an RC Aux function.
// This is primarily for Solo's gimbal is installed on a vehicle other than a Solo.  The usual GoPro controls 
// thorugh the Solo app and Solo controller do not use this, as it is done offboard on the companion computer.
void AP_Camera_SoloGimbal::gopro_capture_mode_toggle()
{
    uint8_t gopro_capture_mode_values[4] = { };
    
    if (gopro_status != STATUS::GOPRO_CONNECTED) {
        gcs().send_text(MAV_SEVERITY_INFO, "GoPro Not Available");
        return;
    }

    if (gopro_capture_mode == CAPTURE::MODE_PHOTO) {
        // Change to video mode
        gopro_capture_mode_values[0] =  CAPTURE::MODE_VIDEO;
        mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_CAPTURE_MODE,gopro_capture_mode_values);
        gcs().send_text(MAV_SEVERITY_INFO, "GoPro changing to mode video");

    } else if (gopro_capture_mode == CAPTURE::MODE_VIDEO) {
        if (gopro_is_recording) {
            // GoPro is recording, cannot change modes
            gcs().send_text(MAV_SEVERITY_INFO, "GoPro recording, can't change modes");
        } else {
            // Change to camera mode
            gopro_capture_mode_values[0] = CAPTURE::MODE_PHOTO;
            mavlink_msg_gopro_set_request_send(heartbeat_channel, mavlink_system.sysid, MAV_COMP_ID_GIMBAL,GOPRO_COMMAND_CAPTURE_MODE,gopro_capture_mode_values);
            gcs().send_text(MAV_SEVERITY_INFO, "GoPro changing to mode photo");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Unsupported GoPro Capture Mode");    
    }
}

// pass a GoPro Heartbeat message to the backend
// this is only used by the Solo's mavlink GoPro gimbal
void AP_Camera_SoloGimbal::handle_gopro_heartbeat(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_gopro_heartbeat_t report_msg;
    mavlink_msg_gopro_heartbeat_decode(&msg, &report_msg);
    gopro_status = report_msg.status;
    gopro_capture_mode = report_msg.capture_mode;
    gopro_is_recording = report_msg.flags & GOPRO_FLAG_RECORDING;
    heartbeat_channel = chan;
}
