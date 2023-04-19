#include "AP_Camera_MAVLinkCamV2.h"

#if AP_CAMERA_MAVLINKCAMV2_ENABLED
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_CAMERA_MAVLINKCAMV2_SEARCH_MS    60000   // search for camera for 60 seconds after startup

// update - should be called at 50hz
void AP_Camera_MAVLinkCamV2::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_camera();
    }

    // call parent update
    AP_Camera_Backend::update();
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_MAVLinkCamV2::trigger_pic()
{
    // exit immediately if have not found camera or does not support taking pictures
    if (_link == nullptr || !(_cap_flags & CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.command = MAV_CMD_IMAGE_START_CAPTURE;
    pkt.param3 = 1;             // number of images to take
    pkt.param4 = image_index+1; // starting sequence number

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Camera_MAVLinkCamV2::record_video(bool start_recording)
{
    // exit immediately if have not found camera or does not support recording video
    if (_link == nullptr || !(_cap_flags & CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};

    if (start_recording) {
        pkt.command = MAV_CMD_VIDEO_START_CAPTURE;
        // param1 = 0, video stream id. 0 for all streams
        // param2 = 0, status frequency.  frequency that CAMERA_CAPTURE_STATUS messages should be sent while recording. 0 for no messages
    } else {
        pkt.command = MAV_CMD_VIDEO_STOP_CAPTURE;
        // param1 = 0, video stream id. 0 for all streams
    }

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set zoom specified as a rate or percentage
bool AP_Camera_MAVLinkCamV2::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if have not found camera or does not support zoom
    if (_link == nullptr || !(_cap_flags & CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.command = MAV_CMD_SET_CAMERA_ZOOM;
    switch (zoom_type) {
    case ZoomType::RATE:
        pkt.param1 = ZOOM_TYPE_CONTINUOUS;
        break;
    case ZoomType::PCT:
        pkt.param1 = ZOOM_TYPE_RANGE;
        break;
    }
    pkt.param2 = zoom_value;            // Zoom Value

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Camera_MAVLinkCamV2::set_manual_focus_step(int8_t focus_step)
{
    // exit immediately if have not found camera or does not support focus
    if (_link == nullptr || !(_cap_flags & CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.command = MAV_CMD_SET_CAMERA_FOCUS;
    pkt.param1 = FOCUS_TYPE_CONTINUOUS; // Focus Type, 0:FOCUS_TYPE_STEP, 1:FOCUS_TYPE_CONTINUOUS, 2:FOCUS_TYPE_RANGE, 3:FOCUS_TYPE_METERS, 4:FOCUS_TYPE_AUTO, 5:FOCUS_TYPE_AUTO_SINGLE, 5:FOCUS_TYPE_AUTO_CONTINUOUS
    pkt.param2 = focus_step;            // Focus Value

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// auto focus.  returns true on success
bool AP_Camera_MAVLinkCamV2::set_auto_focus()
{
    // exit immediately if have not found camera or does not support focus
    if (_link == nullptr || !(_cap_flags & CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.command = MAV_CMD_SET_CAMERA_FOCUS;
    pkt.param1 = FOCUS_TYPE_AUTO;   // Focus Type, 0:FOCUS_TYPE_STEP, 1:FOCUS_TYPE_CONTINUOUS, 2:FOCUS_TYPE_RANGE, 3:FOCUS_TYPE_METERS, 4:FOCUS_TYPE_AUTO, 5:FOCUS_TYPE_AUTO_SINGLE, 5:FOCUS_TYPE_AUTO_CONTINUOUS
    pkt.param2 = 0;                 // Focus Value

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// handle incoming mavlink message including CAMERA_INFORMATION
void AP_Camera_MAVLinkCamV2::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    // handle CAMERA_INFORMATION
    if (msg.msgid == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
        mavlink_camera_information_t cam_info;
        mavlink_msg_camera_information_decode(&msg, &cam_info);

        const uint8_t fw_ver_major = cam_info.firmware_version & 0x000000FF;
        const uint8_t fw_ver_minor = (cam_info.firmware_version & 0x0000FF00) >> 8;
        const uint8_t fw_ver_revision = (cam_info.firmware_version & 0x00FF0000) >> 16;
        const uint8_t fw_ver_build = (cam_info.firmware_version & 0xFF000000) >> 24;

        // display camera info to user
        gcs().send_text(MAV_SEVERITY_INFO, "Camera: %s.32 %s.32 fw:%u.%u.%u.%u",
                cam_info.vendor_name,
                cam_info.model_name,
                (unsigned)fw_ver_major,
                (unsigned)fw_ver_minor,
                (unsigned)fw_ver_revision,
                (unsigned)fw_ver_build);

        // capability flags
        _cap_flags = cam_info.flags;

        _got_camera_info = true;
    }
}

// search for camera in GCS_MAVLink routing table
void AP_Camera_MAVLinkCamV2::find_camera()
{
    // do not look for camera for first 10 seconds so user may see banner
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < 10000) {
        return;
    }

    // search for camera for 60 seconds or until armed
    if ((now_ms > AP_CAMERA_MAVLINKCAMV2_SEARCH_MS) && hal.util->get_soft_armed()) {
        return;
    }

    // search for a mavlink enabled camera
    if (_link == nullptr) {
        // we expect that instance 0 has compid = MAV_COMP_ID_CAMERA, instance 1 has compid = MAV_COMP_ID_CAMERA2, etc
        uint8_t compid = MIN(MAV_COMP_ID_CAMERA + _instance, MAV_COMP_ID_CAMERA6);
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_CAMERA, compid, _sysid);
        if (_link == nullptr) {
            // have not yet found a camera so return
            return;
        }
        _compid = compid;
    }

    // request CAMERA_INFORMATION
    if (!_got_camera_info) {
        if (now_ms - _last_caminfo_req_ms > 1000) {
            _last_caminfo_req_ms = now_ms;
            request_camera_information();
        }
        return;
    }

    _initialised = true;
}

// request CAMERA_INFORMATION (holds vendor and model name)
void AP_Camera_MAVLinkCamV2::request_camera_information() const
{
    if (_link == nullptr) {
        return;
    }

    const mavlink_command_long_t pkt {
        MAVLINK_MSG_ID_CAMERA_INFORMATION,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_REQUEST_MESSAGE,
        _sysid,
        _compid,
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

#endif // AP_CAMERA_MAVLINKCAMV2_ENABLED
