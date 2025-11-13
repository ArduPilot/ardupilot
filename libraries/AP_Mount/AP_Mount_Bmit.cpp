#include "AP_Mount_config.h"

#if HAL_MOUNT_BMIT_ENABLED

#include "AP_Mount_Bmit.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_BMIT_SEARCH_MS  60000  // search for gimbal for 1 minute after startup
#define AP_MOUNT_BMIT_ATTITUDE_INTERVAL_US  20000  // send ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE at 50hz
#define AP_MOUNT_BMIT_SEARCH_DELAY_MS 10000 // delay before starting gimbal search after startup (10 seconds)
#define AP_MOUNT_BMIT_REQ_INTERVAL_MS 1000 // interval between repeated gimbal device information requests (1 second)
// update mount position
void AP_Mount_Bmit::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    // update based on mount mode
    switch (get_mode()) {

    // move mount to a "retracted" position.
    case MAV_MOUNT_MODE_RETRACT: {
        const Vector3f &angle_bf_target = _params.retract_angles.get();
        mnt_target.target_type = MountTargetType::ANGLE;
        mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
        break;
    }
    // move mount to a neutral position, typically pointing forward
    case MAV_MOUNT_MODE_NEUTRAL: {
        const Vector3f &angle_bf_target = _params.neutral_angles.get();
        mnt_target.target_type = MountTargetType::ANGLE;
        mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
        break;
    }
    // move mount with mavlink messages
    case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
        // mavlink targets are stored while handling the incoming message set_angle_target() or set_rate_target()
        break;
    }

    // RC radio manual angle control, but with stabilization from the AHRS
    case MAV_MOUNT_MODE_RC_TARGETING: {
        // update targets using pilot's RC inputs
        update_mnt_target_from_rc_target();
        break;
    }

    // point mount to a GPS point given by the mission planner
    case MAV_MOUNT_MODE_GPS_POINT: {
        if (get_angle_target_to_roi(mnt_target.angle_rad)) {
            mnt_target.target_type = MountTargetType::ANGLE;
        }
        break;
    }
    // point mount to Home location
    case MAV_MOUNT_MODE_HOME_LOCATION: {
        if (get_angle_target_to_home(mnt_target.angle_rad)) {
            mnt_target.target_type = MountTargetType::ANGLE;
        }
        break;
    }
    // point mount to another vehicle
    case MAV_MOUNT_MODE_SYSID_TARGET: {
        if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
            mnt_target.target_type = MountTargetType::ANGLE;
        }
        break;
    }
    default:
        // unknow mode
        break;
    }

    // send target angles or rates depending on the target type
    switch (mnt_target.target_type) {
    case MountTargetType::ANGLE:
        send_gimbal_device_set_attitude(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.yaw, mnt_target.angle_rad.yaw_is_ef);
        break;
    case MountTargetType::RATE:
        send_gimbal_device_set_rate(mnt_target.rate_rads.roll, mnt_target.rate_rads.pitch, mnt_target.rate_rads.yaw, mnt_target.rate_rads.yaw_is_ef);
        break;
    }

}

// return true if healthy
bool AP_Mount_Bmit::healthy() const
{
    // unhealthy until gimbal has been found and replied with device info
    if (_link == nullptr || !_got_device_info) {
        return false;
    }

    // check failure flags
    uint32_t critical_failure_flags = GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT |
                                      GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT |
                                      GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;

    if ((_gimbal_device_attitude_status.failure_flags & critical_failure_flags) > 0) {
        return false;
    }

    // return mount is healthy
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Bmit::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat = _gimbal_device_attitude_status.q;
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_Bmit::find_gimbal()
{
    // stop searching for initial 10 seconds
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < AP_MOUNT_BMIT_SEARCH_DELAY_MS) {
        return;
    }

    // search gimbal for 60 seconds or until armed
    if ((now_ms > AP_MOUNT_BMIT_SEARCH_MS) && hal.util->get_soft_armed()) {
        return;
    }

    // search for a mavlink enabled gimbal
    if (_link == nullptr) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid);
        if (_link == nullptr) {
            // have not yet found a gimbal so return
            return;
        }

        _compid = compid;

    }

    // request GIMBAL_DEVICE_INFORMATION
    if (!_got_device_info) {
        if (now_ms - _last_devinfo_req_ms > AP_MOUNT_BMIT_REQ_INTERVAL_MS) {
            _last_devinfo_req_ms = now_ms;
            request_gimbal_device_information();
        }
        return;
    }

    if (send_attitude_to_gimbal()) {
        _initialised = true;
    }

}

// handle GIMBAL_DEVICE_INFORMATION message
void AP_Mount_Bmit::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_gimbal_device_information_t info;
    mavlink_msg_gimbal_device_information_decode(&msg, &info);

    // set parameter defaults from gimbal information
    _params.roll_angle_min.set_default(degrees(info.roll_min));
    _params.roll_angle_max.set_default(degrees(info.roll_max));
    _params.pitch_angle_min.set_default(degrees(info.pitch_min));
    _params.pitch_angle_max.set_default(degrees(info.pitch_max));
    _params.yaw_angle_min.set_default(degrees(info.yaw_min));
    _params.yaw_angle_max.set_default(degrees(info.yaw_max));

    const uint8_t _fw_version = info.firmware_version & 0x000000FF;
    const uint8_t _gimbal_type = (info.firmware_version & 0xFF000000) >> 24;

    // display gimbal info to user
    gcs().send_text(MAV_SEVERITY_INFO, "Mount: %s %s fw:%u gimbal_type:%u",
                    info.vendor_name,
                    info.model_name,
                    (unsigned)_fw_version,
                    (unsigned)_gimbal_type);

    _got_device_info = true;
}

// request GIMBAL_DEVICE_INFORMATION message
void AP_Mount_Bmit::request_gimbal_device_information() const
{
    if (_link == nullptr) {
        return;
    }

    const mavlink_command_long_t pkt {
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_REQUEST_MESSAGE,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

// start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
bool AP_Mount_Bmit::send_attitude_to_gimbal()
{
    // better safe than sorry:
    if (_link == nullptr) {
        return false;
    }
    // send AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    const MAV_RESULT res = _link->set_message_interval(MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, AP_MOUNT_BMIT_ATTITUDE_INTERVAL_US);

    // return true on success
    return (res == MAV_RESULT_ACCEPTED);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
// earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Bmit::send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const
{
    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    const mavlink_gimbal_device_set_attitude_t pkt {
        {NAN, NAN, NAN, NAN},  // attitude
        roll_rads,  // angular velocity x
        pitch_rads,  // angular velocity y
        yaw_rads,  // angular velocity z
        flags,  // flags
        _sysid,  // target_system
        _compid,  // target_component
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
// earth_frame should be true if yaw_rad target is in earth frame angle, false if body_frame
void AP_Mount_Bmit::send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    // convert euler angles to quaternion
    Quaternion q;
    q.from_euler(roll_rad, pitch_rad, yaw_rad);

    const mavlink_gimbal_device_set_attitude_t pkt {
        {q.q1, q.q2, q.q3, q.q4},
        NAN,  // angular velocity x
        NAN,  // angular velocity y
        NAN,  // angular velocity z
        flags,  // flags
        _sysid,  // target_system
        _compid  // target_component
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);

}

// send camera information message to GCS
void AP_Mount_Bmit::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static const uint8_t vendor_name[32] = "BMIT";
    static uint8_t model_name[32] {};
    const uint32_t fw_version = 0;
    const char cam_definition_uri[140] {};
    float focal_length_mm = 0;

    // capability flags
    const uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                           CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                           CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
                           CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS |
                           CAMERA_CAP_FLAGS_HAS_TRACKING_POINT |
                           CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE;

    // send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send(
        chan,
        AP_HAL::millis(),       // time_boot_ms
        vendor_name,            // vendor_name uint8_t[32]
        model_name,             // model_name uint8_t[32]
        fw_version,             // firmware version uint32_t
        focal_length_mm,        // focal_length float (mm)
        NaNf,                   // sensor_size_h float (mm)
        NaNf,                   // sensor_size_v float (mm)
        0,                      // resolution_h uint16_t (pix)
        0,                      // resolution_v uint16_t (pix)
        0,                      // lens_id uint8_t
        flags,                  // flags uint32_t (CAMERA_CAP_FLAGS)
        0,                      // cam_definition_version uint16_t
        cam_definition_uri,     // cam_definition_uri char[140]
        _instance + 1);         // gimbal_device_id uint8_t

    // display gimbal info to user
    gcs().send_text(MAV_SEVERITY_INFO, "Camera: %s %s fw:%u",
                    vendor_name,
                    model_name,
                    (unsigned)fw_version);
}

//take a picture, returns true on success
bool AP_Mount_Bmit::take_picture()
{
    if (_link == nullptr) {
        return false;
    }

    const mavlink_command_long_t pkt {
        0,  // param1
        0.05,  // param2
        1,  // param3
        _image_index +1,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_IMAGE_START_CAPTURE,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set zoom specified as a rate or percentage
bool AP_Mount_Bmit::set_zoom(ZoomType zoom_type, float zoom_value)
{
    if (_link == nullptr) {
        return false;
    }

    switch (zoom_type) {
    case ZoomType::RATE:
        _zoom_type = ZOOM_TYPE_CONTINUOUS;
        break;

    case ZoomType::PCT:
        _zoom_type = ZOOM_TYPE_RANGE;
        break;
    }

    const mavlink_command_long_t pkt {
        _zoom_type,  // param1
        zoom_value,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_SET_CAMERA_ZOOM,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
    return true;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_Bmit::set_focus(FocusType focus_type, float focus_value)
{
    switch (focus_type) {
    case FocusType::RATE:
        _focus_type = FOCUS_TYPE_CONTINUOUS;
        break;

    case FocusType::PCT:
        _focus_type = FOCUS_TYPE_RANGE;
        break;

    case FocusType::AUTO:
        _focus_type = FOCUS_TYPE_AUTO;
        break;
    }

    const mavlink_command_long_t pkt {
        _focus_type,  // param1
        focus_value,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_SET_CAMERA_FOCUS,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return SetFocusResult::ACCEPTED;
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Mount_Bmit::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    switch (tracking_type) {
    case TrackingType::TRK_NONE:
        return send_stop_tracking_to_gimbal();
        break;
    case TrackingType::TRK_POINT:
        return send_point_tracking_to_gimbal(p1.x, p1.y);
        break;
    case TrackingType::TRK_RECTANGLE:
        return send_rectangle_tracking_to_gimbal(p1.x,p1.y,p2.x,p2.y);
        break;
    }

    return false;
}

// send MAV_CMD_CAMERA_TRACK_POINT
bool AP_Mount_Bmit::send_point_tracking_to_gimbal(float x, float y)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    const mavlink_command_long_t pkt {
        x,  // param1
        y,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_CAMERA_TRACK_POINT,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
    return true;
}

// send MAV_CMD_CAMERA_TRACK_RECTANGLE
bool AP_Mount_Bmit::send_rectangle_tracking_to_gimbal(float x1, float y1, float x2, float y2)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    const mavlink_command_long_t pkt {
        x1,  // param1
        y1,  // param2
        x2,  // param3
        y2,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_CAMERA_TRACK_RECTANGLE,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0    // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
    return true;
}

// send MAV_CMD_CAMERA_STOP_TRACKING
bool AP_Mount_Bmit::send_stop_tracking_to_gimbal()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    const mavlink_command_long_t pkt {
        0,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_CAMERA_STOP_TRACKING,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
    return true;
}

// send camera settings message to GCS
void AP_Mount_Bmit::send_camera_settings(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    uint8_t mode_id = _recording_on ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE;

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        mode_id,             // camera mode (0:image, 1:video, 2:image survey)
        NaNf,           // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaNf);              // focusLevel float, percentage from 0 to 100, NaN if unknown

}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Bmit::record_video(bool start_recording)
{
    // exit immediately if not initialised to reduce mismatch
    // between internal and actual state of recording
    if (!_initialised) {
        return false;
    }

    uint16_t cmd;

    if (start_recording) {
        cmd = MAV_CMD_VIDEO_START_CAPTURE;
        _recording_on = true;
        gcs().send_text(MAV_SEVERITY_DEBUG,"BMIT: start recording");
    } else {
        cmd = MAV_CMD_VIDEO_STOP_CAPTURE;
        _recording_on = false;
        gcs().send_text(MAV_SEVERITY_DEBUG,"BMIT: stop recording");
    }

    const mavlink_command_long_t pkt {
        0,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        cmd,  // command
        _sysid,  // target_system
        _compid,  // target_component
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
    return true;
}

#endif // HAL_MOUNT_BMIT_MSIG_ENABLED
