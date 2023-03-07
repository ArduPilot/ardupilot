#include "AP_Mount_Gremsy.h"

#if HAL_MOUNT_GREMSY_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_GREMSY_RESEND_MS  1000     // resend angle targets to gimbal at least once per second
#define AP_MOUNT_GREMSY_SEARCH_MS  60000    // search for gimbal for 1 minute after startup
#define AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US    20000  // send ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE at 50hz

// update mount position
void AP_Mount_Gremsy::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // update based on mount mode
    switch (get_mode()) {

        // move mount to a "retracted" position.  We disable motors
        case MAV_MOUNT_MODE_RETRACT:
            // handled below
            send_gimbal_device_retract();
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            send_gimbal_device_set_attitude(ToRad(angle_bf_target.x), ToRad(angle_bf_target.y), ToRad(angle_bf_target.z), false);
            }
            break;

        // use angle or rate targets provided by a mavlink message or mission command
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                send_gimbal_device_set_attitude(mavt_target.angle_rad.roll, mavt_target.angle_rad.pitch, mavt_target.angle_rad.yaw, mavt_target.angle_rad.yaw_is_ef);
                break;
            case MountTargetType::RATE:
                send_gimbal_device_set_rate(mavt_target.rate_rads.roll, mavt_target.rate_rads.pitch, mavt_target.rate_rads.yaw, mavt_target.rate_rads.yaw_is_ef);
                break;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's rc inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                send_gimbal_device_set_rate(rc_target.roll, rc_target.pitch, rc_target.yaw, rc_target.yaw_is_ef);
            } else if (get_rc_angle_target(rc_target)) {
                send_gimbal_device_set_attitude(rc_target.roll, rc_target.pitch, rc_target.yaw, rc_target.yaw_is_ef);
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_roi(angle_target_rad)) {
                send_gimbal_device_set_attitude(angle_target_rad.roll, angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        // point mount to home
        case MAV_MOUNT_MODE_HOME_LOCATION: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_home(angle_target_rad)) {
                send_gimbal_device_set_attitude(angle_target_rad.roll, angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        case MAV_MOUNT_MODE_SYSID_TARGET: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_sysid(angle_target_rad)) {
                send_gimbal_device_set_attitude(angle_target_rad.roll, angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        default:
            // unknown mode so do nothing
            break;
    }
}

// return true if healthy
bool AP_Mount_Gremsy::healthy() const
{
    // unhealthy until gimbal has been found and replied with device info
    if (_link == nullptr || !_got_device_info) {
        return false;
    }

    // unhealthy if attitude information NOT received within the last second
    if (AP_HAL::millis() - _last_attitude_status_ms > 1000) {
        return false;
    }

    // check failure flags
    uint32_t critical_failure_flags = GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;

    if ((_gimbal_device_attitude_status.failure_flags & critical_failure_flags) > 0) {
        return false;
    }

    // if we get this far return mount is healthy
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Gremsy::get_attitude_quaternion(Quaternion& att_quat)
{
    // check we have received an updated message
    if (_gimbal_device_attitude_status.time_boot_ms == _sent_gimbal_device_attitude_status_ms) {
        return false;
    }
    _sent_gimbal_device_attitude_status_ms = _gimbal_device_attitude_status.time_boot_ms;

    att_quat = _gimbal_device_attitude_status.q;
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_Gremsy::find_gimbal()
{
    // do not look for gimbal for first 10 seconds so user may see banner
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < 10000) {
        return;
    }

    // search for gimbal for 60 seconds or until armed
    if ((now_ms > AP_MOUNT_GREMSY_SEARCH_MS) && hal.util->get_soft_armed()) {
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
        if (now_ms - _last_devinfo_req_ms > 1000) {
            _last_devinfo_req_ms = now_ms;
            request_gimbal_device_information();
        }
        return;
    }

    // start sending autopilot attitude to gimbal
    if (start_sending_attitude_to_gimbal()) {
        _initialised = true;
    }
}

// handle GIMBAL_DEVICE_INFORMATION message
void AP_Mount_Gremsy::handle_gimbal_device_information(const mavlink_message_t &msg)
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

    const uint8_t fw_ver_major = info.firmware_version & 0x000000FF;
    const uint8_t fw_ver_minor = (info.firmware_version & 0x0000FF00) >> 8;
    const uint8_t fw_ver_revision = (info.firmware_version & 0x00FF0000) >> 16;
    const uint8_t fw_ver_build = (info.firmware_version & 0xFF000000) >> 24;

    // display gimbal info to user
    gcs().send_text(MAV_SEVERITY_INFO, "Mount: %s %s fw:%u.%u.%u.%u",
            info.vendor_name,
            info.model_name,
            (unsigned)fw_ver_major,
            (unsigned)fw_ver_minor,
            (unsigned)fw_ver_revision,
            (unsigned)fw_ver_build);

    _got_device_info = true;
}

// handle GIMBAL_DEVICE_ATTITUDE_STATUS message
void AP_Mount_Gremsy::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    // take copy of message so it can be forwarded onto GCS later
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &_gimbal_device_attitude_status);
    _last_attitude_status_ms = AP_HAL::millis();
}

// request GIMBAL_DEVICE_INFORMATION message
void AP_Mount_Gremsy::request_gimbal_device_information() const
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
        MAV_CMD_REQUEST_MESSAGE,
        _sysid,
        _compid,
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

// start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
bool AP_Mount_Gremsy::start_sending_attitude_to_gimbal()
{
    // better safe than sorry:
    if (_link == nullptr) {
        return false;
    }
    // send AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    const MAV_RESULT res = _link->set_message_interval(MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US);

    // return true on success
    return (res == MAV_RESULT_ACCEPTED);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
void AP_Mount_Gremsy::send_gimbal_device_retract() const
{
    const mavlink_gimbal_device_set_attitude_t pkt {
        {NAN, NAN, NAN, NAN},  // attitude
        0,   // angular velocity x
        0,  // angular velocity y
        0,    // angular velocity z
        GIMBAL_DEVICE_FLAGS_RETRACT,  // flags
        _sysid,
        _compid
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
// earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Gremsy::send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const
{
    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    const mavlink_gimbal_device_set_attitude_t pkt {
        {NAN, NAN, NAN, NAN},  // attitude
        roll_rads,   // angular velocity x
        pitch_rads,  // angular velocity y
        yaw_rads,    // angular velocity z
        flags,
        _sysid,
        _compid
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
// earth_frame should be true if yaw_rad target is in earth frame angle, false if body_frame
void AP_Mount_Gremsy::send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const
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
        flags,
        _sysid,
        _compid
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}

#endif // HAL_MOUNT_GREMSY_ENABLED
