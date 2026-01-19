#include "AP_Mount_config.h"

#if HAL_MOUNT_MAVLINK_ENABLED

#include "AP_Mount_MAVLink.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_MAVLINK_SEARCH_MS  60000    // search for gimbal for 1 minute after startup
#define AP_MOUNT_MAVLINK_ATTITUDE_INTERVAL_US    20000  // send ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE at 50hz

// update mount position
void AP_Mount_MAVLink::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    update_mnt_target();

    // send target angles/rates/retract depending on the target type
    send_target_to_gimbal();
}

// return true if healthy
bool AP_Mount_MAVLink::healthy() const
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
bool AP_Mount_MAVLink::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat = _gimbal_device_attitude_status.q;
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_MAVLink::find_gimbal()
{
    // do not look for gimbal for first 10 seconds so user may see banner
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < 10000) {
        return;
    }

    // search for gimbal for 60 seconds or until armed
    if ((now_ms > AP_MOUNT_MAVLINK_SEARCH_MS) && hal.util->get_soft_armed()) {
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
void AP_Mount_MAVLink::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_gimbal_device_information_t info;
    mavlink_msg_gimbal_device_information_decode(&msg, &info);

    // set parameter defaults from gimbal information
    if (!isnan(info.roll_min) && !isnan(info.roll_max)) {
        _params.roll_angle_min.set_default(degrees(info.roll_min));
        _params.roll_angle_max.set_default(degrees(info.roll_max));
    }
    if (!isnan(info.pitch_min) && !isnan(info.pitch_max)) {
        _params.pitch_angle_min.set_default(degrees(info.pitch_min));
        _params.pitch_angle_max.set_default(degrees(info.pitch_max));
    }
    if (!isnan(info.yaw_min) && !isnan(info.yaw_max)) {
        _params.yaw_angle_min.set_default(degrees(info.yaw_min));
        _params.yaw_angle_max.set_default(degrees(info.yaw_max));
    }

    const uint8_t fw_ver_major = info.firmware_version & 0x000000FF;
    const uint8_t fw_ver_minor = (info.firmware_version & 0x0000FF00) >> 8;
    const uint8_t fw_ver_revision = (info.firmware_version & 0x00FF0000) >> 16;
    const uint8_t fw_ver_build = (info.firmware_version & 0xFF000000) >> 24;

    // display gimbal info to user
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: %s %s fw:%u.%u.%u.%u",
            info.vendor_name,
            info.model_name,
            (unsigned)fw_ver_major,
            (unsigned)fw_ver_minor,
            (unsigned)fw_ver_revision,
            (unsigned)fw_ver_build);

    _got_device_info = true;
}

// handle GIMBAL_DEVICE_ATTITUDE_STATUS message
void AP_Mount_MAVLink::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
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
void AP_Mount_MAVLink::request_gimbal_device_information() const
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
bool AP_Mount_MAVLink::start_sending_attitude_to_gimbal()
{
    // better safe than sorry:
    if (_link == nullptr) {
        return false;
    }
    // send AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    const MAV_RESULT res = _link->set_message_interval(MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, AP_MOUNT_MAVLINK_ATTITUDE_INTERVAL_US);

    // return true on success
    return (res == MAV_RESULT_ACCEPTED);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
void AP_Mount_MAVLink::send_target_retracted()
{
    // update the target angles.  These may be absolutely bogus, of
    // course, but may be useful in logs to see what the gimbal was
    // doing.  This is also preserving existing behaviour in a change...
    mnt_target.angle_rad.set(Vector3f{0,0,0}, false);

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
void AP_Mount_MAVLink::send_target_rates(const MountRateTarget &rate_rads)
{
    const float roll_rads = rate_rads.roll;
    const float pitch_rads = rate_rads.pitch;
    const float yaw_rads = rate_rads.yaw;
    const bool earth_frame = rate_rads.yaw_is_ef;

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
void AP_Mount_MAVLink::send_target_angles(const MountAngleTarget &angle_rad)
{
    const float roll_rad = angle_rad.roll;
    const float pitch_rad = angle_rad.pitch;
    const float yaw_rad = angle_rad.yaw;
    const bool earth_frame = angle_rad.yaw_is_ef;

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

#endif // HAL_MOUNT_MAVLINK_ENABLED
