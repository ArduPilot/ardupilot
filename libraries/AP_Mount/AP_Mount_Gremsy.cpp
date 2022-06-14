#include "AP_Mount_Gremsy.h"

#if HAL_MOUNT_GREMSY_ENABLED

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_GREMSY_RESEND_MS  1000     // resend angle targets to gimbal at least once per second
#define AP_MOUNT_GREMSY_SEARCH_MS  60000    // search for gimbal for 1 minute after startup
#define AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US    10000  // send ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE at 100hz

AP_Mount_Gremsy::AP_Mount_Gremsy(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{}

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
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            send_gimbal_device_set_attitude(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z, false);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // angle targets should have been set by a MOUNT_CONTROL message from GCS
            send_gimbal_device_set_attitude(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z, true);
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            if (_rate_target_rads_valid) {
                send_gimbal_device_set_rate(_rate_target_rads.x, _rate_target_rads.y, _rate_target_rads.z, _state._yaw_lock);
            } else {
                send_gimbal_device_set_attitude(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z, _state._yaw_lock);
            }
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true, false)) {
                send_gimbal_device_set_attitude(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z, true);
            }
            break;

        case MAV_MOUNT_MODE_HOME_LOCATION:
            // constantly update the home location:
            if (!AP::ahrs().home_is_set()) {
                break;
            }
            _state._roi_target = AP::ahrs().get_home();
            _state._roi_target_set = true;
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true, false)) {
                send_gimbal_device_set_attitude(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z, true);
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true, false)) {
                send_gimbal_device_set_attitude(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z, true);
            }
            break;

        default:
            // unknown mode so do nothing
            break;
    }
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Gremsy::send_mount_status(mavlink_channel_t chan)
{
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(chan, GIMBAL_DEVICE_ATTITUDE_STATUS)) {
        return;
    }

    // check we have received an updated message
    if (_gimbal_device_attitude_status.time_boot_ms == _sent_gimbal_device_attitude_status_ms) {
        return;
    }
    _sent_gimbal_device_attitude_status_ms = _gimbal_device_attitude_status.time_boot_ms;

    // forward on message to GCS
    mavlink_msg_gimbal_device_attitude_status_send(chan,
                                                   0,    // target system
                                                   0,    // target component
                                                   AP_HAL::millis(),    // autopilot system time
                                                   _gimbal_device_attitude_status.flags,
                                                   _gimbal_device_attitude_status.q,
                                                   _gimbal_device_attitude_status.angular_velocity_x,
                                                   _gimbal_device_attitude_status.angular_velocity_y,
                                                   _gimbal_device_attitude_status.angular_velocity_z,
                                                   _gimbal_device_attitude_status.failure_flags);
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_Gremsy::find_gimbal()
{
    // do not look for gimbal for first 10 seconds so user may see banner
    if (AP_HAL::millis() < 10000) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_GREMSY_SEARCH_MS) {
        return;
    }

    // search for a mavlink enabled gimbal
    if (!_found_gimbal) {
        mavlink_channel_t chan;
        uint8_t sysid, compid;
        if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, sysid, compid, chan)) {
            if (((_instance == 0) && (compid == MAV_COMP_ID_GIMBAL)) ||
                ((_instance == 1) && (compid == MAV_COMP_ID_GIMBAL2))) {
                _found_gimbal = true;
                _sysid = sysid;
                _compid = compid;
                _chan = chan;
            }
        } else {
            // have not yet found a gimbal so return
            return;
        }
    }

    // request GIMBAL_DEVICE_INFORMATION
    if (!_got_device_info) {
        uint32_t now_ms = AP_HAL::millis();
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
    _state._roll_angle_min.set_default(degrees(info.roll_min) * 100);
    _state._roll_angle_max.set_default(degrees(info.roll_max) * 100);
    _state._tilt_angle_min.set_default(degrees(info.pitch_min) * 100);
    _state._tilt_angle_max.set_default(degrees(info.pitch_max) * 100);
    _state._pan_angle_min.set_default(degrees(info.yaw_min) * 100);
    _state._pan_angle_max.set_default(degrees(info.yaw_max) * 100);

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
}

// request GIMBAL_DEVICE_INFORMATION message
void AP_Mount_Gremsy::request_gimbal_device_information() const
{
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(
        _chan,
        _sysid,
        _compid,
        MAV_CMD_REQUEST_MESSAGE,
        0, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, 0, 0, 0, 0, 0, 0);
}

// start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
bool AP_Mount_Gremsy::start_sending_attitude_to_gimbal()
{
    // send AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    const MAV_RESULT res = gcs().set_message_interval(_chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US);

    // return true on success
    return (res == MAV_RESULT_ACCEPTED);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
void AP_Mount_Gremsy::send_gimbal_device_retract() const
{
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    const float quat_array[4] = {NAN, NAN, NAN, NAN};
    mavlink_msg_gimbal_device_set_attitude_send(_chan,
                                                _sysid,     // target system
                                                _compid,    // target component
                                                GIMBAL_DEVICE_FLAGS_RETRACT,    // gimbal device flags
                                                quat_array, // attitude as a quaternion
                                                0, 0, 0);   // angular velocities
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
// earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Gremsy::send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const
{
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;
    const float quat_array[4] = {NAN, NAN, NAN, NAN};

    // send command_long command containing a do_mount_control command
    mavlink_msg_gimbal_device_set_attitude_send(_chan,
                                                _sysid,     // target system
                                                _compid,    // target component
                                                flags,      // gimbal device flags
                                                quat_array, // attitude as a quaternion
                                                roll_rads, pitch_rads, yaw_rads);   // angular velocities
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
// earth_frame should be true if yaw_rad target is in earth frame angle, false if body_frame
void AP_Mount_Gremsy::send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    // convert euler angles to quaternion
    Quaternion q;
    q.from_euler(roll_rad, pitch_rad, yaw_rad);
    const float quat_array[4] = {q.q1, q.q2, q.q3, q.q4};

    // send command_long command containing a do_mount_control command
    mavlink_msg_gimbal_device_set_attitude_send(_chan,
                                                _sysid,     // target system
                                                _compid,    // target component
                                                flags,      // gimbal device flags
                                                quat_array, // attitude as a quaternion
                                                NAN, NAN, NAN);   // angular velocities
}

#endif // HAL_MOUNT_GREMSY_ENABLED
