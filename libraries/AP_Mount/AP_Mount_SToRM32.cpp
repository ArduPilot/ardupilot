#include "AP_Mount_SToRM32.h"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_STORM32_RESEND_MS  1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_STORM32_SEARCH_MS  60000   // search for gimbal for 1 minute after startup

AP_Mount_SToRM32::AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance),
    _chan(MAVLINK_COMM_0)
{}

// update mount position - should be called periodically
void AP_Mount_SToRM32::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            _angle_rad.roll = radians(target.x);
            _angle_rad.pitch = radians(target.y);
            _angle_rad.yaw = radians(target.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            _angle_rad.roll = radians(target.x);
            _angle_rad.pitch = radians(target.y);
            _angle_rad.yaw = radians(target.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                _angle_rad = mavt_target.angle_rad;
                break;
            case MountTargetType::RATE:
                update_angle_target_from_rate(mavt_target.rate_rads, _angle_rad);
                break;
            }
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                update_angle_target_from_rate(rc_target, _angle_rad);
            } else if (get_rc_angle_target(rc_target)) {
                _angle_rad = rc_target;
            }
            resend_now = true;
            break;
        }

        // point mount to a GPS location
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(_angle_rad)) {
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(_angle_rad)) {
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(_angle_rad)) {
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    if (resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) {
        send_do_mount_control(_angle_rad);
    }
}

// set_mode - sets mount's mode
void AP_Mount_SToRM32::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _mode = mode;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_angle_rad.roll, _angle_rad.pitch, get_bf_yaw_angle(_angle_rad));
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_SToRM32::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_STORM32_SEARCH_MS) {
        return;
    }

    // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
    uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
    if (GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan)) {
        _compid = compid;
        _initialised = true;
    }
}

// send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
void AP_Mount_SToRM32::send_do_mount_control(const MountTarget& angle_target_rad)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    // Note: pitch and yaw are reversed
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  -degrees(angle_target_rad.pitch),
                                  degrees(angle_target_rad.roll),
                                  -degrees(get_bf_yaw_angle(angle_target_rad)),
                                  0, 0, 0,  // param4 ~ param6 unused
                                  MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // store time of send
    _last_send = AP_HAL::millis();
}
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
