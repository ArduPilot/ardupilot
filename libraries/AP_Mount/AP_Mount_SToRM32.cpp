#include "AP_Mount_SToRM32.h"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_STORM32_RESEND_MS  1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_STORM32_SEARCH_MS  60000   // search for gimbal for 1 minute after startup

// update mount position - should be called periodically
void AP_Mount_SToRM32::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mnt_target should have already been populated by set_angle_target() or set_rate_target(). Update target angle from rate if necessary
            if (mnt_target.target_type == MountTargetType::RATE) {
                update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
            }
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            resend_now = true;
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                resend_now = true;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                resend_now = true;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    if (resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) {
        send_do_mount_control(mnt_target.angle_rad);
    }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.get_bf_yaw());
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
        gcs().send_text(MAV_SEVERITY_INFO, "Mount: SToRM32");
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
                                  -degrees(angle_target_rad.get_bf_yaw()),
                                  0, 0, 0,  // param4 ~ param6 unused
                                  MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // store time of send
    _last_send = AP_HAL::millis();
}
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
