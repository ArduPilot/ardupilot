#include "AP_Mount_SoloGimbal.h"
#if HAL_SOLO_GIMBAL_ENABLED

#include "SoloGimbal.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

AP_Mount_SoloGimbal::AP_Mount_SoloGimbal(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance),
    _gimbal()
{}

// init - performs any required initialisation for this instance
void AP_Mount_SoloGimbal::init()
{
    _initialised = true;
    AP_Mount_Backend::init();
}

void AP_Mount_SoloGimbal::update_fast()
{
    _gimbal.update_fast();
}

// update mount position - should be called periodically
void AP_Mount_SoloGimbal::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            _gimbal.set_lockedToBody(true);
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(Vector3f{0,0,0}, false);
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            _gimbal.set_lockedToBody(false);
            const Vector3f &target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // targets are stored while handling the incoming mavlink message
            _gimbal.set_lockedToBody(false);
            if (mnt_target.target_type == MountTargetType::RATE) {
                update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            _gimbal.set_lockedToBody(false);
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
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                _gimbal.set_lockedToBody(false);
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                _gimbal.set_lockedToBody(false);
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                _gimbal.set_lockedToBody(false);
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SoloGimbal::get_attitude_quaternion(Quaternion& att_quat)
{
    if (!_gimbal.aligned()) {
        return false;
    }
    att_quat.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.get_bf_yaw());
    return true;
}

/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_SoloGimbal::handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    _gimbal.update_target(Vector3f{mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.get_bf_yaw()});
    _gimbal.receive_feedback(chan,msg);

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }

    if(!_params_saved && logger->logging_started()) {
        _gimbal.fetch_params();       //last parameter save might not be stored in logger so retry
        _params_saved = true;
    }

    if (_gimbal.get_log_dt() > 1.0f/25.0f) {
        _gimbal.write_logs();
    }
}

void AP_Mount_SoloGimbal::handle_param_value(const mavlink_message_t &msg)
{
    _gimbal.handle_param_value(msg);
}

/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_SoloGimbal::handle_gimbal_torque_report(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    _gimbal.disable_torque_report();
}

#endif // HAL_SOLO_GIMBAL_ENABLED
