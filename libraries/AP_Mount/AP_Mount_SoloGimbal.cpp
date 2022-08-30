#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_SoloGimbal.h"
#if HAL_SOLO_GIMBAL_ENABLED

#include "SoloGimbal.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Mount_SoloGimbal::AP_Mount_SoloGimbal(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance),
    _gimbal()
{}

// init - performs any required initialisation for this instance
void AP_Mount_SoloGimbal::init()
{
    _initialised = true;
    set_mode((enum MAV_MOUNT_MODE)_params.default_mode.get());
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
            // initialise _angle_rad to smooth transition if user changes to RC_TARGETTINg
            _angle_rad = {0, 0, 0, false};
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            _gimbal.set_lockedToBody(false);
            const Vector3f &target = _params.neutral_angles.get();
            _angle_rad.roll = radians(target.x);
            _angle_rad.pitch = radians(target.y);
            _angle_rad.yaw = radians(target.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            _gimbal.set_lockedToBody(false);
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                _angle_rad = mavt_target.angle_rad;
                break;
            case MountTargetType::RATE:
                update_angle_target_from_rate(mavt_target.rate_rads, _angle_rad);
                break;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            _gimbal.set_lockedToBody(false);
            // update targets using pilot's RC inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                update_angle_target_from_rate(rc_target, _angle_rad);
            } else if (get_rc_angle_target(rc_target)) {
                _angle_rad = rc_target;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            _gimbal.set_lockedToBody(false);
            IGNORE_RETURN(get_angle_target_to_roi(_angle_rad));
            break;

        case MAV_MOUNT_MODE_HOME_LOCATION:
            _gimbal.set_lockedToBody(false);
            IGNORE_RETURN(get_angle_target_to_home(_angle_rad));
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            _gimbal.set_lockedToBody(false);
            IGNORE_RETURN(get_angle_target_to_sysid(_angle_rad));
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

// set_mode - sets mount's mode
void AP_Mount_SoloGimbal::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _mode = mode;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SoloGimbal::get_attitude_quaternion(Quaternion& att_quat)
{
    if (!_gimbal.aligned()) {
        return false;
    }
    att_quat.from_euler(_angle_rad.roll, _angle_rad.pitch, get_bf_yaw_angle(_angle_rad));
    return true;
}

/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_SoloGimbal::handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    _gimbal.update_target(Vector3f{_angle_rad.roll, _angle_rad.pitch, get_bf_yaw_angle(_angle_rad)});
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
