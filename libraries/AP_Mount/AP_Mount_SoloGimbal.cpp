#include "AP_Mount_config.h"

#if HAL_SOLO_GIMBAL_ENABLED

#include "AP_Mount_SoloGimbal.h"

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
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    update_mnt_target();

    send_target_to_gimbal();
}

// send angle target in radians to gimbal
void AP_Mount_SoloGimbal::send_target_angles(const MountAngleTarget& angle_rad)
{
    // angles are *actually* inserted into the gimbal in
    // handle_gimbal_report, not here!

    // since angles are now valid, we can unlock from the body:
    _gimbal.set_lockedToBody(false);
 }

void AP_Mount_SoloGimbal::send_target_retracted()
{
    _gimbal.set_lockedToBody(true);
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

#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
#endif
    if(!_params_saved
#if HAL_LOGGING_ENABLED
       && logger->logging_started()
#endif
        ) {
        _gimbal.fetch_params();       //last parameter save might not be stored in logger so retry
        _params_saved = true;
    }

#if HAL_LOGGING_ENABLED
    if (_gimbal.get_log_dt() > 1.0f/25.0f) {
        _gimbal.write_logs();
    }
#endif
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
