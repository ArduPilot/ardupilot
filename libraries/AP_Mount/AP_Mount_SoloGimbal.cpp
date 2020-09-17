#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_SoloGimbal.h"
#if HAL_SOLO_GIMBAL_ENABLED

#include "SoloGimbal.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Mount_SoloGimbal::AP_Mount_SoloGimbal(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _gimbal()
{}

// init - performs any required initialisation for this instance
void AP_Mount_SoloGimbal::init()
{
    _initialised = true;
    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
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
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            _gimbal.set_lockedToBody(false);
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            _gimbal.set_lockedToBody(false);
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            _gimbal.set_lockedToBody(false);
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            _gimbal.set_lockedToBody(false);
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true)) {
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_SoloGimbal::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_SoloGimbal::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_SoloGimbal::send_mount_status(mavlink_channel_t chan)
{
    if (_gimbal.aligned()) {
        mavlink_msg_mount_status_send(chan, 0, 0, degrees(_angle_ef_target_rad.y)*100, degrees(_angle_ef_target_rad.x)*100, degrees(_angle_ef_target_rad.z)*100);
    }
    
    // block heartbeat from transmitting to the GCS
    GCS_MAVLINK::disable_channel_routing(chan);
}

/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_SoloGimbal::handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    _gimbal.update_target(_angle_ef_target_rad);
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

/*
  send a GIMBAL_REPORT message to the GCS
 */
void AP_Mount_SoloGimbal::send_gimbal_report(mavlink_channel_t chan)
{
}

#endif // HAL_SOLO_GIMBAL_ENABLED
