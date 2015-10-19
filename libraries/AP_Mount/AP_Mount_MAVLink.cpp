// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_MAVLink.h>
#include <DataFlash.h>
#if AP_AHRS_NAVEKF_AVAILABLE
#include <GCS_MAVLink.h>
#include <stdio.h>
#include <AP_Gimbal.h>

#if MOUNT_DEBUG
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

AP_Mount_MAVLink::AP_Mount_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _initialised(false),
    _gimbal(frontend._ahrs,frontend._externalParameters),
    _log_dt(0),
    _log_del_ang(),
    _log_del_vel()
{}

// init - performs any required initialisation for this instance
void AP_Mount_MAVLink::init(const AP_SerialManager& serial_manager)
{
    _initialised = true;
    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
}

void AP_Mount_MAVLink::Log_Write_Gimbal(AP_Gimbal &gimbal)
{
    _log_dt += gimbal._measurement.delta_time;
    _log_del_ang += gimbal._measurement.delta_angles;
    _log_del_vel += gimbal._measurement.delta_velocity;

    if (_log_dt >= 1.0f/25.0f) {
        uint32_t tstamp = hal.scheduler->millis();
        Quaternion quatEst;
        gimbal._ekf.getQuat(quatEst);
        Vector3f eulerEst;
        quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);

        struct log_Gimbal1 pkt1 = {
            LOG_PACKET_HEADER_INIT(LOG_GIMBAL1_MSG),
            time_ms : tstamp,
            delta_time      : _log_dt,
            delta_angles_x  : _log_del_ang.x,
            delta_angles_y  : _log_del_ang.y,
            delta_angles_z  : _log_del_ang.z,
            delta_velocity_x : _log_del_vel.x,
            delta_velocity_y : _log_del_vel.y,
            delta_velocity_z : _log_del_vel.z,
            joint_angles_x  : gimbal._measurement.joint_angles.x,
            joint_angles_y  : gimbal._measurement.joint_angles.y,
            joint_angles_z  : gimbal._measurement.joint_angles.z
        };
        _frontend._dataflash->WriteBlock(&pkt1, sizeof(pkt1));

        struct log_Gimbal2 pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_GIMBAL2_MSG),
            time_ms : tstamp,
            est_sta : (uint8_t) gimbal._ekf.getStatus(),
            est_x   : eulerEst.x,
            est_y   : eulerEst.y,
            est_z   : eulerEst.z,
            rate_x  : gimbal.gimbalRateDemVec.x,
            rate_y  : gimbal.gimbalRateDemVec.y,
            rate_z  : gimbal.gimbalRateDemVec.z,
            target_x: gimbal._angle_ef_target_rad.x,
            target_y: gimbal._angle_ef_target_rad.y,
            target_z: gimbal._angle_ef_target_rad.z
        };
        _frontend._dataflash->WriteBlock(&pkt2, sizeof(pkt2));

        _log_dt = 0;
        _log_del_ang.zero();
        _log_del_vel.zero();
    }
}

void AP_Mount_MAVLink::update_fast()
{
    _gimbal.update_fast();
}

// update mount position - should be called periodically
void AP_Mount_MAVLink::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            _gimbal.lockedToBody = true;
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            _gimbal.lockedToBody = false;
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            _gimbal.lockedToBody = false;
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            _gimbal.lockedToBody = false;
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            _gimbal.lockedToBody = false;
            if(_frontend._ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_MAVLink::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_MAVLink::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_MAVLink::status_msg(mavlink_channel_t chan)
{
    if (_gimbal.aligned()) {
        mavlink_msg_mount_status_send(chan, 0, 0, degrees(_angle_ef_target_rad.y)*100, degrees(_angle_ef_target_rad.x)*100, degrees(_angle_ef_target_rad.z)*100);
    }
}

/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_MAVLink::handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
    _gimbal.update_target(_angle_ef_target_rad);
    _gimbal.receive_feedback(chan,msg);
    Log_Write_Gimbal(_gimbal);
    if(!_params_saved && _frontend._dataflash->logging_started()) {
        _frontend._externalParameters.fetch_params();       //last parameter save might not be stored in dataflash so retry
        _params_saved = true;
    }
}

/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_MAVLink::handle_gimbal_torque_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
    _frontend._externalParameters.set_param(GMB_PARAM_GMB_SND_TORQUE, 0);
}

/*
  send a GIMBAL_REPORT message to the GCS
 */
void AP_Mount_MAVLink::send_gimbal_report(mavlink_channel_t chan)
{
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
