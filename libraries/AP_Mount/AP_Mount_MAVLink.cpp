// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_MAVLink.h>
#if AP_AHRS_NAVEKF_AVAILABLE
#include <GCS_MAVLink.h>

#define MOUNT_DEBUG 0

#if MOUNT_DEBUG
#include <stdio.h>
#endif

AP_Mount_MAVLink::AP_Mount_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _initialised(false),
    _ekf(frontend._ahrs)
{}

// init - performs any required initialisation for this instance
void AP_Mount_MAVLink::init(const AP_SerialManager& serial_manager)
{
    _initialised = true;
    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
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
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
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
    // do nothing - we rely on the mount sending the messages directly
}


/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_MAVLink::handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
    // just save it for future processing and reporting to GCS for now
    mavlink_msg_gimbal_report_decode(msg, &_gimbal_report);

    Vector3f delta_angles(_gimbal_report.delta_angle_x,
                          _gimbal_report.delta_angle_y,
                          _gimbal_report.delta_angle_z);
    Vector3f delta_velocity(_gimbal_report.delta_velocity_x,
                            _gimbal_report.delta_velocity_y,
                            _gimbal_report.delta_velocity_z);
    Vector3f joint_angles(_gimbal_report.joint_roll,
                          _gimbal_report.joint_pitch,
                          _gimbal_report.joint_yaw);
    _ekf.RunEKF(_gimbal_report.delta_time, delta_angles, delta_velocity, joint_angles);

    // get the gyro bias data
    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);

    // get the gimbal estimated quaternion
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

    // set the demanded quaternion - tilt down with a roll and yaw of zero
    Quaternion quatDem;
    quatDem.from_euler(_angle_ef_target_rad.x,
                       _angle_ef_target_rad.y,
                       _angle_ef_target_rad.z);

    //divide the demanded quaternion by the estimated to get the error
    Quaternion quatErr = quatDem / quatEst;

    // convert the quaternion to an angle error vector
    Vector3f deltaAngErr;
    float scaler = 1.0f-quatErr[0]*quatErr[0];
    if (scaler > 1e-12) {
        scaler = 1.0f/sqrtf(scaler);
        deltaAngErr.x = quatErr[1] * scaler;
        deltaAngErr.y = quatErr[2] * scaler;
        deltaAngErr.z = quatErr[3] * scaler;
    } else {
        deltaAngErr.zero();
    }

    // multiply the angle error vector by a gain to calculate a demanded gimbal rate
    Vector3f rateDemand = deltaAngErr * 1.0f;

    // Constrain the demanded rate to a length of 0.5 rad /sec
    float length = rateDemand.length();
    if (length > 0.5f) {
        rateDemand = rateDemand * (0.5f / length);
    }

    // send the gimbal control message
    mavlink_msg_gimbal_control_send(chan, 
                                    msg->sysid,
                                    msg->compid,
                                    rateDemand.x, rateDemand.y, rateDemand.z, // demanded rates
                                    gyroBias.x, gyroBias.y, gyroBias.z);
}

/*
  send a GIMBAL_REPORT message to the GCS
 */
void AP_Mount_MAVLink::send_gimbal_report(mavlink_channel_t chan)
{
    mavlink_msg_gimbal_report_send(chan, 
                                   0, 0, // send as broadcast
                                   _gimbal_report.delta_time, 
                                   _gimbal_report.delta_angle_x, 
                                   _gimbal_report.delta_angle_y, 
                                   _gimbal_report.delta_angle_z, 
                                   _gimbal_report.delta_velocity_x, 
                                   _gimbal_report.delta_velocity_y, 
                                   _gimbal_report.delta_velocity_z, 
                                   _gimbal_report.joint_roll, 
                                   _gimbal_report.joint_pitch, 
                                   _gimbal_report.joint_yaw);
    float tilt;
    Vector3f velocity, euler, gyroBias;
    _ekf.getDebug(tilt, velocity, euler, gyroBias);
#if MOUNT_DEBUG
    ::printf("tilt=%.2f euler=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f %.2f)\n",
             tilt,
             degrees(euler.x), degrees(euler.y), degrees(euler.z),
             (velocity.x), (velocity.y), (velocity.z));
#endif
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
