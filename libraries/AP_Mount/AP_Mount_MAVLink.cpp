// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_MAVLink.h>
#if AP_AHRS_NAVEKF_AVAILABLE
#include <GCS_MAVLink.h>

#define MOUNT_DEBUG 0
#define TILT_CONTROL_ONLY 0

#if MOUNT_DEBUG
#include <stdio.h>
#endif

AP_Mount_MAVLink::AP_Mount_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _initialised(false),
    _ekf(frontend._ahrs),
    K_gimbalRate(0.1f),
    angRateLimit(0.5f),
    yawRateFiltPole(10.0f),
    yawErrorLimit(0.1f),
    vehicleYawRateFilt(0)
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

    /*
      we have two different gimbal control algorithms. One does tilt
      control only, but has better control characteristics. The other
      does roll/tilt/yaw, but has worset control characteristics
     */
#if TILT_CONTROL_ONLY
    Vector3f rateDemand = gimbal_update_control2(_angle_ef_target_rad, 
                                                 _gimbal_report.delta_time, delta_angles, delta_velocity, joint_angles);
#else
    Vector3f rateDemand = gimbal_update_control1(_angle_ef_target_rad, 
                                                 _gimbal_report.delta_time, delta_angles, delta_velocity, joint_angles);
#endif

    // for now send a zero gyro bias update and incorporate into the
    // demanded rates
    Vector3f gyroBias(0,0,0);

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

/*
  calculate demanded rates for the gimbal
 */
Vector3f AP_Mount_MAVLink::gimbal_update_control1(const Vector3f &ef_target_euler_rad, 
                                                  float delta_time, 
                                                  const Vector3f &delta_angles,
                                                  const Vector3f &delta_velocity,
                                                  const Vector3f &joint_angles)
{
    // get the gyro bias data
    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);
    
    // get the gimbal estimated quaternion
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

    // set the demanded quaternion - tilt down with a roll and yaw of zero
    Quaternion quatDem;
    quatDem.from_euler(ef_target_euler_rad.x, ef_target_euler_rad.y, ef_target_euler_rad.z);

    //divide the demanded quaternion by the estimated to get the error
    Quaternion quatErr = quatDem / quatEst;

    // convert the quaternion to an angle error vector using a first order approximation
    Vector3f deltaAngErr;
    float scaler;
    if (quatErr[0] >= 0.0f) {
        scaler = 2.0f;
    } else {
        scaler = -2.0f;
    }
    deltaAngErr.x = quatErr[1] * scaler;
    deltaAngErr.y = quatErr[2] * scaler;
    deltaAngErr.z = quatErr[3] * scaler;
 
    // multiply the angle error vector by a gain to calculate a demanded gimbal rate
    Vector3f rateDemand = deltaAngErr * 1.0f;

    // Constrain the demanded rate to a length of 0.5 rad /sec
    float length = rateDemand.length();
    if (length > 0.5f) {
        rateDemand = rateDemand * (0.5f / length);
    }

    return rateDemand;
}


// convert the quaternion to rotation vector
Vector3f AP_Mount_MAVLink::quaternion_to_vector(const Quaternion &quat)
{
    Vector3f vector;
    float scaler = 1.0f-quat[0]*quat[0];
    if (scaler > 1e-12f) {
        scaler = 1.0f/sqrtf(scaler);
        if (quat[0] < 0.0f) {
            scaler *= -1.0f;
        }
        vector.x = quat[1] * scaler;
        vector.y = quat[2] * scaler;
        vector.z = quat[3] * scaler;
    } else {
        vector.zero();
    }
    return vector;
}

// Define rotation matrix using a 312 rotation sequence vector
Matrix3f AP_Mount_MAVLink::vector312_to_rotation_matrix(const Vector3f &vector)
{
    Matrix3f matrix;
    float cosPhi = cosf(vector.x);
    float cosTheta = cosf(vector.y);
    float sinPhi = sinf(vector.x);
    float sinTheta = sinf(vector.y);
    float sinPsi = sinf(vector.z);
    float cosPsi = cosf(vector.z);
    matrix[0][0] = cosTheta*cosPsi-sinPsi*sinPhi*sinTheta;
    matrix[1][0] = -sinPsi*cosPhi;
    matrix[2][0] = cosPsi*sinTheta+cosTheta*sinPsi*sinPhi;
    matrix[0][1] = cosTheta*sinPsi+cosPsi*sinPhi*sinTheta;
    matrix[1][1] = cosPsi*cosPhi;
    matrix[2][1] = sinPsi*sinTheta-cosTheta*cosPsi*sinPhi;
    matrix[0][2] = -sinTheta*cosPhi;
    matrix[1][2] = sinPhi;
    matrix[2][2] = cosTheta*cosPhi;
    return matrix;
}


/*
  calculate the demanded rates for the mount, running the controller
 */
Vector3f AP_Mount_MAVLink::gimbal_update_control2(const Vector3f &ef_target_euler_rad, 
                                                  float delta_time, 
                                                  const Vector3f &delta_angles,
                                                  const Vector3f &delta_velocity,
                                                  const Vector3f &joint_angles)
{
    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);
 
    // Add the control rate vectors
    Vector3f gimbalRateDemVec = 
        getGimbalRateDemVecYaw(ef_target_euler_rad, delta_time, quatEst, joint_angles) + 
        getGimbalRateDemVecTilt(ef_target_euler_rad, quatEst) + 
        getGimbalRateDemVecForward(ef_target_euler_rad, delta_time, quatEst);

    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);

    gimbalRateDemVec += gyroBias;
    return gimbalRateDemVec;
}

Vector3f AP_Mount_MAVLink::getGimbalRateDemVecYaw(const Vector3f &ef_target_euler_rad, float delta_time, const Quaternion &quatEst, const Vector3f &joint_angles)
{
    // Define rotation from vehicle to gimbal using a 312 rotation sequence
    Matrix3f Tvg = vector312_to_rotation_matrix(joint_angles);

    // multiply the yaw joint angle by a gain to calculate a
    // demanded vehicle frame relative rate vector required to
    // keep the yaw joint centred
    Vector3f gimbalRateDemVecYaw(0, 0, - K_gimbalRate * joint_angles.z);

    // Get filtered vehicle turn rate in earth frame
    vehicleYawRateFilt = (1.0f - yawRateFiltPole * delta_time) * vehicleYawRateFilt + yawRateFiltPole * delta_time * _frontend._ahrs.get_yaw_rate_earth();
    Vector3f vehicle_rate_ef(0,0,vehicleYawRateFilt);
    
    // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
    float maxRate = K_gimbalRate * yawErrorLimit;
    float vehicle_rate_mag_ef = vehicle_rate_ef.length();
    float excess_rate_correction = fabs(vehicle_rate_mag_ef) - maxRate; 
    if (vehicle_rate_mag_ef > maxRate) {
        if (vehicle_rate_ef.z>0.0f) {
            gimbalRateDemVecYaw += _frontend._ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);    
        } else {
            gimbalRateDemVecYaw -= _frontend._ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);    
        }            
    }        
    
    // rotate into gimbal frame to calculate the gimbal rate vector required to keep the yaw gimbal centred
    gimbalRateDemVecYaw = Tvg * gimbalRateDemVecYaw;
    return gimbalRateDemVecYaw;
}

Vector3f AP_Mount_MAVLink::getGimbalRateDemVecTilt(const Vector3f &ef_target_euler_rad, const Quaternion &quatEst)
{
    // Calculate the gimbal 321 Euler angle estimates relative to earth frame
    Vector3f eulerEst;
    quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);

    // Calculate a demanded quaternion using the demanded roll and pitch and estimated yaw (yaw is slaved to the vehicle)
    Quaternion quatDem;
    //TODO receive target from AP_Mount
    quatDem.from_euler(0, ef_target_euler_rad.y, eulerEst.z);

    //divide the demanded quaternion by the estimated to get the error
    Quaternion quatErr = quatDem / quatEst;

    // multiply the angle error vector by a gain to calculate a demanded gimbal rate required to control tilt
    Vector3f gimbalRateDemVecTilt = quaternion_to_vector(quatErr) * K_gimbalRate;
    return gimbalRateDemVecTilt;
}

Vector3f AP_Mount_MAVLink::getGimbalRateDemVecForward(const Vector3f &ef_target_euler_rad, float delta_time, const Quaternion &quatEst)
{
    // calculate the delta rotation from the last to the current demand where the demand does not incorporate the copters yaw rotation
    Quaternion quatDemForward;
    quatDemForward.from_euler(0, ef_target_euler_rad.y, 0);
    Quaternion deltaQuat = quatDemForward / lastQuatDem;
    lastQuatDem = quatDemForward;
    
    // convert to a rotation vector and divide by delta time to obtain a forward path rate demand
    Vector3f gimbalRateDemVecForward = quaternion_to_vector(deltaQuat) * (1.0f / delta_time);
    return gimbalRateDemVecForward;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
