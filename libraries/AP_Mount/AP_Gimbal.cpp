// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Gimbal.h"

#if AP_AHRS_NAVEKF_AVAILABLE

#include <stdio.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_NavEKF/AP_SmallEKF.h>
#include <AP_Math/AP_Math.h>

void AP_Gimbal::receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg)
{
    decode_feedback(msg);
    update_state();
    if (_ekf.getStatus() && !isCopterFlipped() && !is_zero(_gimbalParams.K_gimbalRate)){
        send_control(chan);
    }

    Quaternion quatEst;_ekf.getQuat(quatEst);Vector3f eulerEst;quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);
    //::printf("est=%1.1f %1.1f %1.1f %d\t", eulerEst.x,eulerEst.y,eulerEst.z,_ekf.getStatus());    
    //::printf("joint_angles=(%+1.2f %+1.2f %+1.2f)\t", _measurement.joint_angles.x,_measurement.joint_angles.y,_measurement.joint_angles.z);
    //::printf("delta_ang=(%+1.3f %+1.3f %+1.3f)\t",_measurement.delta_angles.x,_measurement.delta_angles.y,_measurement.delta_angles.z);     
    //::printf("delta_vel=(%+1.3f %+1.3f %+1.3f)\t",_measurement.delta_velocity.x,_measurement.delta_velocity.y,_measurement.delta_velocity.z);     
    //::printf("rate=(%+1.3f %+1.3f %+1.3f)\t",gimbalRateDemVec.x,gimbalRateDemVec.y,gimbalRateDemVec.z);
    //::printf("target=(%+1.3f %+1.3f %+1.3f)\t",_angle_ef_target_rad.x,_angle_ef_target_rad.y,_angle_ef_target_rad.z);
    //::printf("\n");
}

void AP_Gimbal::decode_feedback(mavlink_message_t *msg)
{
    mavlink_msg_gimbal_report_decode(msg, &_report_msg);

    _measurement.delta_time = _report_msg.delta_time;
    _measurement.delta_angles.x = _report_msg.delta_angle_x;
    _measurement.delta_angles.y = _report_msg.delta_angle_y;
    _measurement.delta_angles.z = _report_msg.delta_angle_z;
    _measurement.delta_velocity.x = _report_msg.delta_velocity_x,
    _measurement.delta_velocity.y = _report_msg.delta_velocity_y;
    _measurement.delta_velocity.z = _report_msg.delta_velocity_z;
    _measurement.joint_angles.x = _report_msg.joint_roll;
    _measurement.joint_angles.y = _report_msg.joint_el;
    _measurement.joint_angles.z = _report_msg.joint_az;

    //apply joint angle compensation
    _measurement.joint_angles -= _gimbalParams.joint_angles_offsets;
    _measurement.delta_velocity -= _gimbalParams.delta_velocity_offsets;
    _measurement.delta_angles -= _gimbalParams.delta_angles_offsets;
}

/*
  send a gimbal report to the GCS for display purposes
 */
void AP_Gimbal::send_report(mavlink_channel_t chan) const
{
    mavlink_msg_gimbal_report_send(chan, 
                                   0, 0, // send as broadcast
                                   _report_msg.delta_time, 
                                   _report_msg.delta_angle_x, 
                                   _report_msg.delta_angle_y, 
                                   _report_msg.delta_angle_z, 
                                   _report_msg.delta_velocity_x, 
                                   _report_msg.delta_velocity_y, 
                                   _report_msg.delta_velocity_z, 
                                   _report_msg.joint_roll, 
                                   _report_msg.joint_el, 
                                   _report_msg.joint_az);
}

void AP_Gimbal::update_state()
{
    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_measurement.delta_time, _measurement.delta_angles, _measurement.delta_velocity, _measurement.joint_angles);

    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

    // Add the control rate vectors
    gimbalRateDemVec.zero();
    gimbalRateDemVec += getGimbalRateDemVecYaw(quatEst);
    gimbalRateDemVec += getGimbalRateDemVecTilt(quatEst);
    gimbalRateDemVec += getGimbalRateDemVecForward(quatEst);
    gimbalRateDemVec += getGimbalRateDemVecGyroBias();
}

Vector3f AP_Gimbal::getGimbalRateDemVecYaw(const Quaternion &quatEst)
{
        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        float cosPhi = cosf(_measurement.joint_angles.x);
        float cosTheta = cosf(_measurement.joint_angles.y);
        float sinPhi = sinf(_measurement.joint_angles.x);
        float sinTheta = sinf(_measurement.joint_angles.y);
        float sinPsi = sinf(_measurement.joint_angles.z);
        float cosPsi = cosf(_measurement.joint_angles.z);
        Tvg[0][0] = cosTheta*cosPsi-sinPsi*sinPhi*sinTheta;
        Tvg[1][0] = -sinPsi*cosPhi;
        Tvg[2][0] = cosPsi*sinTheta+cosTheta*sinPsi*sinPhi;
        Tvg[0][1] = cosTheta*sinPsi+cosPsi*sinPhi*sinTheta;
        Tvg[1][1] = cosPsi*cosPhi;
        Tvg[2][1] = sinPsi*sinTheta-cosTheta*cosPsi*sinPhi;
        Tvg[0][2] = -sinTheta*cosPhi;
        Tvg[1][2] = sinPhi;
        Tvg[2][2] = cosTheta*cosPhi;

        // multiply the yaw joint angle by a gain to calculate a demanded vehicle frame relative rate vector required to keep the yaw joint centred
        Vector3f gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.z = - _gimbalParams.K_gimbalRate * _measurement.joint_angles.z;

        // Get filtered vehicle turn rate in earth frame
        vehicleYawRateFilt = (1.0f - yawRateFiltPole * _measurement.delta_time) * vehicleYawRateFilt + yawRateFiltPole * _measurement.delta_time * _ahrs.get_yaw_rate_earth();
        Vector3f vehicle_rate_ef(0,0,vehicleYawRateFilt);

         // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
        float maxRate = _gimbalParams.K_gimbalRate * yawErrorLimit;
        float vehicle_rate_mag_ef = vehicle_rate_ef.length();
        float excess_rate_correction = fabsf(vehicle_rate_mag_ef) - maxRate; 
        if (vehicle_rate_mag_ef > maxRate) {
            if (vehicle_rate_ef.z>0.0f){
                gimbalRateDemVecYaw += _ahrs.get_rotation_body_to_ned().transposed()*Vector3f(0,0,excess_rate_correction);
            } else {
                gimbalRateDemVecYaw -= _ahrs.get_rotation_body_to_ned().transposed()*Vector3f(0,0,excess_rate_correction);
            }
        }

        // rotate into gimbal frame to calculate the gimbal rate vector required to keep the yaw gimbal centred
        gimbalRateDemVecYaw = Tvg * gimbalRateDemVecYaw;
        return gimbalRateDemVecYaw;
}

Vector3f AP_Gimbal::getGimbalRateDemVecTilt(const Quaternion &quatEst)
{
        // Calculate the gimbal 321 Euler angle estimates relative to earth frame
        Vector3f eulerEst = quatEst.to_vector312();

        // Calculate a demanded quaternion using the demanded roll and pitch and estimated yaw (yaw is slaved to the vehicle)
        Quaternion quatDem;
        quatDem.from_vector312( _angle_ef_target_rad.x,
                                _angle_ef_target_rad.y,
                                eulerEst.z);

        //divide the demanded quaternion by the estimated to get the error
        Quaternion quatErr = quatDem / quatEst;

        // Convert to a delta rotation using a small angle approximation
        quatErr.normalize();
        Vector3f deltaAngErr;
        float scaler;
        if (quatErr[0] >= 0.0f) {
            scaler = 2.0f;
        } else {
            scaler = -2.0f;
        }
        deltaAngErr.x = scaler * quatErr[1];
        deltaAngErr.y = scaler * quatErr[2];
        deltaAngErr.z = scaler * quatErr[3];

        // multiply the angle error vector by a gain to calculate a demanded gimbal rate required to control tilt
        Vector3f gimbalRateDemVecTilt = deltaAngErr * _gimbalParams.K_gimbalRate;
        return gimbalRateDemVecTilt;
}

Vector3f AP_Gimbal::getGimbalRateDemVecForward(const Quaternion &quatEst)
{
        // quaternion demanded at the previous time step
        static float lastDem;

        // calculate the delta rotation from the last to the current demand where the demand does not incorporate the copters yaw rotation
        float delta = _angle_ef_target_rad.y - lastDem;
        lastDem = _angle_ef_target_rad.y;

        Vector3f gimbalRateDemVecForward;
        gimbalRateDemVecForward.y = delta / _measurement.delta_time;
        return gimbalRateDemVecForward;
}

Vector3f AP_Gimbal::getGimbalRateDemVecGyroBias()
{
    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);
    return gyroBias;
}

void AP_Gimbal::send_control(mavlink_channel_t chan)
{
    mavlink_msg_gimbal_control_send(chan, mavlink_system.sysid, _compid,
        gimbalRateDemVec.x, gimbalRateDemVec.y, gimbalRateDemVec.z);
}

void AP_Gimbal::update_target(Vector3f newTarget)
{
    // Low-pass filter
    _angle_ef_target_rad.y = _angle_ef_target_rad.y + 0.02f*(newTarget.y - _angle_ef_target_rad.y);
    // Update tilt
    _angle_ef_target_rad.y = constrain_float(_angle_ef_target_rad.y,radians(-90),radians(0));
}

Vector3f AP_Gimbal::getGimbalEstimateEF()
{
    Quaternion quatEst;
    _ekf.getQuat(quatEst);
    return quatEst.to_vector312();
}

bool AP_Gimbal::isCopterFlipped()
{
    return (_ahrs.cos_roll()*_ahrs.cos_pitch() < 0.5f);
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
