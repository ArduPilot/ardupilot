// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Gimbal.h>
#include <GCS.h>
#include <GCS_MAVLink.h>
#include <AP_SmallEKF.h>

const AP_Param::GroupInfo AP_Gimbal::var_info[] PROGMEM = {
    AP_GROUPEND
};

uint16_t feedback_error_count;
static float K_gimbalRate = 5.0f;
static float angRateLimit = 0.5f;

void AP_Gimbal::receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg)
{
    update_targets_from_rc();
    decode_feedback(msg);
    update_state();
    if (_ekf.getStatus()){
        send_control(chan);
    }

    Quaternion quatEst;_ekf.getQuat(quatEst);Vector3f eulerEst;quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);
    //::printf("est=%1.1f %1.1f %1.1f %d\t", eulerEst.x,eulerEst.y,eulerEst.z,_ekf.getStatus());    
    //::printf("joint_angles=(%+1.2f %+1.2f %+1.2f)\t", _measurament.joint_angles.x,_measurament.joint_angles.y,_measurament.joint_angles.z);
    //::printf("delta_ang=(%+1.3f %+1.3f %+1.3f)\t",_measurament.delta_angles.x,_measurament.delta_angles.y,_measurament.delta_angles.z);     
    //::printf("delta_vel=(%+1.3f %+1.3f %+1.3f)\t",_measurament.delta_velocity.x,_measurament.delta_velocity.y,_measurament.delta_velocity.z);     
    //::printf("rate=(%+1.3f %+1.3f %+1.3f)\t",gimbalRateDemVec.x,gimbalRateDemVec.y,gimbalRateDemVec.z);
    //::printf("target=(%+1.3f %+1.3f %+1.3f)\t",_angle_ef_target_rad.x,_angle_ef_target_rad.y,_angle_ef_target_rad.z);
    //::printf("\n");
}
    

void AP_Gimbal::decode_feedback(mavlink_message_t *msg)
{
    mavlink_gimbal_report_t report_msg;
    mavlink_msg_gimbal_report_decode(msg, &report_msg);

    _measurament.delta_time = report_msg.delta_time;
    _measurament.delta_angles.x = report_msg.delta_angle_x;
    _measurament.delta_angles.y = report_msg.delta_angle_y,
    _measurament.delta_angles.z = report_msg.delta_angle_z;
    _measurament.delta_velocity.x = report_msg.delta_velocity_x,
    _measurament.delta_velocity.y = report_msg.delta_velocity_y,
    _measurament.delta_velocity.z = report_msg.delta_velocity_z;   
    _measurament.joint_angles.x = report_msg.joint_roll;
    _measurament.joint_angles.y = report_msg.joint_el,
    _measurament.joint_angles.z = report_msg.joint_az;

    //apply joint angle compensation
    _measurament.joint_angles -= _joint_offsets;
}

void AP_Gimbal::update_state()
{
    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_measurament.delta_time, _measurament.delta_angles, _measurament.delta_velocity, _measurament.joint_angles);

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

Vector3f AP_Gimbal::getGimbalRateDemVecYaw(Quaternion quatEst)
{
        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        Tvg.from_euler( _measurament.joint_angles.x,
                        _measurament.joint_angles.y,  
                        _measurament.joint_angles.z);

        // multiply the yaw joint angle by a gain to calculate a demanded vehicle frame relative rate vector required to keep the yaw joint centred
        Vector3f gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.z = - K_gimbalRate * _measurament.joint_angles.z;

        // Get filtered vehicle turn rate in earth frame
        vehicleYawRateFilt = (1.0f - yawRateFiltPole * _measurament.delta_time) * vehicleYawRateFilt + yawRateFiltPole * _measurament.delta_time * _ahrs.get_yaw_rate_earth();
        Vector3f vehicle_rate_ef(0,0,vehicleYawRateFilt);

         // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
        float maxRate = K_gimbalRate * yawErrorLimit;
        float vehicle_rate_mag_ef = vehicle_rate_ef.length();
        float excess_rate_correction = fabs(vehicle_rate_mag_ef) - maxRate; 
        if (vehicle_rate_mag_ef > maxRate) {
            if (vehicle_rate_ef.z>0.0f){
                gimbalRateDemVecYaw += _ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);    
            }else{
                gimbalRateDemVecYaw -= _ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);    
            }            
        }        

        // rotate into gimbal frame to calculate the gimbal rate vector required to keep the yaw gimbal centred
        gimbalRateDemVecYaw = Tvg * gimbalRateDemVecYaw;
        return gimbalRateDemVecYaw;
}

Vector3f AP_Gimbal::getGimbalRateDemVecTilt(Quaternion quatEst)
{
        // Calculate the gimbal 321 Euler angle estimates relative to earth frame
        Vector3f eulerEst;
        quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);

        //TODO receive target from AP_Mount
        Vector3f vectorError;
        vectorError.x = eulerEst.x;
        vectorError.y = eulerEst.y - _angle_ef_target_rad.y;
        vectorError.z = 0;

        Vector3f gimbalRateDemVecTilt = - vectorError * K_gimbalRate;
        return gimbalRateDemVecTilt;
}

Vector3f AP_Gimbal::getGimbalRateDemVecForward(Quaternion quatEst)
{
        // quaternion demanded at the previous time step
        static float lastDem;

        // calculate the delta rotation from the last to the current demand where the demand does not incorporate the copters yaw rotation
        float delta = _angle_ef_target_rad.y - lastDem;
        lastDem = _angle_ef_target_rad.y;

        Vector3f gimbalRateDemVecForward;
        gimbalRateDemVecForward.y = delta / _measurament.delta_time;
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
    mavlink_msg_gimbal_control_send(chan,_sysid, _compid, 
        gimbalRateDemVec.x, gimbalRateDemVec.y, gimbalRateDemVec.z);
}

// returns the angle (radians) that the RC_Channel input is receiving
float angle_input_rad(RC_Channel* rc, float angle_min, float angle_max)
{
    float input =rc->norm_input();
    float angle = input*(angle_max - angle_min) + angle_min;
    
    return radians(angle);
}

// update_targets_from_rc - updates angle targets using input from receiver
void AP_Gimbal::update_targets_from_rc()
{
    float new_tilt = angle_input_rad(RC_Channel::rc_channel(tilt_rc_in-1), _tilt_angle_min, _tilt_angle_max);
    float max_change_rads =_max_tilt_rate * _measurament.delta_time;
    float tilt_change = constrain_float(new_tilt - _angle_ef_target_rad.y,-max_change_rads,+max_change_rads);
    _angle_ef_target_rad.y += tilt_change;
}
