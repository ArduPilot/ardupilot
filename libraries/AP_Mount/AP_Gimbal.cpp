// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Gimbal.h>

#if AP_AHRS_NAVEKF_AVAILABLE

#include <stdio.h>
#include <AP_Common.h>
#include <GCS.h>
#include <AP_SmallEKF.h>

void AP_Gimbal::receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg)
{
    if(!_gimbalParams.received_all()){
        _gimbalParams.receive_missing_parameters(chan);
        return;
    }

    decode_feedback(msg);
    update_state();
    if (_gimbalParams.K_gimbalRate!=0.0f){
        if (lockedToBody){
            send_control(chan);
        }else{
            if (_ekf.getStatus() && !isCopterFlipped()){
                send_control(chan); 
            }
        }
        
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
    mavlink_gimbal_report_t report_msg;
    mavlink_msg_gimbal_report_decode(msg, &report_msg);

    _measurement.delta_time = report_msg.delta_time;
    _measurement.delta_angles.x = report_msg.delta_angle_x;
    _measurement.delta_angles.y = report_msg.delta_angle_y;
    _measurement.delta_angles.z = report_msg.delta_angle_z;
    _measurement.delta_velocity.x = report_msg.delta_velocity_x,
    _measurement.delta_velocity.y = report_msg.delta_velocity_y;
    _measurement.delta_velocity.z = report_msg.delta_velocity_z;
    _measurement.joint_angles.x = report_msg.joint_roll;
    _measurement.joint_angles.y = report_msg.joint_el;
    _measurement.joint_angles.z = report_msg.joint_az;

    //apply joint angle compensation
    _measurement.joint_angles -= _gimbalParams.joint_angles_offsets;
    _measurement.delta_velocity -= _gimbalParams.accelerometer_offsets * _measurement.delta_time;
    _measurement.delta_velocity.x *= _gimbalParams.accelerometer_gains.x;
    _measurement.delta_velocity.y *= _gimbalParams.accelerometer_gains.y;
    _measurement.delta_velocity.z *= _gimbalParams.accelerometer_gains.z;
    _measurement.delta_angles -= _gimbalParams.gyro_offsets * _measurement.delta_time;

    // get complementary filter inputs
    vehicle_to_gimbal_quat.from_vector312(_measurement.joint_angles.x,_measurement.joint_angles.y,_measurement.joint_angles.z);
    vehicle_delta_angles = (last_vehicle_gyro + _ahrs.get_gyro())*_measurement.delta_time*0.5f;
    last_vehicle_gyro = _ahrs.get_gyro();
}

void AP_Gimbal::update_state()
{
    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_measurement.delta_time, _measurement.delta_angles, _measurement.delta_velocity, _measurement.joint_angles);

    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

    // Add the control rate vectors
    if(lockedToBody){
        gimbalRateDemVec.zero();
        gimbalRateDemVec += getGimbalRateBodyLock();
    }else{
        gimbalRateDemVec.zero();
        gimbalRateDemVec += getGimbalRateDemVecYaw(quatEst);
        gimbalRateDemVec += getGimbalRateDemVecTilt(quatEst);
        gimbalRateDemVec += getGimbalRateDemVecForward(quatEst);
        gimbalRateDemVec += getGimbalRateDemVecGyroBias();
    }

    update_joint_angle_est();
}

void AP_Gimbal::update_joint_angle_est()
{
    static const float tc = 1.0f;
    float dt = _measurement.delta_time;
    float alpha = constrain_float(dt/(dt+tc),0.0f,1.0f);

    Matrix3f Tvg; // vehicle frame to gimbal frame
    vehicle_to_gimbal_quat.inverse().rotation_matrix(Tvg);

    Vector3f delta_angle_bias;
    _ekf.getGyroBias(delta_angle_bias);
    delta_angle_bias *= dt;

    Vector3f joint_del_ang;
    gimbal_ang_vel_to_joint_rates((_measurement.delta_angles-delta_angle_bias) - Tvg*vehicle_delta_angles, joint_del_ang);

    filtered_joint_angles += joint_del_ang;
    filtered_joint_angles += (_measurement.joint_angles-filtered_joint_angles)*alpha;

    vehicle_to_gimbal_quat_filt.from_vector312(filtered_joint_angles.x,filtered_joint_angles.y,filtered_joint_angles.z);
}

void AP_Gimbal::gimbal_ang_vel_to_joint_rates(const Vector3f& ang_vel, Vector3f& joint_rates)
{
    float sin_theta = sinf(_measurement.joint_angles.y);
    float cos_theta = cosf(_measurement.joint_angles.y);

    float sin_phi = sinf(_measurement.joint_angles.x);
    float cos_phi = cosf(_measurement.joint_angles.x);
    float sec_phi = 1.0f/cos_phi;
    float tan_phi = sin_phi/cos_phi;

    joint_rates.x = ang_vel.x*cos_theta+ang_vel.z*sin_theta;
    joint_rates.y = ang_vel.x*sin_theta*(sin_phi/cos_phi)-ang_vel.z*cos_theta*(sin_phi/cos_phi)+ang_vel.y;
    joint_rates.z = (1.0f/cos_phi)*(ang_vel.z*cos_theta-ang_vel.x*sin_theta);
}

void AP_Gimbal::joint_rates_to_gimbal_ang_vel(const Vector3f& joint_rates, Vector3f& ang_vel)
{
    float sin_theta = sinf(_measurement.joint_angles.y);
    float cos_theta = cosf(_measurement.joint_angles.y);

    float sin_phi = sinf(_measurement.joint_angles.x);
    float cos_phi = cosf(_measurement.joint_angles.x);

    ang_vel.x = cos_theta*joint_rates.x-sin_theta*cos_phi*joint_rates.z;
    ang_vel.y = joint_rates.y + sin_phi*joint_rates.z;
    ang_vel.z = sin_theta*joint_rates.x+cos_theta*cos_phi*joint_rates.z;
}


Vector3f AP_Gimbal::getGimbalRateDemVecYaw(const Quaternion &quatEst)
{
        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        vehicle_to_gimbal_quat_filt.inverse().rotation_matrix(Tvg);

        // multiply the yaw joint angle by a gain to calculate a demanded vehicle frame relative rate vector required to keep the yaw joint centred
        Vector3f gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.z = - _gimbalParams.K_gimbalRate * filtered_joint_angles.z;

        // Get filtered vehicle turn rate in earth frame
        vehicleYawRateFilt = (1.0f - yawRateFiltPole * _measurement.delta_time) * vehicleYawRateFilt + yawRateFiltPole * _measurement.delta_time * _ahrs.get_yaw_rate_earth();
        Vector3f vehicle_rate_ef(0,0,vehicleYawRateFilt);

         // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
        float maxRate = _gimbalParams.K_gimbalRate * yawErrorLimit;
        float vehicle_rate_mag_ef = vehicle_rate_ef.length();
        float excess_rate_correction = fabsf(vehicle_rate_mag_ef) - maxRate; 
        if (vehicle_rate_mag_ef > maxRate) {
            if (vehicle_rate_ef.z>0.0f){
                gimbalRateDemVecYaw += _ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);
            } else {
                gimbalRateDemVecYaw -= _ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);
            }
        }

        // rotate into gimbal frame to calculate the gimbal rate vector required to keep the yaw gimbal centred
        gimbalRateDemVecYaw = Tvg * gimbalRateDemVecYaw;
        return gimbalRateDemVecYaw;
}

Vector3f AP_Gimbal::getGimbalRateDemVecTilt(const Quaternion &quatEst)
{
        // Calculate the gimbal 321 Euler angle estimates relative to earth frame
        Vector3f eulerEst;
        quatEst.to_vector312(eulerEst.x, eulerEst.y, eulerEst.z);

        // Calculate a demanded quaternion using the demanded roll and pitch and estimated yaw (yaw is slaved to the vehicle)
        Quaternion quatDem;
        quatDem.from_vector312( _angle_ef_target_rad.x,
                                _angle_ef_target_rad.y,
                                eulerEst.z);

        //divide the demanded quaternion by the estimated to get the error
        Quaternion quatErr = quatDem / quatEst;

        // Convert to a delta rotation
        quatErr.normalize();
        Vector3f deltaAngErr;
        quatErr.to_axis_angle(deltaAngErr);

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

Vector3f AP_Gimbal::getGimbalRateBodyLock()
{
        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        vehicle_to_gimbal_quat_filt.inverse().rotation_matrix(Tvg);

        // multiply the joint angles by a gain to calculate a rate vector required to keep the joints centred
        Vector3f gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.x = - _gimbalParams.K_gimbalRate * filtered_joint_angles.x;
        gimbalRateDemVecYaw.y = - _gimbalParams.K_gimbalRate * filtered_joint_angles.y;
        gimbalRateDemVecYaw.z = - _gimbalParams.K_gimbalRate * filtered_joint_angles.z;

        // Add a feedforward term from vehicle gyros
        gimbalRateDemVecYaw += Tvg * _ahrs.get_gyro();

        return gimbalRateDemVecYaw;
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
    Vector3f eulerEst;
    quatEst.to_vector312(eulerEst.x, eulerEst.y, eulerEst.z);
    return eulerEst;
}

bool AP_Gimbal::isCopterFlipped()
{
    return fabsf(_ahrs.roll)>0.7f || _ahrs.pitch > 1.0f || _ahrs.pitch < -0.8f;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
