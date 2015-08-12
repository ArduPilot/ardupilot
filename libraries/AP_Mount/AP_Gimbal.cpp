// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Gimbal.h>

#if AP_AHRS_NAVEKF_AVAILABLE

#include <stdio.h>
#include <AP_Common.h>
#include <GCS.h>
#include <AP_SmallEKF.h>

extern const AP_HAL::HAL& hal;

bool AP_Gimbal::present()
{
    return hal.scheduler->millis() - _last_report_msg_ms < 1000;
}

void AP_Gimbal::receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg)
{
    mavlink_gimbal_report_t report_msg;
    mavlink_msg_gimbal_report_decode(msg, &report_msg);

    _gimbalParams.set_channel(chan);

    if(report_msg.target_system != 1) {
        // gimbal must have been power cycled or reconnected
        _gimbalParams.reset();
        _gimbalParams.set_param(GMB_PARAM_GMB_SYSID, 1);
        return;
    }

    _gimbalParams.update();
    if(!_gimbalParams.initialized()){
        return;
    }

    _last_report_msg_ms = hal.scheduler->millis();

    extract_feedback(report_msg);

    update_mode();
    update_state();

    switch(_mode) {
        case GIMBAL_MODE_IDLE:
            _gimbalParams.set_param(GMB_PARAM_GMB_POS_HOLD, 0);
            break;
        case GIMBAL_MODE_POS_HOLD:
            _gimbalParams.set_param(GMB_PARAM_GMB_POS_HOLD, 1);
            break;
        case GIMBAL_MODE_POS_HOLD_FF:
        case GIMBAL_MODE_STABILIZE:
            send_control(chan);
            _gimbalParams.set_param(GMB_PARAM_GMB_POS_HOLD, 0);
        default:
            break;
    }

    float max_torque;
    _gimbalParams.get_param(GMB_PARAM_GMB_MAX_TORQUE, max_torque, 0);
    if (max_torque != _max_torque && max_torque != 0) {
        _max_torque = max_torque;
    }

    if (!hal.util->get_soft_armed() || joints_near_limits()) {
        _gimbalParams.set_param(GMB_PARAM_GMB_MAX_TORQUE, _max_torque);
    } else {
        _gimbalParams.set_param(GMB_PARAM_GMB_MAX_TORQUE, 0);
    }
}

void AP_Gimbal::readVehicleDeltaAngle(uint8_t ins_index, Vector3f &dAng) {
    const AP_InertialSensor &ins = _ahrs.get_ins();

    if (ins_index < ins.get_gyro_count()) {
        if (!ins.get_delta_angle(ins_index,dAng)) {
            dAng = ins.get_gyro(ins_index) / ins.get_sample_rate();
        }
    }
}

void AP_Gimbal::update_fast() {
    const AP_InertialSensor &ins = _ahrs.get_ins();

    if (ins.get_gyro_health(0) && ins.get_gyro_health(1)) {
        // dual gyro mode - average first two gyros
        Vector3f dAng;
        readVehicleDeltaAngle(0, dAng);
        vehicle_delta_angles += dAng*0.5f;
        readVehicleDeltaAngle(1, dAng);
        vehicle_delta_angles += dAng*0.5f;
    } else {
        // single gyro mode - one of the first two gyros are unhealthy or don't exist
        // just read primary gyro
        Vector3f dAng;
        readVehicleDeltaAngle(ins.get_primary_gyro(), dAng);
        vehicle_delta_angles += dAng;
    }
}

void AP_Gimbal::extract_feedback(const mavlink_gimbal_report_t& report_msg)
{
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

    if (_calibrator.get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        _calibrator.new_sample(_measurement.delta_velocity,_measurement.delta_time);
    }

    _measurement.delta_angles -= _gimbalParams.get_gyro_bias() * _measurement.delta_time;
    _measurement.joint_angles -= _gimbalParams.get_joint_bias();
    _measurement.delta_velocity -= _gimbalParams.get_accel_bias() * _measurement.delta_time;
    _measurement.delta_velocity.x *= _gimbalParams.get_accel_gain().x;
    _measurement.delta_velocity.y *= _gimbalParams.get_accel_gain().y;
    _measurement.delta_velocity.z *= _gimbalParams.get_accel_gain().z;

    // update _ang_vel_mag_filt, used for accel sample readiness
    Vector3f ang_vel = _measurement.delta_angles / _measurement.delta_time;
    Vector3f ekf_gyro_bias;
    _ekf.getGyroBias(ekf_gyro_bias);
    ang_vel -= ekf_gyro_bias;
    float alpha = constrain_float(_measurement.delta_time/(_measurement.delta_time+0.5f),0.0f,1.0f);
    _ang_vel_mag_filt += (ang_vel.length()-_ang_vel_mag_filt)*alpha;
    _ang_vel_mag_filt = min(_ang_vel_mag_filt,20.0f);

    // get complementary filter inputs
    vehicle_to_gimbal_quat.from_vector312(_measurement.joint_angles.x,_measurement.joint_angles.y,_measurement.joint_angles.z);
}

void AP_Gimbal::update_mode()
{
    if (_gimbalParams.get_K_rate()==0.0f || (isCopterFlipped() && !(lockedToBody || _calibrator.running()))) {
        _mode = GIMBAL_MODE_IDLE;
    } else if (!_ekf.getStatus()) {
        _mode = GIMBAL_MODE_POS_HOLD;
    } else if (_calibrator.running() || lockedToBody) {
        _mode = GIMBAL_MODE_POS_HOLD_FF;
    } else {
        _mode = GIMBAL_MODE_STABILIZE;
    }
}

void AP_Gimbal::update_state()
{
    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_measurement.delta_time, _measurement.delta_angles, _measurement.delta_velocity, _measurement.joint_angles);

    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

    update_joint_angle_est();

    gimbalRateDemVec.zero();
    switch(_mode) {
        case GIMBAL_MODE_POS_HOLD_FF:
            gimbalRateDemVec += getGimbalRateBodyLock();
            gimbalRateDemVec += getGimbalRateDemVecGyroBias();
            break;
        case GIMBAL_MODE_STABILIZE:
            gimbalRateDemVec += getGimbalRateDemVecYaw(quatEst);
            gimbalRateDemVec += getGimbalRateDemVecTilt(quatEst);
            gimbalRateDemVec += getGimbalRateDemVecForward(quatEst);
            gimbalRateDemVec += getGimbalRateDemVecGyroBias();
            break;
        default:
        case GIMBAL_MODE_IDLE:
        case GIMBAL_MODE_POS_HOLD:
            break;
    }

    float gimbalRateDemVecLen = gimbalRateDemVec.length();
    if (gimbalRateDemVecLen > radians(400)) {
        gimbalRateDemVec *= radians(400)/gimbalRateDemVecLen;
    }
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

    vehicle_delta_angles.zero();
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
    joint_rates.y = ang_vel.x*sin_theta*tan_phi-ang_vel.z*cos_theta*tan_phi+ang_vel.y;
    joint_rates.z = sec_phi*(ang_vel.z*cos_theta-ang_vel.x*sin_theta);
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
        // define the rotation from vehicle to earth
        Matrix3f Tve = _ahrs.get_dcm_matrix();
        Matrix3f Teg;
        quatEst.inverse().rotation_matrix(Teg);

        // multiply the yaw joint angle by a gain to calculate a demanded vehicle frame relative rate vector required to keep the yaw joint centred
        Vector3f gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.z = - _gimbalParams.get_K_rate() * filtered_joint_angles.z;

        // Get filtered vehicle turn rate in earth frame
        vehicleYawRateFilt = (1.0f - yawRateFiltPole * _measurement.delta_time) * vehicleYawRateFilt + yawRateFiltPole * _measurement.delta_time * _ahrs.get_yaw_rate_earth();
        Vector3f vehicle_rate_ef(0,0,vehicleYawRateFilt);

         // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
        float maxRate = _gimbalParams.get_K_rate() * yawErrorLimit;
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
        gimbalRateDemVecYaw = Tve * gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.x = 0;
        gimbalRateDemVecYaw.y = 0;
        gimbalRateDemVecYaw = Teg * gimbalRateDemVecYaw;
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
        Vector3f gimbalRateDemVecTilt = deltaAngErr * _gimbalParams.get_K_rate();
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
    return gyroBias + _gimbalParams.get_gyro_bias();
}

Vector3f AP_Gimbal::getGimbalRateBodyLock()
{
        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        vehicle_to_gimbal_quat_filt.inverse().rotation_matrix(Tvg);

        // multiply the joint angles by a gain to calculate a rate vector required to keep the joints centred
        Vector3f gimbalRateDemVecBodyLock;
        gimbalRateDemVecBodyLock = filtered_joint_angles * -_gimbalParams.get_K_rate();

        joint_rates_to_gimbal_ang_vel(gimbalRateDemVecBodyLock, gimbalRateDemVecBodyLock);

        // Add a feedforward term from vehicle gyros
        gimbalRateDemVecBodyLock += Tvg * _ahrs.get_gyro();

        return gimbalRateDemVecBodyLock;
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
    return _ahrs.get_dcm_matrix().c.z < 0;
}

bool AP_Gimbal::joints_near_limits()
{
    return fabsf(_measurement.joint_angles.x) > radians(40) || _measurement.joint_angles.y > radians(45) || _measurement.joint_angles.y < -radians(135);
}

AccelCalibrator* AP_Gimbal::_acal_get_calibrator(uint8_t instance)
{
    if(instance==0 && (present() || _calibrator.get_status() == ACCEL_CAL_SUCCESS)) {
        return &_calibrator;
    } else {
        return NULL;
    }
}

bool AP_Gimbal::_acal_ready_to_sample()
{
    return _ang_vel_mag_filt < radians(10);
}

bool AP_Gimbal::_acal_saving()
{
    return _gimbalParams.flashing();
}

void AP_Gimbal::_acal_save_calibrations()
{
    if (_calibrator.get_status() != ACCEL_CAL_SUCCESS) {
        return;
    }
    Vector3f bias;
    Vector3f gain;
    _calibrator.get_calibration(bias,gain);
    _gimbalParams.set_accel_bias(bias);
    _gimbalParams.set_accel_gain(gain);
    _gimbalParams.flash();
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
