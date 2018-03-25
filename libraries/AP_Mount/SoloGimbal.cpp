#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#if AP_AHRS_NAVEKF_AVAILABLE

#include "SoloGimbal.h"

#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

bool SoloGimbal::present()
{
    if (_state != GIMBAL_STATE_NOT_PRESENT && AP_HAL::millis()-_last_report_msg_ms > 3000) {
        // gimbal went away
        _state = GIMBAL_STATE_NOT_PRESENT;
        return false;
    }

    return _state != GIMBAL_STATE_NOT_PRESENT;
}

bool SoloGimbal::aligned()
{
    return present() && _state == GIMBAL_STATE_PRESENT_RUNNING;
}

gimbal_mode_t SoloGimbal::get_mode()
{
    if ((_gimbalParams.initialized() && is_zero(_gimbalParams.get_K_rate())) || (_ahrs.get_rotation_body_to_ned().c.z < 0 && !(_lockedToBody || _calibrator.running()))) {
        return GIMBAL_MODE_IDLE;
    } else if (!_ekf.getStatus()) {
        return GIMBAL_MODE_POS_HOLD;
    } else if (_calibrator.running() || _lockedToBody) {
        return GIMBAL_MODE_POS_HOLD_FF;
    } else {
        return GIMBAL_MODE_STABILIZE;
    }
}

void SoloGimbal::receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg)
{
    mavlink_gimbal_report_t report_msg;
    mavlink_msg_gimbal_report_decode(msg, &report_msg);
    uint32_t tnow_ms = AP_HAL::millis();
    _last_report_msg_ms = tnow_ms;

    _gimbalParams.set_channel(chan);

    if (report_msg.target_system != 1) {
        _state = GIMBAL_STATE_NOT_PRESENT;
    }

    switch(_state) {
        case GIMBAL_STATE_NOT_PRESENT:
            // gimbal was just connected or we just rebooted, transition to PRESENT_INITIALIZING
            _gimbalParams.reset();
            _gimbalParams.set_param(GMB_PARAM_GMB_SYSID, 1);
            _state = GIMBAL_STATE_PRESENT_INITIALIZING;
            break;

        case GIMBAL_STATE_PRESENT_INITIALIZING:
            _gimbalParams.update();
            if (_gimbalParams.initialized()) {
                // parameters done initializing, finalize initialization and transition to aligning
                extract_feedback(report_msg);
                _ang_vel_mag_filt = 20;
                _filtered_joint_angles = _measurement.joint_angles;
                _vehicle_to_gimbal_quat_filt.from_vector312(_filtered_joint_angles.x,_filtered_joint_angles.y,_filtered_joint_angles.z);
                _ekf.reset();
                _state = GIMBAL_STATE_PRESENT_ALIGNING;
            }
            break;

        case GIMBAL_STATE_PRESENT_ALIGNING:
            _gimbalParams.update();
            extract_feedback(report_msg);
            update_estimators();
            if (_ekf.getStatus()) {
                // EKF done aligning, transition to running
                _state = GIMBAL_STATE_PRESENT_RUNNING;
            }
            break;

        case GIMBAL_STATE_PRESENT_RUNNING:
            _gimbalParams.update();
            extract_feedback(report_msg);
            update_estimators();
            break;
    }

    send_controls(chan);
}

void SoloGimbal::send_controls(mavlink_channel_t chan)
{
    if (_state == GIMBAL_STATE_PRESENT_RUNNING) {
        // get the gimbal quaternion estimate
        Quaternion quatEst;
        _ekf.getQuat(quatEst);

        // run rate controller
        _ang_vel_dem_rads.zero();
        switch(get_mode()) {
            case GIMBAL_MODE_POS_HOLD_FF: {
                _ang_vel_dem_rads += get_ang_vel_dem_body_lock();
                _ang_vel_dem_rads += get_ang_vel_dem_gyro_bias();
                float _ang_vel_dem_radsLen = _ang_vel_dem_rads.length();
                if (_ang_vel_dem_radsLen > radians(400)) {
                    _ang_vel_dem_rads *= radians(400)/_ang_vel_dem_radsLen;
                }
                mavlink_msg_gimbal_control_send(chan, mavlink_system.sysid, _compid,
                                                _ang_vel_dem_rads.x, _ang_vel_dem_rads.y, _ang_vel_dem_rads.z);
                break;
            }
            case GIMBAL_MODE_STABILIZE: {
                _ang_vel_dem_rads += get_ang_vel_dem_yaw(quatEst);
                _ang_vel_dem_rads += get_ang_vel_dem_tilt(quatEst);
                _ang_vel_dem_rads += get_ang_vel_dem_feedforward(quatEst);
                _ang_vel_dem_rads += get_ang_vel_dem_gyro_bias();
                float ang_vel_dem_norm = _ang_vel_dem_rads.length();
                if (ang_vel_dem_norm > radians(400)) {
                    _ang_vel_dem_rads *= radians(400)/ang_vel_dem_norm;
                }
                mavlink_msg_gimbal_control_send(chan, mavlink_system.sysid, _compid,
                                                _ang_vel_dem_rads.x, _ang_vel_dem_rads.y, _ang_vel_dem_rads.z);
                break;
            }
            default:
            case GIMBAL_MODE_IDLE:
            case GIMBAL_MODE_POS_HOLD:
                break;
        }
    }

    // set GMB_POS_HOLD
    if (get_mode() == GIMBAL_MODE_POS_HOLD) {
        _gimbalParams.set_param(GMB_PARAM_GMB_POS_HOLD, 1);
    } else {
        _gimbalParams.set_param(GMB_PARAM_GMB_POS_HOLD, 0);
    }

    // set GMB_MAX_TORQUE
    float max_torque;
    _gimbalParams.get_param(GMB_PARAM_GMB_MAX_TORQUE, max_torque, 0);
    if (!is_equal(max_torque,_max_torque) && !is_zero(max_torque)) {
        _max_torque = max_torque;
    }

    if (!hal.util->get_soft_armed() || joints_near_limits()) {
        _gimbalParams.set_param(GMB_PARAM_GMB_MAX_TORQUE, _max_torque);
    } else {
        _gimbalParams.set_param(GMB_PARAM_GMB_MAX_TORQUE, 0);
    }
}

void SoloGimbal::extract_feedback(const mavlink_gimbal_report_t& report_msg)
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
    Vector3f accel_gain = _gimbalParams.get_accel_gain();
    _measurement.delta_velocity.x *= (is_zero(accel_gain.x) ? 1.0f : accel_gain.x);
    _measurement.delta_velocity.y *= (is_zero(accel_gain.y) ? 1.0f : accel_gain.y);
    _measurement.delta_velocity.z *= (is_zero(accel_gain.z) ? 1.0f : accel_gain.z);

    // update _ang_vel_mag_filt, used for accel sample readiness
    Vector3f ang_vel = _measurement.delta_angles / _measurement.delta_time;
    Vector3f ekf_gyro_bias;
    _ekf.getGyroBias(ekf_gyro_bias);
    ang_vel -= ekf_gyro_bias;
    float alpha = constrain_float(_measurement.delta_time/(_measurement.delta_time+0.5f),0.0f,1.0f);
    _ang_vel_mag_filt += (ang_vel.length()-_ang_vel_mag_filt)*alpha;
    _ang_vel_mag_filt = MIN(_ang_vel_mag_filt,20.0f);

    // get complementary filter inputs
    _vehicle_to_gimbal_quat.from_vector312(_measurement.joint_angles.x,_measurement.joint_angles.y,_measurement.joint_angles.z);

    // update log deltas
    _log_dt += _measurement.delta_time;
    _log_del_ang += _measurement.delta_angles;
    _log_del_vel += _measurement.delta_velocity;
}

void SoloGimbal::update_estimators()
{
    if (_state == GIMBAL_STATE_NOT_PRESENT || _state == GIMBAL_STATE_PRESENT_INITIALIZING) {
        return;
    }

    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_measurement.delta_time, _measurement.delta_angles, _measurement.delta_velocity, _measurement.joint_angles);
    update_joint_angle_est();
}

void SoloGimbal::readVehicleDeltaAngle(uint8_t ins_index, Vector3f &dAng) {
    const AP_InertialSensor &ins = AP::ins();

    if (ins_index < ins.get_gyro_count()) {
        if (!ins.get_delta_angle(ins_index,dAng)) {
            dAng = ins.get_gyro(ins_index) / ins.get_sample_rate();
        }
    }
}

void SoloGimbal::update_fast() {
    const AP_InertialSensor &ins = AP::ins();

    if (ins.get_gyro_health(0) && ins.get_gyro_health(1)) {
        // dual gyro mode - average first two gyros
        Vector3f dAng;
        readVehicleDeltaAngle(0, dAng);
        _vehicle_delta_angles += dAng*0.5f;
        readVehicleDeltaAngle(1, dAng);
        _vehicle_delta_angles += dAng*0.5f;
    } else {
        // single gyro mode - one of the first two gyros are unhealthy or don't exist
        // just read primary gyro
        Vector3f dAng;
        readVehicleDeltaAngle(ins.get_primary_gyro(), dAng);
        _vehicle_delta_angles += dAng;
    }
}

void SoloGimbal::update_joint_angle_est()
{
    static const float tc = 1.0f;
    float dt = _measurement.delta_time;
    float alpha = constrain_float(dt/(dt+tc),0.0f,1.0f);

    Matrix3f Tvg; // vehicle frame to gimbal frame
    _vehicle_to_gimbal_quat.inverse().rotation_matrix(Tvg);

    Vector3f delta_angle_bias;
    _ekf.getGyroBias(delta_angle_bias);
    delta_angle_bias *= dt;

    Vector3f joint_del_ang;
    gimbal_ang_vel_to_joint_rates((_measurement.delta_angles-delta_angle_bias) - Tvg*_vehicle_delta_angles, joint_del_ang);

    _filtered_joint_angles += joint_del_ang;
    _filtered_joint_angles += (_measurement.joint_angles-_filtered_joint_angles)*alpha;

    _vehicle_to_gimbal_quat_filt.from_vector312(_filtered_joint_angles.x,_filtered_joint_angles.y,_filtered_joint_angles.z);

    _vehicle_delta_angles.zero();
}

Vector3f SoloGimbal::get_ang_vel_dem_yaw(const Quaternion &quatEst)
{
    static const float tc = 0.1f;
    static const float yawErrorLimit = radians(5.7f);
    float dt = _measurement.delta_time;
    float alpha = dt/(dt+tc);

    Matrix3f Tve = _ahrs.get_rotation_body_to_ned();
    Matrix3f Teg;
    quatEst.inverse().rotation_matrix(Teg);


    //_vehicle_yaw_rate_ef_filt = _ahrs.get_yaw_rate_earth();

    // filter the vehicle yaw rate to remove noise
    _vehicle_yaw_rate_ef_filt += (_ahrs.get_yaw_rate_earth() - _vehicle_yaw_rate_ef_filt) * alpha;

    float yaw_rate_ff = 0;

    // calculate an earth-frame yaw rate feed-forward that prevents gimbal from exceeding the maximum yaw error
    if (_vehicle_yaw_rate_ef_filt > _gimbalParams.get_K_rate()*yawErrorLimit) {
        yaw_rate_ff = _vehicle_yaw_rate_ef_filt-_gimbalParams.get_K_rate()*yawErrorLimit;
    } else if (_vehicle_yaw_rate_ef_filt < -_gimbalParams.get_K_rate()*yawErrorLimit) {
        yaw_rate_ff = _vehicle_yaw_rate_ef_filt+_gimbalParams.get_K_rate()*yawErrorLimit;
    }

    // filter the feed-forward to remove noise
    //_yaw_rate_ff_ef_filt += (yaw_rate_ff - _yaw_rate_ff_ef_filt) * alpha;

    Vector3f gimbalRateDemVecYaw;
    gimbalRateDemVecYaw.z = yaw_rate_ff - _gimbalParams.get_K_rate() * _filtered_joint_angles.z / constrain_float(Tve.c.z,0.5f,1.0f);
    gimbalRateDemVecYaw.z /= constrain_float(Tve.c.z,0.5f,1.0f);

    // rotate the rate demand into gimbal frame
    gimbalRateDemVecYaw = Teg * gimbalRateDemVecYaw;

    return gimbalRateDemVecYaw;
}

Vector3f SoloGimbal::get_ang_vel_dem_tilt(const Quaternion &quatEst)
{
    // Calculate the gimbal 321 Euler angle estimates relative to earth frame
    Vector3f eulerEst = quatEst.to_vector312();

    // Calculate a demanded quaternion using the demanded roll and pitch and estimated yaw (yaw is slaved to the vehicle)
    Quaternion quatDem;
    quatDem.from_vector312( _att_target_euler_rad.x,
                            _att_target_euler_rad.y,
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

Vector3f SoloGimbal::get_ang_vel_dem_feedforward(const Quaternion &quatEst)
{
    // quaternion demanded at the previous time step
    static float lastDem;

    // calculate the delta rotation from the last to the current demand where the demand does not incorporate the copters yaw rotation
    float delta = _att_target_euler_rad.y - lastDem;
    lastDem = _att_target_euler_rad.y;

    Vector3f gimbalRateDemVecForward;
    gimbalRateDemVecForward.y = delta / _measurement.delta_time;
    return gimbalRateDemVecForward;
}

Vector3f SoloGimbal::get_ang_vel_dem_gyro_bias()
{
    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);
    return gyroBias + _gimbalParams.get_gyro_bias();
}

Vector3f SoloGimbal::get_ang_vel_dem_body_lock()
{
    // Define rotation from vehicle to gimbal using a 312 rotation sequence
    Matrix3f Tvg;
    _vehicle_to_gimbal_quat_filt.inverse().rotation_matrix(Tvg);

    // multiply the joint angles by a gain to calculate a rate vector required to keep the joints centred
    Vector3f gimbalRateDemVecBodyLock;
    gimbalRateDemVecBodyLock = _filtered_joint_angles * -_gimbalParams.get_K_rate();

    joint_rates_to_gimbal_ang_vel(gimbalRateDemVecBodyLock, gimbalRateDemVecBodyLock);

    // Add a feedforward term from vehicle gyros
    gimbalRateDemVecBodyLock += Tvg * _ahrs.get_gyro();

    return gimbalRateDemVecBodyLock;
}

void SoloGimbal::update_target(Vector3f newTarget)
{
    // Low-pass filter
    _att_target_euler_rad.y = _att_target_euler_rad.y + 0.02f*(newTarget.y - _att_target_euler_rad.y);
    // Update tilt
    _att_target_euler_rad.y = constrain_float(_att_target_euler_rad.y,radians(-90),radians(0));
}

void SoloGimbal::write_logs()
{
    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash == nullptr) {
        return;
    }

    uint32_t tstamp = AP_HAL::millis();
    Vector3f eulerEst;

    Quaternion quatEst;
    _ekf.getQuat(quatEst);
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
        joint_angles_x  : _measurement.joint_angles.x,
        joint_angles_y  : _measurement.joint_angles.y,
        joint_angles_z  : _measurement.joint_angles.z
    };
    dataflash->WriteBlock(&pkt1, sizeof(pkt1));

    struct log_Gimbal2 pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_GIMBAL2_MSG),
        time_ms : tstamp,
        est_sta : (uint8_t) _ekf.getStatus(),
        est_x   : eulerEst.x,
        est_y   : eulerEst.y,
        est_z   : eulerEst.z,
        rate_x  : _ang_vel_dem_rads.x,
        rate_y  : _ang_vel_dem_rads.y,
        rate_z  : _ang_vel_dem_rads.z,
        target_x: _att_target_euler_rad.x,
        target_y: _att_target_euler_rad.y,
        target_z: _att_target_euler_rad.z
    };
    dataflash->WriteBlock(&pkt2, sizeof(pkt2));

    _log_dt = 0;
    _log_del_ang.zero();
    _log_del_vel.zero();
}

bool SoloGimbal::joints_near_limits()
{
    return fabsf(_measurement.joint_angles.x) > radians(40) || _measurement.joint_angles.y > radians(45) || _measurement.joint_angles.y < -radians(135);
}

AccelCalibrator* SoloGimbal::_acal_get_calibrator(uint8_t instance)
{
    if(instance==0 && (present() || _calibrator.get_status() == ACCEL_CAL_SUCCESS)) {
        return &_calibrator;
    } else {
        return nullptr;
    }
}

bool SoloGimbal::_acal_get_ready_to_sample()
{
    return _ang_vel_mag_filt < radians(10);
}

bool SoloGimbal::_acal_get_saving()
{
    return _gimbalParams.flashing();
}

void SoloGimbal::_acal_save_calibrations()
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

void SoloGimbal::gimbal_ang_vel_to_joint_rates(const Vector3f& ang_vel, Vector3f& joint_rates)
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

void SoloGimbal::joint_rates_to_gimbal_ang_vel(const Vector3f& joint_rates, Vector3f& ang_vel)
{
    float sin_theta = sinf(_measurement.joint_angles.y);
    float cos_theta = cosf(_measurement.joint_angles.y);

    float sin_phi = sinf(_measurement.joint_angles.x);
    float cos_phi = cosf(_measurement.joint_angles.x);

    ang_vel.x = cos_theta*joint_rates.x-sin_theta*cos_phi*joint_rates.z;
    ang_vel.y = joint_rates.y + sin_phi*joint_rates.z;
    ang_vel.z = sin_theta*joint_rates.x+cos_theta*cos_phi*joint_rates.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
