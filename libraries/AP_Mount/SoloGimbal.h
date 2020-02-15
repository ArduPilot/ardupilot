/************************************************************
* SoloGimbal -- library to control a 3 axis rate gimbal.     *
*                                                           *
* Author:  Arthur Benemann, Paul Riseborough;               *
*                                                           *
************************************************************/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#if AP_AHRS_NAVEKF_AVAILABLE

#include "AP_Mount.h"
#include "SoloGimbalEKF.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AccelCal/AP_AccelCal.h>

#include "SoloGimbal_Parameters.h"

enum gimbal_state_t {
    GIMBAL_STATE_NOT_PRESENT = 0,
    GIMBAL_STATE_PRESENT_INITIALIZING,
    GIMBAL_STATE_PRESENT_ALIGNING,
    GIMBAL_STATE_PRESENT_RUNNING
};

enum gimbal_mode_t {
    GIMBAL_MODE_IDLE = 0,
    GIMBAL_MODE_POS_HOLD,
    GIMBAL_MODE_POS_HOLD_FF,
    GIMBAL_MODE_STABILIZE
};

class SoloGimbal : AP_AccelCal_Client
{
public:
    //Constructor
    SoloGimbal() :
        _ekf(),
        _state(GIMBAL_STATE_NOT_PRESENT),
        _vehicle_yaw_rate_ef_filt(0.0f),
        _vehicle_to_gimbal_quat(),
        _vehicle_to_gimbal_quat_filt(),
        _filtered_joint_angles(),
        _last_report_msg_ms(0),
        _max_torque(5000.0f),
        _ang_vel_mag_filt(0),
        _lockedToBody(false),
        _log_dt(0),
        _log_del_ang(),
        _log_del_vel()
    {
        AP_AccelCal::register_client(this);
    }

    void    update_target(const Vector3f &newTarget);
    void    receive_feedback(mavlink_channel_t chan, const mavlink_message_t &msg);

    void update_fast();

    bool present();
    bool aligned();

    void set_lockedToBody(bool val) { _lockedToBody = val; }

    void write_logs();

    float get_log_dt() { return _log_dt; }

    void disable_torque_report() { _gimbalParams.set_param(GMB_PARAM_GMB_SND_TORQUE, 0); }
    void fetch_params() { _gimbalParams.fetch_params(); }

    void handle_param_value(const mavlink_message_t &msg) {
        _gimbalParams.handle_param_value(msg);
    }

private:
    // private methods
    void update_estimators();
    void send_controls(mavlink_channel_t chan);
    void extract_feedback(const mavlink_gimbal_report_t& report_msg);
    void update_joint_angle_est();

    Vector3f get_ang_vel_dem_yaw(const Quaternion &quatEst);
    Vector3f get_ang_vel_dem_tilt(const Quaternion &quatEst);
    Vector3f get_ang_vel_dem_feedforward(const Quaternion &quatEst);
    Vector3f get_ang_vel_dem_gyro_bias();
    Vector3f get_ang_vel_dem_body_lock();

    void gimbal_ang_vel_to_joint_rates(const Vector3f& ang_vel, Vector3f& joint_rates);
    void joint_rates_to_gimbal_ang_vel(const Vector3f& joint_rates, Vector3f& ang_vel);

    void readVehicleDeltaAngle(uint8_t ins_index, Vector3f &dAng);

    void _acal_save_calibrations() override;
    bool _acal_get_ready_to_sample() override;
    bool _acal_get_saving() override;
    AccelCalibrator* _acal_get_calibrator(uint8_t instance) override;

    gimbal_mode_t get_mode();

    bool joints_near_limits();

    // private member variables
    SoloGimbalEKF            _ekf;      // state of small EKF for gimbal

    gimbal_state_t _state;

    struct {
        float delta_time;
        Vector3f delta_angles;
        Vector3f delta_velocity;
        Vector3f joint_angles;
    } _measurement;

    float _vehicle_yaw_rate_ef_filt;

    static const uint8_t _compid = MAV_COMP_ID_GIMBAL;

    // joint angle filter states
    Vector3f _vehicle_delta_angles;

    Quaternion _vehicle_to_gimbal_quat;
    Quaternion _vehicle_to_gimbal_quat_filt;
    Vector3f _filtered_joint_angles;

    uint32_t _last_report_msg_ms;

    float _max_torque;

    float _ang_vel_mag_filt;

    Vector3f    _ang_vel_dem_rads;       // rad/s
    Vector3f    _att_target_euler_rad;   // desired earth-frame roll, tilt and pan angles in radians

    bool _lockedToBody;

    SoloGimbal_Parameters _gimbalParams;

    AccelCalibrator _calibrator;

    float _log_dt;
    Vector3f _log_del_ang;
    Vector3f _log_del_vel;
};

#endif // AP_AHRS_NAVEKF_AVAILABLE
