#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>               // P library
#include <Filter/LowPassFilter2p.h>
#include <Filter/LowPassFilter.h>

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_INDI_ENABLED
    #define CUSTOMCONTROL_INDI_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_INDI_ENABLED

class AC_CustomControl_INDI : public AC_CustomControl_Backend {
public:
    AC_CustomControl_INDI(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // run attitude controller 
    Vector3f run_attitude_controller(Quaternion target, Quaternion meas);

    // run angular velocity controller 
    void run_angvel_controller(Vector3f target, Vector3f meas, Vector3f ang_acc_desired);

    // calculate estimated torque and thrust values using current motor speed
    void calculate_torque_thrust_est(void);

    // calculate attiude error 
    // same as thrust_heading_rotation_angles function in AC_AttitudeControl.cpp
    Vector3f calculate_att_error(Quaternion target, Quaternion meas);

    // add delta angular accleration to current torque to obtain torque command
    void indi_angular_accel(void);

    // set mixer input from the torque and thrust command 
    void scale_torque_cmd(void);

    // assign measured motor speed to _motor_speed_meas_radps 
    void get_motor_speed(void);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    // attitude and angular velocity P controller
    AC_P        _p_angle_x;
    AC_P        _p_angle_y;
    AC_P        _p_angle_z;
    AC_P        _p_ang_rate_x;
    AC_P        _p_ang_rate_y;
    AC_P        _p_ang_rate_z;

    AP_Float    _moment_inertia_xy_kgm2;        // moment of inertia of xy axis in kg.m²
    AP_Float    _moment_inertia_z_kgm2;         // moment of inertia of z axis in kg.m²
    AP_Float    _arm_length_m;                  // distance to motors in m
    AP_Float    _thrust_coefficient;            // thrust coefficient in N/(rad/s)²
    AP_Float    _torque_coefficient;            // torque coefficient in Nm/(rad/s)²
    AP_Float    _throttle2motor_speed;          // coefficient between scaled throttle(between 0-1) command and motor speed in rad/s 
    AP_Float    _ang_acc_filter_cutoff;         // angular acceleration filter cutoff frequency in Hz
    AP_Float    _torque_est_filter_cutoff;      // rpm filter cutoff frequency for torque estimation in Hz
    AP_Float    _yaw_rate_filter_cutoff;        // torque command filter cutoff frequency in Hz

    Quaternion  _att_target_quat;               // target attitude defined using desired yaw and specific thrust command
    float       _att_target_euler_angle_yaw_rad;// target attitude used only for target heading in rad
    Vector3f    _ang_vel_target_radps;          // angular velocity target in body frame in rad/s
    Vector3f    _ang_acc_target_radpss;         // angular acceleration target in body frame in rad/s²
    Vector3f    _ang_acc_desired_radpss;        // feedforwarded angular acceleration in NED frame rad/s²
    Vector3f    _torque_cmd_body_Nm;            // torque command in body frame in Nm
    Vector3f    _torque_cmd_scaled ;            // scaled torque command between -1 ~ +1
    Vector3f    _torque_est_body_Nm;            // estimated torque from motor speed in body frame in Nm

    float _motor_speed_meas_radps[4];           // current motor speed in rad/s

    
    LowPassFilterVector3f _ang_acc_filter;
    LowPassFilterVector3f _torque_est_filter;
    LowPassFilterFloat _yaw_rate_filter;
    Vector3f _gyro_prev;
};

#endif
