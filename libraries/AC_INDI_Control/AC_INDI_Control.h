#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_PID/AC_P.h>               // P library
#include <Filter/LowPassFilter2p.h>
#include <Filter/LowPassFilter.h>

class AC_INDI_Control
{
public:
    /// Constructor
    AC_INDI_Control(AP_AHRS_View& ahrs, const AP_InertialNav& inav);

    /* Do not allow copies */
    AC_INDI_Control(const AC_INDI_Control &other) = delete;
    AC_INDI_Control &operator=(const AC_INDI_Control&) = delete;

    static AC_INDI_Control *get_singleton();

    bool enabled(void) const;

    void write_log(void);

    // run position controller for xy axis in inertial frame
    // accept linear accleration feedforward in inertial frame
    void run_pos_vel_xy_controller(Vector3f target_point_m, Vector3f meas_dist_m, Vector3f vel_des_mps, Vector3f meas_vel_mps, Vector3f accel_des_mpss);

    // run position controller for z axis in inertial frame
    // altitude is above home in unit m 
    // accept linear accleration feedforward in inertial frame
    void run_pos_vel_z_controller(float target_alt_m, float meas_alt_m, float vel_des_mps, float meas_clmb_rate_mps, float accel_des_mpss, bool enable_xy, float accel_max_xy_mpss);

    // run attitude controller 
    Vector3f run_attitude_controller(Quaternion target, Quaternion meas);

    // run angular velocity controller 
    void run_angvel_controller(Vector3f target, Vector3f meas, Vector3f ang_acc_desired);

    // calculate estimated torque and thrust values using current motor speed
    void calculate_torque_thrust_est(void);

    // construct an attitude target using command specific thrust and 
    // yaw(heading) angle in radian
    Quaternion input_acc_des_euler_angle_yaw(float yaw_rad);

    // get throttle command that is send to mixer 
    const float& get_total_thrust_cmd_scaled() const {return _total_thrust_cmd_scaled; }

    // get torque command that is send to mixer
    const Vector3f& get_torque_cmd_scaled() const { return _torque_cmd_scaled; }


    const Vector3f& get_pos_target() const { return _pos_target_neu_m; }
    const Vector3f& get_vel_target() const { return _vel_target_neu_mps; }
    const Vector3f& get_lin_accel_target() const { return _lin_acc_target_ned_mpss; }
    const Vector3f& get_spec_thrst_cmd() const { return _spec_thrust_cmd_ned_mpss; }

    const Quaternion& get_attitude_quad_target() const { return _att_target_quat; }
    const Vector3f& get_ang_vel_target() const { return _ang_vel_target_radps; }
    const Vector3f& get_ang_acc_target() const { return _ang_acc_target_radpss; }
    const Vector3f& get_torque_cmd() const { return _torque_cmd_body_Nm; }


    // get P controllers
    AC_P& get_pos_xy_p() { return _p_pos_xy; }
    AC_P& get_vel_xy_p() { return _p_vel_xy; }
    AC_P& get_pos_z_p() { return _p_pos_z; }
    AC_P& get_pos_p() { return _p_pos_z; }

    AC_P& get_ang_roll_p() { return _p_angle_x; }
    AC_P& get_ang_pitch_p() { return _p_angle_y; }
    AC_P& get_ang_yaw_p() { return _p_angle_z; }
    AC_P& get_rate_roll_p() { return _p_ang_rate_x; }
    AC_P& get_rate_pitch_p() { return _p_ang_rate_y; }
    AC_P& get_rate_yaw_p() { return _p_ang_rate_z; }

    static const struct AP_Param::GroupInfo var_info[];    
protected:

    // add delta linear accleration to current specific thrust to obtain 
    // specific thrust command
    void indi_linear_accel(bool enable_xy, float accel_max_xy_mpss);

    // First limit magnitude of specific thrust command 
    // then limit xy axis of specific thrust command to no 
    // grater than 1 g or z axis command
    void limit_specific_thrust(float accel_max_xy);

    // convert specific thrust command to the scaled thrust command 
    void spec_thrust_to_scaled_thrust(void);

    // calculate attiude error 
    // same as thrust_heading_rotation_angles function in AC_AttitudeControl.cpp
    Vector3f calculate_att_error(Quaternion target, Quaternion meas);

    // add delta angular accleration to current torque to obtain torque command
    void indi_angular_accel(void);

    // set mixer input from the torque and thrust command 
    void scale_torque_cmd(void);

    // allocate torque and thrust cmd to the each motor thrust
    void control_allocation(void);

    // assign measured motor speed to _motor_speed_meas_radps 
    void get_motor_speed(void);

    
    // references to inertial nav and ahrs libraries
    AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;

    // Parameters 
    AP_Int8     enable_chan;

    // position and velocity P controller parameters
    AC_P        _p_pos_xy;
    AC_P        _p_vel_xy;
    AC_P        _p_pos_z;
    AC_P        _p_vel_z;

    // attitude and angular velocity P controller
    AC_P        _p_angle_x;
    AC_P        _p_angle_y;
    AC_P        _p_angle_z;
    AC_P        _p_ang_rate_x;
    AC_P        _p_ang_rate_y;
    AC_P        _p_ang_rate_z;

    // vehicle properties
    AP_Float    _mass_kg;                       // mass in kg               
    AP_Float    _moment_inertia_xy_kgm2;        // moment of inertia of xy axis in kg.m²
    AP_Float    _moment_inertia_z_kgm2;         // moment of inertia of z axis in kg.m²
    AP_Float    _arm_length_m;                  // distance to motors in m
    AP_Float    _thrust_coefficient;            // thrust coefficient in N/(rad/s)²
    AP_Float    _torque_coefficient;            // torque coefficient in Nm/(rad/s)²
    AP_Float    _throttle2motor_speed;          // coefficient between scaled throttle(between 0-1) command and motor speed in rad/s 
    AP_Float    _torque_est_filter_cutoff;      // rpm filter cutoff frequency for torque estimation in Hz
    AP_Float    _spec_thrust_est_filter_cutoff; // rpm filter cutoff frequency for specific thrust estimation in Hz
    AP_Float    _spec_thrust_cmd_filter_cutoff; // specific thrust command filter cutoff frequency in Hz
    AP_Float    _yaw_rate_filter_cutoff;        // torque command filter cutoff frequency in Hz

    Vector3f    _pos_target_neu_m;              // position target in NEU frame in m
    Vector3f    _vel_target_neu_mps;            // velocity target in NEU frame in m/s
    Vector3f    _lin_acc_target_ned_mpss;       // linear accleration target in NED frame m/s²
    Vector3f    _lin_acc_desired_ned_mpss;      // feedforwarded linear acceleration in NED frame m/s²
    Vector3f    _spec_thrust_cmd_ned_mpss;      // specifc thrust command in NED frame in m/s²
    Vector3f    _spec_thrust_est_ned_mpss;      // estimated specific thrust from motor speed in NED frame in m/s²
    float       _total_thrust_cmd_body_N;       // total thrust command in body frame in N
    float       _total_thrust_cmd_scaled;       // scaled total thrust command between 0 ~ 1
    

    Quaternion  _att_target_quat;               // target attitude defined using desired yaw and specific thrust command
    float       _att_target_euler_angle_yaw_rad;// target attitude used only for target heading in rad
    Vector3f    _ang_vel_target_radps;          // angular velocity target in body frame in rad/s
    Vector3f    _ang_acc_target_radpss;         // angular acceleration target in body frame in rad/s²
    Vector3f    _ang_acc_desired_radpss;        // feedforwarded angular acceleration in NED frame rad/s²
    Vector3f    _torque_cmd_body_Nm;            // torque command in body frame in Nm
    Vector3f    _torque_cmd_scaled ;            // scaled torque command between -1 ~ +1
    Vector3f    _torque_est_body_Nm;            // estimated torque from motor speed in body frame in Nm
    
    float _motor_cmd_radps[4];                  // motor command in rad/s   !!!NOT USED
    float _motor_cmd_scaled[4];                 // scaled motor command 0-1 !!!NOT USED
    float _motor_speed_meas_radps[4];           // current motor speed in rad/s

    LowPassFilterVector3f _torque_est_filter;
    LowPassFilter2pFloat _spec_thrust_est_filter;
    LowPassFilterVector3f _spec_thrust_cmd_filter;
    LowPassFilterFloat _yaw_rate_filter;
private:
    static AC_INDI_Control *_singleton;
};

namespace AP {
    AC_INDI_Control &indi_control();
};
