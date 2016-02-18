// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeControl_H
#define AC_AttitudeControl_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

// TODO: change the name or move to AP_Math? eliminate in favor of degrees(100)?
#define AC_ATTITUDE_CONTROL_DEGX100                           5729.57795f      // constant to convert from radians to centidegrees

#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS             radians(40.0f)   // minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS             radians(720.0f)  // maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS              radians(10.0f)   // minimum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS              radians(360.0f)  // maximum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS              1000      // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sed * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS         110000.0f // default maximum acceleration for roll/pitch axis in centidegrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS          27000.0f  // default maximum acceleration for yaw axis in centidegrees/sec/sec

#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          5000.0f // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         4500.0f // body-frame rate controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_YAW_CONTROLLER_OUT_MAX        4500.0f // earth-frame angle controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX          4500.0f // earth-frame angle controller maximum input angle (To-Do: replace with reference to aparm.angle_max)

#define AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD  radians(300.0f) // earth-frame rate stabilize controller's maximum overshoot angle (never limited)
#define AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX_RAD radians(300.0f) // earth-frame rate stabilize controller's maximum overshoot angle (never limited)
#define AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD   radians(10.0f)  // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD  radians(30.0f)  // earth-frame rate stabilize controller's maximum overshoot angle

#define AC_ATTITUDE_100HZ_DT                            0.0100f // delta time in seconds for 100hz update rate
#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          1       // body-frame rate feedforward enabled by default

#define AC_ATTITUDE_CONTROL_ALTHOLD_LEANANGLE_FILT_HZ   1.0f    // filter (in hz) of throttle filter used to limit lean angle so that vehicle does not lose altitude

class AC_AttitudeControl {
public:
    AC_AttitudeControl( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_Motors& motors,
                        AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw
                        ) :
        _dt(AC_ATTITUDE_400HZ_DT),
        _angle_boost(0),
        _att_ctrl_use_accel_limit(true),
        _throttle_in_filt(AC_ATTITUDE_CONTROL_ALTHOLD_LEANANGLE_FILT_HZ),
        _ahrs(ahrs),
        _aparm(aparm),
        _motors(motors),
        _p_angle_roll(pi_angle_roll),
        _p_angle_pitch(pi_angle_pitch),
        _p_angle_yaw(pi_angle_yaw),
        _pid_rate_roll(pid_rate_roll),
        _pid_rate_pitch(pid_rate_pitch),
        _pid_rate_yaw(pid_rate_yaw)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // Empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl() {}

    // Set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec);

    // Gets the roll acceleration limit in centidegrees/s/s
    float get_accel_roll_max() { return _accel_roll_max; }

    // Sets the roll acceleration limit in centidegrees/s/s
    void set_accel_roll_max(float accel_roll_max) { _accel_roll_max = accel_roll_max; }

    // Sets and saves the roll acceleration limit in centidegrees/s/s
    void save_accel_roll_max(float accel_roll_max) { _accel_roll_max = accel_roll_max; _accel_roll_max.save(); }

    // Sets the pitch acceleration limit in centidegrees/s/s
    float get_accel_pitch_max() { return _accel_pitch_max; }

    // Sets the pitch acceleration limit in centidegrees/s/s
    void set_accel_pitch_max(float accel_pitch_max) { _accel_pitch_max = accel_pitch_max; }

    // Sets and saves the pitch acceleration limit in centidegrees/s/s
    void save_accel_pitch_max(float accel_pitch_max) { _accel_pitch_max = accel_pitch_max; _accel_pitch_max.save(); }

    // Gets the yaw acceleration limit in centidegrees/s/s
    float get_accel_yaw_max() { return _accel_yaw_max; }

    // Sets the yaw acceleration limit in centidegrees/s/s
    void set_accel_yaw_max(float accel_yaw_max) { _accel_yaw_max = accel_yaw_max; }

    // Sets and saves the yaw acceleration limit in centidegrees/s/s
    void save_accel_yaw_max(float accel_yaw_max) { _accel_yaw_max = accel_yaw_max; _accel_yaw_max.save(); }

    // Ensure body-frame rate controller has zero errors to relax rate controller output
    void relax_bf_rate_controller();

    // Sets yaw target to vehicle heading
    void set_yaw_target_to_current_heading() { _att_target_euler_rad.z = _ahrs.yaw; }

    // Shifts earth frame yaw target by yaw_shift_cd. yaw_shift_cd should be in centidegrees and is added to the current target heading
    void shift_ef_yaw_target(float yaw_shift_cd);

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_euler_rate_yaw_smooth(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds, float smoothing_gain);

    // Command an euler roll and pitch angle and an euler yaw rate
    void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);

    // Command an euler roll, pitch and yaw angle
    void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

    // Command an euler roll, pitch, and yaw rate
    void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds);

    // Command a quaternion attitude and a body-frame angular velocity
    void input_att_quat_bf_ang_vel(const Quaternion& att_target_quat, const Vector3f& att_target_ang_vel_rads);

    // Command an angular velocity
    virtual void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Run angular velocity controller and send outputs to the motors
    virtual void rate_controller_run();

    // Convert a 321-intrinsic euler angle derivative to an angular velocity vector
    void euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads);

    // Convert an angular velocity vector to a 321-intrinsic euler angle derivative
    // Returns false if the vehicle is pitched 90 degrees up or down
    bool ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads);

    // Configures whether the attitude controller should limit the rate demand to constrain angular acceleration
    void limit_angle_to_rate_request(bool limit_request) { _att_ctrl_use_accel_limit = limit_request; }

    // Return 321-intrinsic euler angles in centidegrees representing the rotation from NED earth frame to the
    // attitude controller's reference attitude.
    Vector3f get_att_target_euler_cd() const { return _att_target_euler_rad*degrees(100.0f); }

    // Return a rotation vector in centidegrees representing the rotation from vehicle body frame to the
    // attitude controller's reference attitude.
    Vector3f get_att_error_rot_vec_cd() const { return _att_error_rot_vec_rad*degrees(100.0f); }

    // Set x-axis angular velocity reference in centidegrees/s
    void rate_bf_roll_target(float rate_cds) { _ang_vel_target_rads.x = radians(rate_cds*0.01f); }

    // Set y-axis angular velocity reference in centidegrees/s
    void rate_bf_pitch_target(float rate_cds) { _ang_vel_target_rads.y = radians(rate_cds*0.01f); }

    // Set z-axis angular velocity reference in centidegrees/s
    void rate_bf_yaw_target(float rate_cds) { _ang_vel_target_rads.z = radians(rate_cds*0.01f); }

    // Return roll rate step size in centidegrees/s that results in maximum output after 4 time steps
    float max_rate_step_bf_roll();

    // Return pitch rate step size in centidegrees/s that results in maximum output after 4 time steps
    float max_rate_step_bf_pitch();

    // Return yaw rate step size in centidegrees/s that results in maximum output after 4 time steps
    float max_rate_step_bf_yaw();

    // Return roll step size in centidegrees that results in maximum output after 4 time steps
    float max_angle_step_bf_roll() { return max_rate_step_bf_roll()/_p_angle_roll.kP(); }

    // Return pitch step size in centidegrees that results in maximum output after 4 time steps
    float max_angle_step_bf_pitch() { return max_rate_step_bf_pitch()/_p_angle_pitch.kP(); }

    // Return yaw step size in centidegrees that results in maximum output after 4 time steps
    float max_angle_step_bf_yaw() { return max_rate_step_bf_yaw()/_p_angle_yaw.kP(); }

    // Return reference angular velocity used in the angular velocity controller
    Vector3f rate_bf_targets() const { return _ang_vel_target_rads*degrees(100.0f); }

    // Enable or disable body-frame feed forward
    void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled = enable_or_disable; }

    // Enable or disable body-frame feed forward and save
    void bf_feedforward_save(bool enable_or_disable) { _rate_bf_ff_enabled.set_and_save(enable_or_disable); }

    // Return body-frame feed forward setting
    bool get_bf_feedforward() { return _rate_bf_ff_enabled; }

    // Enable or disable body-frame feed forward
    void accel_limiting(bool enable_or_disable);

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff);

    // Set output throttle and disable stabilization
    void set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filt_cutoff);

    // Return throttle increase applied for tilt compensation
    int16_t angle_boost() const { return _angle_boost; }

    // Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
    virtual float get_althold_lean_angle_max() const = 0;

    // Return configured tilt angle limit in centidegrees/s
    float lean_angle_max() const { return _aparm.angle_max; }

    // Proportional controller with piecewise sqrt sections to constrain second derivative
    static float sqrt_controller(float error, float p, float second_ord_lim);

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Retrieve a rotation matrix from the vehicle body frame to NED earth frame
    void get_rotation_vehicle_to_ned(Matrix3f& m);

    // Retrieve a rotation matrix from NED earth frame to the vehicle body frame
    void get_rotation_ned_to_vehicle(Matrix3f& m);

    // Retrieve a rotation matrix from reference (setpoint) body frame to NED earth frame
    void get_rotation_reference_to_ned(Matrix3f& m);

    // Retrieve a rotation matrix from NED earth frame to reference (setpoint) body frame
    void get_rotation_ned_to_reference(Matrix3f& m);

    // Retrieve a rotation matrix from vehicle body frame to reference (setpoint) body frame
    void get_rotation_vehicle_to_reference(Matrix3f& m);

    // Retrieve a rotation matrix from reference (setpoint) body frame to vehicle body frame
    void get_rotation_reference_to_vehicle(Matrix3f& m);

    // Update _att_target_euler_rad.x by integrating a 321-intrinsic euler roll angle derivative
    void update_att_target_and_error_roll(float euler_roll_rate_rads, Vector3f &att_error_euler_rad, float overshoot_max_rad);

    // Update _att_target_euler_rad.y by integrating a 321-intrinsic euler pitch angle derivative
    void update_att_target_and_error_pitch(float euler_pitch_rate_rads, Vector3f &att_error_euler_rad, float overshoot_max_rad);

    // Update _att_target_euler_rad.z by integrating a 321-intrinsic euler yaw angle derivative
    void update_att_target_and_error_yaw(float euler_yaw_rate_rads, Vector3f &att_error_euler_rad, float overshoot_max_rad);

    // Integrate vehicle rate into _att_error_rot_vec_rad
    void integrate_bf_rate_error_to_angle_errors();

    // Update _ang_vel_target_rads using _att_error_rot_vec_rad
    void update_ang_vel_target_from_att_error();

    // Run the roll angular velocity PID controller and return the output
    float rate_bf_to_motor_roll(float rate_target_rads);

    // Run the pitch angular velocity PID controller and return the output
    float rate_bf_to_motor_pitch(float rate_target_rads);

    // Run the yaw angular velocity PID controller and return the output
    virtual float rate_bf_to_motor_yaw(float rate_target_rads);

    // Compute a throttle value that is adjusted for the tilt angle of the vehicle
    virtual float get_boosted_throttle(float throttle_in) = 0;

    // Return angle in radians to be added to roll angle. Used by heli to counteract
    // tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
    virtual float get_roll_trim_rad() { return 0;}

    // Return the roll axis acceleration limit in radians/s/s
    float get_accel_roll_max_radss() { return radians(_accel_roll_max*0.01f); }

    // Return the pitch axis acceleration limit in radians/s/s
    float get_accel_pitch_max_radss() { return radians(_accel_pitch_max*0.01f); }

    // Return the yaw axis acceleration limit in radians/s/s
    float get_accel_yaw_max_radss() { return radians(_accel_yaw_max*0.01f); }

    // Return the yaw slew rate limit in radians/s
    float get_slew_yaw_rads() { return radians(_slew_yaw*0.01f); }

    // Return the tilt angle limit in radians
    float get_tilt_limit_rad() { return radians(_aparm.angle_max*0.01f); }

    // Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    AP_Float            _slew_yaw;

    // Maximum rotation acceleration for earth-frame roll axis
    AP_Float            _accel_roll_max;

    // Maximum rotation acceleration for earth-frame pitch axis
    AP_Float            _accel_pitch_max;

    // Maximum rotation acceleration for earth-frame yaw axis
    AP_Float            _accel_yaw_max;

    // Enable/Disable body frame rate feed forward
    AP_Int8             _rate_bf_ff_enabled;

    // Enable/Disable angle boost
    AP_Int8             _angle_boost_enabled;

    // Intersampling period in seconds
    float               _dt;

    // This represents a 321-intrinsic rotation from NED frame to the reference (setpoint)
    // attitude used in the attitude controller, in radians. Formerly _angle_ef_target.
    Vector3f            _att_target_euler_rad;

    // This represents an euler axis-angle rotation vector from the vehicle’s
    // estimated attitude to the reference (setpoint) attitude used in the attitude
    // controller, in radians in the vehicle body frame of reference. Formerly
    // _angle_bf_error.
    Vector3f            _att_error_rot_vec_rad;

    // This represents the angular velocity of the reference (setpoint) attitude used in
    // the attitude controller as 321-intrinsic euler angle derivatives, in radians per
    // second. Formerly _rate_ef_desired.
    Vector3f            _att_target_euler_rate_rads;

    // This represents the angular velocity of the reference (setpoint) attitude used in
    // the attitude controller as an angular velocity vector, in radians per second in
    // the reference attitude frame. Formerly _rate_bf_desired.
    Vector3f            _att_target_ang_vel_rads;

    // This represents the reference (setpoint) angular velocity used in the angular
    // velocity controller, in radians per second. Formerly _rate_bf_target.
    Vector3f            _ang_vel_target_rads;


    // This represents the throttle increase applied for tilt compensation.
    // Used only for logging.
    int16_t             _angle_boost;

    // Specifies whether the attitude controller should use the acceleration limit
    bool                _att_ctrl_use_accel_limit;

    // Filtered throttle input - used to limit lean angle when throttle is saturated
    LowPassFilterFloat  _throttle_in_filt;

    // References to external libraries
    const AP_AHRS&      _ahrs;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;
    AC_P&               _p_angle_roll;
    AC_P&               _p_angle_pitch;
    AC_P&               _p_angle_yaw;
    AC_PID&             _pid_rate_roll;
    AC_PID&             _pid_rate_pitch;
    AC_PID&             _pid_rate_yaw;
};

#define AC_ATTITUDE_CONTROL_LOG_FORMAT(msg) { msg, sizeof(AC_AttitudeControl::log_Attitude),	\
                            "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" }

#endif //AC_AttitudeControl_H
