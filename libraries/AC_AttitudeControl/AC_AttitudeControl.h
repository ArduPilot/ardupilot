// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeControl_H
#define AC_AttitudeControl_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <DataFlash.h>
#include <AC_PID.h>
#include <AC_P.h>

// To-Do: change the name or move to AP_Math?
#define AC_ATTITUDE_CONTROL_DEGX100 5729.57795f                 // constant to convert from radians to centi-degrees
#define AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT         18000   // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT          9000    // maximum rotation rate on yaw axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT            1000    // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sed * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT        0       // default maximum acceleration for roll/pitch axis in centi-degrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT         0       // default maximum acceleration for yaw axis in centi-degrees/sec/sec

#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          5000.0f // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         4500.0f // body-frame rate controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_YAW_CONTROLLER_OUT_MAX        4500.0f // earth-frame angle controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX          4500.0f // earth-frame angle controller maximum input angle (To-Do: replace with reference to aparm.angle_max)

#define AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX  3000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX 3000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX   1000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX  1000.0f // earth-frame rate stabilize controller's maximum overshoot angle

#define AC_ATTITUDE_100HZ_DT                            0.0100f // delta time in seconds for 100hz update rate
#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

#define AC_ATTITUDE_RATE_RP_PID_DTERM_FILTER            20      // D-term filter rate cutoff frequency for Roll and Pitch rate controllers
#define AC_ATTITUDE_RATE_Y_PID_DTERM_FILTER             5       // D-term filter rate cutoff frequency for Yaw rate controller

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          0       // body-frame rate feedforward enabled by default

class AC_AttitudeControl {
public:
	AC_AttitudeControl( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_Motors& motors,
                        AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw
                        ) :
		_ahrs(ahrs),
        _aparm(aparm),
        _motors(motors),
        _p_angle_roll(pi_angle_roll),
        _p_angle_pitch(pi_angle_pitch),
        _p_angle_yaw(pi_angle_yaw),
        _pid_rate_roll(pid_rate_roll),
        _pid_rate_pitch(pid_rate_pitch),
        _pid_rate_yaw(pid_rate_yaw),
        _dt(AC_ATTITUDE_100HZ_DT),
        _angle_boost(0),
        _acro_angle_switch(0)
		{
			AP_Param::setup_object_defaults(this, var_info);

			// initialise flags
			_flags.limit_angle_to_rate_request = true;
		}

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl() {}

    //
    // initialisation functions
    //

    // set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec);

    // relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
    void relax_bf_rate_controller();

    // set_yaw_target_to_current_heading - sets yaw target to current heading
    void set_yaw_target_to_current_heading() { _angle_ef_target.z = _ahrs.yaw_sensor; }

    //
    // methods to be called by upper controllers to request and implement a desired attitude
    //

    // angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
    //      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
    void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);

    // angle_ef_roll_pitch_rate_ef_yaw - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
    void angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef);

    // angle_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw angle (all earth frame)
    //  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the YAW_SLEW parameter
    void angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw);

    // rate_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all earth frame)
    void rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef);

    // rate_bf_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all body frame)
    void rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf);

    //
    // rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
    //      should be called at 100hz or more
    //
    virtual void rate_controller_run();

    //
    // earth-frame <-> body-frame conversion functions
    //
    // frame_conversion_ef_to_bf - converts earth frame angles or rates to body frame
    void frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f &bf_vector);

    // frame_conversion_bf_to_ef - converts body frame angles or rates to earth frame
    //  returns false if conversion fails due to gimbal lock
    bool frame_conversion_bf_to_ef(const Vector3f& bf_vector, Vector3f &ef_vector);

    //
    // public accessor functions
    //

    // limit_angle_to_rate_request
    void limit_angle_to_rate_request(bool limit_request) { _flags.limit_angle_to_rate_request = limit_request; }

    // angle_ef_targets - returns angle controller earth-frame targets (for reporting)
    const Vector3f& angle_ef_targets() const { return _angle_ef_target; }

    // rate_bf_targets - gets rate controller body-frame targets
    void rate_bf_roll_target(float rate_cds) { _rate_bf_target.x = rate_cds; }
    void rate_bf_pitch_target(float rate_cds) { _rate_bf_target.y = rate_cds; }
    void rate_bf_yaw_target(float rate_cds) { _rate_bf_target.z = rate_cds; }

    // enable_bf_feedforward - enable or disable body-frame feed forward
    void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled = enable_or_disable; }

    // enable_bf_feedforward - enable or disable body-frame feed forward
    void accel_limiting(bool enable_or_disable);

    //
    // throttle functions
    //

     // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
     // provide 0 to cut motors
     void set_throttle_out(int16_t throttle_pwm, bool apply_angle_boost);

     // angle_boost - accessor for angle boost so it can be logged
     int16_t angle_boost() const { return _angle_boost; }

    //
    // helper functions
    //

    // lean_angle_max - maximum lean angle of the copter in centi-degrees
    int16_t lean_angle_max() const { return _aparm.angle_max; }

    // sqrt_controller - response based on the sqrt of the error instead of the more common linear response
    static float sqrt_controller(float error, float p, float second_ord_lim);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // attitude control flags
    struct AttControlFlags {
        uint8_t limit_angle_to_rate_request :   1;  // 1 if the earth frame angle controller is limiting it's maximum rate request
    } _flags;
    
    // update_ef_roll_angle_and_error - update _angle_ef_target.x using an earth frame roll rate request
    void update_ef_roll_angle_and_error(float roll_rate_ef, Vector3f &angle_ef_error, float overshoot_max);

    // update_ef_pitch_angle_and_error - update _angle_ef_target.y using an earth frame pitch rate request
    void update_ef_pitch_angle_and_error(float pitch_rate_ef, Vector3f &angle_ef_error, float overshoot_max);

    // update_ef_yaw_angle_and_error - update _angle_ef_target.z using an earth frame yaw rate request
    void update_ef_yaw_angle_and_error(float yaw_rate_ef, Vector3f &angle_ef_error, float overshoot_max);

    // integrate_bf_rate_error_to_angle_errors - calculates body frame angle errors
    //   body-frame feed forward rates (centi-degrees / second) taken from _angle_bf_error
    //   angle errors in centi-degrees placed in _angle_bf_error
    void integrate_bf_rate_error_to_angle_errors();

    // update_rate_bf_targets - converts body-frame angle error to body-frame rate targets for roll, pitch and yaw axis
    //   targets rates in centi-degrees taken from _angle_bf_error
    //   results in centi-degrees/sec put into _rate_bf_target
    void update_rate_bf_targets();

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in centi-degrees/sec) for roll, pitch and yaw
    float rate_bf_to_motor_roll(float rate_target_cds);
    float rate_bf_to_motor_pitch(float rate_target_cds);
    virtual float rate_bf_to_motor_yaw(float rate_target_cds);

    //
    // throttle methods
    //

    // get_angle_boost - calculate total body frame throttle required to produce the given earth frame throttle
    virtual int16_t get_angle_boost(int16_t throttle_pwm);

    // references to external libraries
    const AP_AHRS&      _ahrs;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;
    AC_P&	            _p_angle_roll;
    AC_P&	            _p_angle_pitch;
    AC_P&	            _p_angle_yaw;
    AC_PID&             _pid_rate_roll;
    AC_PID&             _pid_rate_pitch;
    AC_PID&             _pid_rate_yaw;

    // parameters
    AP_Float            _angle_rate_rp_max;     // maximum rate request output from the earth-frame angle controller for roll and pitch axis
    AP_Float            _angle_rate_y_max;      // maximum rate request output from the earth-frame angle controller for yaw axis
    AP_Float            _slew_yaw;              // maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    AP_Float            _accel_rp_max;          // maximum rotation acceleration for earth-frame roll and pitch axis
    AP_Float            _accel_y_max;           // maximum rotation acceleration for earth-frame yaw axis
    AP_Int8             _rate_bf_ff_enabled;    // Enable/Disable body frame rate feed forward

    // internal variables
    // To-Do: make rate targets a typedef instead of Vector3f?
    float               _dt;                    // time delta in seconds
    Vector3f            _angle_ef_target;       // angle controller earth-frame targets
    Vector3f            _angle_bf_error;        // angle controller body-frame error
    Vector3f            _rate_bf_target;        // rate controller body-frame targets
    Vector3f            _rate_ef_desired;       // earth-frame feed forward rates
    Vector3f            _rate_bf_desired;       // body-frame feed forward rates
    int16_t             _angle_boost;           // used only for logging
    int16_t             _acro_angle_switch;           // used only for logging
};

#define AC_ATTITUDE_CONTROL_LOG_FORMAT(msg) { msg, sizeof(AC_AttitudeControl::log_Attitude),	\
                            "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" }

#endif //AC_AttitudeControl_H
