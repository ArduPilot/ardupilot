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
#include <APM_PI.h>

// To-Do: change the name or move to AP_Math?
#define AC_ATTITUDE_CONTROL_DEGX100 5729.57795f                 // constant to convert from radians to centi-degrees
#define AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT         18000   // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT          18000   // maximum rotation rate on yaw axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT            6000    // default yaw slew rate in centi-degrees/sec (i.e. maximum yaw target change in 1second)
#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          5000.0f // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         4500.0f // body-frame rate controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_YAW_CONTROLLER_OUT_MAX        4500.0f // earth-frame angle controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX          4500.0f // earth-frame angle controller maximum input angle (To-Do: replace with reference to aparm.angle_max)

#define AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX  3000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX 3000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX   1000.0f // earth-frame rate stabilize controller's maximum overshoot angle

#define AC_ATTITUDE_100HZ_DT                            0.0100f // delta time in seconds for 100hz update rate
#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

class AC_AttitudeControl {
public:
	AC_AttitudeControl( AP_AHRS &ahrs,
                        AP_InertialSensor& ins,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_Motors& motors,
                        APM_PI& pi_angle_roll, APM_PI& pi_angle_pitch, APM_PI& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw,
                        int16_t& motor_roll, int16_t& motor_pitch, int16_t& motor_yaw, int16_t& motor_throttle
                        ) :
		_ahrs(ahrs),
        _ins(ins),
        _aparm(aparm),
        _motors(motors),
        _pi_angle_roll(pi_angle_roll),
        _pi_angle_pitch(pi_angle_pitch),
        _pi_angle_yaw(pi_angle_yaw),
        _pid_rate_roll(pid_rate_roll),
        _pid_rate_pitch(pid_rate_pitch),
        _pid_rate_yaw(pid_rate_yaw),
        _motor_roll(motor_roll),
        _motor_pitch(motor_pitch),
        _motor_yaw(motor_yaw),
        _motor_throttle(motor_throttle),
        _dt(AC_ATTITUDE_100HZ_DT),
        _angle_boost(0)
		{
			AP_Param::setup_object_defaults(this, var_info);

			// initialise flags
			_flags.limit_angle_to_rate_request = true;
		}

    //
    // initialisation functions
    //

    // set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec) { _dt = delta_sec; }

    // init_targets - resets target angles to current angles
    void init_targets();

    //
    // methods to be called by upper controllers to request and implement a desired attitude
    //

    // angleef_rp_rateef_y - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
    void angleef_rp_rateef_y(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef);

    // angleef_rpy - attempts to maintain a roll, pitch and yaw angle (all earth frame)
    //  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the YAW_SLEW parameter
    void angleef_rpy(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw);

    // rateef_rpy - attempts to maintain a roll, pitch and yaw rate (all earth frame)
    void rateef_rpy(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef);

    // ratebf_rpy - attempts to maintain a roll, pitch and yaw rate (all body frame)
    void ratebf_rpy(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf);

    //
    // rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
    //      should be called at 100hz or more
    //
    virtual void rate_controller_run();

    //
    // earth-frame <-> body-frame conversion functions
    //
    // rate_ef_targets_to_bf - converts earth frame rate targets to body frame rate targets
    void rate_ef_targets_to_bf(const Vector3f& rate_ef_target, Vector3f &rate_bf_target);

    // rate_bf_targets_to_ef - converts body frame rate targets to earth frame rate targets
    void rate_bf_targets_to_ef(const Vector3f& rate_ef_target, Vector3f &rate_bf_target);

    //
    // public accessor functions
    //

    // limit_angle_to_rate_request
    void limit_angle_to_rate_request(bool limit_request) { _flags.limit_angle_to_rate_request = limit_request; }

    // angle_ef_targets - returns angle controller earth-frame targets (for reporting)
    Vector3f angle_ef_targets() const { return _angle_ef_target; }

    // rate_bf_targets - gets rate controller body-frame targets
    void rate_bf_roll_target(float rate_cds) { _rate_bf_target.x = rate_cds; }
    void rate_bf_pitch_target(float rate_cds) { _rate_bf_target.y = rate_cds; }
    void rate_bf_yaw_target(float rate_cds) { _rate_bf_target.z = rate_cds; }

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
    int16_t lean_angle_max() { return _aparm.angle_max; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // attitude control flags
    struct AttControlFlags {
        uint8_t limit_angle_to_rate_request :   1;  // 1 if the earth frame angle controller is limiting it's maximum rate request
    } _flags;

    // update_stab_rate_bf_errors - calculates body frame angle errors
    //   body-frame feed forward rates (centi-degrees / second) taken from _angle_bf_error
    //   angle errors in centi-degrees placed in _angle_bf_error
    void update_stab_rate_bf_errors();

    // update_stab_rate_bf_targets - converts body-frame angle error to body-frame rate targets for roll, pitch and yaw axis
    //   targets rates in centi-degrees taken from _angle_bf_error
    //   results in centi-degrees/sec put into _rate_bf_target
    void update_stab_rate_bf_targets();

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

    //
    // logging
    //

    // log data on internal state of the controller. Called at 10Hz
    void log_data(DataFlash_Class &dataflash, uint8_t msgid);

    // dataflash logging packet
    struct PACKED log_Attitude {
        LOG_PACKET_HEADER;
        int16_t roll_in;
        int16_t roll;
        int16_t pitch_in;
        int16_t pitch;
        int16_t yaw_in;
        uint16_t yaw;
        uint16_t nav_yaw;
    } log_ACAttControl;

    // references to external libraries
    const AP_AHRS&      _ahrs;
    const AP_InertialSensor&  _ins;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;
    APM_PI&	            _pi_angle_roll;
    APM_PI&	            _pi_angle_pitch;
    APM_PI&	            _pi_angle_yaw;
    AC_PID&             _pid_rate_roll;
    AC_PID&             _pid_rate_pitch;
    AC_PID&             _pid_rate_yaw;
    int16_t&            _motor_roll;            // g.rc_1.servo_out
    int16_t&            _motor_pitch;           // g.rc_2.servo_out
    int16_t&            _motor_yaw;             // g.rc_4.servo_out
    int16_t&            _motor_throttle;        // g.rc_3.servo_out

    // parameters
    AP_Float            _angle_rate_rp_max;     // maximum rate request output from the earth-frame angle controller for roll and pitch axis
    AP_Float            _angle_rate_y_max;      // maximum rate request output from the earth-frame angle controller for yaw axis
    AP_Float            _slew_yaw;              // maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes

    // internal variables
    // To-Do: make rate targets a typedef instead of Vector3f?
    float               _dt;                    // time delta in seconds
    Vector3f            _angle_ef_target;       // angle controller earth-frame targets
    Vector3f            _angle_bf_error;        // angle controller earth-frame targets
    Vector3f            _rate_bf_target;        // rate controller body-frame targets
    Vector3f            _rate_bf_feedforward;   // rate controller body-frame targets
    int16_t             _angle_boost;           // used only for logging
};

#define AC_ATTITUDE_CONTROL_LOG_FORMAT(msg) { msg, sizeof(AC_AttitudeControl::log_Attitude),	\
                            "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" }

#endif //AC_AttitudeControl_H
