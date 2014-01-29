// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Heli.h
/// @brief   ArduCopter attitude control library for traditional helicopters

#ifndef AC_ATTITUDECONTROL_HELI_H
#define AC_ATTITUDECONTROL_HELI_H

#include <AC_AttitudeControl.h>

#define AC_ATTITUDE_HELI_ROLL_FF                    0.0f
#define AC_ATTITUDE_HELI_PITCH_FF                   0.0f
#define AC_ATTITUDE_HELI_YAW_FF                     0.0f
#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f

class AC_AttitudeControl_Heli : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Heli( AP_AHRS &ahrs,
                        AP_InertialSensor& ins,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsHeli& motors,
                        APM_PI& pi_angle_roll, APM_PI& pi_angle_pitch, APM_PI& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw,
                        int16_t& motor_roll, int16_t& motor_pitch, int16_t& motor_yaw, int16_t& motor_throttle
                        ) :
        AC_AttitudeControl(ahrs, ins, aparm, motors,
                           pi_angle_roll, pi_angle_pitch, pi_angle_yaw,
                           pid_rate_roll, pid_rate_pitch, pid_rate_yaw,
                           motor_roll, motor_pitch, motor_yaw, motor_throttle)
		{
            AP_Param::setup_object_defaults(this, var_info);
		}

	// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
	// should be called at 100hz or more
	virtual void rate_controller_run();

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage
	void use_leaky_i(bool leaky_i) {  _flags_heli.leaky_i = leaky_i; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // To-Do: move these limits flags into the heli motors class
    struct AttControlHeliFlags {
        uint8_t limit_roll      :   1;  // 1 if we have requested larger roll angle than swash can physically move
        uint8_t limit_pitch     :   1;  // 1 if we have requested larger pitch angle than swash can physically move
        uint8_t leaky_i         :   1;  // 1 if we should use leaky i term for body-frame rate to motor stage
    } _flags_heli;

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in centi-degrees/sec) for roll, pitch and yaw
    void rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds, int16_t& motor_roll, int16_t& motor_pitch);
    virtual float rate_bf_to_motor_yaw(float rate_yaw_cds);

    //
    // throttle methods
    //

    // get_angle_boost - calculate total body frame throttle required to produce the given earth frame throttle
    int16_t get_angle_boost(int16_t throttle_pwm);

    // parameters
    AP_Float _heli_roll_ff;     // body-frame roll rate to motor output feed forward
    AP_Float _heli_pitch_ff;    // body-frame pitch rate to motor output feed forward
    AP_Float _heli_yaw_ff;      // body-frame yaw rate to motor output feed forward
};

#endif //AC_ATTITUDECONTROL_HELI_H
