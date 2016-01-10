// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Motors6DOF.h
/// @brief	Motor control class for ROVs with direct control over 6DOF (or fewer) in movement

#ifndef __AP_MOTORS_6DOF_H__
#define __AP_MOTORS_6DOF_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"



/// @class      AP_MotorsMatrix
class AP_Motors6DOF : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_Motors6DOF(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {};

    void                set_forward(float forward_in) { _forward_in = constrain_float(2*(forward_in-1500),-1000.0f,1000.0f); };        // range 0 ~ 1000
    void                set_strafe(float strafe_in) { _strafe_in = constrain_float(2*(strafe_in-1500),-1000.0f,1000.0f); };           // range 0 ~ 1000

    float               get_forward() const { return _forward_in; }		// range 1100~1900 this is raw pwm value from rc
    float               get_strafe() const { return _strafe_in; }		// range 1100~1900 this is raw pwm value from rc

protected:


    //Override MotorsMatrix method
    void add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float strafe_fac, uint8_t testing_order)

    float               _forward_in;                // last forward input from set_forward caller, raw pwm
    float               _strafe_in;                 // last strafe input from set_strafe caller, raw pwm

    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _strafe_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to strafe (left/right)
};

#endif  // AP_MOTORS6DOF
