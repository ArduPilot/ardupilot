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

    void output_min() override;
protected:


    //Override MotorsMatrix method
    //void add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float strafe_fac, uint8_t testing_order);

    void output_armed_not_stabilizing() override;
    void output_armed_stabilizing() override;


    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _strafe_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to strafe (left/right)
};

#endif  // AP_MOTORS6DOF
