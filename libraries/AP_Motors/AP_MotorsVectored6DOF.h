// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsVectored6DOF.h
/// @brief	Motor control class for 6DOF UV with vectored translational thrusters

#ifndef __AP_MOTORS_VECTORED6DOF_H__
#define __AP_MOTORS_VECTORED6DOF_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_Motors6DOF.h"    // Parent Motors Matrix library

/// @class      AP_MotorsVectored6DOF
class AP_MotorsVectored6DOF : public AP_Motors6DOF {
public:

    /// Constructor
    AP_MotorsVectored6DOF(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors6DOF(loop_rate, speed_hz)
    { };

    // setup_motors - configures the motors for the Vectored 6DOF ROV
    // This takes care of all of the enabling and bitmask stuff seen in the Tricopter motors class
    virtual void        setup_motors();

    void output_armed_stabilizing() override;

protected:

};

#endif  // AP_MotorsVectored6DOF
