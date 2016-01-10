// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsBlueROV.h
/// @brief	Motor control class for Hexacopters

#ifndef __AP_MOTORS_BLUEROV_H__
#define __AP_MOTORS_BLUEROV_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_Motors6DOF.h"    // Parent Motors Matrix library

/// @class      AP_MotorsBlueROV
class AP_MotorsBlueROV : public AP_Motors6DOF {
public:

    /// Constructor
    AP_MotorsBlueROV(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors6DOF(loop_rate, speed_hz)
    { };

    // setup_motors - configures the motors for the BlueROV
    // This takes care of all of the enabling and bitmask stuff seen in the Tricopter motors class
    virtual void        setup_motors();

protected:

};

#endif  // AP_MotorsBlueROV
