// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_MotorsY6.h
/// @brief	Motor control class for Y6 frames

#ifndef AP_MOTORSY6
#define AP_MOTORSY6

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_MotorsMatrix.h>    // Parent Motors Matrix library

#define AP_MOTORS_Y6_YAW_DIRECTION 1    // this really should be a user selectable parameter

/// @class      AP_MotorsY6
class AP_MotorsY6 : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsY6( uint8_t APM_version, APM_RC_Class* rc_out, RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) : AP_MotorsMatrix(APM_version, rc_out, rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz) {
    };

    // setup_motors - configures the motors for a quad
    virtual void        setup_motors();

protected:

};

#endif  // AP_MOTORSY6