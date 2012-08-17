// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_MotorsOctaQuad.h
/// @brief	Motor control class for OctaQuadcopters

#ifndef AP_MOTORSOCTAQUAD
#define AP_MOTORSOCTAQUAD

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_MotorsMatrix.h>    // Parent Motors Matrix library

/// @class      AP_MotorsOcta
class AP_MotorsOctaQuad : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsOctaQuad( uint8_t APM_version, APM_RC_Class* rc_out, RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) : AP_MotorsMatrix(APM_version, rc_out, rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz) {
    };

    // setup_motors - configures the motors for a quad
    virtual void        setup_motors();

protected:

};

#endif  // AP_MOTORSOCTAQUAD