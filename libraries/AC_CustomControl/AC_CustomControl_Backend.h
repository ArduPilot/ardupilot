#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ENABLED

#include "AC_CustomControl.h"

class AC_CustomControl_Backend
{
public:
    AC_CustomControl_Backend(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
        _frontend(frontend),
        _ahrs(ahrs),
        _att_control(att_control),
        _motors(motors)
    {}

    // empty destructor to suppress compiler warning
    virtual ~AC_CustomControl_Backend() {}

    // update controller, return roll, pitch, yaw controller output
    virtual Vector3f update() = 0;

    // reset controller to avoid build up or abrupt response upon switch, ex: integrator, filter
    virtual void reset() = 0;

    // set the PID notch sample rates
    virtual void set_notch_sample_rate(float sample_rate) {};

protected:
    // References to external libraries
    AP_AHRS_View*& _ahrs;
    AC_AttitudeControl*& _att_control;
    AP_MotorsMulticopter*& _motors;
    AC_CustomControl& _frontend;
};

#endif  // AP_CUSTOMCONTROL_ENABLED
