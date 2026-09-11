#pragma once

#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_ENABLED

#include "AP_CustomControl.h"

class AP_CustomControl_Backend
{
public:
    AP_CustomControl_Backend(AP_CustomControl& frontend, float dt) :
        _frontend(frontend),
        _dt(dt)
    {}

    // update controller, return roll, pitch, yaw controller output
    virtual void update() = 0;

    // Check if the conditions are met to run the custom controller this loop.
    virtual bool can_run() { return true; }

    // Reset the custom controllers e.g. to avoid buildups in integrators or initialize filters.
    // This will be called upon engaging the custom controller, so it's a good place to also do output matching.
    virtual void reset() = 0;

#if AP_FILTER_ENABLED
    // set the PID notch sample rates
    virtual void set_notch_sample_rate(float sample_rate) {};
#endif // AP_FILTER_ENABLED

protected:
    // References to external libraries
    AP_CustomControl& _frontend;
    float _dt;

    float get_roll_target_deg() { return _frontend.roll_target_deg; }
    float get_nav_pitch_target_deg() { return _frontend.pitch_target_deg; }
    float get_pitch_target_deg() { return _frontend.pitch_target_deg + _frontend.pitch_trim_deg; }
 };


#endif  // AP_PLANE_CUSTOMCONTROL_ENABLED
