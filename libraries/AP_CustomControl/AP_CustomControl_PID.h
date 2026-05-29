#pragma once

#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_PID_ENABLED

#include "AP_CustomControl/AP_CustomControl.h"
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

#include "AP_CustomControl_Backend.h"

class AP_CustomControl_PID : public AP_CustomControl_Backend {
public:
    AP_CustomControl_PID(AP_CustomControl& frontend, float dt);

    // run lowest level body-frame rate controller and send outputs to the servos
    void update() override;
    bool can_run() override;
    void reset(void) override;

#if AP_FILTER_ENABLED
    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;
#endif // AP_FILTER_ENABLED

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Put controller related variables here.

    // Angle P controller objects
    AC_P _p_angle_roll;
    AC_P _p_angle_pitch;

    // Rate PID controller objects
    AC_PID _pid_rate_roll;
    AC_PID _pid_rate_pitch;
};

#endif  // AP_PLANE_CUSTOMCONTROL_PID_ENABLED
