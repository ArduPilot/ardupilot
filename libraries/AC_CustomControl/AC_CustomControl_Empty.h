#pragma once

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_EMPTY_ENABLED
    #define CUSTOMCONTROL_EMPTY_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_EMPTY_ENABLED

class AC_CustomControl_Empty : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Empty(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

#endif
