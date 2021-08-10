#include "AC_PosControl_Sub.h"

AC_PosControl_Sub::AC_PosControl_Sub(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                     const AP_Motors& motors, AC_AttitudeControl& attitude_control, float dt) :
    AC_PosControl(ahrs, inav, motors, attitude_control, dt),
    _alt_max(0.0f),
    _alt_min(0.0f)
{}
