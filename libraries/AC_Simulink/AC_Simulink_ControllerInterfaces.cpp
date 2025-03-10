
/// @file    AC_Simulink_ControllerInterfaces.cpp
/// @brief   Controller and motor interface of Simulink integration

#include <AP_HAL/AP_HAL.h>
#include "AC_Simulink_ControllerInterfaces.h"

AC_Simulink_ControllerInterfaces::AC_Simulink_ControllerInterfaces(AP_AHRS_View *&ahrs, 
AC_AttitudeControl *&att_control, AC_PosControl *&pos_control, AP_MotorsMulticopter *&motors) {
    //Controller and Motor handles will be used by the Simulink generated code
}