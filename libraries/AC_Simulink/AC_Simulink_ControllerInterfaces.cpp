/// @file    AC_Simulink_ControllerInterfaces.cpp
/// @brief   Controller and motor interface of Simulink integration
#include <AP_HAL/AP_HAL.h>
#include "AC_Simulink_ControllerInterfaces.h"
#include "AC_Simulink_AHRSHandle.h"
#include "AC_Simulink_AttControlHandle.h"
#include "AC_Simulink_PosControlHandle.h"
#include "AC_Simulink_MotorsHandle.h"

AC_Simulink_ControllerInterfaces::AC_Simulink_ControllerInterfaces(AP_AHRS_View *&ahrs, 
AC_AttitudeControl *&att_control, AC_PosControl *&pos_control, AP_MotorsMulticopter *&motors) {
    AC_Simulink_AHRS::initializeInterfaces(ahrs);
    AC_Simulink_AttControlHandle::initializeInterfaces(att_control);
    AC_Simulink_PosControlHandle::initializeInterfaces(pos_control);
    AC_Simulink_MotorsHandle::initializeInterfaces(motors);
}
