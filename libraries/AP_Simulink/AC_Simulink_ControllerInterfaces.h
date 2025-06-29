#pragma once
/// @file    AC_Simulink_ControllerInterfaces.h
/// @brief This class is responsible for storing instances of the AHRS, Attitude & Position Controller,
///        and Motor classes. These instances are provided by the copter class and stored for access by Simulink-generated code.
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_Motors/AP_MotorsMulticopter.h>

class AC_Simulink_ControllerInterfaces
{
public:
    AC_Simulink_ControllerInterfaces(AP_AHRS_View *&ahrs,
                                     AC_AttitudeControl *&_att_control, AC_PosControl *&_pos_control,
                                     AP_MotorsMulticopter *&motors);

    CLASS_NO_COPY(AC_Simulink_ControllerInterfaces);
};
