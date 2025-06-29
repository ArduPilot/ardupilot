// @file    AC_Simulink_AttControlHandle.cpp
// @brief   This class provides AC_PosControl class instance and enables access to AC_PosControl methods from Simulink-generated code.
#include "AC_Simulink_AttControlHandle.h"

AC_Simulink_AttControlHandle *AC_Simulink_AttControlHandle::_singleton = nullptr;

void AC_Simulink_AttControlHandle::initializeInterfaces(AC_AttitudeControl *&att_control)
{
    if (_singleton == nullptr) {
        _singleton = new AC_Simulink_AttControlHandle(att_control);
    }
}

AC_AttitudeControl *&AC_Simulink_AttControlHandle::getHandle_AC_AttControl() const
{
    return _att_control;
}

// get singleton instance
AC_Simulink_AttControlHandle *AC_Simulink_AttControlHandle::get_singleton()
{
    return _singleton;
}

AC_Simulink_AttControlHandle::AC_Simulink_AttControlHandle(AC_AttitudeControl *&att_control) : _att_control(att_control)
{
}
void AC_Simulink_AttControlHandle::init(void)
{
}
Quaternion AC_Simulink_AttControlHandle::getAttitudeSetpointQuat(void)
{
    return _att_control->get_attitude_target_quat();
}
Vector3f AC_Simulink_AttControlHandle::getAngularVelocitySetpoint(void)
{
    return _att_control->get_attitude_target_ang_vel();
}
AC_Simulink_AttControlHandle *sl_attControl_handle = AC_Simulink_AttControlHandle::get_singleton();
