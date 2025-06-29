// @file    AC_Simulink_PosControlHandle.h
// @brief   This class provides AC_PosControl class instance and enables access to AC_PosControl methods from Simulink-generated code.
#include "AC_Simulink_PosControlHandle.h"

AC_Simulink_PosControlHandle *AC_Simulink_PosControlHandle::_singleton = nullptr;

void AC_Simulink_PosControlHandle::initializeInterfaces(AC_PosControl *&pos_control)
{
    if (_singleton == nullptr) {
        _singleton = new AC_Simulink_PosControlHandle(pos_control);
    }
}

AC_PosControl *&AC_Simulink_PosControlHandle::getHandle_AC_PosControl() const
{
    return _pos_control;
}

// get singleton instance
AC_Simulink_PosControlHandle *AC_Simulink_PosControlHandle::get_singleton()
{
    return _singleton;
}

AC_Simulink_PosControlHandle::AC_Simulink_PosControlHandle(AC_PosControl *&pos_control) : _pos_control(pos_control)
{
}
void AC_Simulink_PosControlHandle::init(void)
{
}
Vector3p AC_Simulink_PosControlHandle::getPositionSetPointNEU(void)
{
    return _pos_control->get_pos_target_cm();
}
Vector3f AC_Simulink_PosControlHandle::getVelocitySetPointNEU(void)
{
    return _pos_control->get_vel_desired_cms();
}

AC_Simulink_PosControlHandle *sl_posControl_handle = AC_Simulink_PosControlHandle::get_singleton();
