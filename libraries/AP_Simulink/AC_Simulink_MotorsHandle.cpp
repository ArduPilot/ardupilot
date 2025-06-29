// @file    AC_Simulink_MotorsHandle.cpp
// @brief   This class provides AC_PosControl class instance and enables access to AC_PosControl methods from Simulink-generated code.
#include "AC_Simulink_MotorsHandle.h"

AC_Simulink_MotorsHandle *AC_Simulink_MotorsHandle::_singleton = nullptr;

void AC_Simulink_MotorsHandle::initializeInterfaces(AP_MotorsMulticopter *&motors)
{
    if (_singleton == nullptr) {
        _singleton = new AC_Simulink_MotorsHandle(motors);
    }
}

AP_MotorsMulticopter *&AC_Simulink_MotorsHandle::getHandle_AP_Motors() const
{
    return _motors;
}

// get singleton instance
AC_Simulink_MotorsHandle *AC_Simulink_MotorsHandle::get_singleton()
{
    return _singleton;
}

AC_Simulink_MotorsHandle::AC_Simulink_MotorsHandle(AP_MotorsMulticopter *&motors) : _motors(motors)
{
}
void AC_Simulink_MotorsHandle::init(void)
{
}
void AC_Simulink_MotorsHandle::setTorque(float tauRoll,float tauPitch,float tauYaw)
{
    _motors->set_roll(tauRoll);
    _motors->set_pitch(tauPitch);
    _motors->set_yaw(tauYaw);
}
void AC_Simulink_MotorsHandle::setThrust(float throttle)
{
    _motors->set_throttle(throttle);
}

AC_Simulink_MotorsHandle *sl_motors_handle = AC_Simulink_MotorsHandle::get_singleton();
