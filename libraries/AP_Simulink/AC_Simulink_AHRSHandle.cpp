// @file    AC_Simulink_AHRSHandle.cpp
// @brief   This class provides AP_AHRS_View class instance and enables access to AP_AHRS_View methods from Simulink-generated code.
#include "AC_Simulink_AHRSHandle.h"

AC_Simulink_AHRS *AC_Simulink_AHRS::_singleton = nullptr;

void AC_Simulink_AHRS::initializeInterfaces(AP_AHRS_View *&ahrs)
{
    if (_singleton == nullptr) {
        _singleton = new AC_Simulink_AHRS(ahrs);
    }
}

AP_AHRS_View *&AC_Simulink_AHRS::getHandle_AP_AHRS_View() const
{
    return _ahrs;
}

// get singleton instance
AC_Simulink_AHRS *AC_Simulink_AHRS::get_singleton()
{
    return _singleton;
}

AC_Simulink_AHRS::AC_Simulink_AHRS(AP_AHRS_View *&ahrs) : _ahrs(ahrs)
{
}
void AC_Simulink_AHRS::init(void)
{
}
Vector3f AC_Simulink_AHRS::getAngularVelocity(void)
{
    return _ahrs->get_gyro_latest();
}
void AC_Simulink_AHRS::getAttitudeQuat(Quaternion &attQuat)
{
    _ahrs->get_quat_body_to_ned(attQuat);
}
bool AC_Simulink_AHRS::getPositionNED(Vector3f &posNED)
{
    return _ahrs->get_relative_position_NED_origin(posNED);
}
bool AC_Simulink_AHRS::getVelocityNED(Vector3f &velNED)
{
    return _ahrs->get_velocity_NED(velNED);
}
AC_Simulink_AHRS *sl_ahrs_handle = AC_Simulink_AHRS::get_singleton();
