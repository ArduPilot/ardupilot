
/// @file    AC_Custom_FastTask_Empty.cpp
/// @brief   Empty class implementation for custom fast task instance

#include "AC_Custom_FastTask_Empty.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <GCS_MAVLink/GCS.h>

AC_Custom_FastTask_Empty::AC_Custom_FastTask_Empty()
{
}

void AC_Custom_FastTask_Empty::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Custom Fast Task is initialized ");;

}

void AC_Custom_FastTask_Empty::update()
{
    Quaternion currAttQuat,desAttQuat;
    currAttQuat.initialise();
    desAttQuat.initialise();
    // get current attitude
    AP_AHRS::get_singleton()->get_quat_body_to_ned(currAttQuat);
    // get desired attitude
    desAttQuat = AC_AttitudeControl::get_singleton()->get_attitude_target_quat();
}
