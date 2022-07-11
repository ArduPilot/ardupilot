#pragma once

/// @file    AC_CustomControl.h
/// @brief   ArduCopter custom control library

#include "AC_AttitudeControl_Multi.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

#ifndef CUSTOMCONTROL_ENABLED
    #define CUSTOMCONTROL_ENABLED 0
#endif 

#if CUSTOMCONTROL_ENABLED
class AC_CustomControl : public AC_AttitudeControl_Multi {
public:
    AC_CustomControl(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

    void set_custom_controller(bool enabled) override;
    void log_write(void) override;

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run(void) override;
	
    void reset_custom_controller(void);
    void reset_main_controller(void);
    Vector3f run_custom_controller(void);
    void motor_set(Vector3f rpy);

	bool _custom_controller_active = false;

    enum class  CustomControllerOption {
        ROLL = 1 << 0,
        PITCH = 1 << 1,
        YAW = 1 << 2,
    };

    AP_Int8 _custom_controller_enabled;
    AP_Int8 _custom_controller_mask;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // put controller related variable here
    AC_PID _pid_atti_rate_roll;
    AC_PID _pid_atti_rate_pitch;
    AC_PID _pid_atti_rate_yaw;

private:
	uint64_t _last_active_us = AP_HAL::micros64();
};

#endif