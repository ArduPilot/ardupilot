#pragma once

/// @file    AC_CustomControl.h
/// @brief   ArduCopter custom control library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Logger/AP_Logger.h>

#if AP_CUSTOMCONTROL_ENABLED

#ifndef CUSTOMCONTROL_MAX_TYPES
#define CUSTOMCONTROL_MAX_TYPES 1
#endif

class AC_CustomControl_Backend;

class AC_CustomControl {
public:
    AC_CustomControl(AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& _att_control, AP_MotorsMulticopter*& motors, float dt);

    CLASS_NO_COPY(AC_CustomControl);  /* Do not allow copies */

    void init(void);
    void update(void);
    void motor_set(Vector3f motor_out);
    void set_custom_controller(bool enabled);
    void reset_main_att_controller(void);
    bool is_safe_to_run(void);
    void log_switch(void);

    // zero index controller type param, only use it to acces _backend or _backend_var_info array
    uint8_t get_type() { return _controller_type > 0 ? (_controller_type - 1) : 0; };

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

protected:
    // add custom controller here
    enum class CustomControlType : uint8_t {
        CONT_NONE            = 0,
        CONT_EMPTY           = 1,
    };            // controller that should be used     

    enum class  CustomControlOption {
        ROLL = 1 << 0,
        PITCH = 1 << 1,
        YAW = 1 << 2,
    };

    // Intersampling period in seconds
    float _dt;
    bool _custom_controller_active;

    // References to external libraries
    AP_AHRS_View*& _ahrs;
    AC_AttitudeControl_Multi*& _att_control;
    AP_MotorsMulticopter*& _motors;

    AP_Enum<CustomControlType> _controller_type;
    AP_Int8 _custom_controller_mask;

private:
    AC_CustomControl_Backend *_backend;
};

#endif
