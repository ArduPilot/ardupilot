#include "AP_Gripper.h"

#include "AP_Gripper_Servo.h"

extern const AP_HAL::HAL& hal;

#define GRIPPER_GRAB_PWM_DEFAULT        1900
#define GRIPPER_RELEASE_PWM_DEFAULT     1100

const AP_Param::GroupInfo AP_Gripper::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Gripper Enable/Disable
    // @Description: Gripper enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Gripper, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Gripper Type
    // @Description: Gripper enable/disable
    // @User: Standard
    // @Values: 0:None,1:Servo,2:EPM
    AP_GROUPINFO("TYPE", 1, AP_Gripper, config.type, 0),

    // @Param: GRAB
    // @DisplayName: Gripper Grab PWM
    // @Description: PWM value sent to Gripper to initiate grabbing the cargo
    // @User: Advanced
    // @Range: 1000 2000
    AP_GROUPINFO("GRAB",    2, AP_Gripper, config.grab_pwm, GRIPPER_GRAB_PWM_DEFAULT),

    // @Param: RELEASE
    // @DisplayName: Gripper Release PWM
    // @Description: PWM value sent to Gripper to release the cargo
    // @User: Advanced
    // @Range: 1000 2000
    AP_GROUPINFO("RELEASE", 3, AP_Gripper, config.release_pwm, GRIPPER_RELEASE_PWM_DEFAULT),

    AP_GROUPEND
};

AP_Gripper::AP_Gripper()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Gripper::init()
{
    // return immediately if not enabled
    if (!_enabled.get()) {
        return;
    }

    switch(config.type.get()) {
    case 0:
        break;
    case 1:
        backend = new AP_Gripper_Servo(config);
        break;
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init();
    }
}

// update - should be called at at least 10hz
#define PASS_TO_BACKEND(function_name) \
    void AP_Gripper::function_name()   \
    {                                  \
        if (backend != nullptr) {      \
            backend->function_name();  \
        }                              \
    }

PASS_TO_BACKEND(grab)
PASS_TO_BACKEND(release)
PASS_TO_BACKEND(update)

#undef PASS_TO_BACKEND
