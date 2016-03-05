#include "AP_JSButton.h"

const AP_Param::GroupInfo JSButton::var_info[] = {

    // @Param: FUNCTION
    // @DisplayName: Function for button
    // @Description: Set to 0 to disable or choose a function
    // @Values: 0:Disabled,1:shift,2:arm_toggle,3:arm,4:disarm,5:mode_toggle,6:enter_mode_1,7:enter_mode_2,8:enter_mode_3,9:enter_mode_4,10:enter_mode_5,11:enter_mode_6,21:mount_center,22:mount_tilt_up,23:mount_tilt_down,24:camera_trigger,25:camera_source_toggle,26:mount_pan_right,27:mount_pan_left,31:lights1_cycle,32:lights1_brighter,33:lights1_dimmer,34:lights2_cycle,35:lights2_brighter,36:lights2_dimmer,41:gain_toggle,42:gain_inc,43:gain_dec,44:trim_roll_inc,45:trim_roll_dec,46:trim_pitch_inc,47:trim_pitch_dec,48:input_hold_toggle,51:relay_1_on,52:relay_1_off,53:relay_1_toggle,52:relay_2_on,53:relay_2_off,54:relay_2_toggle,91:custom_1,92:custom_2,93:custom_3,94:custom_4,95:custom_5,96:custom_6
    // @User: Standard
    AP_GROUPINFO("FUNCTION", 1, JSButton, _function, 0),

    // @Param: SFUNCTION
    // @DisplayName: Function for button when the shift mode is toggled on
    // @Description: Set to 0 to disable or choose a function
    // @Values: 0:Disabled,1:shift,2:arm_toggle,3:arm,4:disarm,5:mode_toggle,6:enter_mode_1,7:enter_mode_2,8:enter_mode_3,9:enter_mode_4,10:enter_mode_5,11:enter_mode_6,21:mount_center,22:mount_tilt_up,23:mount_tilt_down,24:camera_trigger,25:camera_source_toggle,26:mount_pan_right,27:mount_pan_left,31:lights1_cycle,32:lights1_brighter,33:lights1_dimmer,34:lights2_cycle,35:lights2_brighter,36:lights2_dimmer,41:gain_toggle,42:gain_inc,43:gain_dec,44:trim_roll_inc,45:trim_roll_dec,46:trim_pitch_inc,47:trim_pitch_dec,48:input_hold_toggle,51:relay_1_on,52:relay_1_off,53:relay_1_toggle,52:relay_2_on,53:relay_2_off,54:relay_2_toggle,91:custom_1,92:custom_2,93:custom_3,94:custom_4,95:custom_5,96:custom_6
    // @User: Standard
    AP_GROUPINFO("SFUNCTION", 2, JSButton, _sfunction, 0),

    AP_GROUPEND
};

JSButton::JSButton(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

uint8_t JSButton::function(bool shift) const
{
    if (shift) {
        return _sfunction;
    } else {
        return _function;
    }
}

void JSButton::set_default(button_function_t f, button_function_t sf)
{
    _function.set_default(f);
    _sfunction.set_default(sf);
}
