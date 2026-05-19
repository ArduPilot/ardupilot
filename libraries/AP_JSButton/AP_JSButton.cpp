#include "AP_JSButton.h"

const AP_Param::GroupInfo JSButton::var_info[] = {

    // @Param: FUNCTION
    // @DisplayName: Function for button
    // @Description: Set to 0 to disable or choose a function
    // @Values: 0:Disabled,1:shift,2:arm_toggle,3:arm,4:disarm,5:mode_manual,6:mode_stabilize,7:mode_depth_hold,8:mode_poshold,9:mode_auto,10:mode_circle,11:mode_guided,12:mode_acro,13:mode_surftrak,21:mount_center,22:mount_tilt_up,23:mount_tilt_down,24:camera_trigger,25:camera_source_toggle,26:mount_pan_right,27:mount_pan_left,31:lights1_cycle,32:lights1_brighter,33:lights1_dimmer,34:lights2_cycle,35:lights2_brighter,36:lights2_dimmer,41:gain_toggle,42:gain_inc,43:gain_dec,44:trim_roll_inc,45:trim_roll_dec,46:trim_pitch_inc,47:trim_pitch_dec,48:input_hold_set,49:roll_pitch_toggle,51:relay_1_on,52:relay_1_off,53:relay_1_toggle,54:relay_2_on,55:relay_2_off,56:relay_2_toggle,57:relay_3_on,58:relay_3_off,59:relay_3_toggle,61:actuator_1_inc,62:actuator_1_dec,63:actuator_1_min,64:actuator_1_max,65:actuator_1_center,66:actuator_2_inc,67:actuator_2_dec,68:actuator_2_min,69:actuator_2_max,70:actuator_2_center,71:actuator_3_inc,72:actuator_3_dec,73:actuator_3_min,74:actuator_3_max,75:actuator_3_center,76:actuator_1_min_momentary,77:actuator_1_max_momentary,78:actuator_1_min_toggle,79:actuator_1_max_toggle,80:actuator_2_min_momentary,81:actuator_2_max_momentary,82:actuator_2_min_toggle,83:actuator_2_max_toggle,84:actuator_3_min_momentary,85:actuator_3_max_momentary,86:actuator_3_min_toggle,87:actuator_3_max_toggle,91:custom_1,92:custom_2,93:custom_3,94:custom_4,95:custom_5,96:custom_6,101:relay_4_on,102:relay_4_off,103:relay_4_toggle,104:relay_1_momentary,105:relay_2_momentary,106:relay_3_momentary,107:relay_4_momentary,108:script_1,109:script_2,110:script_3,111:script_4,112:actuator_4_min,113:actuator_4_max,114:actuator_4_center,115:actuator_4_inc,116:actuator_4_dec,117:actuator_4_min_momentary,118:actuator_4_max_momentary,119:actuator_4_min_toggle,120:actuator_4_max_toggle,121:actuator_5_min,122:actuator_5_max,123:actuator_5_center,124:actuator_5_inc,125:actuator_5_dec,126:actuator_5_min_momentary,127:actuator_5_max_momentary,128:actuator_5_min_toggle,129:actuator_5_max_toggle,130:actuator_6_min,131:actuator_6_max,132:actuator_6_center,133:actuator_6_inc,134:actuator_6_dec,135:actuator_6_min_momentary,136:actuator_6_max_momentary,137:actuator_6_min_toggle,138:actuator_6_max_toggle
    // @User: Standard
    AP_GROUPINFO("FUNCTION", 1, JSButton, _function, 0),

    // @Param: SFUNCTION
    // @DisplayName: Function for button when the shift mode is toggled on
    // @Description: Set to 0 to disable or choose a function
    // @Values: 0:Disabled,1:shift,2:arm_toggle,3:arm,4:disarm,5:mode_manual,6:mode_stabilize,7:mode_depth_hold,8:mode_poshold,9:mode_auto,10:mode_circle,11:mode_guided,12:mode_acro,13:mode_surftrak,21:mount_center,22:mount_tilt_up,23:mount_tilt_down,24:camera_trigger,25:camera_source_toggle,26:mount_pan_right,27:mount_pan_left,31:lights1_cycle,32:lights1_brighter,33:lights1_dimmer,34:lights2_cycle,35:lights2_brighter,36:lights2_dimmer,41:gain_toggle,42:gain_inc,43:gain_dec,44:trim_roll_inc,45:trim_roll_dec,46:trim_pitch_inc,47:trim_pitch_dec,48:input_hold_set,49:roll_pitch_toggle,51:relay_1_on,52:relay_1_off,53:relay_1_toggle,54:relay_2_on,55:relay_2_off,56:relay_2_toggle,57:relay_3_on,58:relay_3_off,59:relay_3_toggle,61:actuator_1_inc,62:actuator_1_dec,63:actuator_1_min,64:actuator_1_max,65:actuator_1_center,66:actuator_2_inc,67:actuator_2_dec,68:actuator_2_min,69:actuator_2_max,70:actuator_2_center,71:actuator_3_inc,72:actuator_3_dec,73:actuator_3_min,74:actuator_3_max,75:actuator_3_center,76:actuator_1_min_momentary,77:actuator_1_max_momentary,78:actuator_1_min_toggle,79:actuator_1_max_toggle,80:actuator_2_min_momentary,81:actuator_2_max_momentary,82:actuator_2_min_toggle,83:actuator_2_max_toggle,84:actuator_3_min_momentary,85:actuator_3_max_momentary,86:actuator_3_min_toggle,87:actuator_3_max_toggle,91:custom_1,92:custom_2,93:custom_3,94:custom_4,95:custom_5,96:custom_6,101:relay_4_on,102:relay_4_off,103:relay_4_toggle,104:relay_1_momentary,105:relay_2_momentary,106:relay_3_momentary,107:relay_4_momentary,108:script_1,109:script_2,110:script_3,111:script_4,112:actuator_4_min,113:actuator_4_max,114:actuator_4_center,115:actuator_4_inc,116:actuator_4_dec,117:actuator_4_min_momentary,118:actuator_4_max_momentary,119:actuator_4_min_toggle,120:actuator_4_max_toggle,121:actuator_5_min,122:actuator_5_max,123:actuator_5_center,124:actuator_5_inc,125:actuator_5_dec,126:actuator_5_min_momentary,127:actuator_5_max_momentary,128:actuator_5_min_toggle,129:actuator_5_max_toggle,130:actuator_6_min,131:actuator_6_max,132:actuator_6_center,133:actuator_6_inc,134:actuator_6_dec,135:actuator_6_min_momentary,136:actuator_6_max_momentary,137:actuator_6_min_toggle,138:actuator_6_max_toggle
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
    }
    return _function;
}

void JSButton::set_default(button_function_t f, button_function_t sf)
{
    _function.set_default(f);
    _sfunction.set_default(sf);
}
