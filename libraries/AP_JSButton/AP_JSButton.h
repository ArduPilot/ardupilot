#pragma once

#include <AP_Param/AP_Param.h>

class JSButton {
public:
    JSButton();

    // Button functions
    // Value list for parameter convenience:
    // @Values: 0:Disabled,1:shift,2:arm_toggle,3:arm,4:disarm,5:mode_toggle,6:mode_stab,7:mode_althold,21:mount_center,22:mount_tilt_up,23:mount_tilt_down,24:camera_trigger,25:camera_source_toggle,26:mount_pan_right,27:mount_pan_left,31:light1_cycle,32:lights1_brighter,
    // 33:lights1_dimmer,34:lights2_cycle,35:lights2_brighter,36:lights2_dimmer,41:gain_toggle,42:gain_inc,43:gain_dec,44:trim_roll_inc,45:trim_roll_dec,46:trim_pitch_inc,47:trim_pitch_dec,48:input_hold_toggle,51:relay_1_on,52:relay_1_off,53:relay_1_toggle,52:relay_2_on,53:relay_2_off,54:relay_2_toggle,91:custom_1,92:custom_2,93:custom_3,94:custom_4,95:custom_5,96:custom_6
    typedef enum {
        k_none                  = 0,            ///< disabled
        k_shift                 = 1,            ///< "shift" buttons to allow more functions
        k_arm_toggle            = 2,            ///< arm/disarm vehicle toggle
        k_arm                   = 3,            ///< arm vehicle
        k_disarm                = 4,            ///< disarm vehicle
        k_mode_toggle           = 5,            ///< toggle through available modes
        k_mode_1                = 6,            ///< enter mode 1
        k_mode_2                = 7,            ///< enter mode 2
        k_mode_3                = 8,            ///< enter mode 3
        k_mode_4                = 9,            ///< enter mode 4
        k_mode_5                = 10,           ///< enter mode 5
        k_mode_6                = 11,           ///< enter mode 6
        // 12-20 reserved for future mode functions
        k_mount_center          = 21,           ///< move mount to center
        k_mount_tilt_up         = 22,           ///< tilt mount up
        k_mount_tilt_down       = 23,           ///< tilt mount down
        k_camera_trigger        = 24,           ///< trigger camera shutter
        k_camera_source_toggle  = 25,           ///< toggle camera source
        k_mount_pan_right       = 26,           ///< pan mount right
        k_mount_pan_left        = 27,           ///< pan mount left
        // 26-30 reserved for future camera functions
        k_lights1_cycle         = 31,           ///< lights 1 cycle
        k_lights1_brighter      = 32,           ///< lights 1 up
        k_lights1_dimmer        = 33,           ///< lights 1 down
        k_lights2_cycle         = 34,           ///< lights 2 cycle
        k_lights2_brighter      = 35,           ///< lights 2 up
        k_lights2_dimmer        = 36,           ///< lights 2 down
        // 37-40 reserved for future light functions
        k_gain_toggle           = 41,           ///< toggle different gain settings
        k_gain_inc              = 42,           ///< increase control gain
        k_gain_dec              = 43,           ///< decrease control gain
        k_trim_roll_inc         = 44,           ///< increase roll trim
        k_trim_roll_dec         = 45,           ///< decrease roll trim
        k_trim_pitch_inc        = 46,           ///< increase pitch trim
        k_trim_pitch_dec        = 47,           ///< decrease pitch trim
        k_input_hold_toggle     = 48,           ///< toggle input hold (trim to current controls)
        // 49-50 reserved for future functions
        k_relay_1_on            = 51,           ///< trigger relay on
        k_relay_1_off           = 52,           ///< trigger relay off
        k_relay_1_toggle        = 53,           ///< trigger relay toggle
        k_relay_2_on            = 54,           ///< trigger relay on
        k_relay_2_off           = 55,           ///< trigger relay off
        k_relay_2_toggle        = 56,           ///< trigger relay toggle
        // 57-90 reserved for future functions
        k_custom_1              = 91,           ///< custom user button 1
        k_custom_2              = 92,           ///< custom user button 2
        k_custom_3              = 93,           ///< custom user button 3
        k_custom_4              = 94,           ///< custom user button 4
        k_custom_5              = 95,           ///< custom user button 5
        k_custom_6              = 96,           ///< custom user button 6
        // 97+ reserved for future functions
        k_nr_btn_functions         ///< This must be the last enum value (only add new values _before_ this one)
    } button_function_t;

    // If shift is false, returns the function assigned to this button
    // If shift is true, returns the shift function assigned to this button
    uint8_t function(bool shift = false) const;

    // Sets the default function and shift function parameter values for this button
    void set_default(button_function_t f, button_function_t sf);

    static const struct AP_Param::GroupInfo var_info[];

private:
    // Button mappings
    AP_Int8 _function;
    AP_Int8 _sfunction;
};
