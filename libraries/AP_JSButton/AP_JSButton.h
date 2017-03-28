#pragma once

#include <AP_Param/AP_Param.h>

class JSButton {
public:
    JSButton();

    // Button functions
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
        // 57-60 reserved for future functions
        k_servo_1_inc           = 61,           ///< increase servo output
        k_servo_1_dec           = 62,           ///< decrease servo output
        k_servo_1_min           = 63,           ///< center servo
        k_servo_1_max           = 64,           ///< set servo output to minimum (SERVOn_MIN)
        k_servo_1_center        = 65,           ///< set servo output to maximum (SERVOn_MAX)

        k_servo_2_inc           = 66,
        k_servo_2_dec           = 67,
        k_servo_2_min           = 68,
        k_servo_2_max           = 69,
        k_servo_2_center        = 70,

        k_servo_3_inc           = 71,
        k_servo_3_dec           = 72,
        k_servo_3_min           = 73,
        k_servo_3_max           = 74,
        k_servo_3_center        = 75,

        // 70-89 reserved for future functions
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
