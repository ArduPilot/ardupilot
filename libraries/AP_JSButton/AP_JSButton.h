/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_JSBUTTON_H
#define AP_JSBUTTON_H

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class JSButton
{
public:
    /// Constructor
    ///
    JSButton();

    /// roll - return input channel number for roll / aileron input
    uint8_t function(bool shift = false) const {
    	if ( shift ) {
    		return _sfunction;
    	} else {
    		return _function;
    	}
    }

    // button function enum
    // value list for parameter convenience:
    // @Values: 0:Disabled,1:shift,2:arm_toggle,3:arm,4:disarm,5:mode_toggle,6:mode_stab,7:mode_althold,21:mount_center,22:mount_tilt_up,23:mount_tilt_down,24:camera_trigger,31:light1_cycle,32:lights1_brighter,
    // 33:lights1_dimmer,34:lights2_cycle,35:lights2_brighter,36:lights2_dimmer,41:gain_toggle,42:gain_inc,43:gain_dec,44:trim_roll_inc,45:trim_roll_dec,46:trim_pitch_inc,47:trim_pitch_dec
    typedef enum
	{
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
		// 8-20 reserved for future mode functions
		k_mount_center          = 21,           ///< move mount to center
		k_mount_tilt_up         = 22,           ///< tilt mount up
		k_mount_tilt_down       = 23,           ///< tilt mount down
		k_camera_trigger        = 24,           ///< trigger camera shutter
		// 24-30 reserved for future camera functions
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
		k_nr_btn_functions         ///< This must be the last enum value (only add new values _before_ this one)
	} button_function_t;

    static const struct AP_Param::GroupInfo var_info[];

private:
    // button mappings
    AP_Int8 _function;
    AP_Int8 _sfunction;
};
#endif
