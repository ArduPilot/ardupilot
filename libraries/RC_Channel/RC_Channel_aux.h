// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel_aux.h
/// @brief	RC_Channel manager for auxiliary channels (5..8), with EEPROM-backed storage of constants.
/// @author Amilcar Lucas

#ifndef RC_CHANNEL_AUX_H_
#define RC_CHANNEL_AUX_H_

#include "RC_Channel.h"

/// @class	RC_Channel_aux
/// @brief	Object managing one aux. RC channel (CH5-8), with information about its function
class RC_Channel_aux : public RC_Channel {
public:
    /// Constructor
    ///
    /// @param key      EEPROM storage key for the channel trim parameters.
    /// @param name     Optional name for the group.
    ///
    RC_Channel_aux(uint8_t ch_out) :
        RC_Channel(ch_out)
    {
    }

    typedef enum
    {
        k_none                  = 0,            ///< disabled
        k_manual                = 1,            ///< manual, just pass-thru the RC in signal
        k_flap                  = 2,            ///< flap
        k_flap_auto             = 3,            ///< flap automated
        k_aileron               = 4,            ///< aileron
        k_flaperon              = 5,            ///< flaperon (flaps and aileron combined, needs two independent servos one for each wing)
        k_mount_pan             = 6,            ///< mount yaw (pan)
        k_mount_tilt    = 7,            ///< mount pitch (tilt)
        k_mount_roll    = 8,            ///< mount roll
        k_mount_open    = 9,            ///< mount open (deploy) / close (retract)
        k_cam_trigger   = 10,           ///< camera trigger
        k_egg_drop              = 11,           ///< egg drop
        k_mount2_pan    = 12,           ///< mount2 yaw (pan)
        k_mount2_tilt   = 13,           ///< mount2 pitch (tilt)
        k_mount2_roll   = 14,           ///< mount2 roll
        k_mount2_open   = 15,           ///< mount2 open (deploy) / close (retract)
		k_dspoiler1     = 16,           ///< differential spoiler 1 (left wing)
		k_dspoiler2     = 17,           ///< differential spoiler 2 (right wing)
        k_aileron_with_input    = 18,            ///< aileron, with rc input
        k_nr_aux_servo_functions         ///< This must be the last enum value (only add new values _before_ this one)
    } Aux_servo_function_t;

    AP_Int8         function;           ///< see Aux_servo_function_t enum

    void            output_ch(unsigned char ch_nr);

	// set radio_out for a function channel
	static void set_radio(Aux_servo_function_t function, int16_t value);

	// set and save the trim for a function channel to radio_in
	static void set_radio_trim(Aux_servo_function_t function);

	// set radio_out to radio_min
	static void set_radio_to_min(Aux_servo_function_t function);

	// set radio_out to radio_max
	static void set_radio_to_max(Aux_servo_function_t function);

	// set radio_out to radio_trim
	static void set_radio_to_trim(Aux_servo_function_t function);

	// copy radio_in to radio_out
	static void copy_radio_in_out(Aux_servo_function_t function);

	// set servo_out
	static void set_servo_out(Aux_servo_function_t function, int16_t value);

	// return true if a function is assigned to a channel
	static bool function_assigned(Aux_servo_function_t function);

	// set a servo_out value, and angle range, then calc_pwm
	static void move_servo(Aux_servo_function_t function,
						   int16_t value, int16_t angle_min, int16_t angle_max);

    static const struct AP_Param::GroupInfo        var_info[];
};

void update_aux_servo_function(RC_Channel_aux* rc_a = NULL, RC_Channel_aux* rc_b = NULL, 
							   RC_Channel_aux* rc_c = NULL, RC_Channel_aux* rc_d = NULL, 
							   RC_Channel_aux* rc_e = NULL, RC_Channel_aux* rc_f = NULL, 
							   RC_Channel_aux* rc_g = NULL);
void enable_aux_servos();

#endif /* RC_CHANNEL_AUX_H_ */
