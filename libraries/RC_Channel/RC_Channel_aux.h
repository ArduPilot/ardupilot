// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel_aux.h
/// @brief	RC_Channel manager for Channels 5..8, with EEPROM-backed storage of constants.
/// @author Amilcar Lucas

#ifndef RC_CHANNEL_AUX_H_
#define RC_CHANNEL_AUX_H_

#include "RC_Channel.h"

// Macro to simplify accessing the auxiliary servos
#define G_RC_AUX(_t)   if (g_rc_function[RC_Channel_aux::_t]) g_rc_function[RC_Channel_aux::_t]

/// @class	RC_Channel_aux
/// @brief	Object managing one aux. RC channel (CH5-8), with information about its function
class RC_Channel_aux : public RC_Channel{
public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	RC_Channel_aux(AP_Var::Key key, const prog_char_t *name) :
		RC_Channel(key, name),
		function  (&_group, 4,  k_none, name ? PSTR("FUNCTION")  : 0) // suppress name if group has no name
	{}

	typedef enum
	{
		k_none			= 0,	// disabled
		k_manual		= 1,	// manual, just pass-thru the RC in signal
		k_flap			= 2,	// flap
		k_flap_auto		= 3,	// flap automated
		k_aileron		= 4,	// aileron
		k_flaperon		= 5,	// flaperon (flaps and aileron combined, needs two independent servos one for each wing)
		k_nr_aux_servo_functions // This must be the last enum value (only add new values _before_ this one)
	} Aux_servo_function_t;

	AP_Int8 	function;	// 0=disabled, 1=manual, 2=flap, 3=flap auto, 4=aileron, 5=flaperon, 6=mount yaw (pan), 7=mount pitch (tilt), 8=mount roll, 9=camera trigger, 10=camera open, 11=egg drop

	void output_ch(unsigned char ch_nr);	// map a function to a servo channel and output it

};

void update_aux_servo_function(RC_Channel_aux* rc_5, RC_Channel_aux* rc_6, RC_Channel_aux* rc_7, RC_Channel_aux* rc_8);

#endif /* RC_CHANNEL_AUX_H_ */
