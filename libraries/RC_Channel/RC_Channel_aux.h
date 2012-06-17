// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel_aux.h
/// @brief	RC_Channel manager for auxiliary channels (5..8), with EEPROM-backed storage of constants.
/// @author Amilcar Lucas
/// @author Gregory Fletcher

#ifndef RC_CHANNEL_AUX_H_
#define RC_CHANNEL_AUX_H_

#include "RC_Channel.h"

// Macro to simplify accessing the auxiliary servos
#define G_RC_AUX(_t)   if (g_rc_function[RC_Channel_aux::_t]) g_rc_function[RC_Channel_aux::_t]

/// @class	RC_Channel_aux
/// @brief	Object managing one aux. RC channel (CH5-8), with information about its function
/// 	    Also contains physical min,max angular deflection, to allow calibrating open-loop servo movements
class RC_Channel_aux : public RC_Channel{
public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	RC_Channel_aux() :
		RC_Channel(),
		function  (0),
		angle_min (-4500), // assume -45 degrees min deflection
		angle_max (4500)   // assume  45 degrees max deflection
	{}

	typedef enum
	{
		k_none			= 0,	///< disabled
		k_manual		= 1,	///< manual, just pass-thru the RC in signal
		k_flap			= 2,	///< flap
		k_flap_auto		= 3,	///< flap automated
		k_aileron		= 4,	///< aileron
		k_flaperon		= 5,	///< flaperon (flaps and aileron combined, needs two independent servos one for each wing)
		k_mount_yaw		= 6,	///< mount yaw (pan)
		k_mount_pitch	= 7,	///< mount pitch (tilt)
		k_mount_roll	= 8,	///< mount roll
		k_mount_open	= 9,	///< mount open (deploy) / close (retract)
		k_cam_trigger	= 10,	///< camera trigger
		k_egg_drop		= 11,	///< egg drop
		k_nr_aux_servo_functions ///< This must be the last enum value (only add new values _before_ this one)
	} Aux_servo_function_t;

	AP_Int8 	function;	///< see Aux_servo_function_t enum
	AP_Int16 	angle_min;	///< min angle limit of actuated surface in 0.01 degree units
	AP_Int16 	angle_max;	///< max angle limit of actuated surface in 0.01 degree units

	int16_t closest_limit(int16_t angle);

	void angle_out(int16_t angle);

	void rc_input(float *control_angle, int16_t angle);

	void output_ch(unsigned char ch_nr);

	static const struct AP_Param::GroupInfo var_info[];
};

void update_aux_servo_function(RC_Channel_aux* rc_5, RC_Channel_aux* rc_6, RC_Channel_aux* rc_7, RC_Channel_aux* rc_8);
extern RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];	// the aux. servo ch. assigned to each function

#endif /* RC_CHANNEL_AUX_H_ */
