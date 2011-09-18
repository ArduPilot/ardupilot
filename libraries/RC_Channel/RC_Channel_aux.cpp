// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <APM_RC.h>
#include "RC_Channel_aux.h"

extern RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];	// the aux. servo ch. assigned to each function

// map a function to a servo channel and output it
void
RC_Channel_aux::output_ch(unsigned char ch_nr)
{
	// take care or two corner cases
	switch(function)
	{
	case k_none: 		// disabled
		return;
		break;
	case k_manual:		// manual
		radio_out = radio_in;
		break;
	}

	APM_RC.OutputCh(ch_nr, radio_out);
}

// Update the g_rc_function array of pointers to rc_x channels
// This is to be done before rc_init so that the channels get correctly initialized.
// It also should be called periodically because the user might change the configuration and
// expects the changes to take effect instantly
void update_aux_servo_function(RC_Channel_aux* rc_5, RC_Channel_aux* rc_6, RC_Channel_aux* rc_7, RC_Channel_aux* rc_8)
{
	// positions 0..3 of this array never get used, but this is a stack array, so the entire array gets freed at the end of the function
	RC_Channel_aux::Aux_servo_function_t aux_servo_function[NUM_CHANNELS];			// the function of the aux. servos
	aux_servo_function[CH_5] = (RC_Channel_aux::Aux_servo_function_t)rc_5->function.get();
	aux_servo_function[CH_6] = (RC_Channel_aux::Aux_servo_function_t)rc_6->function.get();
	aux_servo_function[CH_7] = (RC_Channel_aux::Aux_servo_function_t)rc_7->function.get();
	aux_servo_function[CH_8] = (RC_Channel_aux::Aux_servo_function_t)rc_8->function.get();

	// Assume that no auxiliary function is used
	for (int i = 0; i < RC_Channel_aux::k_nr_aux_servo_functions ; i++)
	{
		g_rc_function[i] = NULL;
	}

	// assign the RC channel to each function
	g_rc_function[aux_servo_function[CH_5]] = rc_5;
	g_rc_function[aux_servo_function[CH_6]] = rc_6;
	g_rc_function[aux_servo_function[CH_7]] = rc_7;
	g_rc_function[aux_servo_function[CH_8]] = rc_8;

	//set auxiliary ranges
	G_RC_AUX(k_flap)->set_range(0,100);
	G_RC_AUX(k_flap_auto)->set_range(0,100);
	G_RC_AUX(k_aileron)->set_angle(4500);
	G_RC_AUX(k_flaperon)->set_range(0,100);
}
