// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Camera.h>
#include <RC_Channel.h>

extern RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];	// the aux. servo ch. assigned to each function
extern long	wp_distance;
extern "C" {
void relay_on();
void relay_off();
}

void
AP_Camera::servo_pic()		// Servo operated camera
{
	if (g_rc_function[RC_Channel_aux::k_cam_trigger])
	{
		g_rc_function[RC_Channel_aux::k_cam_trigger]->radio_out = g_rc_function[RC_Channel_aux::k_cam_trigger]->radio_max;
		keep_cam_trigg_active_cycles = 2;	// leave a message that it should be active for two event loop cycles
	}
}

void
AP_Camera::relay_pic()		// basic relay activation
{
	relay_on();
	keep_cam_trigg_active_cycles = 2;	// leave a message that it should be active for two event loop cycles
}

void
AP_Camera::throttle_pic()		// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
{
// TODO find a way to do this without using the global parameter g
//	g.channel_throttle.radio_out = g.throttle_min;
	if (thr_pic == 10){
		servo_pic();	// triggering method
		thr_pic = 0;
//		g.channel_throttle.radio_out = g.throttle_cruise;
	}
	thr_pic++;
}

void
AP_Camera::distance_pic()		// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
{
// TODO find a way to do this without using the global parameter g
//	g.channel_throttle.radio_out = g.throttle_min;
	if (wp_distance < 3){
		servo_pic();	// triggering method
//		g.channel_throttle.radio_out = g.throttle_cruise;
	}
}

void
AP_Camera::NPN_pic()		// hacked the circuit to run a transistor? use this trigger to send output.
{
	// TODO: Assign pin spare pin for output
	digitalWrite(camtrig, HIGH);
	keep_cam_trigg_active_cycles = 1;	// leave a message that it should be active for two event loop cycles
}

// single entry point to take pictures
void
AP_Camera::trigger_pic()
{
	switch (trigger_type)
	{
	case 0:
		servo_pic();		// Servo operated camera
		break;
	case 1:
		relay_pic();		// basic relay activation
		break;
	case 2:
		throttle_pic();		// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
		break;
	case 3:
		distance_pic();		// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
		break;
	case 4:
		NPN_pic();			// hacked the circuit to run a transistor? use this trigger to send output.
		break;
	}
}

// de-activate the trigger after some delay, but without using a delay() function
void
AP_Camera::trigger_pic_cleanup()
{
	if (keep_cam_trigg_active_cycles)
	{
		keep_cam_trigg_active_cycles --;
	}
	else
	{
		switch (trigger_type)
		{
		case 0:
		case 2:
		case 3:
			if (g_rc_function[RC_Channel_aux::k_cam_trigger])
			{
				g_rc_function[RC_Channel_aux::k_cam_trigger]->radio_out = g_rc_function[RC_Channel_aux::k_cam_trigger]->radio_min;
			}
			break;
		case 1:
			// TODO for some strange reason the function call bellow gives a linker error
			//relay_off();
			PORTL &= ~B00000100; // hardcoded version of relay_off(). Replace with a proper function call later.
			break;
		case 4:
			digitalWrite(camtrig, LOW);
			break;
		}
	}
}
