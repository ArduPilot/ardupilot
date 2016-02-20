/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

#define CONTROL_SWITCH_DEBOUNCE_TIME_MS  200

void Tracker::read_control_switch()
{
	uint32_t tnow_ms = hal.scheduler->millis();

	// calculate position of tracker mode switch
	int8_t switch_position;
	if      (g.channel_mode.radio_in < 1231) switch_position = 0;
	else if (g.channel_mode.radio_in < 1361) switch_position = 1;
	else if (g.channel_mode.radio_in < 1491) switch_position = 2;
	else if (g.channel_mode.radio_in < 1621) switch_position = 3;
	else if (g.channel_mode.radio_in < 1750) switch_position = 4;
	else switch_position = 5;

	if (initializing)
		return;

	// store time that switch last moved
	if(control_switch_state.last_switch_position != switch_position) {
		control_switch_state.last_edge_time_ms = tnow_ms;
	}

	// debounce switch
	bool control_switch_changed = control_switch_state.debounced_switch_position != switch_position;
	bool sufficient_time_elapsed = tnow_ms - control_switch_state.last_edge_time_ms > CONTROL_SWITCH_DEBOUNCE_TIME_MS;

	if (control_switch_changed && sufficient_time_elapsed) {
		// set tracker mode
		if (set_mode(tracker_modes[switch_position])) 
		{
			if (control_switch_state.debounced_switch_position != -1) {
				// alert user to mode change failure (except if autopilot is just starting up)
				AP_Notify::events.user_mode_change = 1;
			}            
		} 
		else if (control_switch_state.last_switch_position != -1) 
		{
			// alert user to mode change failure
			AP_Notify::events.user_mode_change_failed = 1;
		}

		// set the debounced switch position
		control_switch_state.debounced_switch_position = switch_position;
	}

	control_switch_state.last_switch_position = switch_position;
}