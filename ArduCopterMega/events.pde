// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
	This event will be called when the failsafe changes
	boolean failsafe reflects the current state
*/
static void failsafe_on_event()
{
	// This is how to handle a failsafe.
	switch(control_mode)
	{
		case AUTO:
			if (g.throttle_fs_action == 1) {
				set_mode(RTL);
			}
			// 2 = Stay in AUTO and ignore failsafe

		default:
			// not ready to enable yet w/o more testing
			//set_mode(RTL);
			break;
	}
}

static void failsafe_off_event()
{
	if (g.throttle_fs_action == 2){
		// We're back in radio contact
		// return to AP
		// ---------------------------

		// re-read the switch so we can return to our preferred mode
		// --------------------------------------------------------
		reset_control_switch();

		// Reset control integrators
		// ---------------------
		//reset_nav_I();

	}else if (g.throttle_fs_action == 1){
		// We're back in radio contact
		// return to Home
		// we should already be in RTL and throttle set to cruise
		// ------------------------------------------------------
		set_mode(RTL);
	}
}

static void low_battery_event(void)
{
	gcs.send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
	low_batt = true;

	// if we are in Auto mode, come home
	if(control_mode >= AUTO)
		set_mode(RTL);
}


static void update_events()	// Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
	if(event_repeat == 0 || (millis() - event_timer) < event_delay)
		return;

	if (event_repeat > 0){
		event_repeat --;
	}

	if(event_repeat != 0) {		// event_repeat = -1 means repeat forever
		event_timer = millis();

		if (event_id >= CH_5 && event_id <= CH_8) {
			if(event_repeat%2) {
				APM_RC.OutputCh(event_id, event_value); // send to Servos
			} else {
				APM_RC.OutputCh(event_id, event_undo_value);
			}
		}

		if  (event_id == RELAY_TOGGLE) {
			relay_toggle();
		}
	}
}

static void relay_on()
{
	PORTL |= B00000100;
}

static void relay_off()
{
	PORTL &= ~B00000100;
}

static void relay_toggle()
{
	PORTL ^= B00000100;
}

