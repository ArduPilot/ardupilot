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
				// do_rtl sets the altitude to the current altitude by default
				set_mode(RTL);
				// We add an additional 10m to the current altitude
				//next_WP.alt += 1000;
				set_new_altitude(target_altitude + 1000);
			}
			// 2 = Stay in AUTO and ignore failsafe
			break;

		default:
			if(home_is_set == true){
				// same as above ^
				// do_rtl sets the altitude to the current altitude by default
				set_mode(RTL);
				// We add an additional 10m to the current altitude
				//next_WP.alt += 1000;
				set_new_altitude(target_altitude + 1000);
			}else{
				// We have no GPS so we must land
				set_mode(LAND);
			}
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
	gcs_send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
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
			relay.toggle();
		}
	}
}

#if PIEZO == ENABLED
void piezo_on()
{
	digitalWrite(PIEZO_PIN,HIGH);
	//PORTF |= B00100000;
}

void piezo_off()
{
	digitalWrite(PIEZO_PIN,LOW);
	//PORTF &= ~B00100000;
}

void piezo_beep()
{
	// Note: This command should not be used in time sensitive loops
	piezo_on();
	delay(100);
	piezo_off();
}
#endif
