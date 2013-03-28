// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void update_events(void)	// Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
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
				hal.rcout->write(event_id, event_value); // send to Servos
			} else {
				hal.rcout->write(event_id, event_undo_value);
			}
		}

		if  (event_id == RELAY_TOGGLE) {
			relay.toggle();
		}
	}
}
