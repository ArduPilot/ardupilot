// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_long_on_event(int fstype)
{
	// This is how to handle a long loss of control signal failsafe.
	gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
    hal.rcin->clear_overrides();
	failsafe = fstype;
	switch(control_mode)
	{
		case MANUAL: 
		case LEARNING:
		case STEERING:
			set_mode(RTL);
			break;

		case AUTO: 
		case GUIDED: 
            set_mode(RTL);
			break;
			
		case RTL: 
		default:
			break;
	}
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

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
