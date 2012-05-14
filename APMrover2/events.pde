// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_short_on_event(int fstype)
{
	// This is how to handle a short loss of control signal failsafe.
	failsafe = fstype;
	ch3_failsafe_timer = millis();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event on, "));
	switch(control_mode)
	{
		case MANUAL: 
		case LEARNING:
		case FLY_BY_WIRE_A: // middle position
		case FLY_BY_WIRE_B: // middle position
			set_mode(CIRCLE);
			break;

		case AUTO: 
		case GUIDED: 
		case LOITER: 
			if(g.short_fs_action == 1) {
				set_mode(RTL);
			}
			break;
			
		case CIRCLE: 
		case RTL: 
		default:
			break;
	}
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_long_on_event(int fstype)
{
	// This is how to handle a long loss of control signal failsafe.
	gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
	APM_RC.clearOverride();		//  If the GCS is locked up we allow control to revert to RC
	failsafe = fstype;
	switch(control_mode)
	{
		case MANUAL: 
		case LEARNING:
		case FLY_BY_WIRE_A: // middle position
		case FLY_BY_WIRE_B: // middle position
		case CIRCLE: 
			set_mode(RTL);
			break;

		case AUTO: 
		case GUIDED: 
		case LOITER: 
			if(g.long_fs_action == 1) {
				set_mode(RTL);
			}
			break;
			
		case RTL: 
		default:
			break;
	}
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_short_off_event()
{
	// We're back in radio contact
	gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event off"));
	failsafe = FAILSAFE_NONE;

	// re-read the switch so we can return to our preferred mode
	// --------------------------------------------------------
	reset_control_switch();

	// Reset control integrators
	// ---------------------
	reset_I();
}

#if BATTERY_EVENT == ENABLED
static void low_battery_event(void)
{
	gcs_send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
	set_mode(RTL);
	g.throttle_cruise = THROTTLE_CRUISE;
}
#endif

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
