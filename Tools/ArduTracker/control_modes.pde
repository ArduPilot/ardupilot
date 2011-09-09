void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){
		
		set_mode(flight_mode(switchPosition));
		
		oldSwitchPosition = switchPosition;

		// reset navigation integrators
		// -------------------------
		reset_I();
	}
}

byte readSwitch(void){
	uint16_t pulsewidth = APM_RC.InputCh(get(PARAM_FLIGHT_MODE_CH) - 1);
	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
	SendDebug("MSG: reset_control_switch");
	SendDebugln(oldSwitchPosition , DEC);
}

void update_servo_switches()
{
	// up is reverse
	// up is elevon
	mix_mode 		= (PINL & 128) ? 1 : 0;
	if (mix_mode == 0) {
		reverse_roll 	= (PINE & 128) ? 1 : -1;
		reverse_pitch 	= (PINE & 64) ? 1 : -1;
		reverse_rudder 	= (PINL & 64) ? 1 : -1;
	} else {
		reverse_elevons 	= (PINE & 128) ? 1 : -1;
		reverse_ch1_elevon 	= (PINE & 64) ? 1 : -1;
		reverse_ch2_elevon 	= (PINL & 64) ? 1 : -1;
	}
}
