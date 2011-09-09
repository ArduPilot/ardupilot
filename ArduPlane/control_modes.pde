/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){

		set_mode(flight_modes[switchPosition]);

		oldSwitchPosition = switchPosition;
		prev_WP = current_loc;

		// reset navigation integrators
		// -------------------------
		reset_I();
	}

    if (g.inverted_flight_ch != 0) {
        // if the user has configured an inverted flight channel, then
        // fly upside down when that channel goes above INVERTED_FLIGHT_PWM
        inverted_flight = (control_mode != MANUAL && APM_RC.InputCh(g.inverted_flight_ch-1) > INVERTED_FLIGHT_PWM);
    }
}

static byte readSwitch(void){
	uint16_t pulsewidth = APM_RC.InputCh(g.flight_mode_channel - 1);
	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

static void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
	SendDebug_P("MSG: reset_control_switch");
	SendDebugln(oldSwitchPosition , DEC);
}

static void update_servo_switches()
{
	if (!g.switch_enable) {
        // switches are disabled in EEPROM (see SWITCH_ENABLE option)
        // this means the EEPROM control of all channel reversal is enabled
		return;
	}
	// up is reverse
	// up is elevon
	g.mix_mode 		= (PINL & 128) ? 1 : 0; // 1 for elevon mode
	if (g.mix_mode == 0) {
		g.channel_roll.set_reverse((PINE & 128) ? true : false);
		g.channel_pitch.set_reverse((PINE & 64) ? true : false);
		g.channel_rudder.set_reverse((PINL & 64) ? true : false);
	} else {
		g.reverse_elevons 	= (PINE & 128) ? true : false;
		g.reverse_ch1_elevon = (PINE & 64) ? true : false;
		g.reverse_ch2_elevon = (PINL & 64) ? true : false;
	}
}
