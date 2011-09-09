void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){

		set_mode(g.flight_modes[switchPosition]);

		oldSwitchPosition = switchPosition;
		prev_WP = current_loc;

		// reset navigation integrators
		// -------------------------
		reset_I();
	}
}

byte readSwitch(void){
	uint16_t pulsewidth = APM_RC.InputCh(g.flight_mode_channel - 1);
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
	SendDebug_P("MSG: reset_control_switch");
	SendDebugln(oldSwitchPosition , DEC);
}

void update_servo_switches()
{
	// up is reverse
	// up is elevon
	mix_mode 		= (PINL & 128) ? 1 : 0;
	if (mix_mode == 0) {
		reverse_roll 	= (PINE & 128) ? true : false;
		reverse_pitch 	= (PINE & 64) ? true : false;
		reverse_rudder 	= (PINL & 64) ? true : false;
	} else {
		reverse_elevons 	= (PINE & 128) ? -1 : 1;
		reverse_ch1_elevon 	= (PINE & 64) ? -1 : 1;
		reverse_ch2_elevon 	= (PINL & 64) ? -1 : 1;
	}
}


bool trim_flag;
unsigned long trim_timer;

// read at 10 hz
// set this to your trainer switch
void read_trim_switch()
{
	// switch is engaged
	if (g.rc_7.control_in > 800){
		if(trim_flag == false){
			// called once
			trim_timer = millis();
		}
		trim_flag = true;

	}else{ // switch is disengaged

		if(trim_flag){
			// switch was just released
			if((millis() - trim_timer) > 2000){

				g.throttle_cruise.set_and_save(g.channel_throttle.control_in);
				g.angle_of_attack.set_and_save(dcm.pitch_sensor);
				g.airspeed_cruise.set_and_save(airspeed);

			} else {

			}
			trim_flag = false;
		}
	}
}
