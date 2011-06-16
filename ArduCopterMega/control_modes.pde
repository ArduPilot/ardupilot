/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

void read_control_switch()
{
	byte switchPosition = readSwitch();
	//motor_armed = (switchPosition < 5);

	if (oldSwitchPosition != switchPosition){

		set_mode(g.flight_modes[switchPosition]);

		oldSwitchPosition = switchPosition;
		prev_WP = current_loc;

		// reset navigation integrators
		// -------------------------
		//reset_I();
	}
}

byte readSwitch(void){
	int pulsewidth = g.rc_5.radio_in;			// default for Arducopter

	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

void reset_control_switch()
{
	oldSwitchPosition = -1;
	read_control_switch();
	SendDebug("MSG: reset_control_switch ");
	SendDebugln(oldSwitchPosition , DEC);
}

void update_servo_switches()
{

}

boolean trim_flag;
unsigned long trim_timer;

// read at 10 hz
// set this to your trainer switch
void read_trim_switch()
{
	// switch is engaged
	if (g.rc_7.control_in > 800){
		trim_flag = true;

	}else{ // switch is disengaged

		if(trim_flag){
			// set the throttle nominal
			if(g.rc_3.control_in > 150){
				g.throttle_cruise.set_and_save(g.rc_3.control_in);
					//Serial.printf("tnom %d\n", g.throttle_cruise.get());
			}
			trim_flag = false;
		}
	}
}

void auto_trim()
{
	if(auto_level_counter > 0){
		auto_level_counter--;
		trim_accel();
		led_mode = AUTO_TRIM_LEDS;

		if(auto_level_counter == 1){
			led_mode = NORMAL_LEDS;
			clear_leds();
			imu.save();
			Serial.println("Done");
			auto_level_counter = 0;
		}
	}
}



void trim_accel()
{
	g.pid_stabilize_roll.reset_I();
	g.pid_stabilize_pitch.reset_I();

	if(g.rc_1.control_in > 0){
		imu.ay(imu.ay() + 1);
	}else if (g.rc_1.control_in < 0){
		imu.ay(imu.ay() - 1);
	}

	if(g.rc_2.control_in > 0){
		imu.ax(imu.ax() + 1);
	}else if (g.rc_2.control_in < 0){
		imu.ax(imu.ax() - 1);
	}

	/*
	Serial.printf_P(PSTR("r:%ld p:%ld ax:%f, ay:%f, az:%f\n"),
							dcm.roll_sensor,
							dcm.pitch_sensor,
							(float)imu.ax(),
							(float)imu.ay(),
							(float)imu.az());
	//*/
}

