/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void read_control_switch()
{
	static bool switch_debouncer = false;
	byte switchPosition = readSwitch();

	if (oldSwitchPosition != switchPosition){
		if(switch_debouncer){
			// remember the prev location for GS
			prev_WP 			= current_loc;
			oldSwitchPosition 	= switchPosition;
			switch_debouncer 	= false;

			set_mode(flight_modes[switchPosition]);
		}else{
			switch_debouncer 	= true;
		}
	}
}

static byte readSwitch(void){
	int pulsewidth = g.rc_5.radio_in;			// default for Arducopter

	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

static void reset_control_switch()
{
	oldSwitchPosition = -1;
	read_control_switch();
}

static boolean trim_flag;

// read at 10 hz
// set this to your trainer switch
static void read_trim_switch()
{
#if CH7_OPTION == DO_FLIP
	if (g.rc_7.control_in > 800 && g.rc_3.control_in != 0){
		do_flip = true;
	}

#elif CH7_OPTION == DO_SET_HOVER
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
#endif
}

static void auto_trim()
{
	if(auto_level_counter > 0){
		//g.rc_1.dead_zone = 60;		// 60 = .6 degrees
		//g.rc_2.dead_zone = 60;

		auto_level_counter--;
		trim_accel();
		led_mode = AUTO_TRIM_LEDS;

		if(auto_level_counter == 1){
			//g.rc_1.dead_zone = 0;		// 60 = .6 degrees
			//g.rc_2.dead_zone = 0;
			led_mode = NORMAL_LEDS;
			clear_leds();
			imu.save();
			Serial.println("Done");
			auto_level_counter = 0;
		}
	}
}



static void trim_accel()
{
	g.pi_stabilize_roll.reset_I();
	g.pi_stabilize_pitch.reset_I();

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

