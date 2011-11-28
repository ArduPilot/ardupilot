// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plane circling

static void default_dead_zones()
{
	g.rc_1.set_dead_zone(60);
	g.rc_2.set_dead_zone(60);
	#if FRAME_CONFIG == HELI_FRAME
	    g.rc_3.set_dead_zone(20);
		g.rc_4.set_dead_zone(30);
	#else
	    g.rc_3.set_dead_zone(60);
		g.rc_4.set_dead_zone(200);
	#endif
}

static void init_rc_in()
{
	// set rc channel ranges
	g.rc_1.set_angle(4500);
	g.rc_2.set_angle(4500);
	g.rc_3.set_range(0,1000);
    #if FRAME_CONFIG !=	HELI_FRAME
	g.rc_3.scale_output = .9;
	#endif
	g.rc_4.set_angle(4500);

	// reverse: CW = left
	// normal:  CW = left???


	g.rc_1.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_2.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_4.set_type(RC_CHANNEL_ANGLE_RAW);

	// set rc dead zones
	/*g.rc_1.dead_zone = 60;
	g.rc_2.dead_zone = 60;
	g.rc_3.dead_zone = 60;
	g.rc_4.dead_zone = 300;
	*/


	//set auxiliary ranges
	g.rc_5.set_range(0,1000);
	g.rc_6.set_range(0,1000);
	g.rc_7.set_range(0,1000);
	g.rc_8.set_range(0,1000);

}

static void init_rc_out()
{
	APM_RC.Init( &isr_registry );		// APM Radio initialization
	init_motors_out();

	// this is the camera pitch5 and roll6
	APM_RC.OutputCh(CH_5, 1500);
	APM_RC.OutputCh(CH_6, 1500);

	for(byte i = 0; i < 5; i++){
		delay(20);
		read_radio();
	}

    // sanity check - prevent unconfigured radios from outputting
    if(g.rc_3.radio_min >= 1300){
        g.rc_3.radio_min = g.rc_3.radio_in;
    }

	// we are full throttle
	if(g.rc_3.control_in == 800){
		if(g.esc_calibrate == 0){
			// we will enter esc_calibrate mode on next reboot
			g.esc_calibrate.set_and_save(1);
			// send miinimum throttle out to ESC
			output_min();
			// block until we restart
			while(1){
				//Serial.println("esc");
				delay(200);
				dancing_light();
			}
		}else{
			//Serial.println("esc init");
			// clear esc flag
			g.esc_calibrate.set_and_save(0);
			// block until we restart
	        init_esc();
		}
	}else{
		// did we abort the calibration?
		if(g.esc_calibrate == 1)
			g.esc_calibrate.set_and_save(0);

		// send miinimum throttle out to ESC
		output_min();
	}
}

void output_min()
{
    #if FRAME_CONFIG ==	HELI_FRAME
        heli_move_servos_to_mid();
	#else
	    APM_RC.OutputCh(CH_1, 	g.rc_3.radio_min);					// Initialization of servo outputs
	    APM_RC.OutputCh(CH_2, 	g.rc_3.radio_min);
	    APM_RC.OutputCh(CH_3, 	g.rc_3.radio_min);
	    APM_RC.OutputCh(CH_4, 	g.rc_3.radio_min);
	#endif

	APM_RC.OutputCh(CH_7,   g.rc_3.radio_min);
    APM_RC.OutputCh(CH_8,   g.rc_3.radio_min);

	#if FRAME_CONFIG ==	OCTA_FRAME
	APM_RC.OutputCh(CH_10,   g.rc_3.radio_min);
    APM_RC.OutputCh(CH_11,   g.rc_3.radio_min);
	#endif

}
static void read_radio()
{
	if (APM_RC.GetState() == 1){
		new_radio_frame = true;
		g.rc_1.set_pwm(APM_RC.InputCh(CH_1));
		g.rc_2.set_pwm(APM_RC.InputCh(CH_2));
		g.rc_3.set_pwm(APM_RC.InputCh(CH_3));
		g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
		g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
		g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
		g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
		g.rc_8.set_pwm(APM_RC.InputCh(CH_8));

		#if FRAME_CONFIG != HELI_FRAME
			// limit our input to 800 so we can still pitch and roll
			g.rc_3.control_in = min(g.rc_3.control_in, 800);
		#endif

		throttle_failsafe(g.rc_3.radio_in);
	}
}

static void throttle_failsafe(uint16_t pwm)
{
	if(g.throttle_fs_enabled == 0)
		return;

	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (pwm < (unsigned)g.throttle_fs_value){
		// we detect a failsafe from radio
		// throttle has dropped below the mark
		failsafeCounter++;
		if (failsafeCounter == 9){
			SendDebug("MSG FS ON ");
			SendDebugln(pwm, DEC);
		}else if(failsafeCounter == 10) {
			//ch3_failsafe = true;
			set_failsafe(true);
			//failsafeCounter = 10;
		}else if (failsafeCounter > 10){
			failsafeCounter = 11;
		}

	}else if(failsafeCounter > 0){
		// we are no longer in failsafe condition
		// but we need to recover quickly
		failsafeCounter--;
		if (failsafeCounter > 3){
			failsafeCounter = 3;
		}
		if (failsafeCounter == 1){
			SendDebug("MSG FS OFF ");
			SendDebugln(pwm, DEC);
		}else if(failsafeCounter == 0) {
			//ch3_failsafe = false;
			set_failsafe(false);
			//failsafeCounter = -1;
		}else if (failsafeCounter <0){
			failsafeCounter = -1;
		}
	}
}

static void trim_radio()
{
	for (byte i = 0; i < 30; i++){
		read_radio();
	}

	g.rc_1.trim();	// roll
	g.rc_2.trim();	// pitch
	g.rc_4.trim();	// yaw

	g.rc_1.save_eeprom();
	g.rc_2.save_eeprom();
	g.rc_4.save_eeprom();
}

