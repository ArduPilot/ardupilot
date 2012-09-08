// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plane circling


static void init_rc_in()
{
	// set rc reversing
	update_servo_switches();

	// set rc channel ranges
	g.channel_roll.set_angle(SERVO_MAX);
	g.channel_pitch.set_angle(SERVO_MAX);
	g.channel_rudder.set_angle(SERVO_MAX);
	g.channel_throttle.set_range(0, 100);

	// set rc dead zones
	g.channel_roll.set_dead_zone(60);
	g.channel_pitch.set_dead_zone(60);
	g.channel_rudder.set_dead_zone(60);
	g.channel_throttle.set_dead_zone(6);

	//g.channel_roll.dead_zone 	= 60;
	//g.channel_pitch.dead_zone 	= 60;
	//g.channel_rudder.dead_zone 	= 60;
	//g.channel_throttle.dead_zone = 6;

	//set auxiliary ranges
	update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
}

static void init_rc_out()
{
  APM_RC.Init( &isr_registry );		// APM Radio initialization

  APM_RC.enable_out(CH_1);
  APM_RC.enable_out(CH_2);
  APM_RC.enable_out(CH_3);
  APM_RC.enable_out(CH_4);
  APM_RC.enable_out(CH_5);
  APM_RC.enable_out(CH_6);
  APM_RC.enable_out(CH_7);
  APM_RC.enable_out(CH_8);

#if HIL_MODE != HIL_MODE_ATTITUDE
	APM_RC.OutputCh(CH_1, 	g.channel_roll.radio_trim);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	g.channel_pitch.radio_trim);
	APM_RC.OutputCh(CH_3, 	g.channel_throttle.radio_min);
	APM_RC.OutputCh(CH_4, 	g.channel_rudder.radio_trim);

	APM_RC.OutputCh(CH_5, 	g.rc_5.radio_trim);
	APM_RC.OutputCh(CH_6, 	g.rc_6.radio_trim);
	APM_RC.OutputCh(CH_7,   g.rc_7.radio_trim);
        APM_RC.OutputCh(CH_8,   g.rc_8.radio_trim);
#else
	APM_RC.OutputCh(CH_1, 	1500);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	1500);
	APM_RC.OutputCh(CH_3, 	1000);
	APM_RC.OutputCh(CH_4, 	1500);

	APM_RC.OutputCh(CH_5, 	1500);
	APM_RC.OutputCh(CH_6, 	1500);
	APM_RC.OutputCh(CH_7,   1500);
        APM_RC.OutputCh(CH_8,   2000);
#endif

}

static void read_radio()
{
      static uint16_t aileron1;
      static uint16_t aileron2;   
      
	ch1_temp = APM_RC.InputCh(CH_ROLL);
	ch2_temp = APM_RC.InputCh(CH_PITCH);

	if(g.mix_mode == 0){
		g.channel_roll.set_pwm(ch1_temp);
		g.channel_pitch.set_pwm(ch2_temp);
	}else{
		g.channel_roll.set_pwm(BOOL_TO_SIGN(g.reverse_elevons) * (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) - BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500);
		g.channel_pitch.set_pwm((BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) + BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500);
	}

	g.channel_throttle.set_pwm(APM_RC.InputCh(CH_3));
	g.channel_rudder.set_pwm(APM_RC.InputCh(CH_4));
  	g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
 	g.rc_6.set_pwm(APM_RC.InputCh(CH_6));        
	g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
	g.rc_8.set_pwm(APM_RC.InputCh(CH_8));

	//  TO DO  - go through and patch throttle reverse for RC_Channel library compatibility
	#if THROTTLE_REVERSE == 1
		g.channel_throttle.radio_in = g.channel_throttle.radio_max + g.channel_throttle.radio_min - g.channel_throttle.radio_in;
	#endif

	control_failsafe(g.channel_throttle.radio_in);

	g.channel_throttle.servo_out = g.channel_throttle.control_in;

	if (g.channel_throttle.servo_out > 50) {
		if(g.airspeed_enabled == true) {
			airspeed_nudge = (g.flybywire_airspeed_max * 100 - g.airspeed_cruise) * ((g.channel_throttle.norm_input()-0.5) / 0.5);
                        airspeed_nudge = NUDGE_OFFSET + airspeed_nudge;
        } else {
			throttle_nudge = (g.throttle_max - g.throttle_cruise) * ((g.channel_throttle.norm_input()-0.5) / 0.5);
		}
	} else {
		airspeed_nudge = NUDGE_OFFSET;
		throttle_nudge = 0;
	}

	/*
	Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"),
				g.rc_1.control_in,
				g.rc_2.control_in,
				g.rc_3.control_in,
				g.rc_4.control_in);
	*/
}

static void control_failsafe(uint16_t pwm)
{
	if(g.throttle_fs_enabled == 0)
		return;

	// Check for failsafe condition based on loss of GCS control
	if (rc_override_active) {
		if(millis() - rc_override_fs_timer > FAILSAFE_SHORT_TIME) {
			ch3_failsafe = true;
		} else {
			ch3_failsafe = false;
		}

	//Check for failsafe and debounce funky reads
	} else if (g.throttle_fs_enabled) {
		if (pwm < (unsigned)g.throttle_fs_value){
			// we detect a failsafe from radio
			// throttle has dropped below the mark
			failsafeCounter++;
			if (failsafeCounter == 9){
				gcs_send_text_fmt(PSTR("MSG FS ON %u"), (unsigned)pwm);
			}else if(failsafeCounter == 10) {
				ch3_failsafe = true;
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
				gcs_send_text_fmt(PSTR("MSG FS OFF %u"), (unsigned)pwm);
			}else if(failsafeCounter == 0) {
				ch3_failsafe = false;
			}else if (failsafeCounter <0){
				failsafeCounter = -1;
			}
		}
	}
}

static void trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
	if(g.mix_mode == 0){
                if ((g.channel_roll.radio_in > 1400) && (g.channel_pitch.radio_trim > 1400)) {
		g.channel_roll.radio_trim = g.channel_roll.radio_max + g.channel_roll.radio_min - g.channel_roll.radio_in;
		g.channel_pitch.radio_trim = g.channel_pitch.radio_max + g.channel_pitch.radio_min - g.channel_pitch.radio_in;
		g.channel_rudder.radio_trim = g.channel_rudder.radio_max + g.channel_rudder.radio_min - g.channel_rudder.radio_in;
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_aileron);
                } else {
		g.channel_roll.radio_trim 		= 1500;    // case of HIL test without receiver active
		g.channel_pitch.radio_trim 		= 1500;
                g.channel_rudder.radio_trim             = 1500;
                g.channel_throttle.radio_trim 	        = 1000;
                }

	}else{
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		//Recompute values here using new values for elevon1_trim and elevon2_trim
		//We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
		uint16_t center 			= 1500;
		g.channel_roll.radio_trim 	= center;
		g.channel_pitch.radio_trim 	= center;
	}

	// save to eeprom
	g.channel_roll.save_eeprom();
	g.channel_pitch.save_eeprom();
	//g.channel_throttle.save_eeprom();
	g.channel_rudder.save_eeprom();
}

static void trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}

	// Store the trim values
	// ---------------------
	if(g.mix_mode == 0){
                if ((g.channel_roll.radio_in > 1400) && (g.channel_pitch.radio_trim > 1400)) {
		g.channel_roll.radio_trim 		= g.channel_roll.radio_in;
		g.channel_pitch.radio_trim 		= g.channel_pitch.radio_in;
		//g.channel_throttle.radio_trim 	= g.channel_throttle.radio_in;
		g.channel_rudder.radio_trim 	= g.channel_rudder.radio_in;
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_aileron);
                } else {
		g.channel_roll.radio_trim 		= 1500;    // case of HIL test without receiver active
		g.channel_pitch.radio_trim 		= 1500;
                g.channel_rudder.radio_trim             = 1500;
                g.channel_throttle.radio_trim 	        = 1000;
                }

	} else {
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		uint16_t center = 1500;
		g.channel_roll.radio_trim 	= center;
		g.channel_pitch.radio_trim 	= center;
		g.channel_rudder.radio_trim = g.channel_rudder.radio_in;
	}

	// save to eeprom
	g.channel_roll.save_eeprom();
	g.channel_pitch.save_eeprom();
	//g.channel_throttle.save_eeprom();
	g.channel_rudder.save_eeprom();
}
