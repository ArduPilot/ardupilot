// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_rc_in()
{
	// set rc channel ranges
	g.channel_steer.set_angle(SERVO_MAX);
	g.channel_throttle.set_angle(100);

	// set rc dead zones
	g.channel_steer.set_dead_zone(60);
	g.channel_throttle.set_dead_zone(6);

	//set auxiliary ranges
    update_aux_servo_function(&g.rc_2, &g.rc_4, &g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
}

static void init_rc_out()
{
    hal.rcout->enable_ch(CH_1);
    hal.rcout->enable_ch(CH_2);
    hal.rcout->enable_ch(CH_3);
    hal.rcout->enable_ch(CH_4);
    hal.rcout->enable_ch(CH_5);
    hal.rcout->enable_ch(CH_6);
    hal.rcout->enable_ch(CH_7);
    hal.rcout->enable_ch(CH_8);

#if HIL_MODE != HIL_MODE_ATTITUDE
	hal.rcout->write(CH_1, 	g.channel_steer.radio_trim);					// Initialization of servo outputs
	hal.rcout->write(CH_3, 	g.channel_throttle.radio_trim);

	hal.rcout->write(CH_2, 	g.rc_2.radio_trim);
	hal.rcout->write(CH_4, 	g.rc_4.radio_trim);
	hal.rcout->write(CH_5, 	g.rc_5.radio_trim);
	hal.rcout->write(CH_6, 	g.rc_6.radio_trim);
	hal.rcout->write(CH_7,   g.rc_7.radio_trim);
    hal.rcout->write(CH_8,   g.rc_8.radio_trim);
#else
	hal.rcout->write(CH_1, 	1500);					// Initialization of servo outputs
	hal.rcout->write(CH_2, 	1500);
	hal.rcout->write(CH_3, 	1000);
	hal.rcout->write(CH_4, 	1500);

	hal.rcout->write(CH_5, 	1500);
	hal.rcout->write(CH_6, 	1500);
	hal.rcout->write(CH_7,   1500);
    hal.rcout->write(CH_8,   2000);
#endif

}

static void read_radio()
{
    g.channel_steer.set_pwm(hal.rcin->read(CH_STEER));

	g.channel_throttle.set_pwm(hal.rcin->read(CH_3));

  	g.rc_2.set_pwm(hal.rcin->read(CH_2));
  	g.rc_4.set_pwm(hal.rcin->read(CH_4));
  	g.rc_5.set_pwm(hal.rcin->read(CH_5));
 	g.rc_6.set_pwm(hal.rcin->read(CH_6));        
	g.rc_7.set_pwm(hal.rcin->read(CH_7));
	g.rc_8.set_pwm(hal.rcin->read(CH_8));

	control_failsafe(g.channel_throttle.radio_in);

	g.channel_throttle.servo_out = g.channel_throttle.control_in;

	if (g.channel_throttle.servo_out > 50) {
        throttle_nudge = (g.throttle_max - g.throttle_cruise) * ((g.channel_throttle.norm_input()-0.5) / 0.5);
	} else {
		throttle_nudge = 0;
	}

    if (g.skid_steer_in) {
        // convert the two radio_in values from skid steering values
        /*
          mixing rule:
          steering = motor1 - motor2
          throttle = 0.5*(motor1 + motor2)
          motor1 = throttle + 0.5*steering
          motor2 = throttle - 0.5*steering
        */          

        float motor1 = g.channel_steer.norm_input();
        float motor2 = g.channel_throttle.norm_input();
        float steering_scaled = motor1 - motor2;
        float throttle_scaled = 0.5f*(motor1 + motor2);
        int16_t steer = g.channel_steer.radio_trim;
        int16_t thr   = g.channel_throttle.radio_trim;
        if (steering_scaled > 0.0f) {
            steer += steering_scaled*(g.channel_steer.radio_max-g.channel_steer.radio_trim);
        } else {
            steer += steering_scaled*(g.channel_steer.radio_trim-g.channel_steer.radio_min);
        }
        if (throttle_scaled > 0.0f) {
            thr += throttle_scaled*(g.channel_throttle.radio_max-g.channel_throttle.radio_trim);
        } else {
            thr += throttle_scaled*(g.channel_throttle.radio_trim-g.channel_throttle.radio_min);
        }
        g.channel_steer.set_pwm(steer);
        g.channel_throttle.set_pwm(thr);
    }
}

static void control_failsafe(uint16_t pwm)
{
	if (!g.fs_throttle_enabled) {
        // no throttle failsafe
		return;
    }

	// Check for failsafe condition based on loss of GCS control
	if (rc_override_active) {
        failsafe_trigger(FAILSAFE_EVENT_RC, (millis() - failsafe.rc_override_timer) > 1500);
	} else if (g.fs_throttle_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_THROTTLE, pwm < (uint16_t)g.fs_throttle_value);
	}
}

static void trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
    if (g.channel_steer.radio_in > 1400) {
		g.channel_steer.radio_trim = g.channel_steer.radio_in;
        // save to eeprom
        g.channel_steer.save_eeprom();
    }
}

static void trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}
    trim_control_surfaces();
}
