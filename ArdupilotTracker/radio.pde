// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
// static uint8_t failsafeCounter = 0; // we wait a second to take over the throttle and send the plane circling

extern RC_Channel* rc_ch[4];

static void init_rc_in() {
	// set rc channel ranges
	g.channel_azimuth.set_angle(SERVO_MAX);
	g.channel_elevation.set_angle(SERVO_MAX);

	// set rc dead zones
	g.channel_azimuth.set_dead_zone(60);
	g.channel_elevation.set_dead_zone(60);

	rc_ch[CH_1] = &g.channel_azimuth;
	rc_ch[CH_2] = &g.channel_elevation;
	rc_ch[CH_3] = &g.rc_3;
	rc_ch[CH_4] = &g.rc_4;
	//rc_ch[CH_5] = &g.rc_5;
	//rc_ch[CH_6] = &g.rc_6;
	//rc_ch[CH_7] = &g.rc_7;
	//rc_ch[CH_8] = &g.rc_8;

	//set auxiliary ranges
	update_aux_servo_function(&g.rc_3, &g.rc_4);
}

static void init_rc_out() {
	hal.rcout->enable_ch(CH_1);
	hal.rcout->enable_ch(CH_2);
	enable_aux_servos();

	// Initialization of servo outputs
	// OK so this is one initial value: The trim value.
	hal.rcout->write(CH_1, g.channel_azimuth.radio_trim);
	hal.rcout->write(CH_2, g.channel_elevation.radio_trim);
	hal.rcout->write(CH_3, g.rc_3.radio_min);
	hal.rcout->write(CH_4, g.rc_4.radio_trim);
}

/*
 * Update our channel objects from physical radio.
 * Called (as the first thing) from fast_loop.
 */
static void read_radio() {
	uint16_t pwm_azimuth = hal.rcin->read(CH_AZIMUTH);
	uint16_t pwm_elevation = hal.rcin->read(CH_ELEVATION);

	/*
	 * Has same effect as RC_Channel::input() but (apart from setting the radio_in
	 * property) also calculated angles or ranges.
	 */
	g.channel_azimuth.set_pwm(pwm_azimuth);
	g.channel_elevation.set_pwm(pwm_elevation);
	g.rc_3.set_pwm(hal.rcin->read(CH_3));
	g.rc_4.set_pwm(hal.rcin->read(CH_4));

	//  control_failsafe(g.channel_throttle.radio_in);
	//  g.channel_throttle.servo_out = g.channel_throttle.control_in;

	/*
	 *  cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"),
	 *                       (int)g.rc_1.control_in,
	 *                       (int)g.rc_2.control_in,
	 *                       (int)g.rc_3.control_in,
	 *                       (int)g.rc_4.control_in);
	 */
}

static void trim_tracker() {
	read_radio();
	// Store control surface trim values
	// ---------------------------------
	if (g.channel_azimuth.radio_in != 0) {
		g.channel_azimuth.radio_trim = g.channel_azimuth.radio_in;
	}
	if (g.channel_elevation.radio_in != 0) {
		g.channel_elevation.radio_trim = g.channel_elevation.radio_in;
	}

	// save to eeprom
	g.channel_azimuth.save_eeprom();
	g.channel_elevation.save_eeprom();
}

static void trim_radio() {
	for (uint8_t y = 0; y < 30; y++) {
		read_radio();
	}

	trim_tracker();
}
