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
	g.channel_azimuth.set_dead_zone(100);
	g.channel_elevation.set_dead_zone(100);

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

/*
 * Update our channel objects from physical radio.
 * Called (as the first thing) from fast_loop.
 */
static void read_radio() {
	/*
	 * Has same effect as RC_Channel::input() but (apart from setting the radio_in
	 * property) also calculated angles or ranges.
	 */
	g.channel_azimuth.set_pwm( hal.rcin->read(CH_AZIMUTH));
	g.channel_elevation.set_pwm( hal.rcin->read(CH_ELEVATION));
	g.rc_3.set_pwm(hal.rcin->read(CH_3));
	g.rc_4.set_pwm(hal.rcin->read(CH_4));

	// Let's say 1 second at full stick should get us from zero to min/max.
	azimuthRCIntegral += g.channel_azimuth.control_in;
	constrain_int32(azimuthRCIntegral, SERVO_MAX*50L, SERVO_MAX*50L);

	elevationRCIntegral += g.channel_elevation.control_in;
	constrain_int32(elevationRCIntegral, SERVO_MAX*50L, SERVO_MAX*50L);
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
