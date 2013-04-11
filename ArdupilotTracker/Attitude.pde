// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
static void set_servos(void) {
	g.channel_azimuth.servo_out = // g.channel_azimuth.radio_in;
			bearing_error_cd;
	g.channel_azimuth.calc_pwm();
	g.channel_elevation.radio_out = g.channel_elevation.radio_in;
	
    // hal.rcout->write(CH_1, g.channel_azimuth.radio_out);       // send to Servos
    // hal.rcout->write(CH_2, g.channel_elevation.radio_out);      // send to Servos
    
    g.channel_azimuth.output();
    g.channel_elevation.output(); 
       
    //hal.rcout->write(CH_3, g.channel_throttle.radio_out);   // send to Servos
    //hal.rcout->write(CH_4, g.channel_rudder.radio_out);     // send to Servos
/*
	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);
# if CONFIG_HAL_BOARD == HAL_BOARD_APM2
	g.rc_9.output_ch(CH_9);
	g.rc_10.output_ch(CH_10);
	g.rc_11.output_ch(CH_11);
#endif
*/
}

static bool demoing_servos;

static void demo_servos(uint8_t i) {
	while (i > 0) {
		gcs_send_text_P(SEVERITY_LOW, PSTR("Demo Servos!"));
		demoing_servos = true;
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
		hal.rcout->write(1, 1400);
		mavlink_delay(400);
		hal.rcout->write(1, 1600);
		mavlink_delay(200);
		hal.rcout->write(1, 1500);
#endif
		demoing_servos = false;
		mavlink_delay(400);
		i--;
	}
}
