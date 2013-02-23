// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

extern RC_Channel* rc_ch[NUM_CHANNELS];

static void default_dead_zones()
{
    g.rc_1.set_dead_zone(60);
    g.rc_2.set_dead_zone(60);
    g.rc_3.set_dead_zone(60);
    g.rc_4.set_dead_zone(80);
}

static void init_rc_in()
{
	// set rc channel ranges
	g.rc_1.set_angle(MAX_INPUT_YAW_ANGLE);
	g.rc_2.set_angle(MAX_INPUT_PITCH_ANGLE);
	g.rc_3.set_range(0,1000);
	g.rc_4.set_angle(4500);

	g.rc_1.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_2.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_4.set_type(RC_CHANNEL_ANGLE_RAW);

	//set auxiliary ranges
	g.rc_5.set_range(0,1000);
	g.rc_6.set_angle(300);
	g.rc_7.set_range(0,1000);
	g.rc_8.set_range(0,1000);

	//hack
	default_dead_zones();
}

static void init_rc_out()
{
	APM_RC.Init( &isr_registry );		// APM Radio initialization
	init_motors_out();
	motors_output_enable();
}

static void read_radio()
{
	if (APM_RC.GetState() == 1){
		ap_system.new_radio_frame = true;
		g.rc_1.set_pwm(APM_RC.InputCh(CH_1));
		g.rc_2.set_pwm(APM_RC.InputCh(CH_2));
		g.rc_3.set_pwm(APM_RC.InputCh(CH_3));
		g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
		g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
		g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
		g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
		g.rc_8.set_pwm(APM_RC.InputCh(CH_8));
	}
}

static void trim_radio()
{
    for (byte i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}

