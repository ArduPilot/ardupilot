void init_rc_in()
{
	read_EEPROM_radio();		// read Radio limits
	g.rc_1.set_angle(4500);
	g.rc_1.dead_zone = 60;		// 60 = .6 degrees
	g.rc_2.set_angle(4500);
	g.rc_2.dead_zone = 60;	
	g.rc_3.set_range(0,1000);
	g.rc_3.dead_zone = 20;
	g.rc_3.scale_output = .9;
	g.rc_4.set_angle(6000); 
	g.rc_4.dead_zone = 500;
	g.rc_5.set_range(0,1000);
	g.rc_5.set_filter(false);

	// for kP values
	//g.rc_6.set_range(200,800);
	//g.rc_6.set_range(0,1800);  // for faking GPS
	g.rc_6.set_range(0,1000);

	// for camera angles
	//g.rc_6.set_angle(4500);
	//g.rc_6.dead_zone = 60;

	g.rc_7.set_range(0,1000);
	g.rc_8.set_range(0,1000);
}

void init_rc_out()
{
	#if ARM_AT_STARTUP == 1
		motor_armed = 1;
	#endif

	APM_RC.OutputCh(CH_1, 	g.rc_3.radio_min);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	g.rc_3.radio_min);
	APM_RC.OutputCh(CH_3, 	g.rc_3.radio_min);
	APM_RC.OutputCh(CH_4, 	g.rc_3.radio_min);

	APM_RC.OutputCh(CH_7,     g.rc_3.radio_min);
    APM_RC.OutputCh(CH_8,     g.rc_3.radio_min);

	APM_RC.Init();		// APM Radio initialization

	APM_RC.OutputCh(CH_1, 	g.rc_3.radio_min);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	g.rc_3.radio_min);
	APM_RC.OutputCh(CH_3, 	g.rc_3.radio_min);
	APM_RC.OutputCh(CH_4, 	g.rc_3.radio_min);
	
	
	APM_RC.OutputCh(CH_7,     g.rc_3.radio_min);
    APM_RC.OutputCh(CH_8,     g.rc_3.radio_min);

}

void read_radio()
{
	g.rc_1.set_pwm(APM_RC.InputCh(CH_1));
	g.rc_2.set_pwm(APM_RC.InputCh(CH_2));
	g.rc_3.set_pwm(APM_RC.InputCh(CH_3));
	g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
	g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
	g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
	g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
	g.rc_8.set_pwm(APM_RC.InputCh(CH_8));
	//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"), g.rc_1.control_in, g.rc_2.control_in, g.rc_3.control_in, g.rc_4.control_in);
}

void trim_radio()
{
	for (byte i = 0; i < 30; i++){
		read_radio();
	}
	g.rc_1.trim();	// roll
	g.rc_2.trim();	// pitch
	g.rc_4.trim();	// yaw
	
	g.rc_1.save_trim();
	g.rc_2.save_trim();
	g.rc_4.save_trim();	
}

void trim_yaw()
{
	for (byte i = 0; i < 30; i++){
		read_radio();
	}
	g.rc_4.trim();	// yaw
}
