void init_rc_in()
{
	read_EEPROM_radio();		// read Radio limits
	rc_1.set_angle(4500);
	rc_1.dead_zone = 60;
	rc_2.set_angle(4500);
	rc_2.dead_zone = 60;
	rc_3.set_range(0,1000);
	rc_3.dead_zone = 20;
	rc_3.scale_output = .9;
	rc_4.set_angle(6000); 
	rc_4.dead_zone = 500;
	rc_5.set_range(0,1000);
	rc_5.set_filter(false);

	// for kP values
	//rc_6.set_range(200,800);
	rc_6.set_range(0,4000);

	// for camera angles
	//rc_6.set_angle(4500);
	//rc_6.dead_zone = 60;

	rc_7.set_range(0,1000);
	rc_8.set_range(0,1000);
}

void init_rc_out()
{
	#if ARM_AT_STARTUP == 1
		motor_armed = 1;
	#endif

	APM_RC.OutputCh(CH_1, 	rc_3.radio_min);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_3, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_4, 	rc_3.radio_min);

	APM_RC.Init();		// APM Radio initialization

	APM_RC.OutputCh(CH_1, 	rc_3.radio_min);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_3, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_4, 	rc_3.radio_min);
}

void read_radio()
{
	rc_1.set_pwm(APM_RC.InputCh(CH_1));
	rc_2.set_pwm(APM_RC.InputCh(CH_2));
	rc_3.set_pwm(APM_RC.InputCh(CH_3));
	rc_4.set_pwm(APM_RC.InputCh(CH_4));
	rc_5.set_pwm(APM_RC.InputCh(CH_5));
	rc_6.set_pwm(APM_RC.InputCh(CH_6));
	rc_7.set_pwm(APM_RC.InputCh(CH_7));
	rc_8.set_pwm(APM_RC.InputCh(CH_8));
	//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"), rc_1.control_in, rc_2.control_in, rc_3.control_in, rc_4.control_in);
}

void trim_radio()
{
	for (byte i = 0; i < 30; i++){
		read_radio();
	}
	rc_1.trim();	// roll
	rc_2.trim();	// pitch
	rc_4.trim();	// yaw
}

void trim_yaw()
{
	for (byte i = 0; i < 30; i++){
		read_radio();
	}
	rc_4.trim();	// yaw
}
