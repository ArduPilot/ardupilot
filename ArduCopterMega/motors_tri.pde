/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG ==	TRI_FRAME

void output_motors_armed()
{
	int out_min = g.rc_3.radio_min;

	// Throttle is 0 to 1000 only
	g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, 1000);

	if(g.rc_3.servo_out > 0)
		out_min = g.rc_3.radio_min + 90;

	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	int roll_out 		= (float)g.rc_1.pwm_out * .866;
	int pitch_out 		= g.rc_2.pwm_out / 2;

	//left front
	motor_out[CH_2]		= g.rc_3.radio_out + roll_out + pitch_out;
	//right front
	motor_out[CH_1]		= g.rc_3.radio_out - roll_out + pitch_out;
	// rear
	motor_out[CH_4] 	= g.rc_3.radio_out - g.rc_2.pwm_out;

	//motor_out[CH_4]		+= (float)(abs(g.rc_4.control_in)) * .013;

	// limit output so motors don't stop
	motor_out[CH_1]		= max(motor_out[CH_1], 	out_min);
	motor_out[CH_2]		= max(motor_out[CH_2], 	out_min);
	motor_out[CH_4] 	= max(motor_out[CH_4], 	out_min);

		// Send commands to motors
	if(g.rc_3.servo_out > 0){
		APM_RC.OutputCh(CH_1, motor_out[CH_1]);
		APM_RC.OutputCh(CH_2, motor_out[CH_2]);
		APM_RC.OutputCh(CH_4, motor_out[CH_4]);
		// InstantPWM
		APM_RC.Force_Out0_Out1();
		APM_RC.Force_Out2_Out3();
	}else{
		APM_RC.OutputCh(CH_1, g.rc_3.radio_min);
		APM_RC.OutputCh(CH_2, g.rc_3.radio_min);
		APM_RC.OutputCh(CH_4, g.rc_3.radio_min);
	}
}

void output_motors_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle
		// remove safety
		motor_auto_armed = true;
	}

	// fill the motor_out[] array for HIL use
	for (unsigned char i = 0; i < 8; i++) {
		motor_out[i] = g.rc_3.radio_min;
	}

	// Send commands to motors
	APM_RC.OutputCh(CH_1, g.rc_3.radio_min);
	APM_RC.OutputCh(CH_2, g.rc_3.radio_min);
	APM_RC.OutputCh(CH_4, g.rc_3.radio_min);
}

void output_motor_test()
{
	APM_RC.OutputCh(CH_2, g.rc_3.radio_min);
	APM_RC.OutputCh(CH_1, g.rc_3.radio_min + 50);
	delay(1000);

	APM_RC.OutputCh(CH_1, g.rc_3.radio_min);
	APM_RC.OutputCh(CH_4, g.rc_3.radio_min + 50);
	delay(1000);

	APM_RC.OutputCh(CH_4, g.rc_3.radio_min);
	APM_RC.OutputCh(CH_2, g.rc_3.radio_min + 50);
	delay(1000);
}

#endif