/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG ==	TRI_FRAME

static void init_motors_out()
{
	#if INSTANT_PWM == 0
    APM_RC.SetFastOutputChannels( MSK_CH_1 | MSK_CH_2 | MSK_CH_4 );
	#endif
}


static void output_motors_armed()
{
	int out_min = g.rc_3.radio_min;
	int out_max = g.rc_3.radio_max;

	// Throttle is 0 to 1000 only
	g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, MAXIMUM_THROTTLE);

	if(g.rc_3.servo_out > 0)
		out_min = g.rc_3.radio_min + MINIMUM_THROTTLE;

	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();

	int roll_out 		= (float)g.rc_1.pwm_out * .866;
	int pitch_out 		= g.rc_2.pwm_out / 2;

	//left front
	motor_out[CH_2]		= g.rc_3.radio_out + roll_out + pitch_out;
	//right front
	motor_out[CH_1]		= g.rc_3.radio_out - roll_out + pitch_out;
	// rear
	motor_out[CH_4] 	= g.rc_3.radio_out - g.rc_2.pwm_out;

	//motor_out[CH_4]		+= (float)(abs(g.rc_4.control_in)) * .013;

	// Tridge's stability patch
	if (motor_out[CH_1] > out_max) {
		motor_out[CH_2] -= (motor_out[CH_1] - out_max) >> 1;
		motor_out[CH_4] -= (motor_out[CH_1] - out_max) >> 1;
		motor_out[CH_1] = out_max;
	}

	if (motor_out[CH_2] > out_max) {
		motor_out[CH_1] -= (motor_out[CH_2] - out_max) >> 1;
		motor_out[CH_4] -= (motor_out[CH_2] - out_max) >> 1;
		motor_out[CH_2] = out_max;
	}

	if (motor_out[CH_4] > out_max) {
		motor_out[CH_1] -= (motor_out[CH_4] - out_max) >> 1;
		motor_out[CH_2] -= (motor_out[CH_4] - out_max) >> 1;
		motor_out[CH_4] = out_max;
	}

	// limit output so motors don't stop
	motor_out[CH_1]		= max(motor_out[CH_1], 	out_min);
	motor_out[CH_2]		= max(motor_out[CH_2], 	out_min);
	motor_out[CH_4] 	= max(motor_out[CH_4], 	out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(g.rc_3.servo_out == 0){
		motor_out[CH_1]		= g.rc_3.radio_min;
		motor_out[CH_2]		= g.rc_3.radio_min;
		motor_out[CH_4] 	= g.rc_3.radio_min;
	}
	#endif

	APM_RC.OutputCh(CH_1, motor_out[CH_1]);
	APM_RC.OutputCh(CH_2, motor_out[CH_2]);
	APM_RC.OutputCh(CH_4, motor_out[CH_4]);

	#if INSTANT_PWM == 1
	// InstantPWM
	APM_RC.Force_Out0_Out1();
	APM_RC.Force_Out2_Out3();
	#endif
}

static void output_motors_disarmed()
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

static void output_motor_test()
{
	motor_out[CH_1] = g.rc_3.radio_min;
	motor_out[CH_2] = g.rc_3.radio_min;
	motor_out[CH_4] = g.rc_3.radio_min;


	if(g.rc_1.control_in > 3000){	// right
		motor_out[CH_1] += 100;
	}

	if(g.rc_1.control_in < -3000){	// left
		motor_out[CH_2] += 100;
	}

	if(g.rc_2.control_in > 3000){	// back
		motor_out[CH_4] += 100;
	}

	APM_RC.OutputCh(CH_1, motor_out[CH_1]);
	APM_RC.OutputCh(CH_2, motor_out[CH_2]);
	APM_RC.OutputCh(CH_4, motor_out[CH_4]);
}

#endif
