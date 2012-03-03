/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG ==	TRI_FRAME

static void init_motors_out()
{
	#if INSTANT_PWM == 0
    APM_RC.SetFastOutputChannels(_BV(MOT_1) | _BV(MOT_2) | _BV(MOT_4),
                                 g.rc_speed);
	#endif
}

static void motors_output_enable()
{
	APM_RC.enable_out(MOT_1);
	APM_RC.enable_out(MOT_2);
	APM_RC.enable_out(MOT_4);
	APM_RC.enable_out(CH_TRI_YAW);
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
	motor_out[MOT_2]		= g.rc_3.radio_out + roll_out + pitch_out;
	//right front
	motor_out[MOT_1]		= g.rc_3.radio_out - roll_out + pitch_out;
	// rear
	motor_out[MOT_4] 	= g.rc_3.radio_out - g.rc_2.pwm_out;

	//motor_out[MOT_4]		+= (float)(abs(g.rc_4.control_in)) * .013;

	// Tridge's stability patch
	if(motor_out[MOT_1] > out_max){
		motor_out[MOT_2] -= (motor_out[MOT_1] - out_max) >> 1;
		motor_out[MOT_4] -= (motor_out[MOT_1] - out_max) >> 1;
		motor_out[MOT_1] = out_max;
	}

	if(motor_out[MOT_2] > out_max){
		motor_out[MOT_1] -= (motor_out[MOT_2] - out_max) >> 1;
		motor_out[MOT_4] -= (motor_out[MOT_2] - out_max) >> 1;
		motor_out[MOT_2] = out_max;
	}

	if(motor_out[MOT_4] > out_max){
		motor_out[MOT_1] -= (motor_out[MOT_4] - out_max) >> 1;
		motor_out[MOT_2] -= (motor_out[MOT_4] - out_max) >> 1;
		motor_out[MOT_4] = out_max;
	}

	// limit output so motors don't stop
	motor_out[MOT_1]		= max(motor_out[MOT_1], 	out_min);
	motor_out[MOT_2]		= max(motor_out[MOT_2], 	out_min);
	motor_out[MOT_4] 	= max(motor_out[MOT_4], 	out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(g.rc_3.servo_out == 0){
		motor_out[MOT_1]		= g.rc_3.radio_min;
		motor_out[MOT_2]		= g.rc_3.radio_min;
		motor_out[MOT_4] 	= g.rc_3.radio_min;
	}
	#endif

	APM_RC.OutputCh(MOT_1, motor_out[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_out[MOT_2]);
	APM_RC.OutputCh(MOT_4, motor_out[MOT_4]);

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
	for (unsigned char i = 0; i < 8; i++){
		motor_out[i] = g.rc_3.radio_min;
	}

	// Send commands to motors
	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_2, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
}

static void output_motor_test()
{
	motor_out[MOT_1] = g.rc_3.radio_min;
	motor_out[MOT_2] = g.rc_3.radio_min;
	motor_out[MOT_4] = g.rc_3.radio_min;

	 APM_RC.OutputCh(MOT_2, g.rc_2.radio_min);
	delay(4000);
	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
	delay(2000);
	APM_RC.OutputCh(MOT_4, g.rc_1.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_4, g.rc_1.radio_min);
	delay(2000);
	APM_RC.OutputCh(MOT_2, g.rc_4.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_1, motor_out[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_out[MOT_2]);
	APM_RC.OutputCh(MOT_4, motor_out[MOT_4]);
}

#endif
