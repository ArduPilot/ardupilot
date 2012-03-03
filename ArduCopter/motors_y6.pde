/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	Y6_FRAME

#define YAW_DIRECTION 1

static void init_motors_out()
{
	#if INSTANT_PWM == 0
		APM_RC.SetFastOutputChannels(_BV(MOT_1) | _BV(MOT_2) | _BV(MOT_3) | _BV(MOT_4)
                                     | _BV(MOT_5) | _BV(MOT_6),
                                     g.rc_speed);
	#endif
}

static void motors_output_enable()
{
	APM_RC.enable_out(MOT_1);
	APM_RC.enable_out(MOT_2);
	APM_RC.enable_out(MOT_3);
	APM_RC.enable_out(MOT_4);
	APM_RC.enable_out(MOT_5);
	APM_RC.enable_out(MOT_6);
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
	g.rc_4.calc_pwm();

	// Multi-Wii Mix
	//left
	motor_out[MOT_2] 		= (g.rc_3.radio_out * g.top_bottom_ratio) + g.rc_1.pwm_out + (g.rc_2.pwm_out * 2 / 3); // LEFT	 TOP - CW
	motor_out[MOT_3] 		= g.rc_3.radio_out + g.rc_1.pwm_out	+ (g.rc_2.pwm_out * 2 / 3); // BOTTOM_LEFT - CCW
	//right
	motor_out[MOT_5] 		= (g.rc_3.radio_out * g.top_bottom_ratio) - g.rc_1.pwm_out + (g.rc_2.pwm_out * 2 / 3); // RIGHT TOP - CW
	motor_out[MOT_1] 		= g.rc_3.radio_out - g.rc_1.pwm_out + (g.rc_2.pwm_out * 2 / 3); // BOTTOM_RIGHT - CCW
	//back
	motor_out[MOT_6] 		= (g.rc_3.radio_out * g.top_bottom_ratio) - (g.rc_2.pwm_out * 4 / 3); 					// REAR TOP - CCW
	motor_out[MOT_4] 		= g.rc_3.radio_out - (g.rc_2.pwm_out * 4 / 3); 					// BOTTOM_REAR - CW

	//left
	motor_out[MOT_2] 		-= YAW_DIRECTION * g.rc_4.pwm_out; // LEFT TOP - CW
	motor_out[MOT_3] 		+= YAW_DIRECTION * g.rc_4.pwm_out; // LEFT BOTTOM - CCW
	//right
	motor_out[MOT_5] 		-= YAW_DIRECTION * g.rc_4.pwm_out; // RIGHT TOP - CW
	motor_out[MOT_1] 		+= YAW_DIRECTION * g.rc_4.pwm_out; // RIGHT BOTTOM - CCW
	//back
	motor_out[MOT_6] 		+= YAW_DIRECTION * g.rc_4.pwm_out; // REAR TOP - CCW
	motor_out[MOT_4] 		-= YAW_DIRECTION * g.rc_4.pwm_out; // REAR BOTTOM - CW


	/*
	int roll_out 		= (float)g.rc_1.pwm_out * .866;
	int pitch_out 		=	g.rc_2.pwm_out / 2;

	//left
	motor_out[MOT_2]		= ((g.rc_3.radio_out * g.top_bottom_ratio) + roll_out + pitch_out);	// CCW TOP
	motor_out[MOT_3]		=	g.rc_3.radio_out + roll_out + pitch_out;							// CW

	//right
	motor_out[MOT_5]		= ((g.rc_3.radio_out * g.top_bottom_ratio) - roll_out + pitch_out);	// CCW TOP
	motor_out[MOT_1]		=	g.rc_3.radio_out - roll_out + pitch_out;							// CW

	//back
	motor_out[MOT_6]		 = ((g.rc_3.radio_out * g.top_bottom_ratio) - g.rc_2.pwm_out);		// CCW TOP
	motor_out[MOT_4] 	=	g.rc_3.radio_out - g.rc_2.pwm_out;								// CW

	// Yaw
	//top
	motor_out[MOT_2]		+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_5]		+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_6] 	+= g.rc_4.pwm_out;	// CCW

	//bottom
	motor_out[MOT_3]		-= g.rc_4.pwm_out;	// CW
	motor_out[MOT_1]		-= g.rc_4.pwm_out;	// CW
	motor_out[MOT_4]		-= g.rc_4.pwm_out;	// CW
	*/

	// TODO: add stability patch
	motor_out[MOT_1]	= min(motor_out[MOT_1], 	out_max);
	motor_out[MOT_2]	= min(motor_out[MOT_2], 	out_max);
	motor_out[MOT_3]	= min(motor_out[MOT_3], 	out_max);
	motor_out[MOT_4] = min(motor_out[MOT_4], 	out_max);
	motor_out[MOT_5] = min(motor_out[MOT_5],	out_max);
	motor_out[MOT_6]	= min(motor_out[MOT_6],	out_max);

	// limit output so motors don't stop
	motor_out[MOT_1] = max(motor_out[MOT_1],	out_min);
	motor_out[MOT_2] = max(motor_out[MOT_2],	out_min);
	motor_out[MOT_3] = max(motor_out[MOT_3],	out_min);
	motor_out[MOT_4] = max(motor_out[MOT_4],	out_min);
	motor_out[MOT_5] = max(motor_out[MOT_5],	out_min);
	motor_out[MOT_6] = max(motor_out[MOT_6],	out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(g.rc_3.servo_out == 0){
		motor_out[MOT_1]		= g.rc_3.radio_min;
		motor_out[MOT_2]		= g.rc_3.radio_min;
		motor_out[MOT_3]		= g.rc_3.radio_min;
		motor_out[MOT_4] 	= g.rc_3.radio_min;
		motor_out[MOT_5] 	= g.rc_3.radio_min;
		motor_out[MOT_6] 	= g.rc_3.radio_min;
	}
	#endif

	// this filter slows the acceleration of motors vs the deceleration
	// Idea by Denny Rowland to help with his Yaw issue
	for(int8_t m = 0; m <= 6; m++){
			int i = ch_of_mot(m);
		if(motor_filtered[i] < motor_out[i]){
			motor_filtered[i] = (motor_out[i] + motor_filtered[i]) / 2;
		}else{
			// don't filter
			motor_filtered[i] = motor_out[i];
		}
	}

	APM_RC.OutputCh(MOT_1, motor_filtered[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_filtered[MOT_2]);
	APM_RC.OutputCh(MOT_3, motor_filtered[MOT_3]);
	APM_RC.OutputCh(MOT_4, motor_filtered[MOT_4]);
	APM_RC.OutputCh(MOT_5, motor_filtered[MOT_5]);
	APM_RC.OutputCh(MOT_6, motor_filtered[MOT_6]);

	#if INSTANT_PWM == 1
	// InstantPWM
	APM_RC.Force_Out0_Out1();
	APM_RC.Force_Out2_Out3();
	APM_RC.Force_Out6_Out7();
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
	APM_RC.OutputCh(MOT_3, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_5, g.rc_3.radio_min);
	APM_RC.OutputCh(MOT_6, g.rc_3.radio_min);
}

static void output_motor_test()
{
	motor_out[MOT_1] = g.rc_3.radio_min;
	motor_out[MOT_2] = g.rc_3.radio_min;
	motor_out[MOT_3] = g.rc_3.radio_min;
	motor_out[MOT_4] = g.rc_3.radio_min;
	motor_out[MOT_5] = g.rc_3.radio_min;
	motor_out[MOT_6] = g.rc_3.radio_min;

	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
	delay(5000);
	APM_RC.OutputCh(MOT_2, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_2, g.rc_3.radio_min);
	delay(3000);
	APM_RC.OutputCh(MOT_3, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_3, g.rc_3.radio_min);
	delay(3000);
	APM_RC.OutputCh(MOT_6, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_6, g.rc_3.radio_min);
	delay(3000);
	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
	delay(3000);
	APM_RC.OutputCh(MOT_5, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_5, g.rc_3.radio_min);
	delay(3000);
	APM_RC.OutputCh(MOT_1, g.rc_3.radio_min + 100);
	delay(300);

	APM_RC.OutputCh(MOT_1, motor_out[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_out[MOT_2]);
	APM_RC.OutputCh(MOT_3, motor_out[MOT_4]);
	APM_RC.OutputCh(MOT_4, motor_out[MOT_4]);
	APM_RC.OutputCh(MOT_5, motor_out[MOT_5]);
	APM_RC.OutputCh(MOT_6, motor_out[MOT_6]);
}

#endif
