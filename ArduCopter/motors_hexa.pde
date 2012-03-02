/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	HEXA_FRAME

static void init_motors_out()
{
	#if INSTANT_PWM == 0
		APM_RC.SetFastOutputChannels(_BV(MOT_1) | _BV(MOT_2) | _BV(MOT_3) | _BV(MOT_4)
                                     | _BV(MOT_5) | _BV(MOT_6), g.rc_speed);
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
	int roll_out, pitch_out;
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

	if(g.frame_orientation == X_FRAME){
		roll_out 	 	= g.rc_1.pwm_out / 2;
		pitch_out 	 	= (float)g.rc_2.pwm_out * .866;

		//left side
		motor_out[MOT_2]		= g.rc_3.radio_out + g.rc_1.pwm_out;		// CCW Middle
		motor_out[MOT_3]		= g.rc_3.radio_out + roll_out + pitch_out;	// CW Front
		motor_out[MOT_6]		 = g.rc_3.radio_out + roll_out - pitch_out;	// CW Back

		//right side
		motor_out[MOT_1]		= g.rc_3.radio_out - g.rc_1.pwm_out;		// CW Middle
		motor_out[MOT_5] 	= g.rc_3.radio_out - roll_out + pitch_out;	// CCW Front
		motor_out[MOT_4] 	= g.rc_3.radio_out - roll_out - pitch_out;	// CCW Back

	}else{
		roll_out 	 	= (float)g.rc_1.pwm_out * .866;
		pitch_out 	 	= g.rc_2.pwm_out / 2;

		//Front side
		motor_out[MOT_1]		= g.rc_3.radio_out + g.rc_2.pwm_out;		// CW	 FRONT
		motor_out[MOT_5] 	= g.rc_3.radio_out + roll_out + pitch_out;	// CCW	 FRONT LEFT
		motor_out[MOT_4] 	= g.rc_3.radio_out - roll_out + pitch_out;	// CCW	 FRONT RIGHT

		//Back side
		motor_out[MOT_2]		= g.rc_3.radio_out - g.rc_2.pwm_out;		// CCW	BACK
		motor_out[MOT_3]		= g.rc_3.radio_out + roll_out - pitch_out;	// CW, 	BACK LEFT
		motor_out[MOT_6]		= g.rc_3.radio_out - roll_out - pitch_out;	// CW	BACK RIGHT
	}

	// Yaw
	motor_out[MOT_2]		+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_5]		+= g.rc_4.pwm_out;	// CCW
	motor_out[MOT_4] 	+= g.rc_4.pwm_out;	// CCW

	motor_out[MOT_3]		-= g.rc_4.pwm_out;	// CW
	motor_out[MOT_1]		-= g.rc_4.pwm_out;	// CW
	motor_out[MOT_6]		-= g.rc_4.pwm_out;	// CW


	// Tridge's stability patch
		for (int m = 0; m <= 6; m++){
			int c = ch_of_mot(m);
			int c_opp = ch_of_mot(m ^ 1); // m ^ 1 is the opposite motor. c_opp is channel of opposite motor.
			if(motor_out[c] > out_max){
				motor_out[c_opp] -= motor_out[c] - out_max;
				motor_out[c] = out_max;
			}
		}

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
			int c = ch_of_mot(m);
		if(motor_filtered[c] < motor_out[c]){
			motor_filtered[c] = (motor_out[c] + motor_filtered[c]) / 2;
		}else{
			// don't filter
			motor_filtered[c] = motor_out[c];
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
	motors_output_enable();

	motor_out[MOT_1] = g.rc_3.radio_min;
	motor_out[MOT_2] = g.rc_3.radio_min;
	motor_out[MOT_3] = g.rc_3.radio_min;
	motor_out[MOT_4] = g.rc_3.radio_min;
	motor_out[MOT_5] = g.rc_3.radio_min;
	motor_out[MOT_6] = g.rc_3.radio_min;

	if(g.frame_orientation == X_FRAME){
		 APM_RC.OutputCh(MOT_3, g.rc_3.radio_min);
		 delay(4000);
		 APM_RC.OutputCh(MOT_5, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_5, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_1, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_4, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_6, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_6, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_2, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_2, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_3, g.rc_3.radio_min + 100);
		 delay(300);

	} else { /* PLUS_FRAME */
		 APM_RC.OutputCh(MOT_5, g.rc_3.radio_min);
		 delay(4000);
		 APM_RC.OutputCh(MOT_1, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_4, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_4, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_6, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_6, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_2, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_2, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_3, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_3, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_5, g.rc_3.radio_min + 100);
		 delay(300);
		}

	APM_RC.OutputCh(MOT_1, motor_out[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_out[MOT_2]);
	APM_RC.OutputCh(MOT_3, motor_out[MOT_3]);
	APM_RC.OutputCh(MOT_4, motor_out[MOT_4]);
	APM_RC.OutputCh(MOT_5, motor_out[MOT_5]);
	APM_RC.OutputCh(MOT_6, motor_out[MOT_6]);
}

#endif
