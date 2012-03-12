/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	QUAD_FRAME

static void init_motors_out()
{
	#if INSTANT_PWM == 0
	APM_RC.SetFastOutputChannels(_BV(MOT_1) | _BV(MOT_2) | _BV(MOT_3) | _BV(MOT_4),
                                 g.rc_speed);
	#endif
}

static void motors_output_enable()
{
	APM_RC.enable_out(MOT_1);
	APM_RC.enable_out(MOT_2);
	APM_RC.enable_out(MOT_3);
	APM_RC.enable_out(MOT_4);
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
		roll_out 	 	= (float)g.rc_1.pwm_out * 0.707;
		pitch_out 	 	= (float)g.rc_2.pwm_out * 0.707;

		// left
		motor_out[MOT_3]	= g.rc_3.radio_out + roll_out + pitch_out;	// FRONT
		motor_out[MOT_2]	= g.rc_3.radio_out + roll_out - pitch_out;	// BACK

		// right
		motor_out[MOT_1]	= g.rc_3.radio_out - roll_out + pitch_out;	// FRONT
		motor_out[MOT_4] 	= g.rc_3.radio_out - roll_out - pitch_out;	// BACK

	}else{

		roll_out 	 	= g.rc_1.pwm_out;
		pitch_out 	 	= g.rc_2.pwm_out;

		// right motor
		motor_out[MOT_1]	= g.rc_3.radio_out - roll_out;
		// left motor
		motor_out[MOT_2]	= g.rc_3.radio_out + roll_out;
		// front motor
		motor_out[MOT_3]	= g.rc_3.radio_out + pitch_out;
		// back motor
		motor_out[MOT_4] 	= g.rc_3.radio_out - pitch_out;
	}

	// Yaw input
	motor_out[MOT_1]	+=	g.rc_4.pwm_out; 	// CCW
	motor_out[MOT_2]	+=	g.rc_4.pwm_out; 	// CCW
	motor_out[MOT_3]	-=	g.rc_4.pwm_out; 	// CW
	motor_out[MOT_4] 	-=	g.rc_4.pwm_out; 	// CW

    /* We need to clip motor output at out_max. When cipping a motors
		 * output we also need to compensate for the instability by
		 * lowering the opposite motor by the same proportion. This
		 * ensures that we retain control when one or more of the motors
		 * is at its maximum output
		 */
		for (int i = MOT_1; i <= MOT_4; i++){
				if(motor_out[i] > out_max){
		            // note that i^1 is the opposite motor
					motor_out[i ^ 1] -= motor_out[i] - out_max;
					motor_out[i] = out_max;
				}
		}

	// limit output so motors don't stop
	motor_out[MOT_1]	= max(motor_out[MOT_1], 	out_min);
	motor_out[MOT_2]	= max(motor_out[MOT_2], 	out_min);
	motor_out[MOT_3]	= max(motor_out[MOT_3], 	out_min);
	motor_out[MOT_4] 	= max(motor_out[MOT_4], 	out_min);

	#if CUT_MOTORS == ENABLED
	// if we are not sending a throttle output, we cut the motors
	if(g.rc_3.servo_out == 0){
		motor_out[MOT_1]	= g.rc_3.radio_min;
		motor_out[MOT_2]	= g.rc_3.radio_min;
		motor_out[MOT_3]	= g.rc_3.radio_min;
		motor_out[MOT_4] 	= g.rc_3.radio_min;
	}
	#endif

	APM_RC.OutputCh(MOT_1, motor_out[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_out[MOT_2]);
	APM_RC.OutputCh(MOT_3, motor_out[MOT_3]);
	APM_RC.OutputCh(MOT_4, motor_out[MOT_4]);


	#if INSTANT_PWM == 1
	// InstantPWM
	APM_RC.Force_Out0_Out1();
	APM_RC.Force_Out2_Out3();
	#endif

	//debug_motors();
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
}

/*
//static void debug_motors()
{
	Serial.printf("1:%d\t2:%d\t3:%d\t4:%d\n",
				motor_out[MOT_1],
				motor_out[MOT_2],
				motor_out[MOT_3],
				motor_out[MOT_4]);
}
//*/

static void output_motor_test()
{
	motor_out[MOT_1] = g.rc_3.radio_min;
	motor_out[MOT_2] = g.rc_3.radio_min;
	motor_out[MOT_3] = g.rc_3.radio_min;
	motor_out[MOT_4] = g.rc_3.radio_min;


	if(g.frame_orientation == X_FRAME){

		 APM_RC.OutputCh(MOT_3, g.rc_2.radio_min);
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

		 APM_RC.OutputCh(MOT_2, g.rc_4.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_3, g.rc_2.radio_min + 100);
		 delay(300);

	}else{

		 APM_RC.OutputCh(MOT_3, g.rc_2.radio_min);
		 delay(4000);
		 APM_RC.OutputCh(MOT_1, g.rc_3.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_1, g.rc_3.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_2, g.rc_1.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_2, g.rc_1.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_4, g.rc_4.radio_min + 100);
		 delay(300);

		 APM_RC.OutputCh(MOT_4, g.rc_4.radio_min);
		 delay(2000);
		 APM_RC.OutputCh(MOT_3, g.rc_2.radio_min + 100);
		 delay(300);

	}

	APM_RC.OutputCh(MOT_1, motor_out[MOT_1]);
	APM_RC.OutputCh(MOT_2, motor_out[MOT_2]);
	APM_RC.OutputCh(MOT_3, motor_out[MOT_3]);
	APM_RC.OutputCh(MOT_4, motor_out[MOT_4]);
}

#endif
