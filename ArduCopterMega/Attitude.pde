
// desired angle in
// motor commands out (in degrees)
void init_pids()
{
	max_stabilize_dampener 	= pid_stabilize_roll.kP() * 2500;
	stabilze_dampener 		= 5729.57795 * stabilize_rate_roll_pitch;

	max_yaw_dampener		= pid_yaw.kP() * 6000;				// .3 * 6000 = 1800
	stabilze_yaw_dampener 	= 5729.57795 * stabilize_rate_yaw; 	// .3
}

void output_stabilize()
{
	float roll_error, pitch_error;
	Vector3f omega = dcm.get_gyro();
	
	//pitch_sensor = roll_sensor = 0; // testing only
	
	// control +- 45° is mixed with the navigation request by the Autopilot
	// output is in degrees = target pitch and roll of copter
	rc_1.servo_out = rc_1.control_mix(nav_roll);
	rc_2.servo_out = rc_2.control_mix(nav_pitch);
		
	roll_error 		= rc_1.servo_out - roll_sensor;
	pitch_error 	= rc_2.servo_out - pitch_sensor;
	yaw_error		= nav_yaw - yaw_sensor;
	yaw_error 		= wrap_180(yaw_error);
	
	// limit the error we're feeding to the PID
	roll_error 		= constrain(roll_error,  -2500, 2500);
	pitch_error 	= constrain(pitch_error, -2500, 2500);
	yaw_error		= constrain(yaw_error,   -6000, 6000);

	//Serial.printf("s: %d \t mix  %d, err  %d", (int)roll_sensor, (int)rc_1.servo_out, (int)roll_error);

	// write out angles back to servo out - this will be converted to PWM by RC_Channel
	rc_1.servo_out 	= pid_stabilize_roll.get_pid(roll_error,  	deltaMiliSeconds, 1.0);
	rc_2.servo_out 	= pid_stabilize_pitch.get_pid(pitch_error, 	deltaMiliSeconds, 1.0);
	rc_4.servo_out 	= pid_yaw.get_pid(yaw_error, 				deltaMiliSeconds, 1.0); // .3 = 198pwm

	//Serial.printf("\tpid: %d", (int)rc_1.servo_out);

	// We adjust the output by the rate of rotation:
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading
	int roll_dampener 	= (omega.x * stabilze_dampener);// Omega is in radians
	int pitch_dampener 	= (omega.y * stabilze_dampener);
	int yaw_dampener 	= (omega.z * stabilze_yaw_dampener);
	
	// Limit dampening to be equal to propotional term for symmetry
	rc_1.servo_out	-= constrain(roll_dampener,  -max_stabilize_dampener, max_stabilize_dampener);	// +- 15°
	rc_2.servo_out	-= constrain(pitch_dampener, -max_stabilize_dampener, max_stabilize_dampener);	// +- 15°
	rc_4.servo_out	-= constrain(yaw_dampener,   -max_yaw_dampener, 	  max_yaw_dampener);
	

	//Serial.printf(" yaw out: %d, d: %d", (int)rc_4.angle_to_pwm(), yaw_dampener);

	//Serial.printf("\trd: %d", roll_dampener);
	//Serial.printf("\tlimit: %d, PWM: %d", rc_1.servo_out, rc_1.angle_to_pwm());	
}

//	err  -2500	pid: -1100	rd: 1117	limit: -1650, PWM: -152 
//s: -1247 	 mix  0, err  1247	pid: 548	rd: -153	limit: 395, PWM: 35 

void output_rate_control()
{
	Vector3f omega = dcm.get_gyro();

	rc_4.servo_out = rc_4.control_in;
	rc_1.servo_out = rc_2.control_in;
	rc_2.servo_out = rc_2.control_in;
	
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading plus Omega_I, so it´s bias corrected
	rc_1.servo_out 	-= (omega.x * 5729.57795 * acro_rate_roll_pitch);
	rc_2.servo_out 	-= (omega.y * 5729.57795 * acro_rate_roll_pitch);
	rc_4.servo_out  -= (omega.z * 5729.57795 * acro_rate_yaw);

	//Serial.printf("\trated out %d, omega ", rc_1.servo_out);
	//Serial.print((Omega[0] * 5729.57795 * stabilize_rate_roll_pitch), 3);

	// Limit output
	rc_1.servo_out  = constrain(rc_1.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);
	rc_2.servo_out 	= constrain(rc_2.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);
	rc_4.servo_out  = constrain(rc_4.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);	
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
	pid_nav.reset_I();
	pid_throttle.reset_I();
}


/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
void set_servos_4(void)
{
	static byte num;

	// Quadcopter mix
	if (motor_armed == true) {
		int out_min = rc_3.radio_min;
		
		// Throttle is 0 to 1000 only
		rc_3.servo_out 	= constrain(rc_3.servo_out, 0, 1000);	

		if(rc_3.servo_out > 0)
			out_min = rc_3.radio_min + 50;
			
		//Serial.printf("out: %d %d %d %d\t\t", rc_1.servo_out, rc_2.servo_out, rc_3.servo_out, rc_4.servo_out);
		
		// creates the radio_out and pwm_out values
		rc_1.calc_pwm();
		rc_2.calc_pwm();
		rc_3.calc_pwm();
		rc_4.calc_pwm();
		
		//Serial.printf("out: %d %d %d %d\n", rc_1.radio_out, rc_2.radio_out, rc_3.radio_out, rc_4.radio_out);
		//Serial.printf("yaw: %d ", rc_4.radio_out);
		
		if(frame_type == PLUS_FRAME){
			motor_out[RIGHT]	= rc_3.radio_out - rc_1.pwm_out;
			motor_out[LEFT]		= rc_3.radio_out + rc_1.pwm_out;
			motor_out[FRONT]	= rc_3.radio_out + rc_2.pwm_out;
			motor_out[BACK] 	= rc_3.radio_out - rc_2.pwm_out;
		}else{
			int roll_out 	= rc_1.pwm_out / 2;
			int pitch_out 	= rc_2.pwm_out / 2;
			motor_out[RIGHT]	= rc_3.radio_out - roll_out + pitch_out;
			motor_out[LEFT]		= rc_3.radio_out + roll_out - pitch_out;
			motor_out[FRONT]	= rc_3.radio_out + roll_out + pitch_out;
			motor_out[BACK] 	= rc_3.radio_out - roll_out - pitch_out;
		}
		
		//Serial.printf("\tb4: %d %d %d %d ", motor_out[RIGHT], motor_out[LEFT], motor_out[FRONT], motor_out[BACK]);

		motor_out[RIGHT]	+=  rc_4.pwm_out;
		motor_out[LEFT]		+=  rc_4.pwm_out;
		motor_out[FRONT]	-=  rc_4.pwm_out;
		motor_out[BACK] 	-=  rc_4.pwm_out;

		//Serial.printf("\tl8r: %d %d %d %d\n", motor_out[RIGHT], motor_out[LEFT], motor_out[FRONT], motor_out[BACK]);

		motor_out[RIGHT]	= constrain(motor_out[RIGHT], 	out_min, rc_3.radio_max);
		motor_out[LEFT]		= constrain(motor_out[LEFT], 	out_min, rc_3.radio_max);
		motor_out[FRONT]	= constrain(motor_out[FRONT], 	out_min, rc_3.radio_max);
		motor_out[BACK] 	= constrain(motor_out[BACK], 	out_min, rc_3.radio_max);
		
		///*
		int r_out = ((long)(motor_out[RIGHT] - rc_3.radio_min) * 100) / (long)(rc_3.radio_max - rc_3.radio_min);
		int l_out = ((long)(motor_out[LEFT]  - rc_3.radio_min) * 100) / (long)(rc_3.radio_max - rc_3.radio_min);
		int f_out = ((long)(motor_out[FRONT] - rc_3.radio_min) * 100) / (long)(rc_3.radio_max - rc_3.radio_min);
		int b_out = ((long)(motor_out[BACK]  - rc_3.radio_min) * 100) / (long)(rc_3.radio_max - rc_3.radio_min);
		//*/
				
		//
		/* debugging and dynamic kP
		num++;
		if (num > 50){
			num = 0;
			//Serial.printf("control_in: %d ", rc_3.control_in);
			//Serial.printf(" servo: %d %d %d %d\t", rc_1.servo_out, rc_2.servo_out, rc_3.servo_out, rc_4.servo_out);
			//Serial.printf(" cwm: %d %d %d %d, %d\t", rc_1.pwm_out, rc_2.pwm_out, rc_3.pwm_out, rc_4.pwm_out, rc_3.radio_out);
			//Serial.printf(" out: %d %d %d %d\n", r_out, l_out, f_out, b_out);
			//Serial.printf(" pwm: %d, %d %d %d %d\n",rc_3.pwm_out, motor_out[RIGHT], motor_out[LEFT], motor_out[FRONT], motor_out[BACK]);
			
			pid_stabilize_roll.kP((float)rc_6.control_in / 1000);
			stabilize_rate_roll_pitch = pid_stabilize_roll.kP() *.25;
			init_pids();
			
			//Serial.print("kP: ");
			//Serial.println(pid_stabilize_roll.kP(),3);
		}
		// out: 41 38 39 39
 		// pwm: 358, 1412 1380 1395 1389
		//*/
		
		//Serial.printf("set: %d %d %d %d\n", motor_out[RIGHT], motor_out[LEFT], motor_out[FRONT], motor_out[BACK]);		
		//Serial.printf("s: %d %d %d\t\t", (int)roll_sensor, (int)pitch_sensor, (int)yaw_sensor);
		///Serial.printf("outmin: %d\n", out_min);
		
		/*
		write_int(r_out);
		write_int(l_out);
		write_int(f_out);
		write_int(b_out);
		write_int((int)(roll_sensor / 100));
		write_int((int)(pitch_sensor / 100));
		write_int((int)(yaw_sensor / 100));
		write_int((int)(yaw_error / 100));
		write_int((int)(current_loc.alt));
		write_int((int)(altitude_error));
		flush(10);
		//*/
		
		// Send commands to motors
		if(rc_3.servo_out > 0){
			APM_RC.OutputCh(CH_1, motor_out[RIGHT]);
			APM_RC.OutputCh(CH_2, motor_out[LEFT]);
			APM_RC.OutputCh(CH_3, motor_out[FRONT]);
			APM_RC.OutputCh(CH_4, motor_out[BACK]);
		}else{
			APM_RC.OutputCh(CH_1, rc_3.radio_min);
			APM_RC.OutputCh(CH_2, rc_3.radio_min);
			APM_RC.OutputCh(CH_3, rc_3.radio_min);
			APM_RC.OutputCh(CH_4, rc_3.radio_min);
		}
	
		// InstantPWM
		APM_RC.Force_Out0_Out1();
		APM_RC.Force_Out2_Out3();
		
	}else{
	
		// Send commands to motors
		APM_RC.OutputCh(CH_1, rc_3.radio_min);
		APM_RC.OutputCh(CH_2, rc_3.radio_min);
		APM_RC.OutputCh(CH_3, rc_3.radio_min);
		APM_RC.OutputCh(CH_4, rc_3.radio_min);
		
		// reset I terms of PID controls
		reset_I();
		
		// Initialize yaw command to actual yaw when throttle is down...
		rc_4.control_in = ToDeg(yaw);
	}
 }
