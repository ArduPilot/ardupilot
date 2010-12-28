void init_rc_in()
{
	read_EEPROM_radio();		// read Radio limits
	rc_1.set_angle(4500);
	rc_1.dead_zone = 50;
	rc_2.set_angle(4500);
	rc_2.dead_zone = 50;
	rc_3.set_range(0,1000);
	rc_3.dead_zone = 20;
	rc_3.scale_output = .8;
	rc_4.set_angle(6000); 
	rc_4.dead_zone = 500;
	rc_5.set_range(0,1000);
	rc_5.set_filter(false);
	rc_6.set_range(200,800);
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

#define ARM_DELAY 10
#define DISARM_DELAY 10

void arm_motors()
{
	static byte arming_counter;

	// Arm motor output : Throttle down and full yaw right for more than 2 seconds
	if (rc_3.control_in == 0){		
		if (rc_4.control_in > 2700) {
			if (arming_counter > ARM_DELAY) {
				motor_armed = true;
			} else{
				arming_counter++;
			}
		}else if (rc_4.control_in < -2700) {
			if (arming_counter > DISARM_DELAY){
				motor_armed = false;
			}else{
				arming_counter++;
			}			
		}else{
			arming_counter = 0;
		}
	}
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
		
		/*
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
			
			pid_stabilize_roll.kP((float)rc_6.control_in / 1000);
			stabilize_rate_roll_pitch = pid_stabilize_roll.kP() *.25;
			init_pids();
			
			//Serial.print("nav_yaw: ");
			//Serial.println(nav_yaw,DEC);
			
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