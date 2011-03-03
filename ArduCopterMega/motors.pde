/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#define ARM_DELAY 10
#define DISARM_DELAY 10

void arm_motors()
{
	static byte arming_counter;

	// Arm motor output : Throttle down and full yaw right for more than 2 seconds
	if (g.rc_3.control_in == 0){
		if (g.rc_4.control_in > 2700) {
			if (arming_counter > ARM_DELAY) {
				motor_armed = true;
			} else{
				arming_counter++;
			}
		}else if (g.rc_4.control_in < -2700) {
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
void
set_servos_4()
{
	static byte num;
	int out_min;

	// Quadcopter mix
	if (motor_armed == true && motor_auto_safe == true) {
		out_min = g.rc_3.radio_min;

		// Throttle is 0 to 1000 only
		g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, 1000);

		if(g.rc_3.servo_out > 0)
			out_min = g.rc_3.radio_min + 50;

		//Serial.printf("out: %d %d %d %d\t\t", g.rc_1.servo_out, g.rc_2.servo_out, g.rc_3.servo_out, g.rc_4.servo_out);

		// creates the radio_out and pwm_out values
		g.rc_1.calc_pwm();
		g.rc_2.calc_pwm();
		g.rc_3.calc_pwm();
		g.rc_4.calc_pwm();

		//Serial.printf("out: %d %d %d %d\n", g.rc_1.radio_out, g.rc_2.radio_out, g.rc_3.radio_out, g.rc_4.radio_out);
		//Serial.printf("yaw: %d ", g.rc_4.radio_out);

		if(g.frame_type == PLUS_FRAME){
			//Serial.println("P_FRAME");
			motor_out[CH_1]		= g.rc_3.radio_out - g.rc_1.pwm_out;
			motor_out[CH_2]		= g.rc_3.radio_out + g.rc_1.pwm_out;
			motor_out[CH_3]		= g.rc_3.radio_out + g.rc_2.pwm_out;
			motor_out[CH_4] 	= g.rc_3.radio_out - g.rc_2.pwm_out;

			motor_out[CH_1]		+=  g.rc_4.pwm_out; 	// CCW
			motor_out[CH_2]		+=  g.rc_4.pwm_out; 	// CCW
			motor_out[CH_3]		-=  g.rc_4.pwm_out; 	// CW
			motor_out[CH_4] 	-=  g.rc_4.pwm_out; 	// CW

		}else if(g.frame_type == X_FRAME){
			//Serial.println("X_FRAME");
			int roll_out 	 	= g.rc_1.pwm_out * .707;
			int pitch_out 	 	= g.rc_2.pwm_out * .707;

			motor_out[CH_3]	 	= g.rc_3.radio_out + roll_out + pitch_out;
			motor_out[CH_2]	 	= g.rc_3.radio_out + roll_out - pitch_out;

			motor_out[CH_1]		= g.rc_3.radio_out - roll_out + pitch_out;
			motor_out[CH_4] 	= g.rc_3.radio_out - roll_out - pitch_out;

			//Serial.printf("\tb4: %d %d %d %d ", motor_out[CH_1], motor_out[CH_2], motor_out[CH_3], motor_out[CH_4]);

			motor_out[CH_1]		+= g.rc_4.pwm_out;	// CCW
			motor_out[CH_2]		+= g.rc_4.pwm_out;	// CCW
			motor_out[CH_3]		-= g.rc_4.pwm_out;	// CW
			motor_out[CH_4] 	-= g.rc_4.pwm_out;	// CW

			//Serial.printf("\tl8r: %d %d %d %d\n", motor_out[CH_1], motor_out[CH_2], motor_out[CH_3], motor_out[CH_4]);

		}else if(g.frame_type == TRI_FRAME){

			//Serial.println("TRI_FRAME");
			// Tri-copter power distribution

			int roll_out 		= (float)g.rc_1.pwm_out * .866;
			int pitch_out 		= g.rc_2.pwm_out / 2;

			// front two motors
			motor_out[CH_2]		= g.rc_3.radio_out + roll_out + pitch_out;
			motor_out[CH_1]		= g.rc_3.radio_out - roll_out + pitch_out;

			// rear motors
			motor_out[CH_4] 	= g.rc_3.radio_out - g.rc_2.pwm_out;

			// this is a compensation for the angle of the yaw motor. Its linear, but should work ok.
			motor_out[CH_4]		+= (float)(abs(g.rc_4.control_in)) * .013;

			// servo Yaw
			APM_RC.OutputCh(CH_7, g.rc_4.radio_out);


		}else if (g.frame_type == HEXA_FRAME) {
			//Serial.println("6_FRAME");

			int roll_out 		= (float)g.rc_1.pwm_out * .866;
			int pitch_out 		= g.rc_2.pwm_out / 2;

			//left side
			motor_out[CH_2]		= g.rc_3.radio_out + g.rc_1.pwm_out;		// CCW
			motor_out[CH_3]		= g.rc_3.radio_out + roll_out + pitch_out;	// CW
			motor_out[CH_8]     = g.rc_3.radio_out + roll_out - pitch_out;	// CW

			//right side
			motor_out[CH_1]		= g.rc_3.radio_out - g.rc_1.pwm_out;		// CW
            motor_out[CH_7] 	= g.rc_3.radio_out - roll_out + pitch_out;	// CCW
			motor_out[CH_4] 	= g.rc_3.radio_out - roll_out - pitch_out;	// CCW

			motor_out[CH_7]		+= g.rc_4.pwm_out;	// CCW
			motor_out[CH_2]		+= g.rc_4.pwm_out;	// CCW
			motor_out[CH_4] 	+= g.rc_4.pwm_out;	// CCW

			motor_out[CH_3]		-= g.rc_4.pwm_out;	// CW
			motor_out[CH_1]		-= g.rc_4.pwm_out;	// CW
			motor_out[CH_8]     -= g.rc_4.pwm_out;  // CW

    	}else{

			Serial.print("frame error");

		}


		// limit output so motors don't stop
		motor_out[CH_1]		= constrain(motor_out[CH_1], 	out_min, g.rc_3.radio_max.get());
		motor_out[CH_2]		= constrain(motor_out[CH_2], 	out_min, g.rc_3.radio_max.get());
		motor_out[CH_3]		= constrain(motor_out[CH_3], 	out_min, g.rc_3.radio_max.get());
		motor_out[CH_4] 	= constrain(motor_out[CH_4], 	out_min, g.rc_3.radio_max.get());

		if (g.frame_type == HEXA_FRAME) {
			motor_out[CH_7]		= constrain(motor_out[CH_7], 	out_min, g.rc_3.radio_max.get());
			motor_out[CH_8]		= constrain(motor_out[CH_8], 	out_min, g.rc_3.radio_max.get());
		}

		num++;
		if (num > 50){
			num = 0;
			Serial.printf("t_alt:%ld, alt:%ld, thr: %d sen: ",
						target_altitude,
						current_loc.alt,
						g.rc_3.servo_out);

			if(altitude_sensor == BARO){
				Serial.println("Baro");
			}else{
				Serial.println("Sonar");
			}
			//Serial.print("!");
			//debugging with Channel 6

			//g.pid_baro_throttle.kD((float)g.rc_6.control_in / 1000); //  0 to 1
			//g.pid_baro_throttle.kP((float)g.rc_6.control_in / 4000); //  0 to .25

			/*
			// ROLL and PITCH
			// make sure you init_pids() after changing the kP
			g.pid_stabilize_roll.kP((float)g.rc_6.control_in / 1000);
			init_pids();
			//Serial.print("kP: ");
			//Serial.println(g.pid_stabilize_roll.kP(),3);
			//*/

			/*
			// YAW
			// make sure you init_pids() after changing the kP
			g.pid_yaw.kP((float)g.rc_6.control_in / 1000);
			init_pids();
			//*/

			/*
			write_int(motor_out[CH_1]);
			write_int(motor_out[CH_2]);
			write_int(motor_out[CH_3]);
			write_int(motor_out[CH_4]);

			write_int(g.rc_3.servo_out);
			write_int((int)(cos_yaw_x * 100));
			write_int((int)(sin_yaw_y * 100));
			write_int((int)(dcm.yaw_sensor / 100));
			write_int((int)(nav_yaw / 100));

			write_int((int)nav_lat);
			write_int((int)nav_lon);

			write_int((int)nav_roll);
			write_int((int)nav_pitch);

			//24
			write_long(current_loc.lat);	//28
			write_long(current_loc.lng);	//32
			write_int((int)current_loc.alt);	//34

			write_long(next_WP.lat);
			write_long(next_WP.lng);
			write_int((int)next_WP.alt);		//44

			flush(10);
			//*/

			/*Serial.printf("a %ld, e %ld, i %d, t %d, b %4.2f\n",
					current_loc.alt,
					altitude_error,
					(int)g.pid_baro_throttle.get_integrator(),
					nav_throttle,
					angle_boost());
			*/
		}

		// Send commands to motors
		if(g.rc_3.servo_out > 0){

			APM_RC.OutputCh(CH_1, motor_out[CH_1]);
			APM_RC.OutputCh(CH_2, motor_out[CH_2]);
			APM_RC.OutputCh(CH_3, motor_out[CH_3]);
			APM_RC.OutputCh(CH_4, motor_out[CH_4]);
			// InstantPWM
			APM_RC.Force_Out0_Out1();
			APM_RC.Force_Out2_Out3();

			if (g.frame_type == HEXA_FRAME) {
				APM_RC.OutputCh(CH_7, motor_out[CH_7]);
				APM_RC.OutputCh(CH_8, motor_out[CH_8]);
				APM_RC.Force_Out6_Out7();
			}

		}else{

			APM_RC.OutputCh(CH_1, g.rc_3.radio_min);
			APM_RC.OutputCh(CH_2, g.rc_3.radio_min);
			APM_RC.OutputCh(CH_3, g.rc_3.radio_min);
			APM_RC.OutputCh(CH_4, g.rc_3.radio_min);
			// InstantPWM
			APM_RC.Force_Out0_Out1();
			APM_RC.Force_Out2_Out3();

			if (g.frame_type == HEXA_FRAME) {
				APM_RC.OutputCh(CH_7, g.rc_3.radio_min);
				APM_RC.OutputCh(CH_8, g.rc_3.radio_min);
				APM_RC.Force_Out6_Out7();
			}
		}

	}else{
		// our motor is unarmed, we're on the ground
		reset_I();

		if(g.rc_3.control_in > 0){
			// we have pushed up the throttle
			// remove safety
			motor_auto_safe = true;
		}

		// Send commands to motors
		APM_RC.OutputCh(CH_1, g.rc_3.radio_min);
		APM_RC.OutputCh(CH_2, g.rc_3.radio_min);
		APM_RC.OutputCh(CH_3, g.rc_3.radio_min);
		APM_RC.OutputCh(CH_4, g.rc_3.radio_min);

		if (g.frame_type == HEXA_FRAME) {
			APM_RC.OutputCh(CH_7, g.rc_3.radio_min);
			APM_RC.OutputCh(CH_8, g.rc_3.radio_min);
		}

		// reset I terms of PID controls
		reset_I();

		// Initialize yaw command to actual yaw when throttle is down...
		g.rc_4.control_in = ToDeg(dcm.yaw);
	}
 }

