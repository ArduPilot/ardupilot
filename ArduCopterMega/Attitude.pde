
void init_pids()
{
	// create limits to how much dampening we'll allow
	// this creates symmetry with the P gain value preventing oscillations
	
	max_stabilize_dampener 	= pid_stabilize_roll.kP() * 2500;	// = 0.6 * 2500 = 1500 or 15°
	//max_stabilize_dampener += pid_stabilize_roll.imax();		// = 0.1 * 300 = 1500 or 15°
	
	max_yaw_dampener		= pid_yaw.kP() * 6000;				// = .5 * 6000  = 3000
	//max_yaw_dampener 	   += pid_yaw.imax();					// = 0.6 * 2500 = 1500 or 15°
}


void output_stabilize()
{
	float roll_error, pitch_error;
	Vector3f omega = dcm.get_gyro();
	float rate;
	int dampener;
	
	// control +- 45° is mixed with the navigation request by the Autopilot
	// output is in degrees = target pitch and roll of copter
	rc_1.servo_out = rc_1.control_mix(nav_roll);
	rc_2.servo_out = rc_2.control_mix(nav_pitch);
		
	roll_error 		= rc_1.servo_out - roll_sensor;
	pitch_error 	= rc_2.servo_out - pitch_sensor;
	
	// limit the error we're feeding to the PID
	roll_error 		= constrain(roll_error,  -2500, 2500);
	pitch_error 	= constrain(pitch_error, -2500, 2500);

	// write out angles back to servo out - this will be converted to PWM by RC_Channel
	rc_1.servo_out 	= pid_stabilize_roll.get_pid(roll_error,  	deltaMiliSeconds, 1.0);
	rc_2.servo_out 	= pid_stabilize_pitch.get_pid(pitch_error, 	deltaMiliSeconds, 1.0);

	// We adjust the output by the rate of rotation:
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading

	// Limit dampening to be equal to propotional term for symmetry
	rate			= degrees(omega.x) * 100.0; 													// 6rad = 34377
	dampener 		= (rate * stabilize_dampener);										// 34377 * .175 = 6000
	rc_1.servo_out	-= constrain(dampener,  -max_stabilize_dampener, max_stabilize_dampener);	// limit to 1500 based on kP

	rate			= degrees(omega.y) * 100.0; 													// 6rad = 34377
	dampener 		= (rate * stabilize_dampener);										// 34377 * .175 = 6000
	rc_2.servo_out	-= constrain(dampener,  -max_stabilize_dampener, max_stabilize_dampener);	// limit to 1500 based on kP
}

void
clear_yaw_control()
{
	//Serial.print("Clear ");
	rate_yaw_flag  	= false;		// exit rate_yaw_flag
	nav_yaw 		= yaw_sensor;	// save our Yaw
	yaw_error 		= 0;
}


void output_yaw_with_hold(boolean hold)
{
	Vector3f omega 	= dcm.get_gyro();

	if(hold){
		// yaw hold
		
		if(rate_yaw_flag){
			// we are still in motion from rate control
			if(fabs(omega.y) < .15){
				clear_yaw_control();
				hold 			= true;			// just to be explicit
			}else{
				// return to rate control until we slow down.
				hold = false;
			}
		}else{

		}
		
	}else{
		// rate control
		
		// this indicates we are under rate control, when we enter Yaw Hold and 
		// return to 0° per second, we exit rate control and hold the current Yaw		
		rate_yaw_flag 	= true;
		yaw_error 		= 0;
	}
	
	if(hold){
		
		// try and hold the current nav_yaw setting
		yaw_error		= nav_yaw - yaw_sensor; 									// +- 60°
		yaw_error 		= wrap_180(yaw_error);

		// limit the error we're feeding to the PID
		yaw_error		= constrain(yaw_error,   -6000, 6000);						// limit error to 60 degees

		// Apply PID and save the new angle back to RC_Channel
		rc_4.servo_out 	= pid_yaw.get_pid(yaw_error, deltaMiliSeconds, 1.0); 		// .5 * 6000 = 3000
	
		// We adjust the output by the rate of rotation
		long rate		= degrees(omega.z) * 100.0; 									// 3rad = 17188 , 6rad = 34377
		int dampener 	= ((float)rate * hold_yaw_dampener);						// 18000 * .17 = 3000
		
		// Limit dampening to be equal to propotional term for symmetry
		rc_4.servo_out	-= constrain(dampener, -max_yaw_dampener, max_yaw_dampener); 	// -3000
	
	}else{		
		
		// rate control
		long rate		= degrees(omega.z) * 100; 									// 3rad = 17188 , 6rad = 34377
		rate			= constrain(rate, -36000, 36000);							// limit to something fun!
		long error		= ((long)rc_4.control_in * 6) - rate;						// control is += 6000 * 6 = 36000
																					// -error = CCW, 	+error = CW
		rc_4.servo_out 	= pid_acro_rate_yaw.get_pid(error, deltaMiliSeconds, 1.0); 	// .075 * 36000 = 2700
		rc_4.servo_out 	= constrain(rc_4.servo_out, -2400, 2400);					// limit to 2400

	}
}




void output_rate_control()
{
	/*
	Vector3f omega = dcm.get_gyro();

	rc_1.servo_out = rc_2.control_in;
	rc_2.servo_out = rc_2.control_in;
	
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading plus Omega_I, so it´s bias corrected
	rc_1.servo_out 	-= (omega.x * 5729.57795 * acro_dampener);
	rc_2.servo_out 	-= (omega.y * 5729.57795 * acro_dampener);

	//Serial.printf("\trated out %d, omega ", rc_1.servo_out);
	//Serial.print((Omega[0] * 5729.57795 * stabilize_rate_roll_pitch), 3);

	// Limit output
	rc_1.servo_out  = constrain(rc_1.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);
	rc_2.servo_out 	= constrain(rc_2.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);
	*/
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
	pid_nav.reset_I();
	pid_baro_throttle.reset_I();
	pid_sonar_throttle.reset_I();
}



