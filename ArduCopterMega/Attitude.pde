/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

void
init_pids()
{
	// create limits to how much dampening we'll allow
	// this creates symmetry with the P gain value preventing oscillations

	max_stabilize_dampener 	= g.pid_stabilize_roll.kP() * 2500;	// = 0.6 * 2500 = 1500 or 15°
	max_yaw_dampener		= g.pid_yaw.kP() * 6000;				// = .5 * 6000  = 3000
}

void
control_nav_mixer()
{
	// control +- 45° is mixed with the navigation request by the Autopilot
	// output is in degrees = target pitch and roll of copter
	g.rc_1.servo_out = g.rc_1.control_mix(nav_roll);
	g.rc_2.servo_out = g.rc_2.control_mix(nav_pitch);
}

void
simple_mixer()
{
	// control +- 45° is mixed with the navigation request by the Autopilot
	// output is in degrees = target pitch and roll of copter
	g.rc_1.servo_out = nav_roll;
	g.rc_2.servo_out = nav_pitch;
}

void
output_stabilize_roll()
{
	float error, rate;
	int dampener;

	error 		= g.rc_1.servo_out - dcm.roll_sensor;

	// limit the error we're feeding to the PID
	error 		= constrain(error, -2500, 2500);

	// only buildup I if we are trying to roll hard
	if(abs(g.rc_1.servo_out) < 1500){
		// smoother alternative to reset?
		//if(g.pid_stabilize_roll.kI() != 0){
		//	g.pid_stabilize_roll.kI(g.pid_stabilize_roll.kI() * .8);
		//}
		g.pid_stabilize_roll.reset_I();
	}

	// write out angles back to servo out - this will be converted to PWM by RC_Channel
	g.rc_1.servo_out 	= g.pid_stabilize_roll.get_pid(error,  	delta_ms_fast_loop, 1.0);

	// We adjust the output by the rate of rotation:
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading

	// Limit dampening to be equal to propotional term for symmetry
	rate				= degrees(omega.x) * 100.0; 													// 6rad = 34377
	dampener 			= (rate * g.stabilize_dampener);										// 34377 * .175 = 6000
	g.rc_1.servo_out	-= constrain(dampener,  -max_stabilize_dampener, max_stabilize_dampener);	// limit to 1500 based on kP
}

void
output_stabilize_pitch()
{
	float error, rate;
	int dampener;

	error		= g.rc_2.servo_out - dcm.pitch_sensor;

	// limit the error we're feeding to the PID
	error		= constrain(error, -2500, 2500);

	// only buildup I if we are trying to roll hard
	if(abs(g.rc_2.servo_out) < 1500){
		g.pid_stabilize_pitch.reset_I();
	}

	// write out angles back to servo out - this will be converted to PWM by RC_Channel
	g.rc_2.servo_out 	= g.pid_stabilize_pitch.get_pid(error, 	delta_ms_fast_loop, 1.0);

	// We adjust the output by the rate of rotation:
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading

	// Limit dampening to be equal to propotional term for symmetry
	rate				= degrees(omega.y) * 100.0; 													// 6rad = 34377
	dampener 			= (rate * g.stabilize_dampener);										// 34377 * .175 = 6000
	g.rc_2.servo_out	-= constrain(dampener,  -max_stabilize_dampener, max_stabilize_dampener);	// limit to 1500 based on kP
}

void
clear_yaw_control()
{
	//Serial.print("Clear ");
	rate_yaw_flag  	= false;			// exit rate_yaw_flag
	nav_yaw 		= dcm.yaw_sensor;	// save our Yaw
	yaw_error 		= 0;
}

void
output_yaw_with_hold(boolean hold)
{
	if(hold){
		// look to see if we have exited rate control properly - ie stopped turning
		if(rate_yaw_flag){
			// we are still in motion from rate control
			if(fabs(omega.z) < .5){
				clear_yaw_control();
				hold = true;			// just to be explicit
				//Serial.print("C");

			}else{

				// return to rate control until we slow down.
				hold = false;
				//Serial.print("D");
			}
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
		yaw_error			= nav_yaw - dcm.yaw_sensor; 									// +- 60°
		yaw_error 			= wrap_180(yaw_error);

		// limit the error we're feeding to the PID
		yaw_error			= constrain(yaw_error,	 -6000, 6000);						// limit error to 60 degees

		// Apply PID and save the new angle back to RC_Channel
		g.rc_4.servo_out 	= g.pid_yaw.get_pid(yaw_error, delta_ms_fast_loop, 1.0); 		// .5 * 6000 = 3000

	}else{

		// rate control
		long rate		= degrees(omega.z) * 100; 											// 3rad = 17188 , 6rad = 34377
		rate			= constrain(rate, -36000, 36000);									// limit to something fun!
		long error		= ((long)g.rc_4.control_in * 6) - rate;								// control is += 6000 * 6 = 36000
																							// -error = CCW, 	+error = CW

		if(g.rc_4.control_in == 0){
			// we are breaking;
			//g.rc_4.servo_out = (omega.z > 0) ? -600 : 600;
			g.rc_4.servo_out = (int)((float)g.rc_4.servo_out * (omega.z / 6.0));

		}else{

			g.rc_4.servo_out 	= g.pid_acro_rate_yaw.get_pid(error, delta_ms_fast_loop, 1.0); // kP .07 * 36000 = 2520
		}
	}

	// We adjust the output by the rate of rotation
	long rate			= degrees(omega.z) * 100.0; 									// 3rad = 17188 , 6rad = 34377
	int dampener 		= (float)rate * g.hold_yaw_dampener;						// 18000 * .17 = 3000

	// Limit dampening to be equal to propotional term for symmetry
	g.rc_4.servo_out	-= constrain(dampener, -max_yaw_dampener, max_yaw_dampener); 	// -3000

	// Limit Output
	g.rc_4.servo_out 	= constrain(g.rc_4.servo_out, -1800, 1800);							// limit to 24°

	//Serial.printf("%d\n",g.rc_4.servo_out);
}

void
output_rate_roll()
{
	// rate control
	long rate		= degrees(omega.x) * 100; 												// 3rad = 17188 , 6rad = 34377
	rate			= constrain(rate, -36000, 36000);										// limit to something fun!
	long error		= ((long)g.rc_1.control_in * 8) - rate;									// control is += 4500 * 8 = 36000

	g.rc_1.servo_out 	= g.pid_acro_rate_roll.get_pid(error, delta_ms_fast_loop, 1.0); 	// .075 * 36000 = 2700
	g.rc_1.servo_out 	= constrain(g.rc_1.servo_out, -2400, 2400);							// limit to 2400
}

void
output_rate_pitch()
{
	// rate control
	long rate		= degrees(omega.y) * 100; 												// 3rad = 17188 , 6rad = 34377
	rate			= constrain(rate, -36000, 36000);										// limit to something fun!
	long error		= ((long)g.rc_2.control_in * 8) - rate;									// control is += 4500 * 8 = 36000

	g.rc_2.servo_out 	= g.pid_acro_rate_pitch.get_pid(error, delta_ms_fast_loop, 1.0); 	// .075 * 36000 = 2700
	g.rc_2.servo_out 	= constrain(g.rc_2.servo_out, -2400, 2400);							// limit to 2400
}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void
reset_I(void)
{
	g.pid_nav_lat.reset_I();
	g.pid_nav_lon.reset_I();
	g.pid_baro_throttle.reset_I();
	g.pid_sonar_throttle.reset_I();
}


/*************************************************************
throttle control
****************************************************************/

// user input:
// -----------
void output_manual_throttle()
{
	g.rc_3.servo_out = (float)g.rc_3.control_in * angle_boost();
}

// Autopilot
// ---------
void output_auto_throttle()
{
	g.rc_3.servo_out 	= (float)nav_throttle * angle_boost();
	// make sure we never send a 0 throttle that will cut the motors
	g.rc_3.servo_out = max(g.rc_3.servo_out, 1);
}

void calc_nav_throttle()
{
	// limit error
	long error = constrain(altitude_error, -400, 400);
	float scaler = 1.0;

	if(error < 0){
		scaler = (altitude_sensor == BARO) ? .5 : .5;
	}

	if(altitude_sensor == BARO){
		nav_throttle = g.pid_baro_throttle.get_pid(error, delta_ms_fast_loop, scaler);
		nav_throttle = g.throttle_cruise + constrain(nav_throttle, -30, 80);
	}else{
		nav_throttle = g.pid_sonar_throttle.get_pid(error, delta_ms_fast_loop, scaler);
		nav_throttle = g.throttle_cruise + constrain(nav_throttle, -60, 100);
	}

	nav_throttle = (nav_throttle + nav_throttle_old) >> 1;
	nav_throttle_old = nav_throttle;

	//Serial.printf("nav_thr %d, scaler %2.2f ", nav_throttle, scaler);
}

float angle_boost()
{
	float temp = cos_pitch_x * cos_roll_x;
	temp = 2.0 - constrain(temp, .7, 1.0);
	return temp;
}

/*************************************************************
yaw control
****************************************************************/

void output_manual_yaw()
{
	if(g.rc_3.control_in == 0){
		clear_yaw_control();
	}else{
		// Yaw control
		if(g.rc_4.control_in == 0){
			output_yaw_with_hold(true); // hold yaw
		}else{
			output_yaw_with_hold(false); // rate control yaw
		}
	}
}

void auto_yaw()
{
	if(yaw_tracking & TRACK_NEXT_WP){
		nav_yaw = target_bearing;
	}

	output_yaw_with_hold(true); // hold yaw
}
