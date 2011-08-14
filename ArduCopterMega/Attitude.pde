/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// XXX TODO: convert these PI rate controlers to a Class
static int
get_stabilize_roll(long target_angle)
{
	long error;
	long rate;

	error 		= wrap_180(target_angle - dcm.roll_sensor);

	// limit the error we're feeding to the PID
	error 		= constrain(error, -2500, 2500);

	// desired Rate:
	rate 		= g.pid_stabilize_roll.get_pi((float)error, delta_ms_fast_loop, 1.0);
	//Serial.printf("%d\t%d\t%d ", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG != HELI_FRAME  // cannot use rate control for helicopters
	// Rate P:
	error 		= rate - (long)(degrees(omega.x) * 100.0);
	rate 		= g.pid_rate_roll.get_pi((float)error, delta_ms_fast_loop, 1.0);
	//Serial.printf("%d\t%d\n", (int)error, (int)rate);
#endif

	// output control:
	return (int)constrain(rate, -2500, 2500);	
	
}

static int
get_stabilize_pitch(long target_angle)
{
	long error;
	long rate;

	error 		= wrap_180(target_angle - dcm.pitch_sensor);

	// limit the error we're feeding to the PID
	error 		= constrain(error, -2500, 2500);

	// desired Rate:
	rate 		= g.pid_stabilize_pitch.get_pi((float)error, delta_ms_fast_loop, 1.0);
	//Serial.printf("%d\t%d\t%d ", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG != HELI_FRAME  // cannot use rate control for helicopters
	// Rate P:
	error 		= rate - (long)(degrees(omega.y) * 100.0);
	rate 		= g.pid_rate_pitch.get_pi((float)error, delta_ms_fast_loop, 1.0);
	//Serial.printf("%d\t%d\n", (int)error, (int)rate);
#endif

	// output control:
	return (int)constrain(rate, -2500, 2500);
}

static int
get_stabilize_yaw(long target_angle, float scaler)
{
	long error;
	long rate;

	error 		= wrap_180(target_angle - dcm.yaw_sensor);


	// limit the error we're feeding to the PID
	error 		= constrain(error, -4500, 4500);

	// desired Rate:
	rate 		= g.pid_stabilize_yaw.get_pi((float)error, delta_ms_fast_loop, scaler);
	//Serial.printf("%u\t%d\t%d\t", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG == HELI_FRAME  // cannot use rate control for helicopters
	if( ! g.heli_ext_gyro_enabled ) {
		// Rate P:
		error 		= rate - (long)(degrees(omega.z) * 100.0);
		rate 		= g.pid_rate_yaw.get_pi((float)error, delta_ms_fast_loop, 1.0);
	}
#else
	// Rate P:
	error 		= rate - (long)(degrees(omega.z) * 100.0);
	rate 		= g.pid_rate_yaw.get_pi((float)error, delta_ms_fast_loop, 1.0);
	//Serial.printf("%d\t%d\n", (int)error, (int)rate);
#endif

	// output control:
	return (int)constrain(rate, -2500, 2500);
}

static int
get_rate_roll(long target_rate)
{
	long error;
	target_rate 		= constrain(target_rate, -2500, 2500);

	error		= (target_rate * 4.5) - (long)(degrees(omega.x) * 100.0);
	target_rate = g.pid_rate_roll.get_pi((float)error, delta_ms_fast_loop, 1.0);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}

static int
get_rate_pitch(long target_rate)
{
	long error;
	target_rate 		= constrain(target_rate, -2500, 2500);

	error		= (target_rate * 4.5) - (long)(degrees(omega.y) * 100.0);
	target_rate = g.pid_rate_pitch.get_pi((float)error, delta_ms_fast_loop, 1.0);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}

static int
get_rate_yaw(long target_rate)
{
	long error;

	error		= (target_rate * 4.5) - (long)(degrees(omega.z) * 100.0);
	target_rate = g.pid_rate_yaw.get_pi((float)error, delta_ms_fast_loop, 1.0);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void
reset_nav_I(void)
{
	g.pid_nav_lat.reset_I();
	g.pid_nav_lon.reset_I();
	g.pid_nav_wp.reset_I();
	g.pid_crosstrack.reset_I();
	g.pid_throttle.reset_I();
	// I removed these, they don't seem to be needed.
}


/*************************************************************
throttle control
****************************************************************/

// user input:
// -----------
static int
get_throttle(int throttle_input)
{
	throttle_input = (float)throttle_input * angle_boost();
	//throttle_input = max(throttle_slew, throttle_input);
	return  max(throttle_input, 0);
}

static long
get_nav_yaw_offset(int yaw_input, int reset)
{
	long _yaw;

	if(reset == 0){
		// we are on the ground
		return dcm.yaw_sensor;

	}else{
		// re-define nav_yaw if we have stick input
		if(yaw_input != 0){
			// set nav_yaw + or - the current location
			_yaw 	= (long)yaw_input + dcm.yaw_sensor;
			// we need to wrap our value so we can be 0 to 360 (*100)
			return wrap_360(_yaw);

		}else{
			// no stick input, lets not change nav_yaw
			return nav_yaw;
		}
	}
}
/*
static int alt_hold_velocity()
{
	// subtract filtered Accel
	float error	= abs(next_WP.alt - current_loc.alt);
	error = min(error, 200);
	error = 1 - (error/ 200.0);
	return (accels_rot.z + 9.81) * accel_gain * error;
}*/

static float angle_boost()
{
	float temp = cos_pitch_x * cos_roll_x;
	temp = 2.0 - constrain(temp, .5, 1.0);
	return temp;
}

