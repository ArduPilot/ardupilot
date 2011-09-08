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
	rate 		= g.pi_stabilize_roll.get_pi(error, delta_ms_fast_loop);
	//Serial.printf("%d\t%d\t%d ", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG != HELI_FRAME  // cannot use rate control for helicopters
	// Rate P:
	error 		= rate - (long)(degrees(omega.x) * 100.0);
	rate 		= g.pi_rate_roll.get_pi(error, delta_ms_fast_loop);
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
	rate 		= g.pi_stabilize_pitch.get_pi(error, delta_ms_fast_loop);
	//Serial.printf("%d\t%d\t%d ", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG != HELI_FRAME  // cannot use rate control for helicopters
	// Rate P:
	error 		= rate - (long)(degrees(omega.y) * 100.0);
	rate 		= g.pi_rate_pitch.get_pi(error, delta_ms_fast_loop);
	//Serial.printf("%d\t%d\n", (int)error, (int)rate);
#endif

	// output control:
	return (int)constrain(rate, -2500, 2500);
}


#define YAW_ERROR_MAX 2000
static int
get_stabilize_yaw(long target_angle)
{
	long error;
	long rate;

	yaw_error 		= wrap_180(target_angle - dcm.yaw_sensor);

	// limit the error we're feeding to the PID
	yaw_error 		= constrain(yaw_error, -YAW_ERROR_MAX, YAW_ERROR_MAX);
	rate 			= g.pi_stabilize_yaw.get_pi(yaw_error, delta_ms_fast_loop);
	//Serial.printf("%u\t%d\t%d\t", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG == HELI_FRAME  // cannot use rate control for helicopters
	if( ! g.heli_ext_gyro_enabled ) {
		// Rate P:
		error 		= rate - (long)(degrees(omega.z) * 100.0);
		rate 		= g.pi_rate_yaw.get_pi(error, delta_ms_fast_loop);
	}
#else
	// Rate P:
	error 			= rate - (long)(degrees(omega.z) * 100.0);
	rate 			= g.pi_rate_yaw.get_pi(error, delta_ms_fast_loop);
	//Serial.printf("%d\t%d\n", (int)error, (int)rate);
#endif

	// output control:
	return (int)constrain(rate, -2500, 2500);
}

#define ALT_ERROR_MAX 300
static int
get_nav_throttle(long z_error, int target_speed)
{
	int rate_error;
	int throttle;
	float scaler = (float)target_speed/(float)ALT_ERROR_MAX;

	// limit error to prevent I term run up
	z_error 		= constrain(z_error, -ALT_ERROR_MAX, ALT_ERROR_MAX);
	target_speed 	= z_error * scaler;

	rate_error 		= target_speed - altitude_rate;
	rate_error 		= constrain(rate_error, -110, 110);

	throttle 		= g.pi_throttle.get_pi(rate_error, delta_ms_medium_loop);
	return  		  g.throttle_cruise + rate_error;
}


static int
get_rate_roll(long target_rate)
{
	long error;
	target_rate 		= constrain(target_rate, -2500, 2500);

	error		= (target_rate * 4.5) - (long)(degrees(omega.x) * 100.0);
	target_rate = g.pi_rate_roll.get_pi(error, delta_ms_fast_loop);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}

static int
get_rate_pitch(long target_rate)
{
	long error;
	target_rate 		= constrain(target_rate, -2500, 2500);

	error		= (target_rate * 4.5) - (long)(degrees(omega.y) * 100.0);
	target_rate = g.pi_rate_pitch.get_pi(error, delta_ms_fast_loop);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}

static int
get_rate_yaw(long target_rate)
{
	long error;

	error		= (target_rate * 4.5) - (long)(degrees(omega.z) * 100.0);
	target_rate = g.pi_rate_yaw.get_pi(error, delta_ms_fast_loop);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_hold_I(void)
{
	g.pi_loiter_lat.reset_I();
	g.pi_loiter_lat.reset_I();
	g.pi_crosstrack.reset_I();
}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_nav_I(void)
{
	g.pi_nav_lat.reset_I();
	g.pi_nav_lon.reset_I();
}


/*************************************************************
throttle control
****************************************************************/

// user input:
// -----------
static int get_throttle(int throttle_input)
{
	throttle_input = (float)throttle_input * angle_boost();
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
}
*/

static float angle_boost()
{
	float temp = cos_pitch_x * cos_roll_x;
	temp = 2.0 - constrain(temp, .5, 1.0);
	return temp;
}

