/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
static int
get_stabilize_roll(long target_angle)
{
	long error;
	long rate;

	error 		= wrap_180(target_angle - dcm.roll_sensor);

	// limit the error we're feeding to the PID
	error 		= constrain(error, -2500, 2500);

	// desired Rate:
	rate 		= g.pi_stabilize_roll.get_pi(error, G_Dt);
	//Serial.printf("%d\t%d\t%d ", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG != HELI_FRAME  // cannot use rate control for helicopters
	// Rate P:
	error 		= rate - (long)(degrees(omega.x) * 100.0);
	rate 		= g.pi_rate_roll.get_pi(error, G_Dt);
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
	rate 		= g.pi_stabilize_pitch.get_pi(error, G_Dt);
	//Serial.printf("%d\t%d\t%d ", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG != HELI_FRAME  // cannot use rate control for helicopters
	// Rate P:
	error 		= rate - (long)(degrees(omega.y) * 100.0);
	rate 		= g.pi_rate_pitch.get_pi(error, G_Dt);
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
	rate 			= g.pi_stabilize_yaw.get_pi(yaw_error, G_Dt);
	//Serial.printf("%u\t%d\t%d\t", (int)target_angle, (int)error, (int)rate);

#if FRAME_CONFIG == HELI_FRAME  // cannot use rate control for helicopters
	if( ! g.heli_ext_gyro_enabled ) {
		// Rate P:
		error 		= rate - (long)(degrees(omega.z) * 100.0);
		rate 		= g.pi_rate_yaw.get_pi(error, G_Dt);
	}
#else
	// Rate P:
	error 			= rate - (long)(degrees(omega.z) * 100.0);
	rate 			= g.pi_rate_yaw.get_pi(error, G_Dt);
	//Serial.printf("%d\t%d\n", (int)error, (int)rate);
#endif

	// output control:
	return (int)constrain(rate, -2500, 2500);
}

#define ALT_ERROR_MAX 400
static int
get_nav_throttle(long z_error) 
{
	// limit error to prevent I term run up
	z_error 		= constrain(z_error, -ALT_ERROR_MAX, ALT_ERROR_MAX);
	int rate_error 	= g.pi_alt_hold.get_pi(z_error, .1); //_p = .85

	rate_error 		= rate_error - altitude_rate;

	// limit the rate
	rate_error 		= constrain(rate_error, -100, 120);
	return (int)g.pi_throttle.get_pi(rate_error, .1);
}

static int
get_rate_roll(long target_rate)
{
	long error;
	target_rate 	= constrain(target_rate, -2500, 2500);
	error			= (target_rate * 4.5) - (long)(degrees(omega.x) * 100.0);
	target_rate 	= g.pi_rate_roll.get_pi(error, G_Dt);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}

static int
get_rate_pitch(long target_rate)
{
	long error;
	target_rate 	= constrain(target_rate, -2500, 2500);
	error			= (target_rate * 4.5) - (long)(degrees(omega.y) * 100.0);
	target_rate 	= g.pi_rate_pitch.get_pi(error, G_Dt);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}

static int
get_rate_yaw(long target_rate)
{
	long error;
	error		= (target_rate * 4.5) - (long)(degrees(omega.z) * 100.0);
	target_rate = g.pi_rate_yaw.get_pi(error, G_Dt);

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
static void reset_nav(void)
{
	nav_throttle 		= 0;
	invalid_throttle 	= true;

	g.pi_nav_lat.reset_I();
	g.pi_nav_lon.reset_I();

	long_error = 0;
	lat_error  = 0;
}


/*************************************************************
throttle control
****************************************************************/

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
			_yaw = (long)yaw_input + dcm.yaw_sensor;
			// we need to wrap our value so we can be 0 to 360 (*100)
			return wrap_360(_yaw);

		}else{
			// no stick input, lets not change nav_yaw
			return nav_yaw;
		}
	}
}

static int alt_hold_velocity()
{
	#if ACCEL_ALT_HOLD == 1
		// subtract filtered Accel
		float error	= abs(next_WP.alt - current_loc.alt);

		error -= 100;
		error = min(error, 200.0);
		error = max(error, 0.0);
		error = 1 - (error/ 200.0);
		float sum = accels_rot_sum / (float)accels_rot_count;

		accels_rot_sum = 0;
		accels_rot_count = 0;

		int output = (sum + 9.81) * alt_hold_gain * error;

// fast rise
//s: -17.6241, g:0.0000, e:1.0000, o:0
//s: -18.4990, g:0.0000, e:1.0000, o:0
//s: -19.3193, g:0.0000, e:1.0000, o:0
//s: -13.1310, g:47.8700, e:1.0000, o:-158

		//Serial.printf("s: %1.4f, g:%1.4f, e:%1.4f, o:%d\n",sum, alt_hold_gain, error, output);
		return output;
	#else
		return 0;
	#endif
}

static int get_angle_boost()
{
	float temp = cos_pitch_x * cos_roll_x;
	temp = 1.0 - constrain(temp, .5, 1.0);
	return (int)(temp * g.throttle_cruise);
}

