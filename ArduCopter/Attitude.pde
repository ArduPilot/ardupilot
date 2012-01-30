/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static int
get_stabilize_roll(int32_t target_angle)
{
	// angle error
	target_angle 		= wrap_180(target_angle - dcm.roll_sensor);

#if FRAME_CONFIG == HELI_FRAME

	// limit the error we're feeding to the PID
	target_angle 		= constrain(target_angle, -4500, 4500);

	// convert to desired Rate:
	target_angle 		= g.pi_stabilize_roll.get_pi(target_angle, G_Dt);

	// output control:
	return constrain(target_angle, -4500, 4500);
#else

	// limit the error we're feeding to the PID
	target_angle 		= constrain(target_angle, -2500, 2500);

	// conver to desired Rate:
	int32_t target_rate = g.pi_stabilize_roll.get_p(target_angle);
	int16_t iterm 		= g.pi_stabilize_roll.get_i(target_angle, G_Dt);

	return get_rate_roll(target_rate) + iterm;
#endif
}

static int
get_stabilize_pitch(int32_t target_angle)
{
	// angle error
	target_angle 		= wrap_180(target_angle - dcm.pitch_sensor);

#if FRAME_CONFIG == HELI_FRAME
	// limit the error we're feeding to the PID
	target_angle 		= constrain(target_angle, -4500, 4500);

	// convert to desired Rate:
	target_angle 		= g.pi_stabilize_pitch.get_pi(target_angle, G_Dt);

	// output control:
	return constrain(target_angle, -4500, 4500);
#else
	// limit the error we're feeding to the PID
	target_angle 		= constrain(target_angle, -2500, 2500);

	// conver to desired Rate:
	int32_t target_rate = g.pi_stabilize_pitch.get_p(target_angle);
	int16_t iterm 		= g.pi_stabilize_pitch.get_i(target_angle, G_Dt);

	return get_rate_pitch(target_rate) + iterm;
#endif
}

static int
get_stabilize_yaw(int32_t target_angle)
{
	// angle error
	target_angle 		= wrap_180(target_angle - dcm.yaw_sensor);

	// limit the error we're feeding to the PID
	target_angle 		= constrain(target_angle, -2000, 2000);

	// conver to desired Rate:
	int32_t target_rate = g.pi_stabilize_yaw.get_p(target_angle);
	int16_t iterm 		= g.pi_stabilize_yaw.get_i(target_angle, G_Dt);

#if FRAME_CONFIG == HELI_FRAME  // cannot use rate control for helicopters
	if(!g.heli_ext_gyro_enabled){
		return get_rate_yaw(target_rate) + iterm;
	}else{
		return constrain((target_rate + iterm), -4500, 4500);
	}
#else
	return get_rate_yaw(target_rate) + iterm;
#endif
}

static int
get_rate_roll(int32_t target_rate)
{
	static int32_t last_rate 	= 0;
	int32_t current_rate 	= (omega.x * DEGX100);

	// rate control
	target_rate		 		= target_rate - current_rate;
	target_rate 			= g.pid_rate_roll.get_pid(target_rate, G_Dt);

	// Dampening
	target_rate 			-= constrain((current_rate - last_rate) * g.stablize_d, -500, 500);
	last_rate 				= current_rate;

	// output control:
	return constrain(target_rate, -2500, 2500);
}

static int
get_rate_pitch(int32_t target_rate)
{
	static int32_t last_rate 	= 0;
	int32_t current_rate 	= (omega.y * DEGX100);

	// rate control
	target_rate	 			= target_rate - current_rate;
	target_rate 			= g.pid_rate_pitch.get_pid(target_rate, G_Dt);

	// Dampening
	target_rate 			-= constrain((current_rate - last_rate) * g.stablize_d, -500, 500);
	last_rate 				= current_rate;

	// output control:
	return constrain(target_rate, -2500, 2500);
}

static int
get_rate_yaw(int32_t target_rate)
{
	// rate control
	target_rate	 	= target_rate - (omega.z * DEGX100);
	target_rate 	= g.pid_rate_yaw.get_pid(target_rate, G_Dt);

	// output control:
	int16_t yaw_limit = 1400 + abs(g.rc_4.control_in);

	// smoother Yaw control:
	return constrain(target_rate, -yaw_limit, yaw_limit);
}

static int16_t
get_nav_throttle(int32_t z_error)
{
	static int16_t old_output = 0;
	int16_t rate_error = 0;
	int16_t output = 0;

	// convert to desired Rate:
	rate_error 		= g.pi_alt_hold.get_p(z_error);
	rate_error 		= constrain(rate_error, -100, 100);

	// limit error to prevent I term wind up
	z_error 		= constrain(z_error, -400, 400);

	// compensates throttle setpoint error for hovering
	int16_t iterm = g.pi_alt_hold.get_i(z_error, .1);

	// calculate rate error
	rate_error 		= rate_error - climb_rate;

	// limit the rate
	output =  constrain(g.pid_throttle.get_pid(rate_error, .1), -160, 180);

	// light filter of output
	output = (old_output + output) / 2;

	// save our output
	old_output  = output;

	// output control:
	return output + iterm;
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
	// forces us to update nav throttle
	invalid_throttle 		= true;
	nav_throttle 			= 0;

	// always start Circle mode at same angle
	circle_angle			= 0;

	// We must be heading to a new WP, so XTrack must be 0
	crosstrack_error 		= 0;

	// Will be set by new command
	target_bearing 			= 0;

	// Will be set by new command
	wp_distance 			= 0;

	// Will be set by new command, used by loiter
	long_error 				= 0;
	lat_error  				= 0;

	// Will be set by new command, used by loiter
	next_WP.alt				= 0;
}

/*
  reset all I integrators
 */
static void reset_I_all(void)
{
	reset_rate_I();
	reset_stability_I();
	reset_nav_I();
	reset_wind_I();
	reset_throttle_I();
	reset_optflow_I();

	// This is the only place we reset Yaw
	g.pi_stabilize_yaw.reset_I();
}

static void reset_rate_I()
{
	g.pid_rate_roll.reset_I();
	g.pid_rate_pitch.reset_I();
	g.pid_rate_yaw.reset_I();
}

static void reset_optflow_I(void)
{
	g.pid_optflow_roll.reset_I();
	g.pid_optflow_pitch.reset_I();
	of_roll = 0;
	of_pitch = 0;
}

static void reset_wind_I(void)
{
	// Wind Compensation
	g.pi_loiter_lat.reset_I();
	g.pi_loiter_lon.reset_I();
}

static void reset_nav_I(void)
{
	// Rate control for WP navigation
	g.pid_nav_lat.reset_I();
	g.pid_nav_lon.reset_I();
}

static void reset_throttle_I(void)
{
	// For Altitude Hold
	g.pi_alt_hold.reset_I();
	g.pid_throttle.reset_I();
}

static void reset_stability_I(void)
{
	// Used to balance a quad
	// This only needs to be reset during Auto-leveling in flight
	g.pi_stabilize_roll.reset_I();
	g.pi_stabilize_pitch.reset_I();
}


/*************************************************************
throttle control
****************************************************************/

static long
get_nav_yaw_offset(int yaw_input, int reset)
{
	int32_t _yaw;

	if(reset == 0){
		// we are on the ground
		return dcm.yaw_sensor;

	}else{
		// re-define nav_yaw if we have stick input
		if(yaw_input != 0){
			// set nav_yaw + or - the current location
			_yaw = yaw_input + dcm.yaw_sensor;
			// we need to wrap our value so we can be 0 to 360 (*100)
			return wrap_360(_yaw);

		}else{
			// no stick input, lets not change nav_yaw
			return nav_yaw;
		}
	}
}

static int get_angle_boost(int value)
{
	float temp = cos_pitch_x * cos_roll_x;
	temp = 1.0 - constrain(temp, .5, 1.0);
	int16_t output = temp * value;
	return constrain(output, 0, 200);
//	return (int)(temp * value);
}

#define NUM_G_SAMPLES 40

#if ACCEL_ALT_HOLD == 2
// z -14.4306 = going up
// z -6.4306 = going down
static int get_z_damping()
{
	int output;

	Z_integrator 	+= get_world_Z_accel() - Z_offset;
	output 			= Z_integrator * 3;
	Z_integrator 	= Z_integrator * .8;
	output = constrain(output, -100, 100);
	return output;
}

float get_world_Z_accel()
{
	accels_rot = dcm.get_dcm_matrix() * imu.get_accel();
	//Serial.printf("z %1.4f\n", accels_rot.z);
	return accels_rot.z;
}

static void init_z_damper()
{
	Z_offset = 0;
	for (int i = 0; i < NUM_G_SAMPLES; i++){
		delay(5);
		read_AHRS();
		Z_offset += get_world_Z_accel();
	}
	Z_offset /= (float)NUM_G_SAMPLES;
}




// Accelerometer Z dampening by Aurelio R. Ramos
// ---------------------------------------------
#elif ACCEL_ALT_HOLD == 1

// contains G and any other DC offset
static float estimatedAccelOffset = 0;

// state
static float synVelo = 0;
static float synPos = 0;
static float synPosFiltered = 0;
static float posError = 0;
static float prevSensedPos = 0;

// tuning for dead reckoning
static const float dt_50hz = 0.02;
static float synPosP = 10 * dt_50hz;
static float synPosI = 15 * dt_50hz;
static float synVeloP = 1.5 * dt_50hz;
static float maxVeloCorrection = 5 * dt_50hz;
static float maxSensedVelo = 1;
static float synPosFilter = 0.5;


// Z damping term.
static float fullDampP = 0.100;

float get_world_Z_accel()
{
	accels_rot = dcm.get_dcm_matrix() * imu.get_accel();
	return accels_rot.z;
}

static void init_z_damper()
{
	estimatedAccelOffset = 0;
	for (int i = 0; i < NUM_G_SAMPLES; i++){
		delay(5);
		read_AHRS();
		estimatedAccelOffset += get_world_Z_accel();
	}
	estimatedAccelOffset /= (float)NUM_G_SAMPLES;
}

float dead_reckon_Z(float sensedPos, float sensedAccel)
{
	// the following algorithm synthesizes position and velocity from
	// a noisy altitude and accelerometer data.

	// synthesize uncorrected velocity by integrating acceleration
	synVelo += (sensedAccel - estimatedAccelOffset) * dt_50hz;

	// synthesize uncorrected position by integrating uncorrected velocity
	synPos += synVelo * dt_50hz;

	// filter synPos, the better this filter matches the filtering and dead time
	// of the sensed position, the less the position estimate will lag.
	synPosFiltered = synPosFiltered * (1 - synPosFilter) + synPos * synPosFilter;

	// calculate error against sensor position
	posError = sensedPos - synPosFiltered;

	// correct altitude
	synPos += synPosP * posError;

	// correct integrated velocity by posError
	synVelo = synVelo + constrain(posError, -maxVeloCorrection, maxVeloCorrection) * synPosI;

	// correct integrated velocity by the sensed position delta in a small proportion
	// (i.e., the low frequency of the delta)
	float sensedVelo = (sensedPos - prevSensedPos) / dt_50hz;
	synVelo += constrain(sensedVelo - synVelo, -maxSensedVelo, maxSensedVelo) * synVeloP;

	prevSensedPos = sensedPos;
	return synVelo;
}

static int get_z_damping()
{
	float sensedAccel = get_world_Z_accel();
	float sensedPos = current_loc.alt / 100.0;

	float synVelo = dead_reckon_Z(sensedPos, sensedAccel);
	return constrain(fullDampP * synVelo * (-1), -300, 300);
}

#else

static int get_z_damping()
{
	return 0;
}

static void init_z_damper()
{
}
#endif

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t control_roll)
{
#ifdef OPTFLOW_ENABLED
	static float tot_x_cm = 0;  // total distance from target
    static unsigned long last_of_roll_update = 0;
	int32_t new_roll = 0;

	// check if new optflow data available
	if( optflow.last_update != last_of_roll_update) {
	    last_of_roll_update = optflow.last_update;

		// add new distance moved
		tot_x_cm += optflow.x_cm;

		// only stop roll if caller isn't modifying roll
		if( control_roll == 0 && current_loc.alt < 1500) {
			//new_roll = g.pid_optflow_roll.get_pid(-tot_x_cm, 1.0, 1.0);  // we could use the last update time to calculate the time change
		}else{
		    g.pid_optflow_roll.reset_I();
			tot_x_cm = 0;
		}
		// limit amount of change and maximum angle
		of_roll = constrain(new_roll, (of_roll-20), (of_roll+20));
	}

	// limit max angle
    of_roll = constrain(of_roll, -1000, 1000);
    return control_roll+of_roll;
#else
    return control_roll;
#endif
}

static int32_t
get_of_pitch(int32_t control_pitch)
{
#ifdef OPTFLOW_ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static unsigned long last_of_pitch_update = 0;
	int32_t new_pitch = 0;

	// check if new optflow data available
	if( optflow.last_update != last_of_pitch_update ) {
	    last_of_pitch_update = optflow.last_update;

		// add new distance moved
		tot_y_cm += optflow.y_cm;

		// only stop roll if caller isn't modifying pitch
		if( control_pitch == 0 && current_loc.alt < 1500 ) {
			//new_pitch = g.pid_optflow_pitch.get_pid(tot_y_cm, 1.0, 1.0);  // we could use the last update time to calculate the time change
		}else{
		    tot_y_cm = 0;
		    g.pid_optflow_pitch.reset_I();
		}

		// limit amount of change
		of_pitch = constrain(new_pitch, (of_pitch-20), (of_pitch+20));
	}

	// limit max angle
	of_pitch = constrain(of_pitch, -1000, 1000);
    return control_pitch+of_pitch;
#else
    return control_pitch;
#endif
}
