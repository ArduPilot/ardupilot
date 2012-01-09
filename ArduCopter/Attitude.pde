/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static int
get_stabilize_roll(int32_t target_angle)
{
	int32_t error;
	int32_t rate;
	int32_t current_rate;

	int16_t rate_d1 = 0;
	static int16_t rate_d2 = 0;
	static int16_t rate_d3 = 0;
	static int32_t last_rate = 0;

	current_rate 	= (omega.x * DEGX100);

	// playing with double derivatives.
	// History of last 3 dir
	rate_d3			= rate_d2;
	rate_d2			= rate_d1;
	rate_d1 		= current_rate - last_rate;
	last_rate 		= current_rate;

	// angle error
	error 		= wrap_180(target_angle - dcm.roll_sensor);

#if FRAME_CONFIG == HELI_FRAME
	// limit the error we're feeding to the PID
	error 		= constrain(error, -4500, 4500);

	// convert to desired Rate:
	rate 		= g.pi_stabilize_roll.get_pi(error, G_Dt);

	// output control:
	rate = constrain(rate, -4500, 4500);
	return (int)rate;
#else
	// limit the error we're feeding to the PID
	error 		= constrain(error, -2500, 2500);

	// conver to desired Rate:
	rate 		= g.pi_stabilize_roll.get_p(error);

	// experiment to pipe iterm directly into the output
	int16_t iterm = g.pi_stabilize_roll.get_i(error, G_Dt);

	// rate control
	error 		= rate - current_rate;
	rate 		= g.pi_rate_roll.get_pi(error, G_Dt);

	// D term
	// I had tried this before with little result. Recently, someone mentioned to me that
	// MultiWii uses a filter of the last three to get around noise and get a stronger signal.
	// Works well! Thanks!
	int16_t d_temp =  (rate_d1 + rate_d2 + rate_d3) * g.stablize_d;

	rate -= d_temp;

	// output control:
	rate = constrain(rate, -2500, 2500);
	return (int)rate + iterm;
#endif
}

static int
get_stabilize_pitch(int32_t target_angle)
{
	int32_t error;
	int32_t rate;
	int32_t current_rate;

	int16_t rate_d1 = 0;
	static int16_t rate_d2 = 0;
	static int16_t rate_d3 = 0;
	static int32_t last_rate = 0;

	current_rate 	= (omega.y * DEGX100);

	// playing with double derivatives.
	// History of last 3 dir
	rate_d3			= rate_d2;
	rate_d2			= rate_d1;
	rate_d1 		= current_rate - last_rate;
	last_rate 		= current_rate;

	// angle error
	error 		= wrap_180(target_angle - dcm.pitch_sensor);

#if FRAME_CONFIG == HELI_FRAME
	// limit the error we're feeding to the PID
	error 		= constrain(error, -4500, 4500);

	// convert to desired Rate:
	rate 		= g.pi_stabilize_pitch.get_pi(error, G_Dt);

	// output control:
	rate = constrain(rate, -4500, 4500);
	return (int)rate;
#else
	// angle error
	error 		= constrain(error, -2500, 2500);

	// conver to desired Rate:
	rate 		= g.pi_stabilize_pitch.get_p(error);

	// experiment to pipe iterm directly into the output
	int16_t iterm = g.pi_stabilize_pitch.get_i(error, G_Dt);

	// rate control
	error 		= rate - (omega.y * DEGX100);
	rate 		= g.pi_rate_pitch.get_pi(error, G_Dt);

	// D term testing
	int16_t d_temp =  (rate_d1 + rate_d2 + rate_d3) * g.stablize_d;

	rate -= d_temp;

	// output control:
	rate = constrain(rate, -2500, 2500);
	return (int)rate + iterm;
#endif
}


#define YAW_ERROR_MAX 2000
static int
get_stabilize_yaw(int32_t target_angle)
{
	int32_t error;
	int32_t rate;

	// angle error
	error 		= wrap_180(target_angle - dcm.yaw_sensor);

	// limit the error we're feeding to the PID
	error 		= constrain(error, -YAW_ERROR_MAX, YAW_ERROR_MAX);

	// convert to desired Rate:
	rate 		= g.pi_stabilize_yaw.get_p(error);

	// experiment to pipe iterm directly into the output
	int16_t iterm = g.pi_stabilize_yaw.get_i(error, G_Dt);

#if FRAME_CONFIG == HELI_FRAME  // cannot use rate control for helicopters
	if( !g.heli_ext_gyro_enabled ) {
		error 	= rate - (omega.z * DEGX100);
		rate 	= g.pi_rate_yaw.get_pi(error, G_Dt);
	}
	// output control:
	rate = constrain(rate, -4500, 4500);
#else
	error 		= rate - (omega.z * DEGX100);
	rate 		= g.pi_rate_yaw.get_pi(error, G_Dt);

	// output control:
	int16_t yaw_input = 1400 + abs(g.rc_4.control_in);
	// smoother Yaw control:
	rate = constrain(rate, -yaw_input, yaw_input);
#endif

	return (int)rate + iterm;
}

#define ALT_ERROR_MAX 400
static int16_t
get_nav_throttle(int32_t z_error)
{
	static int16_t old_output = 0;

	int16_t rate_error;
	int16_t output;

	// XXX HACK, need a better way not to ramp this i term in large altitude changes.
	float dt = (abs(z_error) < 400) ? .1 : 0.0;

	// limit error to prevent I term run up
	z_error 		= constrain(z_error, -ALT_ERROR_MAX, ALT_ERROR_MAX);

	// convert to desired Rate:
	rate_error 	= g.pi_alt_hold.get_p(z_error); //_p = .85

	// experiment to pipe iterm directly into the output
	int16_t iterm = g.pi_alt_hold.get_i(z_error, dt);

	// calculate rate error
	rate_error 		= rate_error - climb_rate;

	// limit the rate - iterm is not used
	output =  constrain((int)g.pi_throttle.get_p(rate_error), -160, 180);

	// light filter of output
	output = (old_output * 3 + output) / 4;

	// save our output
	old_output  = output;

	// output control:
	return output + iterm;
}

static int
get_rate_roll(int32_t target_rate)
{
	int32_t error	= (target_rate * 3.5) - (omega.x * DEGX100);
	error = constrain(error, -20000, 20000);
	return g.pi_acro_roll.get_pi(error, G_Dt);
}

static int
get_rate_pitch(int32_t target_rate)
{
	int32_t error	= (target_rate * 3.5) - (omega.y * DEGX100);
	error = constrain(error, -20000, 20000);
	return  g.pi_acro_pitch.get_pi(error, G_Dt);
}

static int
get_rate_yaw(int32_t target_rate)
{

	int32_t error	= (target_rate * 4.5) - (omega.z * DEGX100);
	target_rate = g.pi_rate_yaw.get_pi(error, G_Dt);

	// output control:
	return (int)constrain(target_rate, -2500, 2500);
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_hold_I(void)
{
	g.pi_loiter_lat.reset_I();
	g.pi_loiter_lon.reset_I();
}

// Keeps old data out of our calculation / logs
static void reset_nav(void)
{
	invalid_throttle 		= true;
	nav_throttle 			= 0;
	circle_angle			= 0;
	crosstrack_error 		= 0;
	target_bearing 			= 0;
	wp_distance 			= 0;
	long_error 				= 0;
	lat_error  				= 0;
}

static void reset_rate_I()
{
	// balances the quad
	g.pi_stabilize_roll.reset_I();
	g.pi_stabilize_pitch.reset_I();

	// compensates rate error
	g.pi_rate_roll.reset_I();
	g.pi_rate_pitch.reset_I();
	g.pi_acro_roll.reset_I();
	g.pi_acro_pitch.reset_I();
	g.pi_optflow_roll.reset_I();
	g.pi_optflow_pitch.reset_I();
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
	return (int)(temp * value);
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

// calculate modified roll/pitch depending upon optical flow values
static int32_t
get_of_roll(int32_t control_roll)
{
#ifdef OPTFLOW_ENABLED
	//static int32_t of_roll = 0;  // we use global variable to make logging easier
    static unsigned long last_of_roll_update = 0;
	static float prev_value = 0;
	float x_cm;

	// check if new optflow data available
	if( optflow.last_update != last_of_roll_update) {
	    last_of_roll_update = optflow.last_update;

		// filter movement
		x_cm = (optflow.x_cm + prev_value) / 2.0 * 50.0;

		// only stop roll if caller isn't modifying roll
		if( control_roll == 0 && current_loc.alt < 1500) {
			of_roll = g.pi_optflow_roll.get_pi(-x_cm, 1.0);  // we could use the last update time to calculate the time change
		}else{
		    g.pi_optflow_roll.reset_I();
			prev_value = 0;
		}
	}
	// limit maximum angle
	of_roll	= constrain(of_roll, -1000, 1000);

    return control_roll+of_roll;
#else
    return control_roll;
#endif
}

static int32_t
get_of_pitch(int32_t control_pitch)
{
#ifdef OPTFLOW_ENABLED
    //static int32_t of_pitch = 0;  // we use global variable to make logging easier
    static unsigned long last_of_pitch_update = 0;
	static float prev_value = 0;
	float y_cm;

	// check if new optflow data available
	if( optflow.last_update != last_of_pitch_update ) {
	    last_of_pitch_update = optflow.last_update;

		// filter movement
		y_cm = (optflow.y_cm + prev_value) / 2.0 * 50.0;

		// only stop roll if caller isn't modifying roll
		if( control_pitch == 0 && current_loc.alt < 1500 ) {
			of_pitch = g.pi_optflow_pitch.get_pi(y_cm, 1.0);  // we could use the last update time to calculate the time change
		}else{
		    g.pi_optflow_pitch.reset_I();
			prev_value = 0;
		}
	}
	// limit maximum angle
	of_pitch = constrain(of_pitch, -1000, 1000);
    return control_pitch+of_pitch;
#else
    return control_pitch;
#endif
}
