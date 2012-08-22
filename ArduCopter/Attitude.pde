/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static int16_t
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180(target_angle - ahrs.roll_sensor);

#if FRAME_CONFIG == HELI_FRAME

    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

    // convert to desired Rate:
    target_angle            = g.pi_stabilize_roll.get_pi(target_angle, G_Dt);

    // output control:
    return constrain(target_angle, -4500, 4500);
#else

    // convert to desired Rate:
    int32_t target_rate = g.pi_stabilize_roll.get_p(target_angle);

    int16_t i_stab;
    if(labs(ahrs.roll_sensor) < 500) {
        target_angle            = constrain(target_angle, -500, 500);
        i_stab                          = g.pi_stabilize_roll.get_i(target_angle, G_Dt);
    }else{
        i_stab                          = g.pi_stabilize_roll.get_integrator();
    }

    return get_rate_roll(target_rate) + i_stab;
#endif
}

static int16_t
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180(target_angle - ahrs.pitch_sensor);

#if FRAME_CONFIG == HELI_FRAME
    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

    // convert to desired Rate:
    target_angle            = g.pi_stabilize_pitch.get_pi(target_angle, G_Dt);

    // output control:
    return constrain(target_angle, -4500, 4500);
#else

    // convert to desired Rate:
    int32_t target_rate = g.pi_stabilize_pitch.get_p(target_angle);

    int16_t i_stab;
    if(labs(ahrs.pitch_sensor) < 500) {
        target_angle            = constrain(target_angle, -500, 500);
        i_stab                          = g.pi_stabilize_pitch.get_i(target_angle, G_Dt);
    }else{
        i_stab                          = g.pi_stabilize_pitch.get_integrator();
    }
    return get_rate_pitch(target_rate) + i_stab;

#endif
}

static int16_t
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate,i_term;
    int32_t angle_error;
    int32_t output;

    // angle error
    angle_error             = wrap_180(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
#if FRAME_CONFIG == HELI_FRAME
    angle_error             = constrain(angle_error, -4500, 4500);
#else
    angle_error             = constrain(angle_error, -4000, 4000);
#endif

    // convert angle error to desired Rate:
    target_rate = g.pi_stabilize_yaw.get_p(angle_error);
    i_term = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

    // do not use rate controllers for helicotpers with external gyros
#if FRAME_CONFIG == HELI_FRAME
    if(!motors.ext_gyro_enabled) {
        output = get_rate_yaw(target_rate) + i_term;
    }else{
        output = constrain((target_rate + i_term), -4500, 4500);
    }
#else
    output = get_rate_yaw(target_rate) + i_term;
#endif

#if LOGGING_ENABLED == ENABLED
    static int8_t log_counter = 0;              // used to slow down logging of PID values to dataflash
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_KP || g.radio_tuning == CH6_YAW_RATE_KP) ) {
        log_counter++;
        if( log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            log_counter = 0;
            Log_Write_PID(CH6_YAW_KP, angle_error, target_rate, i_term, 0, output, tuning_value);
        }
    }
#endif

    // ensure output does not go beyond barries of what an int16_t can hold
    return constrain(output,-32000,32000);
}

static int16_t
get_acro_roll(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;
    return get_rate_roll(target_rate);
}

static int16_t
get_acro_pitch(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;
    return get_rate_pitch(target_rate);
}

static int16_t
get_acro_yaw(int32_t target_rate)
{
    target_rate = g.pi_stabilize_yaw.get_p(target_rate);
    return get_rate_yaw(target_rate);
}

/*
 *  static int16_t
 *  get_acro_yaw2(int32_t target_rate)
 *  {
 *       int32_t p,i,d;									// used to capture pid values for logging
 *       int32_t	rate_error;								// current yaw rate error
 *       int32_t	current_rate;							// current real yaw rate
 *       int32_t decel_boost;							// gain scheduling if we are overshooting
 *       int32_t output;									// output to rate controller
 *
 *       target_rate = g.pi_stabilize_yaw.get_p(target_rate);
 *       current_rate = omega.z * DEGX100;
 *       rate_error = target_rate - current_rate;
 *
 *       //Gain Scheduling:
 *       //If the yaw input is to the right, but stick is moving to the middle
 *       //and actual rate is greater than the target rate then we are
 *       //going to overshoot the yaw target to the left side, so we should
 *       //strengthen the yaw output to slow down the yaw!
 *
 * #if (FRAME_CONFIG == HELI_FRAME	|| FRAME_CONFIG == TRI_FRAME)
 *       static int32_t last_target_rate = 0;			// last iteration's target rate
 *       if ( target_rate > 0 && last_target_rate > target_rate && rate_error < 0 ){
 *               decel_boost = 1;
 *       } else if (target_rate < 0 && last_target_rate < target_rate && rate_error > 0 ){
 *               decel_boost = 1;
 *       } else if (target_rate == 0 && labs(current_rate) > 1000){
 *               decel_boost = 1;
 *       } else {
 *               decel_boost = 0;
 *       }
 *
 *       last_target_rate = target_rate;
 *
 * #else
 *
 *       decel_boost = 0;
 *
 * #endif
 *
 *       // separately calculate p, i, d values for logging
 *       // we will use d=0, and hold i at it's last value
 *       // since manual inputs are never steady state
 *
 *       p = g.pid_rate_yaw.get_p(rate_error);
 *       i = g.pid_rate_yaw.get_integrator();
 *       d = 0;
 *
 *       if (decel_boost){
 *               p *= 2;
 *       }
 *
 *       output     = p+i+d;
 *
 *       // output control:
 *       // constrain output
 *       output = constrain(output, -4500, 4500);
 *
 * #if LOGGING_ENABLED == ENABLED
 *       static int8_t log_counter = 0;					// used to slow down logging of PID values to dataflash
 *       // log output if PID loggins is on and we are tuning the yaw
 *       if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_KP || g.radio_tuning == CH6_YAW_RATE_KP) ) {
 *               log_counter++;
 *               if( log_counter >= 10 ) {	// (update rate / desired output rate) = (100hz / 10hz) = 10
 *                       log_counter = 0;
 *                       Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
 *               }
 *       }
 * #endif
 *
 *       return output;
 *  }
 */

static int16_t
get_rate_roll(int32_t target_rate)
{
    static int32_t last_rate = 0;                                       // previous iterations rate
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t rate_d;                                                                     // roll's acceleration
    int32_t output;                                                                     // output from pid controller
    int32_t rate_d_dampener;                                                    // value to dampen output based on acceleration

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // calculate and filter the acceleration
    rate_d                  = roll_rate_d_filter.apply(current_rate - last_rate);

    // store rate for next iteration
    last_rate               = current_rate;

    // call pid controller
    rate_error      = target_rate - current_rate;
    p                       = g.pid_rate_roll.get_p(rate_error);
    i                       = g.pid_rate_roll.get_i(rate_error, G_Dt);
    d                       = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output          = p + i + d;

    // Dampening output with D term
    rate_d_dampener = rate_d * roll_scale_d;
    rate_d_dampener = constrain(rate_d_dampener, -400, 400);
    output -= rate_d_dampener;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    static int8_t log_counter = 0;                                      // used to slow down logging of PID values to dataflash
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        log_counter++;
        if( log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d-rate_d_dampener, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    static int32_t last_rate = 0;                                       // previous iterations rate
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t rate_d;                                                                     // roll's acceleration
    int32_t output;                                                                     // output from pid controller
    int32_t rate_d_dampener;                                                    // value to dampen output based on acceleration

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // calculate and filter the acceleration
    rate_d                  = pitch_rate_d_filter.apply(current_rate - last_rate);

    // store rate for next iteration
    last_rate               = current_rate;

    // call pid controller
    rate_error      = target_rate - current_rate;
    p                       = g.pid_rate_pitch.get_p(rate_error);
    i                       = g.pid_rate_pitch.get_i(rate_error, G_Dt);
    d                       = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output          = p + i + d;

    // Dampening output with D term
    rate_d_dampener = rate_d * pitch_scale_d;
    rate_d_dampener = constrain(rate_d_dampener, -400, 400);
    output -= rate_d_dampener;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    static int8_t log_counter = 0;                                      // used to slow down logging of PID values to dataflash
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        log_counter++;
        if( log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            log_counter = 0;
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, d-rate_d_dampener, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    static int8_t log_counter = 0;                                      // used to slow down logging of PID values to dataflash
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_KP || g.radio_tuning == CH6_YAW_RATE_KP) ) {
        log_counter++;
        if( log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

#if FRAME_CONFIG == HELI_FRAME || FRAME_CONFIG == TRI_FRAME
    // constrain output
    return output;
#else
    // output control:
    int16_t yaw_limit = 2200 + abs(g.rc_4.control_in);
    // smoother Yaw control:
    return constrain(output, -yaw_limit, yaw_limit);
#endif

}

static int16_t
get_throttle_rate(int16_t z_target_speed)
{
    int16_t z_rate_error, output;

    // calculate rate error
#if INERTIAL_NAV == ENABLED
    z_rate_error    = z_target_speed - accels_velocity.z;                       // calc the speed error
#else
    z_rate_error    = z_target_speed - climb_rate;              // calc the speed error
#endif

    int32_t tmp     = (z_target_speed * z_target_speed * (int32_t)g.throttle_cruise) / 200000;

    if(z_target_speed < 0) tmp = -tmp;

    output                  = constrain(tmp, -3200, 3200);

    // limit the rate
    output +=  constrain(g.pid_throttle.get_pid(z_rate_error, .02), -80, 120);

    return output;
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    nav_throttle                    = 0;

    // always start Circle mode at same angle
    circle_angle                    = 0;

    // We must be heading to a new WP, so XTrack must be 0
    crosstrack_error                = 0;

    // Will be set by new command
    target_bearing                  = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by new command, used by loiter
    long_error                              = 0;
    lat_error                               = 0;

    // We want to by default pass WPs
    slow_wp = false;

    // make sure we stick to Nav yaw on takeoff
    auto_yaw = nav_yaw;

    // revert to smaller radius set in params
    waypoint_radius = g.waypoint_radius;
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_stability_I();
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
    // this i is not currently being used, but we reset it anyway
    // because someone may modify it and not realize it, causing a bug
    g.pi_loiter_lat.reset_I();
    g.pi_loiter_lon.reset_I();

    g.pid_loiter_rate_lat.reset_I();
    g.pid_loiter_rate_lon.reset_I();

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
 *  throttle control
 ****************************************************************/

/* Depricated
 *
 *  static long
 *  //get_nav_yaw_offset(int yaw_input, int reset)
 *  {
 *       int32_t _yaw;
 *
 *       if(reset == 0){
 *               // we are on the ground
 *               return ahrs.yaw_sensor;
 *
 *       }else{
 *               // re-define nav_yaw if we have stick input
 *               if(yaw_input != 0){
 *                       // set nav_yaw + or - the current location
 *                       _yaw = yaw_input + ahrs.yaw_sensor;
 *                       // we need to wrap our value so we can be 0 to 360 (*100)
 *                       return wrap_360(_yaw);
 *               }else{
 *                       // no stick input, lets not change nav_yaw
 *                       return nav_yaw;
 *               }
 *       }
 *  }
 */

static int16_t get_angle_boost(int16_t value)
{
    float temp = cos_pitch_x * cos_roll_x;
    temp = constrain(temp, .75, 1.0);
    return ((float)(g.throttle_cruise + 80) / temp) - (g.throttle_cruise + 80);
}

#if FRAME_CONFIG == HELI_FRAME
// heli_angle_boost - adds a boost depending on roll/pitch values
// equivalent of quad's angle_boost function
// throttle value should be 0 ~ 1000
static int16_t heli_get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0 - constrain(angle_boost_factor, .5, 1.0);
    int16_t throttle_above_mid = max(throttle - motors.throttle_mid,0);
    return throttle + throttle_above_mid*angle_boost_factor;

}
#endif // HELI_FRAME

#define NUM_G_SAMPLES 40

#if ACCEL_ALT_HOLD == 2
// z -14.4306 = going up
// z -6.4306 = going down
static int16_t get_z_damping()
{
    int16_t output;

    Z_integrator    += get_world_Z_accel() - Z_offset;
    output                  = Z_integrator * 3;
    Z_integrator    = Z_integrator * .8;
    output = constrain(output, -100, 100);
    return output;
}

float get_world_Z_accel()
{
    accels_rot = ahrs.get_dcm_matrix() * imu.get_accel();
    //Serial.printf("z %1.4f\n", accels_rot.z);
    return accels_rot.z;
}

static void init_z_damper()
{
    Z_offset = 0;
    for (int16_t i = 0; i < NUM_G_SAMPLES; i++) {
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
    accels_rot = ahrs.get_dcm_matrix() * imu.get_accel();
    return accels_rot.z;
}

static void init_z_damper()
{
    estimatedAccelOffset = 0;
    for (int16_t i = 0; i < NUM_G_SAMPLES; i++) {
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

static int16_t get_z_damping()
{
    float sensedAccel = get_world_Z_accel();
    float sensedPos = current_loc.alt / 100.0;

    float synVelo = dead_reckon_Z(sensedPos, sensedAccel);
    return constrain(fullDampP * synVelo * (-1), -300, 300);
}

#else

static int16_t get_z_damping()
{
    return 0;
}

static void init_z_damper()
{
}
#endif

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#ifdef OPTFLOW_ENABLED
    static float tot_x_cm = 0;      // total distance from target
    static uint32_t last_of_roll_update = 0;
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_roll_update) {
        last_of_roll_update = optflow.last_update;

        // add new distance moved
        tot_x_cm += optflow.x_cm;

        // only stop roll if caller isn't modifying roll
        if( input_roll == 0 && current_loc.alt < 1500) {
            p = g.pid_optflow_roll.get_p(-tot_x_cm);
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain(new_roll, (of_roll-20), (of_roll+20));

 #if LOGGING_ENABLED == ENABLED
        static int8_t log_counter = 0;                                  // used to slow down logging of PID values to dataflash
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            log_counter++;
            if( log_counter >= 5 ) {                    // (update rate / desired output rate) = (100hz / 10hz) = 10
                log_counter = 0;
                Log_Write_PID(CH6_OPTFLOW_KP, tot_x_cm, p, i, d, of_roll, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_roll = constrain(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#ifdef OPTFLOW_ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static uint32_t last_of_pitch_update = 0;
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = optflow.last_update;

        // add new distance moved
        tot_y_cm += optflow.y_cm;

        // only stop roll if caller isn't modifying pitch
        if( input_pitch == 0 && current_loc.alt < 1500 ) {
            p = g.pid_optflow_pitch.get_p(tot_y_cm);
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain(new_pitch, (of_pitch-20), (of_pitch+20));

 #if LOGGING_ENABLED == ENABLED
        static int8_t log_counter = 0;                                  // used to slow down logging of PID values to dataflash
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            log_counter++;
            if( log_counter >= 5 ) {                    // (update rate / desired output rate) = (100hz / 10hz) = 10
                log_counter = 0;
                Log_Write_PID(CH6_OPTFLOW_KP+100, tot_y_cm, p, i, d, of_pitch, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_pitch = constrain(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}
