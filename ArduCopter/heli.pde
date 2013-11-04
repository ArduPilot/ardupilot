/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      100     // we are in "dynamic flight" when the speed is over 1m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// Tradheli flags
static struct {
    uint8_t dynamic_flight  : 1;    // 0   // true if we are moving at a significant speed (used to turn on/off leaky I terms)
} heli_flags;

// heli_check_dynamic_flight - updates the dynamic_flight flag based on our horizontal velocity
// should be called at 50hz
static void check_dynamic_flight(void)
{
    if (!motors.armed() || throttle_mode == THROTTLE_LAND || !motors.motor_runup_complete()) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // with GPS lock use inertial nav to determine if we are moving
    if (GPS_ok()) {
        // get horizontal velocity
        float velocity = inertial_nav.get_velocity_xy();
        moving = (velocity >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    }else{
        // with no GPS lock base it on throttle and forward lean angle
        moving = (g.rc_3.servo_out > 800 || ahrs.pitch_sensor < -1500);
    }

    if (moving) {
        // if moving for 2 seconds, set the dynamic flight flag
        if (!heli_flags.dynamic_flight) {
            heli_dynamic_flight_counter++;
            if (heli_dynamic_flight_counter >= 100) {
                heli_flags.dynamic_flight = true;
                heli_dynamic_flight_counter = 100;
            }
        }
    }else{
        // if not moving for 2 seconds, clear the dynamic flight flag
        if (heli_flags.dynamic_flight) {
            if (heli_dynamic_flight_counter > 0) {
                heli_dynamic_flight_counter--;
            }else{
                heli_flags.dynamic_flight = false;
            }
        }
    }
}

// init_rate_controllers - set-up filters for rate controller inputs
void init_rate_controllers()
{
   // initalise low pass filters on rate controller inputs
   // 1st parameter is time_step, 2nd parameter is time_constant
   // rate_roll_filter.set_cutoff_frequency(0.01f, 0.1f);
   // rate_pitch_filter.set_cutoff_frequency(0.01f, 0.1f);
}

static void heli_integrated_swash_controller(int32_t target_roll_rate, int32_t target_pitch_rate)
{
    int32_t         roll_p, roll_i, roll_d, roll_ff;            // used to capture pid values for logging
    int32_t         pitch_p, pitch_i, pitch_d, pitch_ff;
	int32_t         current_roll_rate, current_pitch_rate;	    // this iteration's rate
    int32_t         roll_rate_error, pitch_rate_error;          // simply target_rate - current_rate
    int32_t         roll_output, pitch_output;                  // output from pid controller
    static bool     roll_pid_saturated, pitch_pid_saturated;    // tracker from last loop if the PID was saturated
    
    current_roll_rate = (omega.x * DEGX100);                    // get current roll rate
    current_pitch_rate = (omega.y * DEGX100);                   // get current pitch rate
	
    roll_rate_error = target_roll_rate - current_roll_rate;
    pitch_rate_error = target_pitch_rate - current_pitch_rate;
    
    roll_p = g.pid_rate_roll.get_p(roll_rate_error);
    pitch_p = g.pid_rate_pitch.get_p(pitch_rate_error);

    if (roll_pid_saturated){
        roll_i = g.pid_rate_roll.get_integrator();                                                      // Locked Integrator due to PID saturation on previous cycle
    } else {
        if (motors.has_flybar()) {                                                                      // Mechanical Flybars get regular integral for rate auto trim
            if (target_roll_rate > -50 && target_roll_rate < 50){								        // Frozen at high rates
                roll_i = g.pid_rate_roll.get_i(roll_rate_error, G_Dt);
            } else {
                roll_i = g.pid_rate_roll.get_integrator();
            }
        } else {
            if (heli_flags.dynamic_flight){
                roll_i = g.pid_rate_roll.get_i(roll_rate_error, G_Dt);
            } else {
                roll_i = g.pid_rate_roll.get_leaky_i(roll_rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
            }
        }
    }
    
    if (pitch_pid_saturated){
        pitch_i = g.pid_rate_pitch.get_integrator();                                                    // Locked Integrator due to PID saturation on previous cycle
    } else {
        if (motors.has_flybar()) {                                                                      // Mechanical Flybars get regular integral for rate auto trim
            if (target_pitch_rate > -50 && target_pitch_rate < 50){								        // Frozen at high rates
                pitch_i = g.pid_rate_pitch.get_i(pitch_rate_error, G_Dt);
            } else {
                pitch_i = g.pid_rate_pitch.get_integrator();
            }
        } else {
            if (heli_flags.dynamic_flight){
                pitch_i = g.pid_rate_pitch.get_i(pitch_rate_error, G_Dt);
            } else {
                pitch_i = g.pid_rate_pitch.get_leaky_i(pitch_rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
            }
        }
    }
	
	roll_d = g.pid_rate_roll.get_d(target_roll_rate, G_Dt);
	pitch_d = g.pid_rate_pitch.get_d(target_pitch_rate, G_Dt);
    
	roll_ff = g.heli_roll_ff * target_roll_rate;
    pitch_ff = g.heli_pitch_ff * target_pitch_rate;

    // Joly, I think your PC and CC code goes here
    
    roll_output = roll_p + roll_i + roll_d + roll_ff;
    pitch_output = pitch_p + pitch_i + pitch_d + pitch_ff;

    if (labs(roll_output) > 4500){
        roll_output = constrain_int32(roll_output, -4500, 4500);         // constrain output
        roll_pid_saturated = true;                                       // freeze integrator next cycle
    } else {
        roll_pid_saturated = false;                                      // unfreeze integrator
    }
    
    if (labs(pitch_output) > 4500){
        pitch_output = constrain_int32(pitch_output, -4500, 4500);        // constrain output
        pitch_pid_saturated = true;                                       // freeze integrator next cycle
    } else {
        pitch_pid_saturated = false;                                      // unfreeze integrator
    }

    g.rc_1.servo_out = roll_output;
	g.rc_2.servo_out = pitch_output;
}

static int16_t
get_heli_rate_yaw(int32_t target_rate)
{
    int32_t         p,i,d,ff;               // used to capture pid values for logging
	int32_t         current_rate;           // this iteration's rate
    int32_t         rate_error;
    int32_t         output;
    static bool     pid_saturated;          // tracker from last loop if the PID was saturated

    current_rate = (omega.z * DEGX100);                         // get current rate
	
    // rate control
    rate_error = target_rate - current_rate;

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    
    if (pid_saturated){
        i = g.pid_rate_yaw.get_integrator();                    // Locked Integrator due to PID saturation on previous cycle
    } else {
        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);
	
	ff = g.heli_yaw_ff*target_rate;

    output = p + i + d + ff;
    
    if (labs(output) > 4500){
        output = constrain_int32(output, -4500, 4500);          // constrain output
        pid_saturated = true;                                   // freeze integrator next cycle
    } else {
        pid_saturated = false;                                  // unfreeze integrator
    }

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_RATE_KP || g.radio_tuning == CH6_YAW_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

	return output;                                              // output control
}

#endif  // FRAME_CONFIG == TRAD_HELI