/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      500     // we are in "dynamic flight" when the speed is over 1m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// Tradheli flags
static struct {
    uint8_t dynamic_flight  : 1;    // 0   // true if we are moving at a significant speed (used to turn on/off leaky I terms)
} heli_flags;

#if HELI_CC_COMP == ENABLED
static LowPassFilterFloat rate_dynamics_filter;     // Rate Dynamics filter
#endif

// heli_init - perform any special initialisation required for the tradheli
static void heli_init()
{
#if HELI_CC_COMP == ENABLED
    rate_dynamics_filter.set_cutoff_frequency(0.01f, 4.0f);
#endif
}

// get_pilot_desired_collective - converts pilot input (from 0 ~ 1000) to a value that can be fed into the g.rc_3.servo_out function
static int16_t get_pilot_desired_collective(int16_t control_in)
{
    // return immediately if reduce collective range for manual flight has not been configured
    if (g.heli_stab_col_min == 0 && g.heli_stab_col_max == 1000) {
        return control_in;
    }

    // scale pilot input to reduced collective range
    float scalar = ((float)(g.heli_stab_col_max - g.heli_stab_col_min))/1000.0f;
    int16_t collective_out = g.heli_stab_col_min + control_in * scalar;
    collective_out = constrain_int16(collective_out, 0, 1000);
    return collective_out;
}

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

// heli_integrated_swash_controller - convert desired roll and pitch rate to roll and pitch swash angles
// should be called at 100hz
// output placed directly into g.rc_1.servo_out and g.rc_2.servo_out
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
    
    roll_output = roll_p + roll_i + roll_d + roll_ff;
    pitch_output = pitch_p + pitch_i + pitch_d + pitch_ff;

#if HELI_CC_COMP == ENABLED
// Do cross-coupling compensation for low rpm helis
// Credit: Jolyon Saunders
// Note: This is not widely tested at this time.  Will not be used by default yet.
    float cc_axis_ratio = 2.0f; // Ratio of compensation on pitch vs roll axes. Number >1 means pitch is affected more than roll
    float cc_kp = 0.0002f;      // Compensation p term. Setting this to zero gives h_phang only, while increasing it will increase the p term of correction
    float cc_kd = 0.127f;       // Compensation d term, scaled. This accounts for flexing of the blades, dampers etc. Originally was (motors.ext_gyro_gain * 0.0001)
    float cc_angle, cc_total_output;
    uint32_t cc_roll_d, cc_pitch_d, cc_sum_d;
    int32_t cc_scaled_roll;
    int32_t cc_roll_output;     // Used to temporarily hold output while rotation is being calculated
    int32_t cc_pitch_output;    // Used to temporarily hold output while rotation is being calculated
    static int32_t last_roll_output = 0;
    static int32_t last_pitch_output = 0;

    cc_scaled_roll  = roll_output / cc_axis_ratio; // apply axis ratio to roll
    cc_total_output = safe_sqrt(cc_scaled_roll * cc_scaled_roll + pitch_output * pitch_output) * cc_kp;
    
    // find the delta component
    cc_roll_d  = (roll_output - last_roll_output) / cc_axis_ratio;
    cc_pitch_d = pitch_output - last_pitch_output;
    cc_sum_d = safe_sqrt(cc_roll_d * cc_roll_d + cc_pitch_d * cc_pitch_d);

    // do the magic.
    cc_angle = cc_kd * cc_sum_d * cc_total_output - cc_total_output * motors.get_phase_angle();

    // smooth angle variations, apply constraints
    cc_angle = rate_dynamics_filter.apply(cc_angle);
    cc_angle = constrain_float(cc_angle, -90.0f, 0.0f);
    cc_angle = radians(cc_angle);

    // Make swash rate vector
    Vector2f swashratevector;
    swashratevector.x = cosf(cc_angle);
    swashratevector.y = sinf(cc_angle);
    swashratevector.normalize();

    // rotate the output
    cc_roll_output  = roll_output;
    cc_pitch_output = pitch_output;
    roll_output     = - (cc_pitch_output * swashratevector.y - cc_roll_output * swashratevector.x);
    pitch_output    =    cc_pitch_output * swashratevector.x + cc_roll_output * swashratevector.y;

    // make current outputs old, for next iteration
    last_roll_output  = cc_roll_output;
    last_pitch_output = cc_pitch_output;
# endif // HELI_CC_COMP   
    
#if HELI_PIRO_COMP == ENABLED
    if (control_mode <= ACRO){
    
        int32_t         piro_roll_i, piro_pitch_i;            // used to hold i term while doing prio comp
    
        piro_roll_i  = roll_i;
        piro_pitch_i = pitch_i;

        Vector2f yawratevector;
        yawratevector.x     = cos(-omega.z/100);
        yawratevector.y     = sin(-omega.z/100);
        yawratevector.normalize();
            
        roll_i      = piro_roll_i * yawratevector.x - piro_pitch_i * yawratevector.y;
        pitch_i     = piro_pitch_i * yawratevector.x + piro_roll_i * yawratevector.y;

        g.pid_rate_pitch.set_integrator(pitch_i);
        g.pid_rate_roll.set_integrator(roll_i); 
    }
#endif //HELI_PIRO_COMP   

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
        if (motors.motor_runup_complete()){
            i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
        } else {
            i = g.pid_rate_yaw.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);	// If motor is not running use leaky I-term to avoid excessive build-up
        }
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

// heli_update_landing_swash - sets swash plate flag so higher minimum is used when landed or landing
// should be called soon after update_land_detector in main code
static void heli_update_landing_swash()
{
    switch(throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
        case THROTTLE_MANUAL_HELI:
            // manual modes always uses full swash range
            motors.set_collective_for_landing(false);
            break;

        case THROTTLE_LAND:
            // landing always uses limit swash range
            motors.set_collective_for_landing(true);
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
        default:
            // auto and hold use limited swash when landed
            motors.set_collective_for_landing(ap.land_complete || !ap.auto_armed);
            break;
    }
}

// heli_update_rotor_speed_targets - reads pilot input and passes new rotor speed targets to heli motors object
static void heli_update_rotor_speed_targets()
{
    // get rotor control method
    uint8_t rsc_control_mode = motors.get_rsc_mode();

    switch (rsc_control_mode) {
        case AP_MOTORS_HELI_RSC_MODE_NONE:
            // even though pilot passes rotors speed directly to rotor ESC via receiver, motor lib needs to know if
            // rotor is spinning in case we are using direct drive tailrotor which must be spun up at same time
        case AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH:
            // pass through pilot desired rotor speed
            motors.set_desired_rotor_speed(g.rc_8.control_in);
            break;
        case AP_MOTORS_HELI_RSC_MODE_SETPOINT:
            // pass setpoint through as desired rotor speed
            if (g.rc_8.control_in > 0) {
                motors.set_desired_rotor_speed(motors.get_rsc_setpoint());
            }else{
                motors.set_desired_rotor_speed(0);
            }
            break;
    }
}

#endif  // FRAME_CONFIG == HELI_FRAME