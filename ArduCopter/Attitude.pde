/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
static void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
    static float _scaler = 1.0;
    static int16_t _angle_max = 0;

    // range check the input
    roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

    // return immediately if no scaling required
    if (g.angle_max == ROLL_PITCH_INPUT_MAX) {
        roll_out = roll_in;
        pitch_out = pitch_in;
        return;
    }

    // check if angle_max has been updated and redo scaler
    if (g.angle_max != _angle_max) {
        _angle_max = g.angle_max;
        _scaler = (float)g.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    }

    // convert pilot input to lean angle
    roll_out = roll_in * _scaler;
    pitch_out = pitch_in * _scaler;
}

static void
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle = wrap_180_cd(target_angle - ahrs.roll_sensor);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_roll.kP() * target_angle;

    // constrain the target rate
    if (!ap.disable_stab_rate_limit) {
        target_rate = constrain_int32(target_rate, -g.angle_rate_max, g.angle_rate_max);
    }

    // set targets for rate controller
    set_roll_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - ahrs.pitch_sensor);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_pitch.kP() * target_angle;

    // constrain the target rate
    if (!ap.disable_stab_rate_limit) {
        target_rate = constrain_int32(target_rate, -g.angle_rate_max, g.angle_rate_max);
    }

    // set targets for rate controller
    set_pitch_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate;
    int32_t angle_error;

    // angle error
    angle_error = wrap_180_cd(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error = constrain_int32(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = g.pi_stabilize_yaw.kP() * angle_error;

    // do not use rate controllers for helicotpers with external gyros
#if FRAME_CONFIG == HELI_FRAME
    if(motors.tail_type() == AP_MOTORS_HELI_TAILTYPE_SERVO_EXTGYRO) {
        g.rc_4.servo_out = constrain_int32(target_rate, -4500, 4500);
    }
#endif

    // set targets for rate controller
    set_yaw_rate_target(target_rate, EARTH_FRAME);
}

// get_acro_level_rates - calculate earth frame rate corrections to pull the copter back to level while in ACRO mode
static void
get_acro_level_rates()
{
    // zero earth frame leveling if trainer is disabled
    if (g.acro_trainer == ACRO_TRAINER_DISABLED) {
        set_roll_rate_target(0, BODY_EARTH_FRAME);
        set_pitch_rate_target(0, BODY_EARTH_FRAME);
        set_yaw_rate_target(0, BODY_EARTH_FRAME);
        return;
    }

    // Calculate trainer mode earth frame rate command for roll
    int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
    int32_t target_rate = 0;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        if (roll_angle > g.angle_max){
            target_rate =  g.pi_stabilize_roll.get_p(g.angle_max-roll_angle);
        }else if (roll_angle < -g.angle_max) {
            target_rate =  g.pi_stabilize_roll.get_p(-g.angle_max-roll_angle);
        }
    }
    roll_angle   = constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE);
    target_rate -= roll_angle * g.acro_balance_roll;

    // add earth frame targets for roll rate controller
    set_roll_rate_target(target_rate, BODY_EARTH_FRAME);

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
    target_rate = 0;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        if (pitch_angle > g.angle_max){
            target_rate =  g.pi_stabilize_pitch.get_p(g.angle_max-pitch_angle);
        }else if (pitch_angle < -g.angle_max) {
            target_rate =  g.pi_stabilize_pitch.get_p(-g.angle_max-pitch_angle);
        }
    }
    pitch_angle  = constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE);
    target_rate -= pitch_angle * g.acro_balance_pitch;

    // add earth frame targets for pitch rate controller
    set_pitch_rate_target(target_rate, BODY_EARTH_FRAME);

    // add earth frame targets for yaw rate controller
    set_yaw_rate_target(0, BODY_EARTH_FRAME);
}

// Roll with rate input and stabilized in the body frame
static void
get_roll_rate_stabilized_bf(int32_t stick_angle)
{
    static float angle_error = 0;

    // convert the input to the desired body frame roll rate
    int32_t rate_request = stick_angle * g.acro_rp_p;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_request += acro_roll_rate;
    }else{        
        // Scale pitch leveling by stick input
        acro_roll_rate = (float)acro_roll_rate*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_request)-labs(acro_roll_rate));

        rate_request += acro_roll_rate;
        rate_request = constrain_int32(rate_request, -rate_limit, rate_limit);
    }

    // add automatic correction
    int32_t rate_correction = g.pi_stabilize_roll.get_p(angle_error);

    // set body frame targets for rate controller
    set_roll_rate_target(rate_request+rate_correction, BODY_FRAME);

    // Calculate integrated body frame rate error
    angle_error += (rate_request - (omega.x * DEGX100)) * G_Dt;

    // don't let angle error grow too large
    angle_error = constrain_float(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
           angle_error = 0;
    }
#else
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME
}

// Pitch with rate input and stabilized in the body frame
static void
get_pitch_rate_stabilized_bf(int32_t stick_angle)
{
    static float angle_error = 0;

    // convert the input to the desired body frame pitch rate
    int32_t rate_request = stick_angle * g.acro_rp_p;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_request += acro_pitch_rate;
    }else{
        // Scale pitch leveling by stick input
        acro_pitch_rate = (float)acro_pitch_rate*acro_level_mix;
        
        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_request)-labs(acro_pitch_rate));
        
        rate_request += acro_pitch_rate;
        rate_request = constrain_int32(rate_request, -rate_limit, rate_limit);
    }

    // add automatic correction
    int32_t rate_correction = g.pi_stabilize_pitch.get_p(angle_error);

    // set body frame targets for rate controller
    set_pitch_rate_target(rate_request+rate_correction, BODY_FRAME);

    // Calculate integrated body frame rate error
    angle_error += (rate_request - (omega.y * DEGX100)) * G_Dt;

    // don't let angle error grow too large
    angle_error = constrain_float(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
           angle_error = 0;
    }
#else
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME
}

// Yaw with rate input and stabilized in the body frame
static void
get_yaw_rate_stabilized_bf(int32_t stick_angle)
{
    static float angle_error = 0;

    // convert the input to the desired body frame yaw rate
    int32_t rate_request = stick_angle * g.acro_yaw_p;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_request += acro_yaw_rate;
    }else{
        // Scale pitch leveling by stick input
        acro_yaw_rate = (float)acro_yaw_rate*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_request)-labs(acro_yaw_rate));

        rate_request += acro_yaw_rate;
        rate_request = constrain_int32(rate_request, -rate_limit, rate_limit);
    }

    // add automatic correction
    int32_t rate_correction = g.pi_stabilize_yaw.get_p(angle_error);

    // set body frame targets for rate controller
    set_yaw_rate_target(rate_request+rate_correction, BODY_FRAME);

    // Calculate integrated body frame rate error
    angle_error += (rate_request - (omega.z * DEGX100)) * G_Dt;

    // don't let angle error grow too large
    angle_error = constrain_float(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
           angle_error = 0;
    }
#else
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME
}

// Roll with rate input and stabilized in the earth frame
static void
get_roll_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired roll rate
    int32_t target_rate = stick_angle * g.acro_rp_p - (acro_roll * g.acro_balance_roll);

    // convert the input to the desired roll rate
    acro_roll += target_rate * G_Dt;
    acro_roll = wrap_180_cd(acro_roll);

    // ensure that we don't reach gimbal lock
    if (labs(acro_roll) > g.angle_max) {
        acro_roll  = constrain_int32(acro_roll, -g.angle_max, g.angle_max);
        angle_error = wrap_180_cd(acro_roll - ahrs.roll_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180_cd(acro_roll - ahrs.roll_sensor);
        angle_error  = constrain_int32(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);
    }

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
        angle_error = 0;
    }
#else      
    // reset target angle to current angle if motors not spinning
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME

    // update acro_roll to be within max_angle_overshoot of our current heading
    acro_roll = wrap_180_cd(angle_error + ahrs.roll_sensor);

    // set earth frame targets for rate controller
  set_roll_rate_target(g.pi_stabilize_roll.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Pitch with rate input and stabilized in the earth frame
static void
get_pitch_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired pitch rate
    int32_t target_rate = stick_angle * g.acro_rp_p - (acro_pitch * g.acro_balance_pitch);

    // convert the input to the desired pitch rate
    acro_pitch += target_rate * G_Dt;
    acro_pitch = wrap_180_cd(acro_pitch);

    // ensure that we don't reach gimbal lock
    if (labs(acro_pitch) > g.angle_max) {
        acro_pitch  = constrain_int32(acro_pitch, -g.angle_max, g.angle_max);
        angle_error = wrap_180_cd(acro_pitch - ahrs.pitch_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180_cd(acro_pitch - ahrs.pitch_sensor);
        angle_error  = constrain_int32(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);
    }

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
        angle_error = 0;
    }
#else       
    // reset target angle to current angle if motors not spinning
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME

    // update acro_pitch to be within max_angle_overshoot of our current heading
    acro_pitch = wrap_180_cd(angle_error + ahrs.pitch_sensor);

    // set earth frame targets for rate controller
    set_pitch_rate_target(g.pi_stabilize_pitch.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Yaw with rate input and stabilized in the earth frame
static void
get_yaw_rate_stabilized_ef(int32_t stick_angle)
{

    int32_t angle_error = 0;

    // convert the input to the desired yaw rate
    int32_t target_rate = stick_angle * g.acro_yaw_p;

    // convert the input to the desired yaw rate
    control_yaw += target_rate * G_Dt;
    control_yaw = wrap_360_cd(control_yaw);

    // calculate difference between desired heading and current heading
    angle_error = wrap_180_cd(control_yaw - ahrs.yaw_sensor);

    // limit the maximum overshoot
    angle_error	= constrain_int32(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
    	angle_error = 0;
    }
#else   
    // reset target angle to current heading if motors not spinning
    if (!motors.armed() || g.rc_3.servo_out == 0) {
    	angle_error = 0;
    }
#endif // HELI_FRAME

    // update control_yaw to be within max_angle_overshoot of our current heading
    control_yaw = wrap_360_cd(angle_error + ahrs.yaw_sensor);

    // set earth frame targets for rate controller
	set_yaw_rate_target(g.pi_stabilize_yaw.get_p(angle_error)+target_rate, EARTH_FRAME);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        yaw_rate_target_bf = desired_rate;
    }else{
        yaw_rate_target_ef = desired_rate;
    }
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf     = roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        pitch_rate_target_bf    = cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf      = cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }else if( rate_targets_frame == BODY_EARTH_FRAME ) {
        // add converted earth frame rates to body frame rates
        acro_roll_rate = roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        acro_pitch_rate = cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        acro_yaw_rate = cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
#if FRAME_CONFIG == HELI_FRAME
    // convert desired roll and pitch rate to roll and pitch swash angles
    heli_integrated_swash_controller(roll_rate_target_bf, pitch_rate_target_bf);
    // helicopters only use rate controllers for yaw and only when not using an external gyro
    if(motors.tail_type() != AP_MOTORS_HELI_TAILTYPE_SERVO_EXTGYRO) {
        g.rc_4.servo_out = get_heli_rate_yaw(yaw_rate_target_bf);
    }else{
        // do not use rate controllers for helicotpers with external gyros
        g.rc_4.servo_out = constrain_int32(yaw_rate_target_bf, -4500, 4500);
    }
#else
    // call rate controllers
    g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);
#endif

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

#if FRAME_CONFIG != HELI_FRAME
static int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // get i term
    i = g.pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = g.pid_rate_roll.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);

    // get i term
    i = g.pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

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

    // get i term
    i = g.pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    }

    // get d value
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain_int32(output, -4500, 4500);

    // constrain output
    return output;
}
#endif // !HELI_FRAME

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#if OPTFLOW == ENABLED
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
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0f);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain_int32(new_roll, (of_roll-20), (of_roll+20));
    }

    // limit max angle
    of_roll = constrain_int32(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#if OPTFLOW == ENABLED
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
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0f);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain_int32(new_pitch, (of_pitch-20), (of_pitch+20));
    }

    // limit max angle
    of_pitch = constrain_int32(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}

/*************************************************************
 * yaw controllers
 *************************************************************/

 // get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
static void get_circle_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    // if circle radius is zero do panorama
    if( g.circle_radius == 0 ) {
        // slew yaw towards circle angle
        control_yaw = get_yaw_slew(control_yaw, ToDeg(circle_angle)*100, AUTO_YAW_SLEW_RATE);
    }else{
        look_at_yaw_counter++;
        if( look_at_yaw_counter >= 10 ) {
            look_at_yaw_counter = 0;
            yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
        }
        // slew yaw
        control_yaw = get_yaw_slew(control_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    }

    // call stabilize yaw controller
    get_stabilize_yaw(control_yaw);
}

// get_look_at_yaw - updates bearing to location held in look_at_yaw_WP and calls stabilize yaw controller
// should be called at 100hz
static void get_look_at_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    look_at_yaw_counter++;
    if( look_at_yaw_counter >= 10 ) {
        look_at_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
    }

    // slew yaw and call stabilize controller
    control_yaw = get_yaw_slew(control_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    get_stabilize_yaw(control_yaw);
}

static void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps->fix && g_gps->ground_speed_cm > YAW_LOOK_AHEAD_MIN_SPEED) {
        control_yaw = get_yaw_slew(control_yaw, g_gps->ground_course_cd, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360_cd(control_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        control_yaw += pilot_yaw * g.acro_yaw_p * G_Dt;
        control_yaw = wrap_360_cd(control_yaw);
        get_stabilize_yaw(control_yaw);
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
static void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        g.throttle_cruise = throttle_avg;
    }
}

#if FRAME_CONFIG == HELI_FRAME
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
// for traditional helicopters
static int16_t get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0f - constrain_float(angle_boost_factor, .5f, 1.0f);
    int16_t throttle_above_mid = max(throttle - motors.get_collective_mid(),0);

    // to allow logging of angle boost
    angle_boost = throttle_above_mid*angle_boost_factor;

    return throttle + angle_boost;
}
#else   // all multicopters
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
static int16_t get_angle_boost(int16_t throttle)
{
    float temp = cos_pitch_x * cos_roll_x;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-max(labs(ahrs.roll_sensor),labs(ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    throttle_out = constrain_float((float)(throttle-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}
#endif // FRAME_CONFIG == HELI_FRAME

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        g.rc_3.servo_out = get_angle_boost(throttle_out);
    }else{
        g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }

    // update compass with throttle value
    compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    throttle_accel_target_ef = desired_acceleration;
    throttle_accel_controller_active = true;
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    throttle_accel_controller_active = false;
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
static void
set_throttle_takeoff()
{
    // set alt target
    controller_desired_alt = current_loc.alt + ALT_HOLD_TAKEOFF_JUMP;

    // clear i term from acceleration controller
    if (g.pid_throttle_accel.get_integrator() < 0) {
        g.pid_throttle_accel.reset_I();
    }
    // tell motors to do a slow start
    motors.slow_start(true);
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
static int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164f * (constrain_float(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle_accel.get_p(z_accel_error);

    // get i term
    i = g.pid_throttle_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!motors.limit.throttle_lower && !motors.limit.throttle_upper) || (i>0&&z_accel_error<0) || (i<0&&z_accel_error>0)) {
        i = g.pid_throttle_accel.get_i(z_accel_error, .01f);
    }

    d = g.pid_throttle_accel.get_d(z_accel_error, .01f);

    output =  constrain_float(p+i+d+g.throttle_cruise, g.throttle_min, g.throttle_max);

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THROTTLE_IN_DEADBAND)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THROTTLE_IN_DEADBAND)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( failsafe.radio ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
static int32_t
get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)
{
    int32_t target_alt;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    int32_t linear_velocity;      // the velocity we swap between linear and sqrt.

    linear_velocity = ALT_HOLD_ACCEL_MAX/g.pi_alt_hold.kP();

    if (abs(climb_rate_cms) < linear_velocity) {
        target_alt = alt_cm + climb_rate_cms/g.pi_alt_hold.kP();
    } else {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if (climb_rate_cms > 0){
            target_alt = alt_cm + linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX);
        } else {
            target_alt = alt_cm - ( linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX) );
        }
    }
    return constrain_int32(target_alt, alt_cm - ALT_HOLD_INIT_MAX_OVERSHOOT, alt_cm + ALT_HOLD_INIT_MAX_OVERSHOOT);
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
static void
get_throttle_rate(float z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    static float z_target_speed_filt = 0;   // The filtered requested speed
    float z_target_speed_delta;   // The change in requested speed
    int32_t p;          // used to capture pid values for logging
    int32_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error = 0;
        z_target_speed_filt = z_target_speed;
        output = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085f * ((z_target_speed - climb_rate) - z_rate_error);
        // feed forward acceleration based on change in the filtered desired speed.
        z_target_speed_delta = 0.20085f * (z_target_speed - z_target_speed_filt);
        z_target_speed_filt    = z_target_speed_filt + z_target_speed_delta;
        output = z_target_speed_delta * 50.0f;   // To-Do: replace 50 with dt
    }
    last_call_ms = now;

    // calculate p
    p = g.pid_throttle_rate.kP() * z_rate_error;

    // consolidate and constrain target acceleration
    output += p;
    output = constrain_int32(output, -32000, 32000);

    // set target for accel based throttle controller
    set_throttle_accel_target(output);

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
static void
get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    float desired_rate;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.

    // calculate altitude error
    alt_error    = target_alt - current_loc.alt;

    // check kP to avoid division by zero
    if( g.pi_alt_hold.kP() != 0 ) {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(-alt_error-linear_distance));
        }else{
            desired_rate = g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain_float(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
    altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    float alt_change = target_alt-controller_desired_alt;
    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !motors.limit.throttle_lower) || (alt_change>0 && !motors.limit.throttle_upper)) {
        controller_desired_alt += constrain_float(alt_change, min_climb_rate*0.02f, max_climb_rate*0.02f);
    }

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    // adjust desired alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        controller_desired_alt += target_rate * 0.02f;
    }

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

#if AC_FENCE == ENABLED
    // do not let target altitude be too close to the fence
    // To-Do: add this to other altitude controllers
    if((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        float alt_limit = fence.get_safe_alt() * 100.0f;
        if (controller_desired_alt > alt_limit) {
            controller_desired_alt = alt_limit;
        }
    }
#endif

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
static void
get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        get_throttle_althold_with_slew(LAND_START_ALT, -wp_nav.get_descent_velocity(), -abs(g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(g.land_speed));

        // disarm when the landing detector says we've landed and throttle is at min (or we're in failsafe so we have no pilot thorottle input)
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        if( ap.land_complete && (g.rc_3.control_in == 0 || failsafe.radio) ) {
#else
        if (ap.land_complete) {
#endif
            init_disarm_motors();
        }
    }
}

// reset_land_detector - initialises land detector
static void reset_land_detector()
{
    set_land_complete(false);
    land_detector = 0;
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
static bool update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(climb_rate) < 20 && motors.limit.throttle_lower) {
        if (!ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = 0;
            }
        }
    }else if (g.rc_3.control_in != 0 || failsafe.radio){    // zero throttle locks land_complete as true
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        if(ap.land_complete) {
            set_land_complete(false);
        }
    }

    // return current state of landing
    return ap.land_complete;
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
static void
get_throttle_surface_tracking(int16_t target_rate)
{
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 200 ) {
        target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
    }
    last_call_ms = now;

    // adjust sonar target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_sonar_alt += target_rate * 0.02f;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain_float(target_sonar_alt,sonar_alt-750,sonar_alt+750);

    // calc desired velocity correction from target sonar alt vs actual sonar alt
    distance_error = target_sonar_alt-sonar_alt;
    velocity_correction = distance_error * g.sonar_gain;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // call regular rate stabilize alt hold controller
    get_throttle_rate_stabilized(target_rate + velocity_correction);
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_throttle_I();
    reset_optflow_I();
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

static void reset_throttle_I(void)
{
    // For Altitude Hold
    g.pi_alt_hold.reset_I();
    g.pid_throttle_accel.reset_I();
}

static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}
