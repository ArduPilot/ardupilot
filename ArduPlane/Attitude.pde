// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************


/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain_float(speed_scaler, 0.5, 2.0);
    } else {
        if (channel_throttle->servo_out > 0) {
            speed_scaler = 0.5 + ((float)THROTTLE_CRUISE / channel_throttle->servo_out / 2.0);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6, 1.67);
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
static bool stick_mixing_enabled(void)
{
    if (auto_throttle_mode) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing != STICK_MIXING_DISABLED &&
            geofence_stickmixing() &&
            failsafe.state == FAILSAFE_NONE &&
            !throttle_failsafe_level()) {
            // we're in an auto mode, and haven't triggered failsafe
            return true;
        } else {
            return false;
        }
    }

    if (failsafe.ch3_failsafe && g.short_fs_action == 2) {
        // don't do stick mixing in FBWA glide mode
        return false;
    }

    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
static void stabilize_roll(float speed_scaler)
{
    if (inverted_flight) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    bool disable_integrator = false;
    if (control_mode == STABILIZE && channel_roll->control_in != 0) {
        disable_integrator = true;
    }
    channel_roll->servo_out = rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, 
                                                           speed_scaler, 
                                                           disable_integrator);
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
static void stabilize_pitch(float speed_scaler)
{
    int32_t demanded_pitch = nav_pitch_cd + g.pitch_trim_cd + channel_throttle->servo_out * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (control_mode == STABILIZE && channel_pitch->control_in != 0) {
        disable_integrator = true;
    }
    channel_pitch->servo_out = pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, 
                                                             speed_scaler, 
                                                             disable_integrator);
}

/*
  perform stick mixing on one channel
  This type of stick mixing reduces the influence of the auto
  controller as it increases the influence of the users stick input,
  allowing the user full deflection if needed
 */
static void stick_mix_channel(RC_Channel *channel)
{
    float ch_inf;
        
    ch_inf = (float)channel->radio_in - (float)channel->radio_trim;
    ch_inf = fabsf(ch_inf);
    ch_inf = min(ch_inf, 400.0);
    ch_inf = ((400.0 - ch_inf) / 400.0);
    channel->servo_out *= ch_inf;
    channel->servo_out += channel->pwm_to_angle();
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
static void stabilize_stick_mixing_direct()
{
    if (!stick_mixing_enabled() ||
        control_mode == ACRO ||
        control_mode == FLY_BY_WIRE_A ||
        control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE ||
        control_mode == TRAINING) {
        return;
    }
    stick_mix_channel(channel_roll);
    stick_mix_channel(channel_pitch);
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
static void stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        control_mode == ACRO ||
        control_mode == FLY_BY_WIRE_A ||
        control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE ||
        control_mode == TRAINING ||
        (control_mode == AUTO && g.auto_fbw_steer)) {
        return;
    }
    // do FBW style stick mixing. We don't treat it linearly
    // however. For inputs up to half the maximum, we use linear
    // addition to the nav_roll and nav_pitch. Above that it goes
    // non-linear and ends up as 2x the maximum, to ensure that
    // the user can direct the plane in any direction with stick
    // mixing.
    float roll_input = channel_roll->norm_input();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
    
    float pitch_input = channel_pitch->norm_input();
    if (fabsf(pitch_input) > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    }
    if (inverted_flight) {
        pitch_input = -pitch_input;
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max_cd;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min_cd);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  stabilize the yaw axis. There are 3 modes of operation:

    - hold a specific heading with ground steering
    - rate controlled with ground steering
    - yaw control for coordinated flight    
 */
static void stabilize_yaw(float speed_scaler)
{
    bool ground_steering = (channel_roll->control_in == 0 && fabsf(relative_altitude()) < g.ground_steer_alt);

    if (steer_state.hold_course_cd != -1 && ground_steering) {
        calc_nav_yaw_course();
    } else if (ground_steering) {
        calc_nav_yaw_ground();
    } else {
        calc_nav_yaw_coordinated(speed_scaler);
    }
}


/*
  a special stabilization function for training mode
 */
static void stabilize_training(float speed_scaler)
{
    if (training_manual_roll) {
        channel_roll->servo_out = channel_roll->control_in;
    } else {
        // calculate what is needed to hold
        stabilize_roll(speed_scaler);
        if ((nav_roll_cd > 0 && channel_roll->control_in < channel_roll->servo_out) ||
            (nav_roll_cd < 0 && channel_roll->control_in > channel_roll->servo_out)) {
            // allow user to get out of the roll
            channel_roll->servo_out = channel_roll->control_in;            
        }
    }

    if (training_manual_pitch) {
        channel_pitch->servo_out = channel_pitch->control_in;
    } else {
        stabilize_pitch(speed_scaler);
        if ((nav_pitch_cd > 0 && channel_pitch->control_in < channel_pitch->servo_out) ||
            (nav_pitch_cd < 0 && channel_pitch->control_in > channel_pitch->servo_out)) {
            // allow user to get back to level
            channel_pitch->servo_out = channel_pitch->control_in;            
        }
    }

    stabilize_yaw(speed_scaler);
}


/*
  this is the ACRO mode stabilization function. It does rate
  stabilization on roll and pitch axes
 */
static void stabilize_acro(float speed_scaler)
{
    float roll_rate = (channel_roll->control_in/4500.0f) * g.acro_roll_rate;
    float pitch_rate = (channel_pitch->control_in/4500.0f) * g.acro_pitch_rate;

    /*
      check for special roll handling near the pitch poles
     */
    if (g.acro_locking && roll_rate == 0) {
        /*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
         */
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * G_Dt;
        }
        int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
        nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // try to reduce the integrated angular error to zero. We set
        // 'stabilze' to true, which disables the roll integrator
        channel_roll->servo_out  = rollController.get_servo_out(roll_error_cd,
                                                                speed_scaler,
                                                                true);
    } else {
        /*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
         */
        acro_state.locked_roll = false;
        channel_roll->servo_out  = rollController.get_rate_out(roll_rate,  speed_scaler);
    }

    if (g.acro_locking && pitch_rate == 0) {
        /*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
         */
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // try to hold the locked pitch. Note that we have the pitch
        // integrator enabled, which helps with inverted flight
        nav_pitch_cd = acro_state.locked_pitch_cd;
        channel_pitch->servo_out  = pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor,
                                                                  speed_scaler,
                                                                  false);
    } else {
        /*
          user has non-zero pitch input, use a pure rate controller
         */
        acro_state.locked_pitch = false;
        channel_pitch->servo_out = pitchController.get_rate_out(pitch_rate, speed_scaler);
    }

    /*
      manual rudder for now
     */
    channel_rudder->servo_out = channel_rudder->control_in;
}

/*
  main stabilization function for all 3 axes
 */
static void stabilize()
{
    if (control_mode == MANUAL) {
        // nothing to do
        return;
    }
    float speed_scaler = get_speed_scaler();

    if (control_mode == TRAINING) {
        stabilize_training(speed_scaler);
    } else if (control_mode == ACRO) {
        stabilize_acro(speed_scaler);
    } else {
        if (g.stick_mixing == STICK_MIXING_FBW && control_mode != STABILIZE) {
            stabilize_stick_mixing_fbw();
        }
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        if (g.stick_mixing == STICK_MIXING_DIRECT || control_mode == STABILIZE) {
            stabilize_stick_mixing_direct();
        }
        stabilize_yaw(speed_scaler);
    }

    /*
      see if we should zero the attitude controller integrators. 
     */
    if (channel_throttle->control_in == 0 &&
        relative_altitude_abs_cm() < 500 && 
        fabs(barometer.get_climb_rate()) < 0.5f &&
        g_gps->ground_speed_cm < 300) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();
    }
}


static void calc_throttle()
{
    if (aparm.throttle_cruise <= 1) {
        // user has asked for zero throttle - this may be done by a
        // mission which wants to turn off the engine for a parachute
        // landing
        channel_throttle->servo_out = 0;
        return;
    }

    channel_throttle->servo_out = SpdHgt_Controller->get_throttle_demand();
}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

/*
  calculate yaw control for coordinated flight
 */
static void calc_nav_yaw_coordinated(float speed_scaler)
{
    bool disable_integrator = false;
    if (control_mode == STABILIZE && channel_rudder->control_in != 0) {
        disable_integrator = true;
    }
    channel_rudder->servo_out = yawController.get_servo_out(speed_scaler, disable_integrator);

    // add in rudder mixing from roll
    channel_rudder->servo_out += channel_roll->servo_out * g.kff_rudder_mix;
    channel_rudder->servo_out += channel_rudder->control_in;
    channel_rudder->servo_out = constrain_int16(channel_rudder->servo_out, -4500, 4500);
}

/*
  calculate yaw control for ground steering with specific course
 */
static void calc_nav_yaw_course(void)
{
    // holding a specific navigation course on the ground. Used in
    // auto-takeoff and landing
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    channel_rudder->servo_out = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        stick_mix_channel(channel_rudder);
    }
    channel_rudder->servo_out = constrain_int16(channel_rudder->servo_out, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
static void calc_nav_yaw_ground(void)
{
    if (g_gps->ground_speed_cm < 100 && 
        channel_throttle->control_in == 0) {
        // manual rudder control while still
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        channel_rudder->servo_out = channel_rudder->control_in;
        return;
    }

    float steer_rate = (channel_rudder->control_in/4500.0f) * g.ground_steer_dps;
    if (steer_rate != 0) {
        // pilot is giving rudder input
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // pilot has released the rudder stick or we are still - lock the course
        steer_state.locked_course = true;
        steer_state.locked_course_err = 0;
    }
    if (!steer_state.locked_course) {
        // use a rate controller at the pilot specified rate
        channel_rudder->servo_out = steerController.get_steering_out_rate(steer_rate);
    } else {
        // use a error controller on the summed error
        steer_state.locked_course_err += ahrs.get_gyro().z * G_Dt;
        int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
        channel_rudder->servo_out = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    channel_rudder->servo_out = constrain_int16(channel_rudder->servo_out, -4500, 4500);
}


static void calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    nav_pitch_cd = SpdHgt_Controller->get_pitch_demand();
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


static void calc_nav_roll()
{
    nav_roll_cd = nav_controller->nav_roll_cd();
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
}


/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (aparm.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = aparm.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}


/*   Check for automatic takeoff conditions being met using the following sequence:
 *   1) Check for adequate GPS lock - if not return false
 *   2) Check the gravity compensated longitudinal acceleration against the threshold and start the timer if true
 *   3) Wait until the timer has reached the specified value (increments of 0.1 sec) and then check the GPS speed against the threshold
 *   4) If the GPS speed is above the threshold and the attitude is within limits then return true and reset the timer
 *   5) If the GPS speed and attitude within limits has not been achieved after 2.5 seconds, return false and reset the timer
 *   6) If the time lapsed since the last timecheck is greater than 0.2 seconds, return false and reset the timer
 *   NOTE : This function relies on the TECS 50Hz processing for its acceleration measure.
 */
static bool auto_takeoff_check(void)
{
    // this is a more advanced check that relies on TECS
    uint32_t now = hal.scheduler->millis();
    static bool launchTimerStarted;
    static uint32_t last_tkoff_arm_time;
    static uint32_t last_check_ms;

    // Reset states if process has been interrupted
    if (last_check_ms && (now - last_check_ms) > 200) {
        gcs_send_text_fmt(PSTR("Timer Interrupted AUTO"));
	    launchTimerStarted = false;
	    last_tkoff_arm_time = 0;
        last_check_ms = now;
        return false;
    }

    last_check_ms = now;

    // Check for bad GPS
    if (g_gps == NULL || g_gps->status() != GPS::GPS_OK_FIX_3D) {
        // no auto takeoff without GPS lock
        return false;
    }

    // Check for launch acceleration or timer started. NOTE: relies on TECS 50Hz processing
    if (!launchTimerStarted &&
        g.takeoff_throttle_min_accel != 0.0 &&
        SpdHgt_Controller->get_VXdot() < g.takeoff_throttle_min_accel) {
        goto no_launch;
    }

    // we've reached the acceleration threshold, so start the timer
    if (!launchTimerStarted) {
        launchTimerStarted = true;
        last_tkoff_arm_time = now;
        gcs_send_text_fmt(PSTR("Armed AUTO, xaccel = %.1f m/s/s, waiting %.1f sec"), 
                          SpdHgt_Controller->get_VXdot(), 0.1f*float(min(g.takeoff_throttle_delay,25)));
    }

    // Only perform velocity check if not timed out
    if ((now - last_tkoff_arm_time) > 2500) {
        gcs_send_text_fmt(PSTR("Timeout AUTO"));
        goto no_launch;
    }

    // Check aircraft attitude for bad launch
    if (ahrs.pitch_sensor <= -3000 ||
        ahrs.pitch_sensor >= 4500 ||
        abs(ahrs.roll_sensor) > 3000) {
        gcs_send_text_fmt(PSTR("Bad Launch AUTO"));
        goto no_launch;
    }

    // Check ground speed and time delay
    if (((g_gps->ground_speed_cm > g.takeoff_throttle_min_speed*100.0f || g.takeoff_throttle_min_speed == 0.0)) && 
        ((now - last_tkoff_arm_time) >= min(uint16_t(g.takeoff_throttle_delay)*100,2500))) {
        gcs_send_text_fmt(PSTR("Triggered AUTO, GPSspd = %.1f"), g_gps->ground_speed_cm*0.01f);
        launchTimerStarted = false;
        last_tkoff_arm_time = 0;
        return true;
    }

    // we're not launching yet, but the timer is still going
    return false;

no_launch:
    launchTimerStarted = false;
    last_tkoff_arm_time = 0;
    return false;
}


/**
  Do we think we are flying?
  This is a heuristic so it could be wrong in some cases.  In particular, if we don't have GPS lock we'll fall
  back to only using altitude.  (This is probably more optimistic than what suppress_throttle wants...)
*/
static bool is_flying(void)
{
    // If we don't have a GPS lock then don't use GPS for this test
    bool gpsMovement = (g_gps == NULL ||
                        g_gps->status() < GPS::GPS_OK_FIX_2D ||
                        g_gps->ground_speed_cm >= 500);
    
    bool airspeedMovement = !airspeed.use() || airspeed.get_airspeed() >= 5;
    
    // we're more than 5m from the home altitude
    bool inAir = relative_altitude_abs_cm() > 500;

    return inAir && gpsMovement && airspeedMovement;
}


/* We want to supress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode or takeoff speed/accel not yet reached
   *       OR
   *       5 - Home location is not set
*/
static bool suppress_throttle(void)
{
    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (!auto_throttle_mode) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    if (control_mode==AUTO && g.auto_fbw_steer) {
        // user has throttle control
        return false;
    }

    if (control_mode==AUTO && takeoff_complete == false && auto_takeoff_check()) {
        // we're in auto takeoff 
        throttle_suppressed = false;
        if (steer_state.hold_course_cd != -1) {
            // update takeoff course hold, if already initialised
            steer_state.hold_course_cd = ahrs.yaw_sensor;
            gcs_send_text_fmt(PSTR("Holding course %ld"), steer_state.hold_course_cd);
        }
        return false;
    }
    
    if (relative_altitude_abs_cm() >= 1000) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (g_gps != NULL && 
        g_gps->status() >= GPS::GPS_OK_FIX_2D && 
        g_gps->ground_speed_cm >= 500) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents throttle up due to spiky GPS
        // groundspeed with bad GPS reception
        if (!airspeed.use() || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            throttle_suppressed = false;
            return false;        
        }
    }

    // throttle remains suppressed
    return true;
}

/*
  implement a software VTail or elevon mixer. There are 4 different mixing modes
 */
static void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out)
{
    int16_t c1, c2;
    int16_t v1, v2;

    // first get desired elevator and rudder as -500..500 values
    c1 = chan1_out - 1500;
    c2 = chan2_out - 1500;

    v1 = (c1 - c2) * g.mixing_gain;
    v2 = (c1 + c2) * g.mixing_gain;

    // now map to mixed output
    switch (mixing_type) {
    case MIXING_DISABLED:
        return;

    case MIXING_UPUP:
        break;

    case MIXING_UPDN:
        v2 = -v2;
        break;

    case MIXING_DNUP:
        v1 = -v1;
        break;

    case MIXING_DNDN:
        v1 = -v1;
        v2 = -v2;
        break;
    }

    // scale for a 1500 center and 900..2100 range, symmetric
    v1 = constrain_int16(v1, -600, 600);
    v2 = constrain_int16(v2, -600, 600);

    chan1_out = 1500 + v1;
    chan2_out = 1500 + v2;
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = channel_throttle->radio_out;

    if (control_mode == MANUAL) {
        // do a direct pass through of radio values
        if (g.mix_mode == 0 || g.elevon_output != MIXING_DISABLED) {
            channel_roll->radio_out                = channel_roll->radio_in;
            channel_pitch->radio_out               = channel_pitch->radio_in;
        } else {
            channel_roll->radio_out                = channel_roll->read();
            channel_pitch->radio_out               = channel_pitch->read();
        }
        channel_throttle->radio_out    = channel_throttle->radio_in;
        channel_rudder->radio_out              = channel_rudder->radio_in;

        // setup extra channels. We want this to come from the
        // main input channel, but using the 2nd channels dead
        // zone, reverse and min/max settings. We need to use
        // pwm_to_angle_dz() to ensure we don't trim the value for the
        // deadzone of the main aileron channel, otherwise the 2nd
        // aileron won't quite follow the first one
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->pwm_to_angle_dz(0));
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->pwm_to_angle_dz(0));
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, channel_rudder->pwm_to_angle_dz(0));

        // this variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);

        if (g.mix_mode == 0 && g.elevon_output == MIXING_DISABLED) {
            // set any differential spoilers to follow the elevons in
            // manual mode. 
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler1, channel_roll->radio_out);
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler2, channel_pitch->radio_out);
        }
    } else {
        if (g.mix_mode == 0) {
            // both types of secondary aileron are slaved to the roll servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron_with_input, channel_roll->servo_out);

            // both types of secondary elevator are slaved to the pitch servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator_with_input, channel_pitch->servo_out);

            // setup secondary rudder
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, channel_rudder->servo_out);
        }else{
            /*Elevon mode*/
            float ch1;
            float ch2;
            ch1 = channel_pitch->servo_out - (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->servo_out);
            ch2 = channel_pitch->servo_out + (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->servo_out);

			/* Differential Spoilers
               If differential spoilers are setup, then we translate
               rudder control into splitting of the two ailerons on
               the side of the aircraft where we want to induce
               additional drag.
             */
			if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
				float ch3 = ch1;
				float ch4 = ch2;
				if ( BOOL_TO_SIGN(g.reverse_elevons) * channel_rudder->servo_out < 0) {
				    ch1 += abs(channel_rudder->servo_out);
				    ch3 -= abs(channel_rudder->servo_out);
				} else {
					ch2 += abs(channel_rudder->servo_out);
				    ch4 -= abs(channel_rudder->servo_out);
				}
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler1, ch3);
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler2, ch4);
			}

            // directly set the radio_out values for elevon mode
            channel_roll->radio_out  =     elevon.trim1 + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0/ SERVO_MAX));
            channel_pitch->radio_out =     elevon.trim2 + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0/ SERVO_MAX));
        }

#if OBC_FAILSAFE == ENABLED
        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        if (obc.crash_plane()) {
            channel_roll->servo_out = -4500;
            channel_pitch->servo_out = -4500;
            channel_rudder->servo_out = -4500;
            channel_throttle->servo_out = 0;
        }
#endif
        

        // push out the PWM values
        if (g.mix_mode == 0) {
            channel_roll->calc_pwm();
            channel_pitch->calc_pwm();
        }
        channel_rudder->calc_pwm();

#if THROTTLE_OUT == 0
        channel_throttle->servo_out = 0;
#else
        // convert 0 to 100% into PWM
        channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                       aparm.throttle_min.get(), 
                                                       aparm.throttle_max.get());

        if (suppress_throttle()) {
            // throttle is suppressed in auto mode
            channel_throttle->servo_out = 0;
            if (g.throttle_suppress_manual) {
                // manual pass through of throttle while throttle is suppressed
                channel_throttle->radio_out = channel_throttle->radio_in;
            } else {
                channel_throttle->calc_pwm();                
            }
        } else if (g.throttle_passthru_stabilize && 
                   (control_mode == STABILIZE || 
                    control_mode == TRAINING ||
                    control_mode == ACRO ||
                    control_mode == FLY_BY_WIRE_A)) {
            // manual pass through of throttle while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            channel_throttle->radio_out = channel_throttle->radio_in;
        } else if (control_mode == GUIDED && 
                   guided_throttle_passthru) {
            // manual pass through of throttle while in GUIDED
            channel_throttle->radio_out = channel_throttle->radio_in;
        } else {
            // normal throttle calculation based on servo_out
            channel_throttle->calc_pwm();
        }
#endif
    }

    // Auto flap deployment
    if(control_mode < FLY_BY_WIRE_B) {
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);
    } else if (control_mode >= FLY_BY_WIRE_B) {
        int16_t flapSpeedSource = 0;

        // FIXME: use target_airspeed in both FBW_B and g.airspeed_enabled cases - Doug?
        if (control_mode == FLY_BY_WIRE_B) {
            flapSpeedSource = target_airspeed_cm * 0.01;
        } else if (airspeed.use()) {
            flapSpeedSource = g.airspeed_cruise_cm * 0.01;
        } else {
            flapSpeedSource = aparm.throttle_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, 0);
        } else if (g.flap_2_speed != 0 && flapSpeedSource > g.flap_2_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_1_percent);
        } else {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_2_percent);
        }
    }

    if (control_mode >= FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }

    if (control_mode == TRAINING) {
        // copy rudder in training mode
        channel_rudder->radio_out   = channel_rudder->radio_in;
    }

    if (g.vtail_output != MIXING_DISABLED) {
        channel_output_mixer(g.vtail_output, channel_pitch->radio_out, channel_rudder->radio_out);
    } else if (g.elevon_output != MIXING_DISABLED) {
        channel_output_mixer(g.elevon_output, channel_pitch->radio_out, channel_roll->radio_out);
    }

    //send throttle to 0 or MIN_PWM if not yet armed
    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) { 
            case AP_Arming::YES_MIN_PWM:
                channel_throttle->radio_out = channel_throttle->radio_min;
            break;
            case AP_Arming::YES_ZERO_PWM:
                channel_throttle->radio_out = 0;
            break;
            default:
                //keep existing behavior: do nothing to radio_out
                //(don't disarm throttle channel even if AP_Arming class is)
            break;
        }
    }

#if HIL_MODE != HIL_MODE_DISABLED
    // get the servos to the GCS immediately for HIL
    if (comm_get_txspace(MAVLINK_COMM_0) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN) {
        send_servo_out(MAVLINK_COMM_0);
    }
    if (!g.hil_servos) {
        return;
    }
#endif

    // send values to the PWM timers for output
    // ----------------------------------------
    channel_roll->output();
    channel_pitch->output();
    channel_throttle->output();
    channel_rudder->output();
    // Route configurable aux. functions to their respective servos
    g.rc_5.output_ch(CH_5);
    g.rc_6.output_ch(CH_6);
    g.rc_7.output_ch(CH_7);
    g.rc_8.output_ch(CH_8);
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_9.output_ch(CH_9);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_12.output_ch(CH_12);
 #endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) 
{
    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
        servo_write(1, 1400);
        mavlink_delay(400);
        servo_write(1, 1600);
        mavlink_delay(200);
        servo_write(1, 1500);
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}

