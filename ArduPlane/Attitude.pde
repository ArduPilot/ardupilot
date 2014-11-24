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
        if (aspeed > auto_state.highest_airspeed) {
            auto_state.highest_airspeed = aspeed;
        }
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain_float(speed_scaler, 0.5, 2.0);
    } else {
        if (channel_throttle->servo_out > 0) {
            speed_scaler = 0.5f + ((float)THROTTLE_CRUISE / channel_throttle->servo_out / 2.0f);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67f;
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
            !rc_failsafe_active()) {
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
    if (fly_inverted()) {
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
    int8_t force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // we are holding the tail down during takeoff. Just covert
        // from a percentage to a -4500..4500 centidegree angle
        channel_pitch->servo_out = 45*force_elevator;
        return;
    }
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
static void stick_mix_channel(RC_Channel *channel, int16_t &servo_out)
{
    float ch_inf;
        
    ch_inf = (float)channel->radio_in - (float)channel->radio_trim;
    ch_inf = fabsf(ch_inf);
    ch_inf = min(ch_inf, 400.0f);
    ch_inf = ((400.0f - ch_inf) / 400.0f);
    servo_out *= ch_inf;
    servo_out += channel->pwm_to_angle();
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
static void stabilize_stick_mixing_direct()
{
    if (!stick_mixing_enabled() ||
        control_mode == ACRO ||
        control_mode == FLY_BY_WIRE_A ||
        control_mode == AUTOTUNE ||
        control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE ||
        control_mode == TRAINING) {
        return;
    }
    stick_mix_channel(channel_roll, channel_roll->servo_out);
    stick_mix_channel(channel_pitch, channel_pitch->servo_out);
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
        control_mode == AUTOTUNE ||
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
    if (fly_inverted()) {
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
    if (control_mode == AUTO && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        // in land final setup for ground steering
        steering_control.ground_steering = true;
    } else {
        // otherwise use ground steering when no input control and we
        // are below the GROUND_STEER_ALT
        steering_control.ground_steering = (channel_roll->control_in == 0 && 
                                            fabsf(relative_altitude()) < g.ground_steer_alt);
        if (control_mode == AUTO && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
            // don't use ground steering on landing approach
            steering_control.ground_steering = false;
        }
    }


    /*
      first calculate steering_control.steering for a nose or tail
      wheel.
      We use "course hold" mode for the rudder when either in the
      final stage of landing (when the wings are help level) or when
      in course hold in FBWA mode (when we are below GROUND_STEER_ALT)
     */
    if ((control_mode == AUTO && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) ||
        (steer_state.hold_course_cd != -1 && steering_control.ground_steering)) {
        calc_nav_yaw_course();
    } else if (steering_control.ground_steering) {
        calc_nav_yaw_ground();
    }

    /*
      now calculate steering_control.rudder for the rudder
     */
    calc_nav_yaw_coordinated(speed_scaler);
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
    steering_control.steering = steering_control.rudder = channel_rudder->control_in;
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
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        gps.ground_speed() < 3) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // if moving very slowly also zero the steering integrator
        if (gps.ground_speed() < 1) {
            steerController.reset_I();            
        }
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
    steering_control.rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

    // add in rudder mixing from roll
    steering_control.rudder += channel_roll->servo_out * g.kff_rudder_mix;
    steering_control.rudder += channel_rudder->control_in;
    steering_control.rudder = constrain_int16(steering_control.rudder, -4500, 4500);
}

/*
  calculate yaw control for ground steering with specific course
 */
static void calc_nav_yaw_course(void)
{
    // holding a specific navigation course on the ground. Used in
    // auto-takeoff and landing
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    steering_control.steering = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        stick_mix_channel(channel_rudder, steering_control.steering);
    }
    steering_control.steering = constrain_int16(steering_control.steering, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
static void calc_nav_yaw_ground(void)
{
    if (gps.ground_speed() < 1 && 
        channel_throttle->control_in == 0 &&
        flight_stage != AP_SpdHgtControl::FLIGHT_TAKEOFF) {
        // manual rudder control while still
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        steering_control.steering = channel_rudder->control_in;
        return;
    }

    float steer_rate = (channel_rudder->control_in/4500.0f) * g.ground_steer_dps;
    if (flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF) {
        steer_rate = 0;
    }
    if (steer_rate != 0) {
        // pilot is giving rudder input
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // pilot has released the rudder stick or we are still - lock the course
        steer_state.locked_course = true;
        if (flight_stage != AP_SpdHgtControl::FLIGHT_TAKEOFF) {
            steer_state.locked_course_err = 0;
        }
    }
    if (!steer_state.locked_course) {
        // use a rate controller at the pilot specified rate
        steering_control.steering = steerController.get_steering_out_rate(steer_rate);
    } else {
        // use a error controller on the summed error
        int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
        steering_control.steering = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    steering_control.steering = constrain_int16(steering_control.steering, -4500, 4500);
}


/*
  calculate a new nav_pitch_cd from the speed height controller
 */
static void calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    nav_pitch_cd = SpdHgt_Controller->get_pitch_demand();
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  calculate a new nav_roll_cd from the navigation controller
 */
static void calc_nav_roll()
{
    nav_roll_cd = nav_controller->nav_roll_cd();
    update_load_factor();
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
}


/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    uint8_t slewrate = aparm.throttle_slewrate;
    if (control_mode==AUTO && auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
        slewrate = g.takeoff_throttle_slewrate;
    }
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}

/*****************************************
Flap slew limit
*****************************************/
static void flap_slew_limit(int8_t &last_value, int8_t &new_value)
{
    uint8_t slewrate = g.flap_slewrate;
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate) {                   
        // limit flap change by the given percentage per second
        float temp = slewrate * G_Dt;
        // allow a minimum change of 1% per cycle. This means the
        // slowest flaps we can do is full change over 2 seconds
        if (temp < 1) {
            temp = 1;
        }
        new_value = constrain_int16(new_value, last_value - temp, last_value + temp);
    }
    last_value = new_value;
}



/**
  Do we think we are flying?
  This is a heuristic so it could be wrong in some cases.  In particular, if we don't have GPS lock we'll fall
  back to only using altitude.  (This is probably more optimistic than what suppress_throttle wants...)
*/
static bool is_flying(void)
{
    // If we don't have a GPS lock then don't use GPS for this test
    bool gpsMovement = (gps.status() < AP_GPS::GPS_OK_FIX_2D ||
                        gps.ground_speed() >= 5);
    
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

    if (control_mode==AUTO && 
        auto_state.takeoff_complete == false && 
        auto_takeoff_check()) {
        // we're in auto takeoff 
        throttle_suppressed = false;
        return false;
    }
    
    if (relative_altitude_abs_cm() >= 1000) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D && 
        gps.ground_speed() >= 5) {
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

/*
  setup flaperon output channels
 */
static void flaperon_update(int8_t flap_percent)
{
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon1) ||
        !RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon2)) {
        return;
    }
    int16_t ch1, ch2;
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.

      Use k_flaperon1 and k_flaperon2 channel trims to center servos.
      Then adjust aileron trim for level flight (note that aileron trim is affected
      by mixing gain). flapin_channel's trim is not used.
     */
     
    ch1 = channel_roll->radio_out;
    // The *5 is to take a percentage to a value from -500 to 500 for the mixer
    ch2 = 1500 - flap_percent * 5;
    channel_output_mixer(g.flaperon_output, ch1, ch2);
    RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::k_flaperon1, ch1);
    RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::k_flaperon2, ch2);
}


/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = channel_throttle->radio_out;

    /*
      see if we are doing ground steering.
     */
    if (!steering_control.ground_steering) {
        // we are not at an altitude for ground steering. Set the nose
        // wheel to the rudder just in case the barometer has drifted
        // a lot
        steering_control.steering = steering_control.rudder;
    } else if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_steering)) {
        // we are within the ground steering altitude but don't have a
        // dedicated steering channel. Set the rudder to the ground
        // steering output
        steering_control.rudder = steering_control.steering;
    }
    channel_rudder->servo_out = steering_control.rudder;

    // clear ground_steering to ensure manual control if the yaw stabilizer doesn't run
    steering_control.ground_steering = false;

    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, steering_control.rudder);
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering, steering_control.steering);

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

        // this variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input);

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
            channel_roll->radio_out  =     elevon.trim1 + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0f/ SERVO_MAX));
            channel_pitch->radio_out =     elevon.trim2 + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0f/ SERVO_MAX));
        }

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
        uint8_t min_throttle = aparm.throttle_min.get();
        uint8_t max_throttle = aparm.throttle_max.get();
        if (control_mode == AUTO && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
            min_throttle = 0;
        }
        if (control_mode == AUTO && flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF) {
            max_throttle = takeoff_throttle();
        }
        channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                      min_throttle,
                                                      max_throttle);

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
                    control_mode == FLY_BY_WIRE_A ||
                    control_mode == AUTOTUNE)) {
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
    int8_t auto_flap_percent = 0;
    int8_t manual_flap_percent = 0;
    static int8_t last_auto_flap;
    static int8_t last_manual_flap;

    // work out any manual flap input
    RC_Channel *flapin = RC_Channel::rc_channel(g.flapin_channel-1);
    if (flapin != NULL && !failsafe.ch3_failsafe && failsafe.ch3_counter == 0) {
        flapin->input();
        manual_flap_percent = flapin->percent_input();
    }

    if (auto_throttle_mode) {
        int16_t flapSpeedSource = 0;
        if (airspeed.use()) {
            flapSpeedSource = target_airspeed_cm * 0.01f;
        } else {
            flapSpeedSource = aparm.throttle_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            auto_flap_percent = 0;
        } else if (g.flap_2_speed != 0 && flapSpeedSource <= g.flap_2_speed) {
            auto_flap_percent = g.flap_2_percent;
        } else {
            auto_flap_percent = g.flap_1_percent;
        }

        /*
          special flap levels for takeoff and landing. This works
          better than speed based flaps as it leads to less
          possibility of oscillation
         */
        if (control_mode == AUTO) {
            switch (flight_stage) {
            case AP_SpdHgtControl::FLIGHT_TAKEOFF:
                if (g.takeoff_flap_percent != 0) {
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_SpdHgtControl::FLIGHT_LAND_APPROACH:
            case AP_SpdHgtControl::FLIGHT_LAND_FINAL:
                if (g.land_flap_percent != 0) {
                    auto_flap_percent = g.land_flap_percent;
                }
                break;
            default:
                break;
            }
        }
    }

    // manual flap input overrides auto flap input
    if (abs(manual_flap_percent) > auto_flap_percent) {
        auto_flap_percent = manual_flap_percent;
    }

    flap_slew_limit(last_auto_flap, auto_flap_percent);
    flap_slew_limit(last_manual_flap, manual_flap_percent);

    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, auto_flap_percent);
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap, manual_flap_percent);

    if (control_mode >= FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }

    if (control_mode == TRAINING) {
        // copy rudder in training mode
        channel_rudder->radio_out   = channel_rudder->radio_in;
    }

    if (g.flaperon_output != MIXING_DISABLED && g.elevon_output == MIXING_DISABLED && g.mix_mode == 0) {
        flaperon_update(auto_flap_percent);
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

#if OBC_FAILSAFE == ENABLED
    // this is to allow the failsafe module to deliberately crash 
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
    obc.check_crash_plane();
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    // get the servos to the GCS immediately for HIL
    if (comm_get_txspace(MAVLINK_COMM_0) >= 
        MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
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
    RC_Channel_aux::output_ch_all();
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

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low throttle. It makes
  FBWA landings without stalling much easier.
 */
static void adjust_nav_pitch_throttle(void)
{
    uint8_t throttle = throttle_percentage();
    if (throttle < aparm.throttle_cruise) {
        float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}


/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
static void update_load_factor(void)
{
    float demanded_roll = fabsf(nav_roll_cd*0.01f);
    if (demanded_roll > 85) {
        // limit to 85 degrees to prevent numerical errors
        demanded_roll = 85;
    }
    aerodynamic_load_factor = 1.0f / safe_sqrt(cos(radians(demanded_roll)));

    if (!aparm.stall_prevention) {
        // stall prevention is disabled
        return;
    }
    if (fly_inverted()) {
        // no roll limits when inverted
        return;
    }

    float max_load_factor = smoothed_airspeed / aparm.airspeed_min;
    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodymamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be maneuvered with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
    }    
}
