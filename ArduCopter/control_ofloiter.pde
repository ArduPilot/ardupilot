/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_ofloiter.pde - init and run calls for of_loiter (optical flow loiter) flight mode
 */

#if OPTFLOW == ENABLED

#define OPTFLOW_ALT_MAX_CM  1500        // maximum altitude above home that optical flow sensor will be used
#define OPTFLOW_TIMEOUT_MS  200         // timeout in milliseconds after which we will give up on optical flow readings and return control to the pilot
#define OPTFLOW_RP_RATE_LIM (2000/MAIN_LOOP_RATE)   // limit in centi-degrees/sec on rate of change of roll-pitch target.  Equal to 20deg/sec

// ofloiter_init - initialise ofloiter controller
static bool ofloiter_init(bool ignore_checks)
{
    if (optflow.enabled() || ignore_checks) {

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        // initialise of_roll, pitch to current attitude
        of_roll = ahrs.roll_sensor;
        of_pitch = ahrs.pitch_sensor;
        reset_optflow_I();

        return true;
    }else{
        return false;
    }
}

// ofloiter_run - runs the optical flow loiter controller
// should be called at 100hz or more
static void ofloiter_run()
{
    int16_t target_roll, target_pitch;
    float final_roll, final_pitch;
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        of_roll = ahrs.roll_sensor;
        of_pitch = ahrs.pitch_sensor;
        reset_optflow_I();
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
        of_roll = ahrs.roll_sensor;
        of_pitch = ahrs.pitch_sensor;
    }else{
        // mix in user control with optical flow
        get_of_roll_pitch(target_roll, target_pitch, final_roll, final_pitch);

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(final_roll, final_pitch, target_yaw_rate, get_smoothing_gain());

        // run altitude controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// calculate modified roll/pitch depending upon optical flow calculated position
static void get_of_roll_pitch(int16_t input_roll, int16_t input_pitch, float &roll_out, float &pitch_out)
{
    static uint32_t last_of_update = 0;
    float dt;
    Vector2f vel;

    // To-Do: pass input_roll, input_pitch through to roll_out, pitch_out if input is non-zero or previous iteration was non-zero

    // check if new optflow data available
    if (optflow.last_update() != last_of_update) {

        // calculate dt and sanity check
        dt = (optflow.last_update() - last_of_update) / 1000.0f;
        if (dt > 0.2f) {
            dt = 0.0f;
            g.pid_optflow_roll.reset_I();
            g.pid_optflow_pitch.reset_I();
        }
        last_of_update = optflow.last_update();

        // get latest velocity from sensor
        vel = optflow.velocity();
    }

    // calculate time since last update
    uint32_t time_since_update_ms = millis() - last_of_update;

    // use pilot roll input if input is non-zero, altitude above 15m or optical flow sensor has timed out
    if (input_roll != 0 || current_loc.alt > OPTFLOW_ALT_MAX_CM || time_since_update_ms > OPTFLOW_TIMEOUT_MS) {
        roll_out = input_roll;
    } else {
        // run velocity through pid controller
        roll_out = g.pid_optflow_roll.get_pid(-vel.x, dt);

        // limit amount of change and maximum angle
        // To-Do: replace reliance on of_roll, of_pitch within this function
        roll_out = constrain_float(roll_out, (of_roll-OPTFLOW_RP_RATE_LIM), (of_roll+OPTFLOW_RP_RATE_LIM));
    }

    // use pilot pitch input if input is non-zero, altitude above 15m or optical flow sensor has timed out
    if (input_pitch != 0 || current_loc.alt > OPTFLOW_ALT_MAX_CM || time_since_update_ms > OPTFLOW_TIMEOUT_MS) {
        pitch_out = input_pitch;
    } else {
        // run velocity through pid controller
        pitch_out = g.pid_optflow_pitch.get_pid(vel.y, dt);

        // limit amount of change and maximum angle
        // To-Do: replace reliance on of_roll, of_pitch within this function
        pitch_out = constrain_float(pitch_out, (of_pitch-OPTFLOW_RP_RATE_LIM), (of_pitch+OPTFLOW_RP_RATE_LIM));
    }
}

// reset_optflow_I - reset optflow position hold I terms
static void reset_optflow_I(void)
{
    g.pid_optflow_roll.reset_I();
    g.pid_optflow_pitch.reset_I();
}

#endif // OPTFLOW == ENABLED
