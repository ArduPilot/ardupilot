/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

static bool land_with_gps;

static uint32_t land_start_time;
static bool land_pause;

// land_init - initialise land controller
bool Copter::land_init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    land_with_gps = position_ok();
    if (land_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
    
    land_start_time = millis();

    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    ap.land_repo_active = false;

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void Copter::land_run()
{
    if (land_with_gps) {
        land_gps_run();
    }else{
        land_nogps_run();
    }
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Copter::land_gps_run()
{
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        wp_nav.init_loiter_target();

        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
        return;
    }
    
    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    // pause before beginning land descent
    if(land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }
    
    land_run_horizontal_control();
    land_run_vertical_control(land_pause);
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Copter::land_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif

        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // pause before beginning land descent
    if(land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }

    land_run_vertical_control(land_pause);
}

void Copter::land_run_vertical_control(bool pause_descent)
{
    bool navigating = pos_control.is_active_xy();

#if PRECISION_LANDING == ENABLED
    bool doing_precision_landing = !ap.land_repo_active && precland.target_acquired() && navigating;
#else
    bool doing_precision_landing = false;
#endif

    // compute desired velocity
    const float precland_acceptable_error = 25.0f;
    const float precland_min_descent_speed = -10.0f;
    int32_t alt_above_ground;
    if (rangefinder_alt_ok()) {
        alt_above_ground = rangefinder_state.alt_cm_filt.get();
    } else {
        if (!navigating || !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, alt_above_ground)) {
            alt_above_ground = current_loc.alt;
        }
    }

    float cmb_rate = 0;
    if (!pause_descent) {
        cmb_rate = AC_AttitudeControl::sqrt_controller(LAND_START_ALT-alt_above_ground, g.p_alt_hold.kP(), pos_control.get_accel_z());
        cmb_rate = constrain_float(cmb_rate, pos_control.get_speed_down(), -abs(g.land_speed));

        if (doing_precision_landing && alt_above_ground < 300.0f) {
            float land_slowdown = MAX(0.0f, pos_control.get_horizontal_error()*(abs(g.land_speed)/precland_acceptable_error));
            cmb_rate = MIN(precland_min_descent_speed, cmb_rate+land_slowdown);
        }
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

void Copter::land_run_horizontal_control()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    
    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }
    
    // process pilot inputs
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->get_control_in();
            pitch_control = channel_pitch->get_control_in();

            // record if pilot has overriden roll or pitch
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

#if PRECISION_LANDING == ENABLED
    bool doing_precision_landing = !ap.land_repo_active && precland.target_acquired();
    // run precision landing
    if (doing_precision_landing && precland_last_update_ms != precland.last_update_ms()) {
        Vector3f target_pos;
        precland.get_target_position(target_pos);
        pos_control.set_xy_target(target_pos.x, target_pos.y);
        pos_control.freeze_ff_xy();
        precland_last_update_ms = precland.last_update_ms();
    }
#else
    bool doing_precision_landing = false;
#endif
    
    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());
}

// land_do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void Copter::land_do_not_use_GPS()
{
    land_with_gps = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(mode_reason_t reason)
{
    set_mode(LAND, reason);
    land_pause = true;

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS() {
    return (control_mode == LAND && land_with_gps);
}
