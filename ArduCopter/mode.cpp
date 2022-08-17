#include "Copter.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(copter.g),
    g2(copter.g2),
    wp_nav(copter.wp_nav),
    loiter_nav(copter.loiter_nav),
    pos_control(copter.pos_control),
    inertial_nav(copter.inertial_nav),
    ahrs(copter.ahrs),
    attitude_control(copter.attitude_control),
    motors(copter.motors),
    channel_roll(copter.channel_roll),
    channel_pitch(copter.channel_pitch),
    channel_throttle(copter.channel_throttle),
    channel_yaw(copter.channel_yaw),
    G_Dt(copter.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Copter::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
#if MODE_ACRO_ENABLED == ENABLED
        case Mode::Number::ACRO:
            ret = &mode_acro;
            break;
#endif

        case Mode::Number::STABILIZE:
            ret = &mode_stabilize;
            break;

        case Mode::Number::ALT_HOLD:
            ret = &mode_althold;
            break;

#if MODE_AUTO_ENABLED == ENABLED
        case Mode::Number::AUTO:
            ret = &mode_auto;
            break;
#endif

#if MODE_CIRCLE_ENABLED == ENABLED
        case Mode::Number::CIRCLE:
            ret = &mode_circle;
            break;
#endif

#if MODE_LOITER_ENABLED == ENABLED
        case Mode::Number::LOITER:
            ret = &mode_loiter;
            break;
#endif

#if MODE_GUIDED_ENABLED == ENABLED
        case Mode::Number::GUIDED:
            ret = &mode_guided;
            break;
#endif

        case Mode::Number::LAND:
            ret = &mode_land;
            break;

#if MODE_RTL_ENABLED == ENABLED
        case Mode::Number::RTL:
            ret = &mode_rtl;
            break;
#endif

#if MODE_DRIFT_ENABLED == ENABLED
        case Mode::Number::DRIFT:
            ret = &mode_drift;
            break;
#endif

#if MODE_SPORT_ENABLED == ENABLED
        case Mode::Number::SPORT:
            ret = &mode_sport;
            break;
#endif

#if MODE_FLIP_ENABLED == ENABLED
        case Mode::Number::FLIP:
            ret = &mode_flip;
            break;
#endif

#if AUTOTUNE_ENABLED == ENABLED
        case Mode::Number::AUTOTUNE:
            ret = &mode_autotune;
            break;
#endif

#if MODE_POSHOLD_ENABLED == ENABLED
        case Mode::Number::POSHOLD:
            ret = &mode_poshold;
            break;
#endif

#if MODE_BRAKE_ENABLED == ENABLED
        case Mode::Number::BRAKE:
            ret = &mode_brake;
            break;
#endif

#if MODE_THROW_ENABLED == ENABLED
        case Mode::Number::THROW:
            ret = &mode_throw;
            break;
#endif

#if HAL_ADSB_ENABLED
        case Mode::Number::AVOID_ADSB:
            ret = &mode_avoid_adsb;
            break;
#endif

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED
        case Mode::Number::GUIDED_NOGPS:
            ret = &mode_guided_nogps;
            break;
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
        case Mode::Number::SMART_RTL:
            ret = &mode_smartrtl;
            break;
#endif

#if MODE_FLOWHOLD_ENABLED == ENABLED
        case Mode::Number::FLOWHOLD:
            ret = (Mode *)g2.mode_flowhold_ptr;
            break;
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
        case Mode::Number::FOLLOW:
            ret = &mode_follow;
            break;
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
        case Mode::Number::ZIGZAG:
            ret = &mode_zigzag;
            break;
#endif

#if MODE_SYSTEMID_ENABLED == ENABLED
        case Mode::Number::SYSTEMID:
            ret = (Mode *)g2.mode_systemid_ptr;
            break;
#endif

#if MODE_AUTOROTATE_ENABLED == ENABLED
        case Mode::Number::AUTOROTATE:
            ret = &mode_autorotate;
            break;
#endif

#if MODE_TURTLE_ENABLED == ENABLED
        case Mode::Number::TURTLE:
            ret = &mode_turtle;
            break;
#endif

        default:
            break;
    }

    return ret;
}


// called when an attempt to change into a mode is unsuccessful:
void Copter::mode_change_failed(const Mode *mode, const char *reason)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to %s failed: %s", mode->name(), reason);
    AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode->mode_number()));
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
}

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(Mode::Number mode, ModeReason reason)
{
    // update last reason
    const ModeReason last_reason = _last_reason;
    _last_reason = reason;

    // return immediately if we are already in the desired mode
    if (mode == flightmode->mode_number()) {
        control_mode_reason = reason;
        // set yaw rate time constant during autopilot startup
        if (reason == ModeReason::INITIALISED && mode == Mode::Number::STABILIZE) {
            attitude_control->set_yaw_rate_tc(g2.command_model_pilot.get_rate_tc());
        }
        // make happy noise
        if (copter.ap.initialised && (reason != last_reason)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

#if MODE_AUTO_ENABLED == ENABLED
    if (mode == Mode::Number::AUTO_RTL) {
        // Special case for AUTO RTL, not a true mode, just AUTO in disguise
        return mode_auto.jump_to_landing_sequence_auto_RTL(reason);
    }
#endif

    Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !new_flightmode->has_manual_throttle() &&
        (motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_UP || motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_DOWN)) {
        #if MODE_AUTOROTATE_ENABLED == ENABLED
            //if the mode being exited is the autorotation mode allow mode change despite rotor not being at
            //full speed.  This will reduce altitude loss on bail-outs back to non-manual throttle modes
            bool in_autorotation_check = (flightmode != &mode_autorotate || new_flightmode != &mode_autorotate);
        #else
            bool in_autorotation_check = false;
        #endif

        if (!in_autorotation_check) {
            mode_change_failed(new_flightmode, "runup not complete");
            return false;
        }
    }
#endif

#if FRAME_CONFIG != HELI_FRAME
    // ensure vehicle doesn't leap off the ground if a user switches
    // into a manual throttle mode from a non-manual-throttle mode
    // (e.g. user arms in guided, raises throttle to 1300 (not enough to
    // trigger auto takeoff), then switches into manual):
    bool user_throttle = new_flightmode->has_manual_throttle();
#if MODE_DRIFT_ENABLED == ENABLED
    if (new_flightmode == &mode_drift) {
        user_throttle = true;
    }
#endif
    if (!ignore_checks &&
        ap.land_complete &&
        user_throttle &&
        !copter.flightmode->has_manual_throttle() &&
        new_flightmode->get_pilot_desired_throttle() > copter.get_non_takeoff_throttle()) {
        mode_change_failed(new_flightmode, "throttle too high");
        return false;
    }
#endif

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !copter.position_ok()) {
        mode_change_failed(new_flightmode, "requires position");
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !copter.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        mode_change_failed(new_flightmode, "need alt estimate");
        return false;
    }

    if (!new_flightmode->init(ignore_checks)) {
        mode_change_failed(new_flightmode, "initialisation failed");
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // store previous flight mode (only used by tradeheli's autorotation)
    prev_control_mode = flightmode->mode_number();

    // update flight mode
    flightmode = new_flightmode;
    control_mode_reason = reason;
    logger.Write_Mode((uint8_t)flightmode->mode_number(), reason);
    gcs().send_message(MSG_HEARTBEAT);

#if HAL_ADSB_ENABLED
    adsb.set_is_auto_mode((mode == Mode::Number::AUTO) || (mode == Mode::Number::RTL) || (mode == Mode::Number::GUIDED));
#endif

#if AP_FENCE_ENABLED
    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    fence.manual_recovery_start();
#endif

#if CAMERA == ENABLED
    camera.set_is_auto_mode(flightmode->mode_number() == Mode::Number::AUTO);
#endif

    // set rate shaping time constants
#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    attitude_control->set_roll_pitch_rate_tc(g2.command_model_acro_rp.get_rate_tc());
#endif
    attitude_control->set_yaw_rate_tc(g2.command_model_pilot.get_rate_tc());
#if MODE_ACRO_ENABLED == ENABLED || MODE_DRIFT_ENABLED == ENABLED
    if (mode== Mode::Number::ACRO || mode== Mode::Number::DRIFT) {
        attitude_control->set_yaw_rate_tc(g2.command_model_acro_y.get_rate_tc());
    }
#endif

    // update notify object
    notify_flight_mode();

    // make happy noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }

    // return success
    return true;
}

bool Copter::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && copter.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return copter.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
    surface_tracking.invalidate_for_logging();  // invalidate surface tracking alt, flight mode will set to true if used

    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{
    // smooth throttle transition when switching from manual to automatic flight modes
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    // cancel any takeoffs in progress
    old_flightmode->takeoff_stop();

    // perform cleanup required for each flight mode
    old_flightmode->exit();

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_flightmode == &mode_acro) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    // if we are changing from a mode that did not use manual throttle,
    // stab col ramp value should be pre-loaded to the correct value to avoid a twitch
    // heli_stab_col_ramp should really only be active switching between Stabilize and Acro modes
    if (!old_flightmode->has_manual_throttle()){
        if (new_flightmode == &mode_stabilize){
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_flightmode == &mode_acro){
            input_manager.set_stab_col_ramp(0.0);
        }
    }
#endif //HELI_FRAME
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode() {
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)flightmode->mode_number();
    notify.set_flight_mode_str(flightmode->name4());
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Mode::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        roll_out_cd = 0.0;
        pitch_out_cd = 0.0;
        return;
    }

    //transform pilot's normalised roll or pitch stick input into a roll and pitch euler angle command
    float roll_out_deg;
    float pitch_out_deg;
    rc_input_to_roll_pitch(channel_roll->get_control_in()*(1.0/ROLL_PITCH_YAW_INPUT_MAX), channel_pitch->get_control_in()*(1.0/ROLL_PITCH_YAW_INPUT_MAX), angle_max_cd * 0.01,  angle_limit_cd * 0.01, roll_out_deg, pitch_out_deg);

    // Convert to centi-degrees
    roll_out_cd = roll_out_deg * 100.0;
    pitch_out_cd = pitch_out_deg * 100.0;
}

// transform pilot's roll or pitch input into a desired velocity
Vector2f Mode::get_pilot_desired_velocity(float vel_max) const
{
    Vector2f vel;

    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return vel;
    }
    // fetch roll and pitch inputs
    float roll_out = channel_roll->get_control_in();
    float pitch_out = channel_pitch->get_control_in();

    // convert roll and pitch inputs to -1 to +1 range
    float scaler = 1.0 / (float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // convert roll and pitch inputs into velocity in NE frame
    vel = Vector2f(-pitch_out, roll_out);
    if (vel.is_zero()) {
        return vel;
    }
    copter.rotate_body_frame_to_NE(vel.x, vel.y);

    // Transform square input range to circular output
    // vel_scaler is the vector to the edge of the +- 1.0 square in the direction of the current input
    Vector2f vel_scaler = vel / MAX(fabsf(vel.x), fabsf(vel.y));
    // We scale the output by the ratio of the distance to the square to the unit circle and multiply by vel_max
    vel *= vel_max / vel_scaler.length();
    return vel;
}

bool Mode::_TakeOff::triggered(const float target_climb_rate) const
{
    if (!copter.ap.land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }

    if (copter.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // hold aircraft on the ground until rotor speed runup has finished
        return false;
    }

    return true;
}

bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return true;
    }
    return false;
}

void Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    if (spool_up) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

void Mode::zero_throttle_and_hold_attitude()
{
    // run attitude controller
    attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

// handle situations where the vehicle is on the ground waiting for takeoff
// force_throttle_unlimited should be true in cases where we want to keep the motors spooled up
// (instead of spooling down to ground idle).  This is required for tradheli's in Guided and Auto
// where we always want the motor spooled up in Guided or Auto mode.  Tradheli's main rotor stops 
// when spooled down to ground idle.
// ultimately it forces the motor interlock to be obeyed in auto and guided modes when on the ground.
void Mode::make_safe_ground_handling(bool force_throttle_unlimited)
{
    if (force_throttle_unlimited) {
        // keep rotors turning 
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // spool down to ground idle
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    // aircraft is landed, integrator terms must be reset regardless of spool state
    attitude_control->reset_rate_controller_I_terms_smoothly();
 
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // reset yaw targets and rates during idle states
        attitude_control->reset_yaw_target_and_rate();
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // while transitioning though active states continue to operate normally
        break;
    }

    pos_control->relax_velocity_controller_xy();
    pos_control->update_xy_controller();
    pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
    pos_control->update_z_controller();
    // we may need to move this out
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
}

/*
  get a height above ground estimate for landing
 */
int32_t Mode::get_alt_above_ground_cm(void)
{
    int32_t alt_above_ground_cm;
    if (copter.get_rangefinder_height_interpolated_cm(alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }
    if (!pos_control->is_active_xy()) {
        return copter.current_loc.alt;
    }
    if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }

    // Assume the Earth is flat:
    return copter.current_loc.alt;
}

void Mode::land_run_vertical_control(bool pause_descent)
{
    float cmb_rate = 0;
    bool ignore_descent_limit = false;
    if (!pause_descent) {

        // do not ignore limits until we have slowed down for landing
        ignore_descent_limit = (MAX(g2.land_alt_low,100) > get_alt_above_ground_cm()) || copter.ap.land_complete_maybe;

        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            max_land_descent_velocity = pos_control->get_max_speed_down_cms();
        }

        // Don't speed up for landing.
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
        cmb_rate = sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

#if PRECISION_LANDING == ENABLED
        const bool navigating = pos_control->is_active_xy();
        bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired() && navigating;

        if (doing_precision_landing) {
            // prec landing is active
            Vector2f target_pos;
            float target_error_cm = 0.0f;
            if (copter.precland.get_target_position_cm(target_pos)) {
                const Vector2f current_pos = inertial_nav.get_position_xy_cm();
                // target is this many cm away from the vehicle
                target_error_cm = (target_pos - current_pos).length();
            }
            // check if we should descend or not
            const float max_horiz_pos_error_cm = copter.precland.get_max_xy_error_before_descending_cm();
            if (target_error_cm > max_horiz_pos_error_cm && !is_zero(max_horiz_pos_error_cm)) {
                // doing precland but too far away from the obstacle
                // do not descend
                cmb_rate = 0.0f;
            } else if (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm > 35.0f && copter.rangefinder_state.alt_cm < 200.0f) {
                // very close to the ground and doing prec land, lets slow down to make sure we land on target
                // compute desired descent velocity
                const float precland_acceptable_error_cm = 15.0f;
                const float precland_min_descent_speed_cms = 10.0f;
                const float max_descent_speed_cms = abs(g.land_speed)*0.5f;
                const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent_speed_cms/precland_acceptable_error_cm));
                cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms+land_slowdown);
            }
        }
#endif
    }

    // update altitude target and call position controller
    pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control->update_z_controller();
}

void Mode::land_run_horizontal_control()
{
    Vector2f vel_correction;
    float target_yaw_rate = 0;

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to reposition velocity
            // use half maximum acceleration as the maximum velocity to ensure aircraft will
            // stop from full reposition speed in less than 1 second.
            const float max_pilot_vel = wp_nav->get_wp_acceleration() * 0.5;
            vel_correction = get_pilot_desired_velocity(max_pilot_vel);

            // record if pilot has overridden roll or pitch
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // this variable will be updated if prec land target is in sight and pilot isn't trying to reposition the vehicle
    copter.ap.prec_land_active = false;
#if PRECISION_LANDING == ENABLED
    copter.ap.prec_land_active = !copter.ap.land_repo_active && copter.precland.target_acquired();
    // run precision landing
    if (copter.ap.prec_land_active) {
        Vector2f target_pos, target_vel;
        if (!copter.precland.get_target_position_cm(target_pos)) {
            target_pos = inertial_nav.get_position_xy_cm();
        }
         // get the velocity of the target
        copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

        Vector2f zero;
        Vector2p landing_pos = target_pos.topostype();
        // target vel will remain zero if landing target is stationary
        pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    }
#endif

    if (!copter.ap.prec_land_active) {
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel_correction, accel);
    }

    // run pos controller
    pos_control->update_xy_controller();
    Vector3f thrust_vector = pos_control->get_thrust_vector();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        const float attitude_limit_cd = linear_interpolate(700, copter.aparm.angle_max, get_alt_above_ground_cm(),
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        const float thrust_vector_max = sinf(radians(attitude_limit_cd * 0.01f)) * GRAVITY_MSS * 100.0f;
        const float thrust_vector_mag = thrust_vector.xy().length();
        if (thrust_vector_mag > thrust_vector_max) {
            float ratio = thrust_vector_max / thrust_vector_mag;
            thrust_vector.x *= ratio;
            thrust_vector.y *= ratio;

            // tell position controller we are applying an external limit
            pos_control->set_externally_limited_xy();
        }
    }

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(thrust_vector, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.yaw());
    }
}

// run normal or precision landing (if enabled)
// pause_descent is true if vehicle should not descend
void Mode::land_run_normal_or_precland(bool pause_descent)
{
#if PRECISION_LANDING == ENABLED
    if (pause_descent || !copter.precland.enabled()) {
        // we don't want to start descending immediately or prec land is disabled
        // in both cases just run simple land controllers
        land_run_horiz_and_vert_control(pause_descent);
    } else {
        // prec land is enabled and we have not paused descent
        // the state machine takes care of the entire prec landing procedure
        precland_run();
    }
#else
    land_run_horiz_and_vert_control(pause_descent);
#endif
}

#if PRECISION_LANDING == ENABLED
// Go towards a position commanded by prec land state machine in order to retry landing
// The passed in location is expected to be NED and in m
void Mode::precland_retry_position(const Vector3f &retry_pos)
{
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // allow user to take control during repositioning. Note: copied from land_run_horizontal_control()
        // To-Do: this code exists at several different places in slightly different forms and that should be fixed
        if (g.land_repositioning) {
            float target_roll = 0.0f;
            float target_pitch = 0.0f;
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                // this flag will be checked by prec land state machine later and any further landing retires will be cancelled
                copter.ap.land_repo_active = true;
            }
        }

        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    Vector3p retry_pos_NEU{retry_pos.x, retry_pos.y, retry_pos.z * -1.0f};
    // pos controller expects input in NEU cm's
    retry_pos_NEU = retry_pos_NEU * 100.0f;
    pos_control->input_pos_xyz(retry_pos_NEU, 0.0f, 1000.0f);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    const Vector3f thrust_vector{pos_control->get_thrust_vector()};

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(thrust_vector, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.yaw());
    }
}

// Run precland statemachine. This function should be called from any mode that wants to do precision landing.
// This handles everything from prec landing, to prec landing failures, to retries and failsafe measures
void Mode::precland_run()
{
    // if user is taking control, we will not run the statemachine, and simply land (may or may not be on target)
    if (!copter.ap.land_repo_active) {
        // This will get updated later to a retry pos if needed
        Vector3f retry_pos;

        switch (copter.precland_statemachine.update(retry_pos)) {
        case AC_PrecLand_StateMachine::Status::RETRYING:
            // we want to retry landing by going to another position
            precland_retry_position(retry_pos);
            break;

        case AC_PrecLand_StateMachine::Status::FAILSAFE: {
            // we have hit a failsafe. Failsafe can only mean two things, we either want to stop permanently till user takes over or land
            switch (copter.precland_statemachine.get_failsafe_actions()) {
            case AC_PrecLand_StateMachine::FailSafeAction::DESCEND:
                // descend normally, prec land target is definitely not in sight
                land_run_horiz_and_vert_control();
                break;
            case AC_PrecLand_StateMachine::FailSafeAction::HOLD_POS:
                // sending "true" in this argument will stop the descend
                land_run_horiz_and_vert_control(true);
                break;
            }
            break;
        }
        case AC_PrecLand_StateMachine::Status::ERROR:
            // should never happen, is certainly a bug. Report then descend
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            FALLTHROUGH;
        case AC_PrecLand_StateMachine::Status::DESCEND:
            // run land controller. This will descend towards the target if prec land target is in sight
            // else it will just descend vertically
            land_run_horiz_and_vert_control();
            break;
        }
    } else {
        // just land, since user has taken over controls, it does not make sense to run any retries or failsafe measures
        land_run_horiz_and_vert_control();
    }
}
#endif

float Mode::throttle_hover() const
{
    return motors->get_throttle_hover();
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1
// returns throttle output 0 to 1
float Mode::get_pilot_desired_throttle() const
{
    const float thr_mid = throttle_hover();
    int16_t throttle_control = channel_throttle->get_control_in();

    int16_t mid_stick = copter.get_throttle_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    } else {
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }

    const float expo = constrain_float(-(thr_mid-0.5f)/0.375f, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

float Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
#if AC_AVOID_ENABLED == ENABLED
    AP::ac_avoid()->adjust_velocity_z(pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), target_rate, G_Dt);
    return target_rate;
#else
    return target_rate;
#endif
}

// send output to the motors, can be overridden by subclasses
void Mode::output_to_motors()
{
    motors->output();
}

Mode::AltHoldModeState Mode::get_alt_hold_state(float target_climb_rate_cms)
{
    // Alt Hold State Machine Determination
    if (!motors->armed()) {
        // the aircraft should moved to a shut down state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // transition through states as aircraft spools down
        switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::SHUT_DOWN:
            return AltHold_MotorStopped;

        case AP_Motors::SpoolState::GROUND_IDLE:
            return AltHold_Landed_Ground_Idle;

        default:
            return AltHold_Landed_Pre_Takeoff;
        }

    } else if (takeoff.running() || takeoff.triggered(target_climb_rate_cms)) {
        // the aircraft is currently landed or taking off, asking for a positive climb rate and in THROTTLE_UNLIMITED
        // the aircraft should progress through the take off procedure
        return AltHold_Takeoff;

    } else if (!copter.ap.auto_armed || copter.ap.land_complete) {
        // the aircraft is armed and landed
        if (target_climb_rate_cms < 0.0f && !copter.ap.using_interlock) {
            // the aircraft should move to a ground idle state
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        } else {
            // the aircraft should prepare for imminent take off
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            // the aircraft is waiting in ground idle
            return AltHold_Landed_Ground_Idle;

        } else {
            // the aircraft can leave the ground at any time
            return AltHold_Landed_Pre_Takeoff;
        }

    } else {
        // the aircraft is in a flying state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        return AltHold_Flying;
    }
}

// transform pilot's yaw input into a desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Mode::get_pilot_desired_yaw_rate(float yaw_in)
{
    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return 0.0f;
    }

    // convert pilot input to the desired yaw rate
    return g2.command_model_pilot.get_rate() * 100.0 * input_expo(yaw_in, g2.command_model_pilot.get_expo());
}

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.
float Mode::get_pilot_desired_climb_rate(float throttle_control)
{
    return copter.get_pilot_desired_climb_rate(throttle_control);
}

float Mode::get_non_takeoff_throttle()
{
    return copter.get_non_takeoff_throttle();
}

void Mode::update_simple_mode(void) {
    copter.update_simple_mode();
}

bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return copter.set_mode(mode, reason);
}

void Mode::set_land_complete(bool b)
{
    return copter.set_land_complete(b);
}

GCS_Copter &Mode::gcs()
{
    return copter.gcs();
}

// set_throttle_takeoff - allows modes to tell throttle controller we
// are taking off so I terms can be cleared
void Mode::set_throttle_takeoff()
{
    // initialise the vertical position controller
    pos_control->init_z_controller();
}

uint16_t Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}
