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

#if ADSB_ENABLED == ENABLED
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

#if OPTFLOW == ENABLED
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

        default:
            break;
    }

    return ret;
}


// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(Mode::Number mode, ModeReason reason)
{

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Mode *new_flightmode = mode_from_mode_num((Mode::Number)mode);
    if (new_flightmode == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING,"No such mode");
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
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
            gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
            AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
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
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: throttle too high");
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }
#endif

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !copter.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!new_flightmode->init(ignore_checks)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // store previous flight mode (only used by tradeheli's autorotation)
    prev_control_mode = control_mode;

    // update flight mode
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
    logger.Write_Mode((uint8_t)control_mode, reason);
    gcs().send_message(MSG_HEARTBEAT);

#if ADSB_ENABLED == ENABLED
    adsb.set_is_auto_mode((mode == Mode::Number::AUTO) || (mode == Mode::Number::RTL) || (mode == Mode::Number::GUIDED));
#endif

#if AC_FENCE == ENABLED
    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    fence.manual_recovery_start();
#endif

#if CAMERA == ENABLED
    camera.set_is_auto_mode(control_mode == Mode::Number::AUTO);
#endif

    // update notify object
    notify_flight_mode();

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
#if AUTOTUNE_ENABLED == ENABLED
    if (old_flightmode == &mode_autotune) {
        mode_autotune.stop();
    }
#endif

    // stop mission when we leave auto mode
#if MODE_AUTO_ENABLED == ENABLED
    if (old_flightmode == &mode_auto) {
        if (mode_auto.mission.state() == AP_Mission::MISSION_RUNNING) {
            mode_auto.mission.stop();
        }
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
    }
#endif

    // smooth throttle transition when switching from manual to automatic flight modes
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    // cancel any takeoffs in progress
    old_flightmode->takeoff_stop();

#if MODE_SMARTRTL_ENABLED == ENABLED
    // call smart_rtl cleanup
    if (old_flightmode == &mode_smartrtl) {
        mode_smartrtl.exit();
    }
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
    if (old_flightmode == &mode_follow) {
        mode_follow.exit();
    }
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
    if (old_flightmode == &mode_zigzag) {
        mode_zigzag.exit();
    }
#endif

#if MODE_ACRO_ENABLED == ENABLED
    if (old_flightmode == &mode_acro) {
        mode_acro.exit();
    }
#endif

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
    AP_Notify::flags.flight_mode = (uint8_t)control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}

void Mode::update_navigation()
{
    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Mode::get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit) const
{
    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        roll_out = 0;
        pitch_out = 0;
        return;
    }
    // fetch roll and pitch inputs
    roll_out = channel_roll->get_control_in();
    pitch_out = channel_pitch->get_control_in();

    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 1000.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));

    // roll_out and pitch_out are returned
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

void Mode::make_safe_spool_down()
{
    // command aircraft to initiate the shutdown process
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    switch (motors->get_spool_state()) {

    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // relax controllers during idle states
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // while transitioning though active states continue to operate normally
        break;
    }

    pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
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
    if (!pause_descent) {
        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            max_land_descent_velocity = pos_control->get_max_speed_down();
        }

        // Don't speed up for landing.
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
        cmb_rate = AC_AttitudeControl::sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z(), G_Dt);

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

#if PRECISION_LANDING == ENABLED
        const bool navigating = pos_control->is_active_xy();
        bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired() && navigating;

        if (doing_precision_landing && copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm > 35.0f && copter.rangefinder_state.alt_cm < 200.0f) {
            // compute desired velocity
            const float precland_acceptable_error = 15.0f;
            const float precland_min_descent_speed = 10.0f;

            float max_descent_speed = abs(g.land_speed)*0.5f;
            float land_slowdown = MAX(0.0f, pos_control->get_horizontal_error()*(max_descent_speed/precland_acceptable_error));
            cmb_rate = MIN(-precland_min_descent_speed, -max_descent_speed+land_slowdown);
        }
#endif
    }

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control->update_z_controller();
}

void Mode::land_run_horizontal_control()
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
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

            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

#if PRECISION_LANDING == ENABLED
    bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired();
    // run precision landing
    if (doing_precision_landing) {
        Vector2f target_pos, target_vel_rel;
        if (!copter.precland.get_target_position_cm(target_pos)) {
            target_pos.x = inertial_nav.get_position().x;
            target_pos.y = inertial_nav.get_position().y;
        }
        if (!copter.precland.get_target_velocity_relative_cms(target_vel_rel)) {
            target_vel_rel.x = -inertial_nav.get_velocity().x;
            target_vel_rel.y = -inertial_nav.get_velocity().y;
        }
        pos_control->set_xy_target(target_pos.x, target_pos.y);
        pos_control->override_vehicle_velocity_xy(-target_vel_rel);
    }
#endif

    // process roll, pitch inputs
    loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

    // run loiter controller
    loiter_nav->update();

    float nav_roll  = loiter_nav->get_roll();
    float nav_pitch = loiter_nav->get_pitch();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        float attitude_limit_cd = linear_interpolate(700, copter.aparm.angle_max, get_alt_above_ground_cm(),
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        float total_angle_cd = norm(nav_roll, nav_pitch);
        if (total_angle_cd > attitude_limit_cd) {
            float ratio = attitude_limit_cd / total_angle_cd;
            nav_roll *= ratio;
            nav_pitch *= ratio;

            // tell position controller we are applying an external limit
            pos_control->set_limit_accel_xy();
        }
    }

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(nav_roll, nav_pitch, auto_yaw.yaw(), true);
    }
}

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

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.
float Mode::get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    return copter.get_pilot_desired_yaw_rate(stick_angle);
}

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
    // tell position controller to reset alt target and reset I terms
    pos_control->init_takeoff();
}

float Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
    return copter.get_avoidance_adjusted_climbrate(target_rate);
}

uint16_t Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}
