#include "Copter.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

/*
  constructor for Mode object
 */
Copter::Mode::Mode(void) :
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
    G_Dt(copter.G_Dt),
    ap(copter.ap)
{ };

float Copter::Mode::auto_takeoff_no_nav_alt_cm = 0;

// return the static controller object corresponding to supplied mode
Copter::Mode *Copter::mode_from_mode_num(const uint8_t mode)
{
    Copter::Mode *ret = nullptr;

    switch (mode) {
#if MODE_ACRO_ENABLED == ENABLED
        case ACRO:
            ret = &mode_acro;
            break;
#endif

        case STABILIZE:
            ret = &mode_stabilize;
            break;

        case ALT_HOLD:
            ret = &mode_althold;
            break;

#if MODE_AUTO_ENABLED == ENABLED
        case AUTO:
            ret = &mode_auto;
            break;
#endif

#if MODE_CIRCLE_ENABLED == ENABLED
        case CIRCLE:
            ret = &mode_circle;
            break;
#endif

#if MODE_LOITER_ENABLED == ENABLED
        case LOITER:
            ret = &mode_loiter;
            break;
#endif

#if MODE_GUIDED_ENABLED == ENABLED
        case GUIDED:
            ret = &mode_guided;
            break;
#endif

        case LAND:
            ret = &mode_land;
            break;

#if MODE_RTL_ENABLED == ENABLED
        case RTL:
            ret = &mode_rtl;
            break;
#endif

#if MODE_DRIFT_ENABLED == ENABLED
        case DRIFT:
            ret = &mode_drift;
            break;
#endif

#if MODE_SPORT_ENABLED == ENABLED
        case SPORT:
            ret = &mode_sport;
            break;
#endif

#if MODE_FLIP_ENABLED == ENABLED
        case FLIP:
            ret = &mode_flip;
            break;
#endif

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            ret = &mode_autotune;
            break;
#endif

#if MODE_POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            ret = &mode_poshold;
            break;
#endif

#if MODE_BRAKE_ENABLED == ENABLED
        case BRAKE:
            ret = &mode_brake;
            break;
#endif

#if MODE_THROW_ENABLED == ENABLED
        case THROW:
            ret = &mode_throw;
            break;
#endif

#if ADSB_ENABLED == ENABLED
        case AVOID_ADSB:
            ret = &mode_avoid_adsb;
            break;
#endif

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED
        case GUIDED_NOGPS:
            ret = &mode_guided_nogps;
            break;
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
        case SMART_RTL:
            ret = &mode_smartrtl;
            break;
#endif

#if OPTFLOW == ENABLED
        case FLOWHOLD:
            ret = (Copter::Mode *)g2.mode_flowhold_ptr;
            break;
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
        case FOLLOW:
            ret = &mode_follow;
            break;
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
        case ZIGZAG:
            ret = &mode_zigzag;
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
bool Copter::set_mode(control_mode_t mode, mode_reason_t reason)
{

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Copter::Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING,"No such mode");
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
        return false;
    }

    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !new_flightmode->has_manual_throttle() && !motors->rotor_runup_complete()){
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed");
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
        return false;
    }
#endif

    if (!new_flightmode->init(ignore_checks)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed");
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // update flight mode
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
    logger.Write_Mode(control_mode, reason);

#if ADSB_ENABLED == ENABLED
    adsb.set_is_auto_mode((mode == AUTO) || (mode == RTL) || (mode == GUIDED));
#endif

#if AC_FENCE == ENABLED
    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    fence.manual_recovery_start();
#endif

#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.update_control_mode(control_mode);
#endif
#if DEVO_TELEM_ENABLED == ENABLED
    devo_telemetry.update_control_mode(control_mode);
#endif

#if CAMERA == ENABLED
    camera.set_is_auto_mode(control_mode == AUTO);
#endif

    // update notify object
    notify_flight_mode();

    // return success
    return true;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
    target_rangefinder_alt_used = false;

    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(Copter::Mode *&old_flightmode,
                       Copter::Mode *&new_flightmode)
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
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
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
    AP_Notify::flags.flight_mode = control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}

void Copter::Mode::update_navigation()
{
    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Copter::Mode::get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit) const
{
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

bool Copter::Mode::_TakeOff::triggered(const float target_climb_rate) const
{
    if (!copter.ap.land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }
#if FRAME_CONFIG == HELI_FRAME
    if (!copter.motors->rotor_runup_complete()) {
        // hold heli on the ground until rotor speed runup has finished
        return false;
    }
#endif
    return true;
}

void Copter::Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    if (spool_up) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
    }
#if FRAME_CONFIG == HELI_FRAME
    // Helicopters always stabilize roll/pitch/yaw
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
#else
    // multicopters do not stabilize roll/pitch/yaw when disarmed
    attitude_control->set_throttle_out_unstabilized(0.0f, true, copter.g.throttle_filt);
#endif
}

/*
  get a height above ground estimate for landing
 */
int32_t Copter::Mode::get_alt_above_ground(void)
{
    int32_t alt_above_ground;
    if (copter.rangefinder_alt_ok()) {
        alt_above_ground = copter.rangefinder_state.alt_cm_filt.get();
    } else {
        bool navigating = pos_control->is_active_xy();
        if (!navigating || !copter.current_loc.get_alt_cm(Location::ALT_FRAME_ABOVE_TERRAIN, alt_above_ground)) {
            alt_above_ground = copter.current_loc.alt;
        }
    }
    return alt_above_ground;
}

void Copter::Mode::land_run_vertical_control(bool pause_descent)
{
#if PRECISION_LANDING == ENABLED
    const bool navigating = pos_control->is_active_xy();
    bool doing_precision_landing = !ap.land_repo_active && copter.precland.target_acquired() && navigating;
#else
    bool doing_precision_landing = false;
#endif

    // compute desired velocity
    const float precland_acceptable_error = 15.0f;
    const float precland_min_descent_speed = 10.0f;
    const int32_t alt_above_ground = get_alt_above_ground();

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
        cmb_rate = AC_AttitudeControl::sqrt_controller(MAX(g2.land_alt_low,100)-alt_above_ground, pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z(), G_Dt);

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

        if (doing_precision_landing && copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm > 35.0f && copter.rangefinder_state.alt_cm < 200.0f) {
            float max_descent_speed = abs(g.land_speed)*0.5f;
            float land_slowdown = MAX(0.0f, pos_control->get_horizontal_error()*(max_descent_speed/precland_acceptable_error));
            cmb_rate = MIN(-precland_min_descent_speed, -max_descent_speed+land_slowdown);
        }
    }

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control->update_z_controller();
}

void Copter::Mode::land_run_horizontal_control()
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            copter.Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

            // record if pilot has overriden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!ap.land_repo_active) {
                    copter.Log_Write_Event(DATA_LAND_REPO_ACTIVE);
                }
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

#if PRECISION_LANDING == ENABLED
    bool doing_precision_landing = !ap.land_repo_active && copter.precland.target_acquired();
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

    int32_t nav_roll  = loiter_nav->get_roll();
    int32_t nav_pitch = loiter_nav->get_pitch();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        const int alt_above_ground = get_alt_above_ground();
        float attitude_limit_cd = linear_interpolate(700, copter.aparm.angle_max, alt_above_ground,
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
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate);
}

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.

float Copter::Mode::get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt)
{
    return copter.get_surface_tracking_climb_rate(target_rate, current_alt_target, dt);
}

float Copter::Mode::get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    return copter.get_pilot_desired_yaw_rate(stick_angle);
}

float Copter::Mode::get_pilot_desired_climb_rate(float throttle_control)
{
    return copter.get_pilot_desired_climb_rate(throttle_control);
}

float Copter::Mode::get_pilot_desired_throttle(int16_t throttle_control, float thr_mid)
{
    return copter.get_pilot_desired_throttle(throttle_control, thr_mid);
}

float Copter::Mode::get_non_takeoff_throttle()
{
    return copter.get_non_takeoff_throttle();
}

void Copter::Mode::update_simple_mode(void) {
    copter.update_simple_mode();
}

bool Copter::Mode::set_mode(control_mode_t mode, mode_reason_t reason)
{
    return copter.set_mode(mode, reason);
}

void Copter::Mode::set_land_complete(bool b)
{
    return copter.set_land_complete(b);
}

GCS_Copter &Copter::Mode::gcs()
{
    return copter.gcs();
}

void Copter::Mode::Log_Write_Event(Log_Event id)
{
    return copter.logger.Write_Event(id);
}

void Copter::Mode::set_throttle_takeoff()
{
    return copter.set_throttle_takeoff();
}

float Copter::Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
    return copter.get_avoidance_adjusted_climbrate(target_rate);
}

uint16_t Copter::Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}
