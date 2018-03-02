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
    ap(copter.ap),
    takeoff_state(copter.takeoff_state),
    ekfGndSpdLimit(copter.ekfGndSpdLimit),
    ekfNavVelGainScaler(copter.ekfNavVelGainScaler),
#if FRAME_CONFIG == HELI_FRAME
    heli_flags(copter.heli_flags),
#endif
    auto_yaw_mode(copter.auto_yaw_mode)
{ };

// return the static controller object corresponding to supplied mode
Copter::Mode *Copter::mode_from_mode_num(const uint8_t mode)
{
    Copter::Mode *ret = nullptr;

    switch (mode) {
        case ACRO:
            ret = &mode_acro;
            break;

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

        case FLIP:
            ret = &mode_flip;
            break;

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

        case THROW:
            ret = &mode_throw;
            break;

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
    DataFlash.Log_Write_Mode(control_mode, reason);

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
    // Update EKF speed limit - used to limit speed when we are using optical flow
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

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
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
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
    takeoff_stop();

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

bool Copter::Mode::takeoff_triggered(const float target_climb_rate) const
{
    if (!ap.land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }
#if FRAME_CONFIG == HELI_FRAME
    if (!motors->rotor_runup_complete()) {
        // hold heli on the ground until rotor speed runup has finished
        return false;
    }
#endif
    return true;
}

void Copter::Mode::zero_throttle_and_relax_ac()
{
#if FRAME_CONFIG == HELI_FRAME
    // Helicopters always stabilize roll/pitch/yaw
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f, get_smoothing_gain());
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
#else
    motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    // multicopters do not stabilize roll/pitch/yaw when disarmed
    attitude_control->set_throttle_out_unstabilized(0.0f, true, copter.g.throttle_filt);
#endif
}


// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.
void Copter::Mode::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    copter.get_pilot_desired_lean_angles(roll_in, pitch_in, roll_out, pitch_out, angle_max);
}

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

float Copter::Mode::get_smoothing_gain() {
    return copter.get_smoothing_gain();
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

void Copter::Mode::Log_Write_Event(uint8_t id)
{
    return copter.Log_Write_Event(id);
}

void Copter::Mode::set_throttle_takeoff()
{
    return copter.set_throttle_takeoff();
}

void Copter::Mode::set_auto_yaw_mode(uint8_t yaw_mode)
{
    return copter.set_auto_yaw_mode(yaw_mode);
}

void Copter::Mode::set_auto_yaw_rate(float turn_rate_cds)
{
    return copter.set_auto_yaw_rate(turn_rate_cds);
}

void Copter::Mode::set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle)
{
    return copter.set_auto_yaw_look_at_heading(angle_deg, turn_rate_dps, direction, relative_angle);
}

void Copter::Mode::takeoff_timer_start(float alt_cm)
{
    return copter.takeoff_timer_start(alt_cm);
}

void Copter::Mode::takeoff_stop()
{
    return copter.takeoff_stop();
}

void Copter::Mode::takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate)
{
    return copter.takeoff_get_climb_rates(pilot_climb_rate, takeoff_climb_rate);
}

float Copter::Mode::get_auto_heading()
{
    return copter.get_auto_heading();
}

float Copter::Mode::get_auto_yaw_rate_cds()
{
    return copter.get_auto_yaw_rate_cds();
}

float Copter::Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
    return copter.get_avoidance_adjusted_climbrate(target_rate);
}

uint16_t Copter::Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}
