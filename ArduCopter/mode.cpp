#include "Copter.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

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

        case AUTO:
            ret = &mode_auto;
            break;

        case CIRCLE:
            ret = &mode_circle;
            break;

        case LOITER:
            ret = &mode_loiter;
            break;

        case GUIDED:
            ret = &mode_guided;
            break;

        case LAND:
            ret = &mode_land;
            break;

        case RTL:
            ret = &mode_rtl;
            break;

        case DRIFT:
            ret = &mode_drift;
            break;

        case SPORT:
            ret = &mode_sport;
            break;

        case FLIP:
            ret = &mode_flip;
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            ret = &mode_autotune;
            break;
#endif

        case POSHOLD:
            ret = &mode_poshold;
            break;

        case BRAKE:
            ret = &mode_brake;
            break;

        case THROW:
            ret = &mode_throw;
            break;

        case AVOID_ADSB:
            ret = &mode_avoid_adsb;
            break;

        case GUIDED_NOGPS:
            ret = &mode_guided_nogps;
            break;

        case SMART_RTL:
            ret = &mode_smartrtl;
            break;

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

    if (!new_flightmode->init(ignore_checks)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed");
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
        return false;
    }

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !new_flightmode->has_manual_throttle() && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // update flight mode
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
    DataFlash.Log_Write_Mode(control_mode);

    adsb.set_is_auto_mode((mode == AUTO) || (mode == RTL) || (mode == GUIDED));

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
    if (old_flightmode == &mode_auto) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    // cancel any takeoffs in progress
    takeoff_stop();

    // call smart_rtl cleanup
    if (old_flightmode == &mode_smartrtl) {
        mode_smartrtl.exit();
    }

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
    notify.set_flight_mode_str(flightmode->name4());
}

void Copter::Mode::update_navigation()
{
    // run autopilot to make high level decisions about control modes
    run_autopilot();
}
