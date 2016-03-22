#include "Copter.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(control_mode_t mode, mode_reason_t reason)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        prev_control_mode = control_mode;
        prev_control_mode_reason = control_mode_reason;

        control_mode_reason = reason;
        return true;
    }

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !mode_has_manual_throttle(mode) && !motors->rotor_runup_complete()){
        goto failed;
    }
#endif

    // for transition, we assume no flightmode object will be used in
    // the new mode, and if the transition fails we reset the
    // flightmode to the previous value
    Copter::FlightMode* old_flightmode = flightmode;
    flightmode = nullptr;

    switch (mode) {
        case ACRO:
                success = flightmode_acro.init(ignore_checks);
                if (success) {
                    flightmode = &flightmode_acro;
                }
            break;

        case STABILIZE:
                success = flightmode_stabilize.init(ignore_checks);
                if (success) {
                    flightmode = &flightmode_stabilize;
                }
            break;

        case ALT_HOLD:
            success = flightmode_althold.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_althold;
            }
            break;

        case AUTO:
            success = flightmode_auto.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_auto;
            }
            break;

        case CIRCLE:
            success = flightmode_circle.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_circle;
            }
            break;

        case LOITER:
            success = flightmode_loiter.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_loiter;
            }
            break;

        case GUIDED:
            success = flightmode_guided.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_guided;
            }
            break;

        case LAND:
            success = flightmode_land.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_land;
            }
            break;

        case RTL:
            success = flightmode_rtl.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_rtl;
            }
            break;

        case DRIFT:
            success = flightmode_drift.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_drift;
            }
            break;

        case SPORT:
            success = flightmode_sport.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_sport;
            }
            break;

        case FLIP:
            success = flightmode_flip.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_flip;
            }
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            success = flightmode_autotune.init(ignore_checks);
            if (success) {
                flightmode = &flightmode_autotune;
            }
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            success = poshold_init(ignore_checks);
            break;
#endif

        case BRAKE:
            success = brake_init(ignore_checks);
            break;

        case THROW:
            success = throw_init(ignore_checks);
            break;

        case AVOID_ADSB:
            success = avoid_adsb_init(ignore_checks);
            break;

        case GUIDED_NOGPS:
            success = guided_nogps_init(ignore_checks);
            break;

        case SMART_RTL:
            success = smart_rtl_init(ignore_checks);
            break;

        default:
            success = false;
            break;
    }

#if FRAME_CONFIG == HELI_FRAME
failed:
#endif
    
    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        
        prev_control_mode = control_mode;
        prev_control_mode_reason = control_mode_reason;

        control_mode = mode;
        control_mode_reason = reason;
        DataFlash.Log_Write_Mode(control_mode, control_mode_reason);

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
        
    } else {
        flightmode = old_flightmode;
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed");
    }

    // update notify object
    if (success) {
        notify_flight_mode();
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
    // Update EKF speed limit - used to limit speed when we are using optical flow
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    if (flightmode != nullptr) {
        flightmode->run();
    }

    switch (control_mode) {

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            poshold_run();
            break;
#endif

        case BRAKE:
            brake_run();
            break;

        case THROW:
            throw_run();
            break;

        case AVOID_ADSB:
            avoid_adsb_run();
            break;

        case GUIDED_NOGPS:
            guided_nogps_run();
            break;


        case SMART_RTL:
            smart_rtl_run();
            break;

        default:
            break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode)
{
#if AUTOTUNE_ENABLED == ENABLED
    if (old_control_mode == AUTOTUNE) {
        flightmode_autotune.autotune_stop();
    }
#endif

    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (mode_has_manual_throttle(old_control_mode) && !mode_has_manual_throttle(new_control_mode) && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    // cancel any takeoffs in progress
    takeoff_stop();

    // call smart_rtl cleanup
    if (old_control_mode == SMART_RTL) {
        smart_rtl_exit();
    }

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_control_mode == ACRO) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    // if we are changing from a mode that did not use manual throttle,
    // stab col ramp value should be pre-loaded to the correct value to avoid a twitch
    // heli_stab_col_ramp should really only be active switching between Stabilize and Acro modes
    if (!mode_has_manual_throttle(old_control_mode)){
        if (new_control_mode == STABILIZE){
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_control_mode == ACRO){
            input_manager.set_stab_col_ramp(0.0);
        }
    }
#endif //HELI_FRAME
}

// returns true or false whether current control mode requires GPS
bool Copter::mode_requires_GPS()
{
    if (flightmode != nullptr) {
        return flightmode->requires_GPS();
    }
    switch (control_mode) {
        case SMART_RTL:
        case POSHOLD:
        case BRAKE:
        case AVOID_ADSB:
        case THROW:
            return true;
        default:
            return false;
    }
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
bool Copter::mode_has_manual_throttle(control_mode_t mode)
{
    switch (mode) {
        case ACRO:
        case STABILIZE:
            return true;
        default:
            return false;
    }
}

// mode_allows_arming - returns true if vehicle can be armed in the current mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
bool Copter::mode_allows_arming(bool arming_from_gcs)
{
    if (flightmode != nullptr) {
        return flightmode->allows_arming(arming_from_gcs);
    }
    control_mode_t mode = control_mode;
    if (mode_has_manual_throttle(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || mode == DRIFT || mode == SPORT || mode == THROW || (arming_from_gcs && (mode == GUIDED || mode == GUIDED_NOGPS))) {
        return true;
    }
    return false;
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode()
{
    AP_Notify::flags.flight_mode = control_mode;

    if (flightmode != nullptr) {
        AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
        notify.set_flight_mode_str(flightmode->name4());
        return;
    }
    switch (control_mode) {
        case AVOID_ADSB:
        case GUIDED_NOGPS:
        case SMART_RTL:
            // autopilot modes
            AP_Notify::flags.autopilot_mode = true;
            break;
    default:
            // all other are manual flight modes
            AP_Notify::flags.autopilot_mode = false;
            break;
    }

    // set flight mode string
    switch (control_mode) {
        case POSHOLD:
            notify.set_flight_mode_str("PHLD");
            break;
        case BRAKE:
            notify.set_flight_mode_str("BRAK");
            break;
        case THROW:
            notify.set_flight_mode_str("THRW");
            break;
        case AVOID_ADSB:
            notify.set_flight_mode_str("AVOI");
            break;
        case GUIDED_NOGPS:
            notify.set_flight_mode_str("GNGP");
            break;
        default:
            notify.set_flight_mode_str("----");
            break;
    }
}
