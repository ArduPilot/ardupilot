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

    switch (mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_acro_init(ignore_checks);
            #else
                success = acro_init(ignore_checks);
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_stabilize_init(ignore_checks);
            #else
                success = stabilize_init(ignore_checks);
            #endif
            break;

        case ALT_HOLD:
            success = althold_init(ignore_checks);
            break;

        case AUTO:
            success = auto_init(ignore_checks);
            break;

        case CIRCLE:
            success = circle_init(ignore_checks);
            break;

        case LOITER:
            success = loiter_init(ignore_checks);
            break;

        case GUIDED:
            success = guided_init(ignore_checks);
            break;

        case LAND:
            success = land_init(ignore_checks);
            break;

        case RTL:
            success = rtl_init(ignore_checks);
            break;

        case DRIFT:
            success = drift_init(ignore_checks);
            break;

        case SPORT:
            success = sport_init(ignore_checks);
            break;

        case FLIP:
            success = flip_init(ignore_checks);
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            success = autotune_init(ignore_checks);
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

        default:
            success = false;
            break;
    }

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
        
    } else {
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
        gcs_send_text(MAV_SEVERITY_WARNING,"Flight mode change failed");
    }

    // update notify object
    if (success) {
        notify_flight_mode(control_mode);
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

    switch (control_mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                heli_acro_run();
            #else
                acro_run();
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                heli_stabilize_run();
            #else
                stabilize_run();
            #endif
            break;

        case ALT_HOLD:
            althold_run();
            break;

        case AUTO:
            auto_run();
            break;

        case CIRCLE:
            circle_run();
            break;

        case LOITER:
            loiter_run();
            break;

        case GUIDED:
            guided_run();
            break;

        case LAND:
            land_run();
            break;

        case RTL:
            rtl_run();
            break;

        case DRIFT:
            drift_run();
            break;

        case SPORT:
            sport_run();
            break;

        case FLIP:
            flip_run();
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            autotune_run();
            break;
#endif

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

        default:
            break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode)
{
#if AUTOTUNE_ENABLED == ENABLED
    if (old_control_mode == AUTOTUNE) {
        autotune_stop();
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

// returns true or false whether mode requires GPS
bool Copter::mode_requires_GPS(control_mode_t mode)
{
    switch (mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
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

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
bool Copter::mode_allows_arming(control_mode_t mode, bool arming_from_gcs)
{
    if (mode_has_manual_throttle(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || mode == DRIFT || mode == SPORT || mode == THROW || (arming_from_gcs && (mode == GUIDED || mode == GUIDED_NOGPS))) {
        return true;
    }
    return false;
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode(control_mode_t mode)
{
    AP_Notify::flags.flight_mode = mode;

    switch (mode) {
        case AUTO:
        case GUIDED:
        case RTL:
        case CIRCLE:
        case AVOID_ADSB:
        case GUIDED_NOGPS:
        case LAND:
            // autopilot modes
            AP_Notify::flags.autopilot_mode = true;
            break;
        default:
            // all other are manual flight modes
            AP_Notify::flags.autopilot_mode = false;
            break;
    }

    // set flight mode string
    switch (mode) {
        case STABILIZE:
            notify.set_flight_mode_str("STAB");
            break;
        case ACRO:
            notify.set_flight_mode_str("ACRO");
            break;
        case ALT_HOLD:
            notify.set_flight_mode_str("ALTH");
            break;
        case AUTO:
            notify.set_flight_mode_str("AUTO");
            break;
        case GUIDED:
            notify.set_flight_mode_str("GUID");
            break;
        case LOITER:
            notify.set_flight_mode_str("LOIT");
            break;
        case RTL:
            notify.set_flight_mode_str("RTL ");
            break;
        case CIRCLE:
            notify.set_flight_mode_str("CIRC");
            break;
        case LAND:
            notify.set_flight_mode_str("LAND");
            break;
        case DRIFT:
            notify.set_flight_mode_str("DRIF");
            break;
        case SPORT:
            notify.set_flight_mode_str("SPRT");
            break;
        case FLIP:
            notify.set_flight_mode_str("FLIP");
            break;
        case AUTOTUNE:
            notify.set_flight_mode_str("ATUN");
            break;
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

//
// print_flight_mode - prints flight mode to serial port.
//
void Copter::print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->printf("STABILIZE");
        break;
    case ACRO:
        port->printf("ACRO");
        break;
    case ALT_HOLD:
        port->printf("ALT_HOLD");
        break;
    case AUTO:
        port->printf("AUTO");
        break;
    case GUIDED:
        port->printf("GUIDED");
        break;
    case LOITER:
        port->printf("LOITER");
        break;
    case RTL:
        port->printf("RTL");
        break;
    case CIRCLE:
        port->printf("CIRCLE");
        break;
    case LAND:
        port->printf("LAND");
        break;
    case DRIFT:
        port->printf("DRIFT");
        break;
    case SPORT:
        port->printf("SPORT");
        break;
    case FLIP:
        port->printf("FLIP");
        break;
    case AUTOTUNE:
        port->printf("AUTOTUNE");
        break;
    case POSHOLD:
        port->printf("POSHOLD");
        break;
    case BRAKE:
        port->printf("BRAKE");
        break;
    case THROW:
        port->printf("THROW");
        break;
    case AVOID_ADSB:
        port->printf("AVOID_ADSB");
        break;
    case GUIDED_NOGPS:
        port->printf("GUIDED_NOGPS");
        break;
    default:
        port->printf("Mode(%u)", (unsigned)mode);
        break;
    }
}

