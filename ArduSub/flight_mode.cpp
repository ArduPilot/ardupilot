#include "Sub.h"

// change flight mode and perform any necessary initialisation
// returns true if mode was successfully set
bool Sub::set_mode(control_mode_t mode, ModeReason reason)
{
    // boolean to record if flight mode could be set
    bool success = false;

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        prev_control_mode = control_mode;

        control_mode_reason = reason;
        return true;
    }

    switch (mode) {
    case ACRO:
        success = acro_init();
        break;

    case STABILIZE:
        success = stabilize_init();
        break;

    case ALT_HOLD:
        success = althold_init();
        break;

    case AUTO:
        success = auto_init();
        break;

    case CIRCLE:
        success = circle_init();
        break;

    case GUIDED:
        success = guided_init();
        break;

    case SURFACE:
        success = surface_init();
        break;

#if POSHOLD_ENABLED == ENABLED
    case POSHOLD:
        success = poshold_init();
        break;
#endif

    case MANUAL:
        success = manual_init();
        break;

    case MOTOR_DETECT:
        success = motordetect_init();
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

        control_mode = mode;
        control_mode_reason = reason;
        logger.Write_Mode(control_mode, control_mode_reason);
        gcs().send_message(MSG_HEARTBEAT);

        // update notify object
        notify_flight_mode(control_mode);

#if AP_CAMERA_ENABLED
        camera.set_is_auto_mode(control_mode == AUTO);
#endif

#if AP_FENCE_ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
#endif
    } else {
        // Log error that we failed to enter desired flight mode
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
    }

    // return success or failure
    return success;
}

bool Sub::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(control_mode_t) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return sub.set_mode((control_mode_t)new_mode, reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Sub::update_flight_mode()
{
    switch (control_mode) {
    case ACRO:
        acro_run();
        break;

    case STABILIZE:
        stabilize_run();
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

    case GUIDED:
        guided_run();
        break;

    case SURFACE:
        surface_run();
        break;

#if POSHOLD_ENABLED == ENABLED
    case POSHOLD:
        poshold_run();
        break;
#endif

    case MANUAL:
        manual_run();
        break;

    case MOTOR_DETECT:
        motordetect_run();
        break;

    default:
        break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Sub::exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode)
{
    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
    }
}

// returns true or false whether mode requires GPS
bool Sub::mode_requires_GPS(control_mode_t mode)
{
    switch (mode) {
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
        return true;
    default:
        return false;
    }

    return false;
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
bool Sub::mode_has_manual_throttle(control_mode_t mode)
{
    switch (mode) {
    case ACRO:
    case STABILIZE:
    case MANUAL:
        return true;
    default:
        return false;
    }

    return false;
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
bool Sub::mode_allows_arming(control_mode_t mode, bool arming_from_gcs)
{
    return (mode_has_manual_throttle(mode)
        || mode == ALT_HOLD
        || mode == POSHOLD
        || (arming_from_gcs&& mode == GUIDED)
    );
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
void Sub::notify_flight_mode(control_mode_t mode)
{
    switch (mode) {
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case SURFACE:
        // autopilot modes
        AP_Notify::flags.autopilot_mode = true;
        break;
    default:
        // all other are manual flight modes
        AP_Notify::flags.autopilot_mode = false;
        break;
    }
}
