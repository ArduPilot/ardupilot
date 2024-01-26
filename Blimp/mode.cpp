#include "Blimp.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

#include <AP_Vehicle/AP_MultiCopter.h>

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(blimp.g),
    g2(blimp.g2),
    inertial_nav(blimp.inertial_nav),
    ahrs(blimp.ahrs),
    motors(blimp.motors),
    loiter(blimp.loiter),
    channel_right(blimp.channel_right),
    channel_front(blimp.channel_front),
    channel_up(blimp.channel_up),
    channel_yaw(blimp.channel_yaw),
    G_Dt(blimp.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Blimp::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
    case Mode::Number::LAND:
        ret = &mode_land;
        break;
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::VELOCITY:
        ret = &mode_velocity;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    default:
        break;
    }

    return ret;
}


// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// Some modes can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Blimp::set_mode(Mode::Number mode, ModeReason reason)
{

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Mode *new_flightmode = mode_from_mode_num((Mode::Number)mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !blimp.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !blimp.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s need alt estimate", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!new_flightmode->init(ignore_checks)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
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
#if HAL_LOGGING_ENABLED
    logger.Write_Mode((uint8_t)control_mode, reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);

    // update notify object
    notify_flight_mode();

    // return success
    return true;
}

bool Blimp::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && blimp.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return blimp.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Blimp::update_flight_mode()
{
    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Blimp::exit_mode(Mode *&old_flightmode,
                      Mode *&new_flightmode) {}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Blimp::notify_flight_mode()
{
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}

void Mode::update_navigation()
{
    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// returns desired thrust/acceleration
void Mode::get_pilot_input(Vector3f &pilot, float &yaw)
{
    // throttle failsafe check
    if (blimp.failsafe.radio || !blimp.ap.rc_receiver_present) {
        pilot.y = 0;
        pilot.x = 0;
        pilot.z = 0;
        yaw = 0;
        return;
    }
    // fetch pilot inputs
    pilot.y = channel_right->get_control_in() / float(RC_SCALE);
    pilot.x = channel_front->get_control_in() / float(RC_SCALE);
    //TODO: need to make this channel_up instead, and then have it .negative. before being sent to pilot.z -> this is "throttle" channel, so higher = up.
    pilot.z = -channel_up->get_control_in() / float(RC_SCALE);
    yaw = channel_yaw->get_control_in() / float(RC_SCALE);
}

bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !blimp.ap.auto_armed || blimp.ap.land_complete) {
        return true;
    }
    return false;
}

bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return blimp.set_mode(mode, reason);
}

GCS_Blimp &Mode::gcs()
{
    return blimp.gcs();
}
