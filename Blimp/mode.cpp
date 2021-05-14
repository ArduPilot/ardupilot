#include "Blimp.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(blimp.g),
    g2(blimp.g2),
    inertial_nav(blimp.inertial_nav),
    ahrs(blimp.ahrs),
    motors(blimp.motors),
    channel_right(blimp.channel_right),
    channel_front(blimp.channel_front),
    channel_down(blimp.channel_down),
    channel_yaw(blimp.channel_yaw),
    G_Dt(blimp.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Blimp::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::LAND:
        ret = &mode_land;
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
bool Blimp::set_mode(Mode::Number mode, ModeReason reason)
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

    // ensure vehicle doesn't leap off the ground if a user switches
    // into a manual throttle mode from a non-manual-throttle mode
    // (e.g. user arms in guided, raises throttle to 1300 (not enough to
    // trigger auto takeoff), then switches into manual):
    bool user_throttle = new_flightmode->has_manual_throttle();
    if (!ignore_checks &&
        ap.land_complete &&
        user_throttle &&
        !blimp.flightmode->has_manual_throttle() &&
        new_flightmode->get_pilot_desired_throttle() > blimp.get_non_takeoff_throttle()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: throttle too high");
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !blimp.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !blimp.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s need alt estimate", new_flightmode->name());
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
    // surface_tracking.invalidate_for_logging();  // invalidate surface tracking alt, flight mode will set to true if used

    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Blimp::exit_mode(Mode *&old_flightmode,
                      Mode *&new_flightmode)
{
}

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

// returns desired angle in centi-degrees
void Mode::get_pilot_desired_accelerations(float &right_out, float &front_out) const
{
    // throttle failsafe check
    if (blimp.failsafe.radio || !blimp.ap.rc_receiver_present) {
        right_out = 0;
        front_out = 0;
        return;
    }
    // fetch roll and pitch inputs
    right_out = channel_right->get_control_in();
    front_out = channel_front->get_control_in();
}

bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !blimp.ap.auto_armed || blimp.ap.land_complete) {
        return true;
    }
    return false;
}

void Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    if (spool_up) {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::SHUT_DOWN);
    }
}

/*
  get a height above ground estimate for landing
 */
int32_t Mode::get_alt_above_ground_cm(void)
{
    int32_t alt_above_ground_cm;

    if (blimp.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }

    // Assume the Earth is flat:
    return blimp.current_loc.alt;
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
    int16_t throttle_control = channel_down->get_control_in();

    int16_t mid_stick = blimp.get_throttle_mid();
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

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.
float Mode::get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    return blimp.get_pilot_desired_yaw_rate(stick_angle);
}

float Mode::get_pilot_desired_climb_rate(float throttle_control)
{
    return blimp.get_pilot_desired_climb_rate(throttle_control);
}

float Mode::get_non_takeoff_throttle()
{
    return blimp.get_non_takeoff_throttle();
}

bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return blimp.set_mode(mode, reason);
}

void Mode::set_land_complete(bool b)
{
    return blimp.set_land_complete(b);
}

GCS_Blimp &Mode::gcs()
{
    return blimp.gcs();
}

uint16_t Mode::get_pilot_speed_dn()
{
    return blimp.get_pilot_speed_dn();
}
