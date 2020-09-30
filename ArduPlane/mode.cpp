#include "Plane.h"

Mode::Mode()
{
}

void Mode::exit()
{
    // call sub-classes exit
    _exit();
}

bool Mode::enter()
{
    // cancel inverted flight
    plane.auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    plane.auto_state.next_wp_crosstrack = false;

    // reset landing check
    plane.auto_state.checked_for_autoland = false;

    // zero locked course
    plane.steer_state.locked_course_err = 0;

    // reset crash detection
    plane.crash_state.is_crashed = false;
    plane.crash_state.impact_detected = false;

    // reset external attitude guidance
    plane.guided_state.last_forced_rpy_ms.zero();
    plane.guided_state.last_forced_throttle_ms = 0;

#if OFFBOARD_GUIDED == ENABLED
    plane.guided_state.target_heading = -4; // radians here are in range -3.14 to 3.14, so a default value needs to be outside that range
    plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
    plane.guided_state.target_airspeed_cm = -1; // same as above, although an airspeed of -1 is rare on plane.
    plane.guided_state.target_alt = -1; // same as above, although a target alt of -1 is rare on plane.
    plane.guided_state.last_target_alt = 0;
#endif

#if CAMERA == ENABLED
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = plane.ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    plane.prev_WP_loc = plane.current_loc;

    // new mode means new loiter
    plane.loiter.start_time_ms = 0;

    // record time of mode change
    plane.last_mode_change_ms = AP_HAL::millis();

    // assume non-VTOL mode
    plane.auto_state.vtol_mode = false;
    plane.auto_state.vtol_loiter = false;

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plane.throttle_suppressed = plane.auto_throttle_mode;
#if HAL_ADSB_ENABLED
        plane.adsb.set_is_auto_mode(plane.auto_navigation_mode);
#endif

        // reset steering integrator on mode change
        plane.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        plane.control_failsafe();
    }

    return enter_result;
}

bool Mode::is_vtol_man_throttle() const
{
    if (!plane.quadplane.in_vtol_mode() &&
        plane.quadplane.is_tailsitter() && 
        plane.quadplane.tailsitter_transition_fw_complete() && 
        plane.quadplane.assisted_flight) {
        // a tailsitter that has fully transisisoned to Q-assisted forward flight
        // in this case the forward throttle directly drives the vertical throttle
        // set vertical throttle state to match the forward throttle state. Confusingly the booleans are inverted,
        // forward throttle uses 'auto_throttle_mode' whereas vertical used 'is_vtol_man_throttle'
        return !plane.auto_throttle_mode;
    }
    return false;
}
