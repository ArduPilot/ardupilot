#include "mode.h"
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


 #if FRSKY_TELEM_ENABLED == ENABLED
     plane.frsky_telemetry.update_control_mode(mode_number());
 #endif
 #if DEVO_TELEM_ENABLED == ENABLED
     plane.devo_telemetry.update_control_mode(mode_number());
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

     // reset steering integrator on mode change
     plane.steerController.reset_I();

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plane.throttle_suppressed = plane.auto_throttle_mode;

        plane.adsb.set_is_auto_mode(plane.auto_navigation_mode);
    }

    return enter_result;
}

