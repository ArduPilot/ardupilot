#include "Plane.h"

Mode::Mode() :
    ahrs(plane.ahrs)
#if HAL_QUADPLANE_ENABLED
    , quadplane(plane.quadplane),
    pos_control(plane.quadplane.pos_control),
    attitude_control(plane.quadplane.attitude_control),
    loiter_nav(plane.quadplane.loiter_nav),
    poscontrol(plane.quadplane.poscontrol)
#endif
{
}

void Mode::exit()
{
    // call sub-classes exit
    _exit();
    // stop autotuning if not AUTOTUNE mode
    if (plane.control_mode != &plane.mode_autotune){
        plane.autotune_restore();
    }

}

bool Mode::enter()
{
#if AP_SCRIPTING_ENABLED
    // reset nav_scripting.enabled
    plane.nav_scripting.enabled = false;
#endif

    // cancel inverted flight
    plane.auto_state.inverted_flight = false;
    
    // cancel waiting for rudder neutral
    plane.takeoff_state.waiting_for_rudder_neutral = false;

    // don't cross-track when starting a mission
    plane.auto_state.next_wp_crosstrack = false;

    // reset landing check
    plane.auto_state.checked_for_autoland = false;

    // zero locked course
    plane.steer_state.locked_course_err = 0;
    plane.steer_state.locked_course = false;

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
    plane.guided_state.target_alt_time_ms = 0;
    plane.guided_state.last_target_alt = 0;
#endif

#if AP_CAMERA_ENABLED
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    plane.prev_WP_loc = plane.current_loc;

    // new mode means new loiter
    plane.loiter.start_time_ms = 0;

    // record time of mode change
    plane.last_mode_change_ms = AP_HAL::millis();

    // set VTOL auto state
    plane.auto_state.vtol_mode = is_vtol_mode();
    plane.auto_state.vtol_loiter = false;

    // initialize speed variable used in AUTO and GUIDED for DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;
    
    // clear postponed long failsafe if mode change (from GCS) occurs before recall of long failsafe
    plane.long_failsafe_pending = false;

#if HAL_QUADPLANE_ENABLED
    quadplane.mode_enter();
#endif

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plane.throttle_suppressed = does_auto_throttle();
#if HAL_ADSB_ENABLED
        plane.adsb.set_is_auto_mode(does_auto_navigation());
#endif

        // reset steering integrator on mode change
        plane.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        plane.control_failsafe();

#if AP_FENCE_ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        plane.fence.manual_recovery_start();
#endif
    }

    return enter_result;
}

bool Mode::is_vtol_man_throttle() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.tailsitter.is_in_fw_flight() &&
        plane.quadplane.assisted_flight) {
        // We are a tailsitter that has fully transitioned to Q-assisted forward flight.
        // In this case the forward throttle directly drives the vertical throttle so
        // set vertical throttle state to match the forward throttle state. Confusingly the booleans are inverted,
        // forward throttle uses 'does_auto_throttle' whereas vertical uses 'is_vtol_man_throttle'.
        return !does_auto_throttle();
    }
#endif
    return false;
}

void Mode::update_target_altitude()
{
    Location target_location;

    if (plane.landing.is_flaring()) {
        // during a landing flare, use TECS_LAND_SINK as a target sink
        // rate, and ignores the target altitude
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.landing.is_on_approach()) {
        plane.landing.setup_landing_glide_slope(plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.target_altitude.offset_cm);
        plane.landing.adjust_landing_slope_for_rangefinder_bump(plane.rangefinder_state, plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.auto_state.wp_distance, plane.target_altitude.offset_cm);
    } else if (plane.landing.get_target_altitude_location(target_location)) {
        plane.set_target_altitude_location(target_location);
#if HAL_SOARING_ENABLED
    } else if (plane.g2.soaring_controller.is_active() && plane.g2.soaring_controller.get_throttle_suppressed()) {
        // Reset target alt to current alt, to prevent large altitude errors when gliding.
        plane.set_target_altitude_location(plane.current_loc);
        plane.reset_offset_altitude();
#endif
    } else if (plane.reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.target_altitude.offset_cm != 0 && 
               !plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc)) {
        // control climb/descent rate
        plane.set_target_altitude_proportion(plane.next_WP_loc, 1.0f-plane.auto_state.wp_proportion);

        // stay within the range of the start and end locations in altitude
        plane.constrain_target_altitude_location(plane.next_WP_loc, plane.prev_WP_loc);
    } else {
        plane.set_target_altitude_location(plane.next_WP_loc);
    }

    plane.altitude_error_cm = plane.calc_altitude_error_cm();
}

// returns true if the vehicle can be armed in this mode
bool Mode::pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!_pre_arm_checks(buflen, buffer)) {
        if (strlen(buffer) == 0) {
            // If no message is provided add a generic one
            hal.util->snprintf(buffer, buflen, "mode not armable");
        }
        return false;
    }

    return true;
}

// Auto and Guided do not call this to bypass the q-mode check.
bool Mode::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled() && !is_vtol_mode() &&
            plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO)) {
        hal.util->snprintf(buffer, buflen, "not Q mode");
        return false;
    }
#endif
    return true;
}

void Mode::run()
{
    // Direct stick mixing functionality has been removed, so as not to remove all stick mixing from the user completely
    // the old direct option is now used to enable fbw mixing, this is easier than doing a param conversion.
    if ((plane.g.stick_mixing == StickMixing::FBW) || (plane.g.stick_mixing == StickMixing::DIRECT_REMOVED)) {
        plane.stabilize_stick_mixing_fbw();
    }
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}

// Reset rate and steering controllers
void Mode::reset_controllers()
{
    // reset integrators
    plane.rollController.reset_I();
    plane.pitchController.reset_I();
    plane.yawController.reset_I();

    // reset steering controls
    plane.steer_state.locked_course = false;
    plane.steer_state.locked_course_err = 0;
}

bool Mode::is_taking_off() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF);
}

// Helper to output to both k_rudder and k_steering servo functions
void Mode::output_rudder_and_steering(float val)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, val);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, val);
}
