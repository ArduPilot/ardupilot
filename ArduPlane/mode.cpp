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

bool Mode::has_manual_input() const
{
    return plane.g.stick_mixing != STICK_MIXING_DISABLED;
}

bool Mode::q_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_navigation_mode = false;
    if (!plane.quadplane.init_mode() && plane.previous_mode != nullptr) {
        plane.control_mode = plane.previous_mode;
    } else {
        plane.auto_throttle_mode = false;
        plane.auto_state.vtol_mode = true;
    }

    return true;
}

void Mode::q_update()
{
    // set nav_roll and nav_pitch using sticks
    int16_t roll_limit = MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    plane.nav_roll_cd  = (plane.channel_roll->get_control_in() / 4500.0) * roll_limit;
    plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -roll_limit, roll_limit);
    float pitch_input = plane.channel_pitch->norm_input();
    // Scale from normalized input [-1,1] to centidegrees
    if (plane.quadplane.tailsitter_active()) {
        // For tailsitters, the pitch range is symmetrical: [-Q_ANGLE_MAX,Q_ANGLE_MAX]
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    } else {
        // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
        // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
        if (pitch_input > 0) {
            plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max_cd, plane.quadplane.aparm.angle_max);
        } else {
            plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min_cd, plane.quadplane.aparm.angle_max);
        }
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
    }
}

bool Mode::guided_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    plane.guided_WP_loc = plane.current_loc;
    plane.set_guided_WP();

    return true;
}

void Mode::guided_update()
{
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
    } else {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void Mode::fbwa_update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    }
    if (plane.g.fbwa_tdrag_chan > 0) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = (RC_Channels::get_radio_in(plane.g.fbwa_tdrag_chan-1) > 1700);
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

