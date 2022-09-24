#include "Copter.h"

#if MODE_RTL_ENABLED == ENABLED

/*
 * Init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool ModeRTL::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init(g.rtl_speed_cms);
    _state = SubMode::STARTING;
    _state_complete = true; // see run() method below
    terrain_following_allowed = !copter.failsafe.terrain;
    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

#if PRECISION_LANDING == ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

// re-start RTL with terrain following disabled
void ModeRTL::restart_without_terrain()
{
    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
    terrain_following_allowed = false;
    _state = SubMode::STARTING;
    _state_complete = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
}

ModeRTL::RTLAltType ModeRTL::get_alt_type() const
{
    // sanity check parameter
    if (g.rtl_alt_type < 0 || g.rtl_alt_type > (int)RTLAltType::RTL_ALTTYPE_TERRAIN) {
        return RTLAltType::RTL_ALTTYPE_RELATIVE;
    }
    return (RTLAltType)g.rtl_alt_type.get();
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void ModeRTL::run(bool disarm_on_land)
{
    if (!motors->armed()) {
        return;
    }

    // check if we need to move to next state
    if (_state_complete) {
        switch (_state) {
        case SubMode::STARTING:
            build_path();
            climb_start();
            break;
        case SubMode::INITIAL_CLIMB:
            return_start();
            break;
        case SubMode::RETURN_HOME:
            loiterathome_start();
            break;
        case SubMode::LOITER_AT_HOME:
            if (rtl_path.land || copter.failsafe.radio) {
                land_start();
            } else {
                descent_start();
            }
            break;
        case SubMode::FINAL_DESCENT:
            // do nothing
            break;
        case SubMode::LAND:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (_state) {

    case SubMode::STARTING:
        // should not be reached:
        _state = SubMode::INITIAL_CLIMB;
        FALLTHROUGH;

    case SubMode::INITIAL_CLIMB:
        climb_return_run();
        break;

    case SubMode::RETURN_HOME:
        climb_return_run();
        break;

    case SubMode::LOITER_AT_HOME:
        loiterathome_run();
        break;

    case SubMode::FINAL_DESCENT:
        descent_run();
        break;

    case SubMode::LAND:
        land_run(disarm_on_land);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void ModeRTL::climb_start()
{
    _state = SubMode::INITIAL_CLIMB;
    _state_complete = false;

    // set the destination
    if (!wp_nav->set_wp_destination_loc(rtl_path.climb_target) || !wp_nav->set_wp_destination_next_loc(rtl_path.return_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }

    // hold current yaw during initial climb
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

// rtl_return_start - initialise return to home
void ModeRTL::return_start()
{
    _state = SubMode::RETURN_HOME;
    _state_complete = false;

    if (!wp_nav->set_wp_destination_loc(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    auto_yaw.set_mode_to_default(true);
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::climb_return_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// loiterathome_start - initialise return to home
void ModeRTL::loiterathome_start()
{
    _state = SubMode::LOITER_AT_HOME;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if (auto_yaw.default_mode(true) != AutoYaw::Mode::HOLD) {
        auto_yaw.set_mode(AutoYaw::Mode::RESETTOARMEDYAW);
    } else {
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw.mode() == AutoYaw::Mode::RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (abs(wrap_180_cd(ahrs.yaw_sensor-copter.initial_armed_bearing)) <= 200) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            _state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void ModeRTL::descent_start()
{
    _state = SubMode::FINAL_DESCENT;
    _state_complete = false;

    // initialise altitude target to stopping point
    pos_control->init_z_controller_stopping_point();

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AP_FENCE_ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void ModeRTL::descent_run()
{
    Vector2f vel_correction;

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's input
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!copter.set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to reposition velocity
            vel_correction = get_pilot_desired_velocity(wp_nav->get_wp_acceleration() * 0.5);

            // record if pilot has overridden roll or pitch
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector2f accel;
    pos_control->input_vel_accel_xy(vel_correction, accel);
    pos_control->update_xy_controller();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->set_alt_target_with_slew(rtl_path.descent_target.alt);
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've reached within 20cm of final altitude
    _state_complete = labs(rtl_path.descent_target.alt - copter.current_loc.alt) < 20;
}

// land_start - initialise controllers to loiter over home
void ModeRTL::land_start()
{
    _state = SubMode::LAND;
    _state_complete = false;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise loiter target destination
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AP_FENCE_ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif
}

bool ModeRTL::is_landing() const
{
    return _state == SubMode::LAND;
}

// land_run - run the landing controllers to put the aircraft on the ground
// called by rtl_run at 100hz or more
void ModeRTL::land_run(bool disarm_on_land)
{
    // check if we've completed this stage of RTL
    _state_complete = copter.ap.land_complete;

    // disarm when the landing detector says we've landed
    if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

void ModeRTL::build_path()
{
    // origin point is our stopping point
    Vector3p stopping_point;
    pos_control->get_stopping_point_xy_cm(stopping_point.xy());
    pos_control->get_stopping_point_z_cm(stopping_point.z);
    rtl_path.origin_point = Location(stopping_point.tofloat(), Location::AltFrame::ABOVE_ORIGIN);
    rtl_path.origin_point.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // compute return target
    compute_return_target();

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target = Location(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location::AltFrame::ABOVE_HOME);

    // set land flag
    rtl_path.land = g.rtl_alt_final <= 0;
}

// compute the return target - home or rally point
//   return target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void ModeRTL::compute_return_target()
{
    // set return target to nearest rally point or home position (Note: alt is absolute)
#if HAL_RALLY_ENABLED
    rtl_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = copter.current_loc.alt;

    // determine altitude type of return journey (alt-above-home, alt-above-terrain using range finder or alt-above-terrain using terrain database)
    ReturnTargetAltType alt_type = ReturnTargetAltType::RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == RTLAltType::RTL_ALTTYPE_TERRAIN)) {
        // convert RTL_ALT_TYPE and WPNAV_RFNG_USE parameters to ReturnTargetAltType
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            alt_type = ReturnTargetAltType::RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            alt_type = ReturnTargetAltType::RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            alt_type = ReturnTargetAltType::TERRAINDATABASE;
            break;
        }
    }

    // set curr_alt and return_target.alt from range finder
    if (alt_type == ReturnTargetAltType::RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_cm(curr_alt)) {
            // set return_target.alt
            rtl_path.return_target.set_alt_cm(MAX(curr_alt + MAX(0, g.rtl_climb_min), MAX(g.rtl_altitude, RTL_ALT_MIN)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
        }
    }

    // set curr_alt and return_target.alt from terrain database
    if (alt_type == ReturnTargetAltType::TERRAINDATABASE) {
        // set curr_alt to current altitude above terrain
        // convert return_target.alt from an abs (above MSL) to altitude above terrain
        //   Note: the return_target may be a rally point with the alt set above the terrain alt (like the top of a building)
        int32_t curr_terr_alt;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt) &&
            rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            curr_alt = curr_terr_alt;
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // for the default case we must convert return-target alt (which is an absolute alt) to alt-above-home
    if (alt_type == ReturnTargetAltType::RELATIVE) {
        if (!rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_path.return_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
            gcs().send_text(MAV_SEVERITY_WARNING, "RTL: unexpected error calculating target alt");
        }
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon rtl_alt_type
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    int32_t target_alt = MAX(rtl_path.return_target.alt, 0);

    // increase target to maximum of current altitude + climb_min and rtl altitude
    target_alt = MAX(target_alt, curr_alt + MAX(0, g.rtl_climb_min));
    target_alt = MAX(target_alt, MAX(g.rtl_altitude, RTL_ALT_MIN));

    // reduce climb if close to return target
    float rtl_return_dist_cm = rtl_path.return_target.get_distance(rtl_path.origin_point) * 100.0f;
    // don't allow really shallow slopes
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        target_alt = MAX(curr_alt, MIN(target_alt, MAX(rtl_return_dist_cm*g.rtl_cone_slope, curr_alt+RTL_ABS_MIN_CLIMB)));
    }

    // set returned target alt to new target_alt (don't change altitude type)
    rtl_path.return_target.set_alt_cm(target_alt, (alt_type == ReturnTargetAltType::RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AP_FENCE_ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: because the rtl_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target as alt-above-home so it can be compared to fence's alt
        if (rtl_path.return_target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt)) {
            float fence_alt = copter.fence.get_safe_alt_max()*100.0f;
            if (target_alt > fence_alt) {
                // reduce target alt to the fence alt
                rtl_path.return_target.alt -= (target_alt - fence_alt);
            }
        }
    }
#endif

    // ensure we do not descend
    rtl_path.return_target.alt = MAX(rtl_path.return_target.alt, curr_alt);
}

bool ModeRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (_state) {
    case SubMode::STARTING:
    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
    case SubMode::LOITER_AT_HOME:
    case SubMode::FINAL_DESCENT:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

uint32_t ModeRTL::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeRTL::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeRTL::use_pilot_yaw(void) const
{
    return (copter.g2.rtl_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
}

#endif
