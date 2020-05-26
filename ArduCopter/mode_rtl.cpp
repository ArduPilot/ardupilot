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
    wp_nav->wp_and_spline_init();
    _state = RTL_Starting;
    _state_complete = true; // see run() method below
    terrain_following_allowed = !copter.failsafe.terrain;
    return true;
}

// re-start RTL with terrain following disabled
void ModeRTL::restart_without_terrain()
{
    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
    terrain_following_allowed = false;
    _state = RTL_Starting;
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
        case RTL_Starting:
            build_path();
            climb_start();
            break;
        case RTL_InitialClimb:
            return_start();
            break;
        case RTL_ReturnHome:
            loiterathome_start();
            break;
        case RTL_LoiterAtHome:
            if (rtl_path.land || copter.failsafe.radio) {
                land_start();
            }else{
                descent_start();
            }
            break;
        case RTL_FinalDescent:
            // do nothing
            break;
        case RTL_Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (_state) {

    case RTL_Starting:
        // should not be reached:
        _state = RTL_InitialClimb;
        FALLTHROUGH;

    case RTL_InitialClimb:
        climb_return_run();
        break;

    case RTL_ReturnHome:
        climb_return_run();
        break;

    case RTL_LoiterAtHome:
        loiterathome_run();
        break;

    case RTL_FinalDescent:
        descent_run();
        break;

    case RTL_Land:
        land_run(disarm_on_land);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void ModeRTL::climb_start()
{
    _state = RTL_InitialClimb;
    _state_complete = false;

    // RTL_SPEED == 0 means use WPNAV_SPEED
    if (g.rtl_speed_cms != 0) {
        wp_nav->set_speed_xy(g.rtl_speed_cms);
    }

    // set the destination
    if (!wp_nav->set_wp_destination(rtl_path.climb_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }
    wp_nav->set_fast_waypoint(true);

    // hold current yaw during initial climb
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
void ModeRTL::return_start()
{
    _state = RTL_ReturnHome;
    _state_complete = false;

    if (!wp_nav->set_wp_destination(rtl_path.return_target)) {
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
        make_safe_spool_down();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(),true);
    }

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// rtl_loiterathome_start - initialise return to home
void ModeRTL::loiterathome_start()
{
    _state = RTL_LoiterAtHome;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(auto_yaw.default_mode(true) != AUTO_YAW_HOLD) {
        auto_yaw.set_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(),true);
    }

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw.mode() == AUTO_YAW_RESETTOARMEDYAW) {
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
    _state = RTL_FinalDescent;
    _state_complete = false;

    // Set wp navigation target to above home
    loiter_nav->init_target(wp_nav->get_wp_destination());

    // initialise altitude target to stopping point
    pos_control->set_target_to_stopping_point_z();

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void ModeRTL::descent_run()
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f;

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
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

            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // process roll, pitch inputs
    loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

    // run loiter controller
    loiter_nav->update();

    // call z-axis position controller
    pos_control->set_alt_target_with_slew(rtl_path.descent_target.alt, G_Dt);
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

    // check if we've reached within 20cm of final altitude
    _state_complete = labs(rtl_path.descent_target.alt - copter.current_loc.alt) < 20;
}

// rtl_loiterathome_start - initialise controllers to loiter over home
void ModeRTL::land_start()
{
    _state = RTL_Land;
    _state_complete = false;

    // Set wp navigation target to above home
    loiter_nav->init_target(wp_nav->get_wp_destination());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
}

bool ModeRTL::is_landing() const
{
    return _state == RTL_Land;
}

// rtl_returnhome_run - return home
//      called by rtl_run at 100hz or more
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
        make_safe_spool_down();
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    land_run_horizontal_control();
    land_run_vertical_control();
}

void ModeRTL::build_path()
{
    // origin point is our stopping point
    Vector3f stopping_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->get_stopping_point_z(stopping_point);
    rtl_path.origin_point = Location(stopping_point);
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
#if AC_RALLY == ENABLED
    rtl_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = copter.current_loc.alt;

    // determine altitude type of return journey (alt-above-home, alt-above-terrain using range finder or alt-above-terrain using terrain database)
    ReturnTargetAltType alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == RTLAltType::RTL_ALTTYPE_TERRAIN)) {
        // convert RTL_ALT_TYPE and WPNAV_RFNG_USE parameters to ReturnTargetAltType
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_TERRAINDATABASE;
            break;
        }
    }

    // set curr_alt and return_target.alt from range finder
    if (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_cm(curr_alt)) {
            // set return_target.alt
            rtl_path.return_target.set_alt_cm(MAX(curr_alt + MAX(0, g.rtl_climb_min), MAX(g.rtl_altitude, RTL_ALT_MIN)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
        }
    }

    // set curr_alt and return_target.alt from terrain database
    if (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_TERRAINDATABASE) {
        // set curr_alt to current altitude above terrain
        // convert return_target.alt from an abs (above MSL) to altitude above terrain
        //   Note: the return_target may be a rally point with the alt set above the terrain alt (like the top of a building)
        int32_t curr_terr_alt;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt) &&
            rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            curr_alt = curr_terr_alt;
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // for the default case we must convert return-target alt (which is an absolute alt) to alt-above-home
    if (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE) {
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
    rtl_path.return_target.set_alt_cm(target_alt, (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AC_FENCE == ENABLED
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

bool ModeRTL::get_wp(Location& destination)
{
    // provide target in states which use wp_nav
    switch (_state) {
    case RTL_Starting:
    case RTL_InitialClimb:
    case RTL_ReturnHome:
    case RTL_LoiterAtHome:
    case RTL_FinalDescent:
        return wp_nav->get_oa_wp_destination(destination);
    case RTL_Land:
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

#endif
