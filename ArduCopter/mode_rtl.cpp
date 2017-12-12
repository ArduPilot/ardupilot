#include "Copter.h"

/*
 * Init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool Copter::ModeRTL::init(bool ignore_checks)
{
    if (_copter.position_ok() || ignore_checks) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
        build_path(!_copter.failsafe.terrain);
        climb_start();
        return true;
    }else{
        return false;
    }
}

// re-start RTL with terrain following disabled
void Copter::ModeRTL::restart_without_terrain()
{
    // log an error
    _copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_RESTARTED_RTL);
    if (rtl_path.terrain_used) {
        build_path(false);
        climb_start();
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void Copter::ModeRTL::run(bool disarm_on_land)
{
    // check if we need to move to next state
    if (_state_complete) {
        switch (_state) {
        case RTL_InitialClimb:
            return_start();
            break;
        case RTL_ReturnHome:
            loiterathome_start();
            break;
        case RTL_LoiterAtHome:
            if (rtl_path.land || _copter.failsafe.radio) {
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
void Copter::ModeRTL::climb_start()
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
        _copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        _copter.set_mode(LAND, MODE_REASON_TERRAIN_FAILSAFE);
        return;
    }
    wp_nav->set_fast_waypoint(true);

    // hold current yaw during initial climb
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
void Copter::ModeRTL::return_start()
{
    _state = RTL_ReturnHome;
    _state_complete = false;

    if (!wp_nav->set_wp_destination(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    set_auto_yaw_mode(_copter.get_default_auto_yaw_mode(true));
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Copter::ModeRTL::climb_return_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!_copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    _copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (_copter.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
    }

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// rtl_loiterathome_start - initialise return to home
void Copter::ModeRTL::loiterathome_start()
{
    _state = RTL_LoiterAtHome;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(_copter.get_default_auto_yaw_mode(true) != AUTO_YAW_HOLD) {
        set_auto_yaw_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Copter::ModeRTL::loiterathome_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!_copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    _copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (_copter.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
    }

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (_copter.auto_yaw_mode == AUTO_YAW_RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (labs(wrap_180_cd(ahrs.yaw_sensor-_copter.initial_armed_bearing)) <= 200) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            _state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void Copter::ModeRTL::descent_start()
{
    _state = RTL_FinalDescent;
    _state_complete = false;

    // Set wp navigation target to above home
    wp_nav->init_loiter_target(wp_nav->get_wp_destination());

    // initialise altitude target to stopping point
    pos_control->set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void Copter::ModeRTL::descent_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // set target to current position
        wp_nav->init_loiter_target();
        return;
    }

    // process pilot's input
    if (!_copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && _copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!_copter.set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                _copter.set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->get_control_in();
            pitch_control = channel_pitch->get_control_in();
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // process roll, pitch inputs
    wp_nav->set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    pos_control->set_alt_target_with_slew(rtl_path.descent_target.alt, G_Dt);
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());

    // check if we've reached within 20cm of final altitude
    _state_complete = labs(rtl_path.descent_target.alt - _copter.current_loc.alt) < 20;
}

// rtl_loiterathome_start - initialise controllers to loiter over home
void Copter::ModeRTL::land_start()
{
    _state = RTL_Land;
    _state_complete = false;

    // Set wp navigation target to above home
    wp_nav->init_loiter_target(wp_nav->get_wp_destination());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

bool Copter::ModeRTL::landing_gear_should_be_deployed()
{
    switch(_state) {
    case RTL_LoiterAtHome:
    case RTL_Land:
    case RTL_FinalDescent:
        return true;
    default:
        return false;
    }
    return false;
}

// rtl_returnhome_run - return home
//      called by rtl_run at 100hz or more
void Copter::ModeRTL::land_run(bool disarm_on_land)
{
    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // set target to current position
        wp_nav->init_loiter_target();

        // disarm when the landing detector says we've landed
        if (ap.land_complete && disarm_on_land) {
            _copter.init_disarm_motors();
        }

        // check if we've completed this stage of RTL
        _state_complete = ap.land_complete;
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    _copter.land_run_horizontal_control();
    _copter.land_run_vertical_control();

    // check if we've completed this stage of RTL
    _state_complete = ap.land_complete;
}

void Copter::ModeRTL::build_path(bool terrain_following_allowed)
{
    // origin point is our stopping point
    Vector3f stopping_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->get_stopping_point_z(stopping_point);
    rtl_path.origin_point = Location_Class(stopping_point);
    rtl_path.origin_point.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME);

    // compute return target
    compute_return_target(terrain_following_allowed);

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location_Class(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target = Location_Class(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location_Class::ALT_FRAME_ABOVE_HOME);

    // set land flag
    rtl_path.land = g.rtl_alt_final <= 0;
}

// compute the return target - home or rally point
//   return altitude in cm above home at which vehicle should return home
//   return target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void Copter::ModeRTL::compute_return_target(bool terrain_following_allowed)
{
    // set return target to nearest rally point or home position (Note: alt is absolute)
#if AC_RALLY == ENABLED
    rtl_path.return_target = _copter.rally.calc_best_rally_or_home_location(_copter.current_loc, ahrs.get_home().alt);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = _copter.current_loc.alt;

    // decide if we should use terrain altitudes
    rtl_path.terrain_used = _copter.terrain_use() && terrain_following_allowed;
    if (rtl_path.terrain_used) {
        // attempt to retrieve terrain alt for current location, stopping point and origin
        int32_t origin_terr_alt, return_target_terr_alt;
        if (!rtl_path.origin_point.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, origin_terr_alt) ||
            !rtl_path.return_target.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, return_target_terr_alt) ||
            !_copter.current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, curr_alt)) {
            rtl_path.terrain_used = false;
            _copter.Log_Write_Error(ERROR_SUBSYSTEM_TERRAIN, ERROR_CODE_MISSING_TERRAIN_DATA);
        }
    }

    // convert return-target alt (which is an absolute alt) to alt-above-home or alt-above-terrain
    if (!rtl_path.terrain_used || !rtl_path.return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_TERRAIN)) {
        if (!rtl_path.return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_path.return_target.set_alt_cm(0, Location_Class::ALT_FRAME_ABOVE_HOME);
        }
        rtl_path.terrain_used = false;
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon use_terrain
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

    // set returned target alt to new target_alt
    rtl_path.return_target.set_alt_cm(target_alt, rtl_path.terrain_used ? Location_Class::ALT_FRAME_ABOVE_TERRAIN : Location_Class::ALT_FRAME_ABOVE_HOME);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: because the rtl_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((_copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target as alt-above-home so it can be compared to fence's alt
        if (rtl_path.return_target.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, target_alt)) {
            float fence_alt = _copter.fence.get_safe_alt_max()*100.0f;
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
