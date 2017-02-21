/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

/*
 * control_rtl.pde - init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool Sub::rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
    	rtl_build_path(!failsafe.terrain);
        rtl_climb_start();
        return true;
    }else{
        return false;
    }
}

// re-start RTL with terrain following disabled
void Sub::rtl_restart_without_terrain()
{
    // log an error
    Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_RESTARTED_RTL);
    if (rtl_path.terrain_used) {
        rtl_build_path(false);
        rtl_climb_start();
        gcs_send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void Sub::rtl_run()
{
    // check if we need to move to next state
    if (rtl_state_complete) {
        switch (rtl_state) {
        case RTL_InitialClimb:
            rtl_return_start();
            break;
        case RTL_ReturnHome:
            rtl_loiterathome_start();
            break;
        case RTL_LoiterAtHome:
            if (g.rtl_alt_final > 0 && !failsafe.radio) {
                rtl_descent_start();
            }else{
                rtl_land_start();
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
    switch (rtl_state) {

    case RTL_InitialClimb:
        rtl_climb_return_run();
        break;

    case RTL_ReturnHome:
        rtl_climb_return_run();
        break;

    case RTL_LoiterAtHome:
        rtl_loiterathome_run();
        break;

    case RTL_FinalDescent:
        rtl_descent_run();
        break;

    case RTL_Land:
        rtl_land_run();
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void Sub::rtl_climb_start()
{
    rtl_state = RTL_InitialClimb;
    rtl_state_complete = false;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // RTL_SPEED == 0 means use WPNAV_SPEED
    if (g.rtl_speed_cms != 0) {
        wp_nav.set_speed_xy(g.rtl_speed_cms);
    }

    // set the destination
    if (!wp_nav.set_wp_destination(rtl_path.climb_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        set_mode(LAND, MODE_REASON_TERRAIN_FAILSAFE);
        return;
    }
    wp_nav.set_fast_waypoint(true);

    // hold current yaw during initial climb
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
void Sub::rtl_return_start()
{
    rtl_state = RTL_ReturnHome;
    rtl_state_complete = false;

    if (!wp_nav.set_wp_destination(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        rtl_restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    set_auto_yaw_mode(get_default_auto_yaw_mode(true));
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Sub::rtl_climb_return_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    	// multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true, get_smoothing_gain());
    }

    // check if we've completed this stage of RTL
    rtl_state_complete = wp_nav.reached_wp_destination();
}

// rtl_loiterathome_start - initialise return to home
void Sub::rtl_loiterathome_start()
{
    rtl_state = RTL_LoiterAtHome;
    rtl_state_complete = false;
    rtl_loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(get_default_auto_yaw_mode(true) != AUTO_YAW_HOLD) {
        set_auto_yaw_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Sub::rtl_loiterathome_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    	// multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true, get_smoothing_gain());
    }

    // check if we've completed this stage of RTL
    if ((millis() - rtl_loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw_mode == AUTO_YAW_RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (labs(wrap_180_cd(ahrs.yaw_sensor-initial_armed_bearing)) <= 200) {
                rtl_state_complete = true;
            }
        } else {
            // we have loitered long enough
            rtl_state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void Sub::rtl_descent_start()
{
    rtl_state = RTL_FinalDescent;
    rtl_state_complete = false;

    // Set wp navigation target to above home
    wp_nav.init_loiter_target(wp_nav.get_wp_destination());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void Sub::rtl_descent_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    	// multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // set target to current position
        wp_nav.init_loiter_target();
        return;
    }

    // process pilot's input
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }
        
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    pos_control.set_alt_target_with_slew(rtl_path.descent_target.alt, G_Dt);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());

    // check if we've reached within 20cm of final altitude
    rtl_state_complete = fabsf(rtl_path.descent_target.alt - current_loc.alt) < 20.0f;
}

// rtl_loiterathome_start - initialise controllers to loiter over home
void Sub::rtl_land_start()
{
    rtl_state = RTL_Land;
    rtl_state_complete = false;

    // Set wp navigation target to above home
    wp_nav.init_loiter_target(wp_nav.get_wp_destination());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_returnhome_run - return home
//      called by rtl_run at 100hz or more
void Sub::rtl_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    	// multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // set target to current position
        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif

        // check if we've completed this stage of RTL
        rtl_state_complete = ap.land_complete;
        return;
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->control_in;
            pitch_control = channel_pitch->control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

     // process pilot's roll and pitch input
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    float cmb_rate = get_land_descent_speed();
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;
}

void Sub::rtl_build_path(bool terrain_following_allowed)
{
    // origin point is our stopping point
    Vector3f stopping_point;
    pos_control.get_stopping_point_xy(stopping_point);
    pos_control.get_stopping_point_z(stopping_point);
    rtl_path.origin_point = Location_Class(stopping_point);
    rtl_path.origin_point.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME);

    // set return target to nearest rally point or home position
#if AC_RALLY == ENABLED
    rtl_path.return_target = rally.calc_best_rally_or_home_location(current_loc, ahrs.get_home().alt);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // compute return altitude
    rtl_compute_return_alt(rtl_path.origin_point, rtl_path.return_target, terrain_following_allowed);

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location_Class(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target = Location_Class(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location_Class::ALT_FRAME_ABOVE_HOME);

    // set land flag
    rtl_path.land = g.rtl_alt_final <= 0;
}

// return altitude in cm above home at which vehicle should return home
//   rtl_origin_point is the stopping point of the vehicle when rtl is initiated
//   rtl_return_target is the home or rally point that the vehicle is returning to.  It's lat, lng and alt values must already have been filled in before this function is called
//   rtl_return_target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void Sub::rtl_compute_return_alt(const Location_Class &rtl_origin_point, Location_Class &rtl_return_target, bool terrain_following_allowed)
{
    float rtl_return_dist_cm = rtl_return_target.get_distance(rtl_origin_point) * 100.0f;

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = current_loc.alt;

    // decide if we should use terrain altitudes
    rtl_path.terrain_used = terrain_use() && terrain_following_allowed;
    if (rtl_path.terrain_used) {
        // attempt to retrieve terrain alt for current location, stopping point and origin
        int32_t origin_terr_alt, return_target_terr_alt;
        if (!rtl_origin_point.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, origin_terr_alt) ||
            !rtl_origin_point.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, return_target_terr_alt) ||
            !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, curr_alt)) {
            rtl_path.terrain_used = false;
            Log_Write_Error(ERROR_SUBSYSTEM_TERRAIN, ERROR_CODE_MISSING_TERRAIN_DATA);
        }
    }

    // maximum of current altitude + climb_min and rtl altitude
    float ret = MAX(curr_alt + MAX(0, g.rtl_climb_min), MAX(g.rtl_altitude, RTL_ALT_MIN));

    // don't allow really shallow slopes
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        ret = MAX(curr_alt, MIN(ret, MAX(rtl_return_dist_cm*g.rtl_cone_slope, curr_alt+RTL_ABS_MIN_CLIMB)));
    }

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: we are assuming the fence alt is the same frame as ret
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        ret = MIN(ret, fence.get_safe_alt()*100.0f);
    }
#endif

    // ensure we do not descend
    ret = MAX(ret, curr_alt);

    // convert return-target to alt-above-home or alt-above-terrain
    if (!rtl_path.terrain_used || !rtl_return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_TERRAIN)) {
        if (!rtl_return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_return_target.set_alt_cm(0, Location_Class::ALT_FRAME_ABOVE_HOME);
        }
    }

    // add ret to altitude
    rtl_return_target.alt += ret;
}

