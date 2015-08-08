/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_rtl.pde - init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool Copter::rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        rtl_climb_start();
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void Copter::rtl_run()
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
void Copter::rtl_climb_start()
{
    rtl_state = RTL_InitialClimb;
    rtl_state_complete = false;
    rtl_alt = get_RTL_alt();

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // get horizontal stopping point
    Vector3f destination;
    wp_nav.get_wp_stopping_point_xy(destination);

#if AC_RALLY == ENABLED
    // rally_point.alt will be the altitude of the nearest rally point or the RTL_ALT. uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, rtl_alt+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = max(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    destination.z = pv_alt_above_origin(rally_point.alt);
#else
    destination.z = pv_alt_above_origin(rtl_alt);
#endif

    // set the destination
    wp_nav.set_wp_destination(destination);
    wp_nav.set_fast_waypoint(true);

    // hold current yaw during initial climb
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
void Copter::rtl_return_start()
{
    rtl_state = RTL_ReturnHome;
    rtl_state_complete = false;

    // set target to above home/rally point
#if AC_RALLY == ENABLED
    // rally_point will be the nearest rally point or home.  uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, rtl_alt+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = max(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    Vector3f destination = pv_location_to_vector(rally_point);
#else
    Vector3f destination = pv_location_to_vector(ahrs.get_home());
    destination.z = pv_alt_above_origin(rtl_alt));
#endif

    wp_nav.set_wp_destination(destination);

    // initialise yaw to point home (maybe)
    set_auto_yaw_mode(get_default_auto_yaw_mode(true));
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void Copter::rtl_climb_return_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
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

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }

    // check if we've completed this stage of RTL
    rtl_state_complete = wp_nav.reached_wp_destination();
}

// rtl_return_start - initialise return to home
void Copter::rtl_loiterathome_start()
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
void Copter::rtl_loiterathome_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
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

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
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
void Copter::rtl_descent_start()
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
void Copter::rtl_descent_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // set target to current position
        wp_nav.init_loiter_target();
        return;
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

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    pos_control.set_alt_target_with_slew(pv_alt_above_origin(g.rtl_alt_final), G_Dt);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // check if we've reached within 20cm of final altitude
    rtl_state_complete = fabsf(pv_alt_above_origin(g.rtl_alt_final) - inertial_nav.get_altitude()) < 20.0f;
}

// rtl_loiterathome_start - initialise controllers to loiter over home
void Copter::rtl_land_start()
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
void Copter::rtl_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    // if not auto armed or landing completed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
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
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;
}

// get_RTL_alt - return altitude which vehicle should return home at
//      altitude is in cm above home
float Copter::get_RTL_alt()
{
    // maximum of current altitude + climb_min and rtl altitude
    float ret = max(current_loc.alt + max(0, g.rtl_climb_min), g.rtl_altitude);
    ret = max(ret, RTL_ALT_MIN);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        ret = min(ret, fence.get_safe_alt()*100.0f);
    }
#endif

    return ret;
}

