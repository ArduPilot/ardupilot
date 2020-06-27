#include "AC_WPNav_Heli.h"
#include <AP_HAL/AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_WPNav_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_WPNav, 0),

    // @Param: USE_L1_NAV
    // @DisplayName: Waypoint mission use L1 navigation with speed/height control
    // @Description: This controls if waypoint missions use L1 navigation
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("USE_L1_NAV",   1, AC_WPNav_Heli, _l1_nav_use, 0),

    // @Param: L1_LOIT_RAD
    // @DisplayName: Loiter Radius for L1 Navigation
    // @Description: This sets the circle radius for the L1 loiter mode
    // @Range: 25 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("L1_LTR_RAD",   2, AC_WPNav_Heli, _loiter_radius, 50.0f),

    AP_GROUPEND
};

AC_WPNav_Heli::AC_WPNav_Heli( const AP_InertialNav& inav,
                   AP_AHRS_View& ahrs,
                   AC_PosControl& pos_control,
                   const AC_AttitudeControl& attitude_control,
                   const AP_L1_Control_Heli& L1_controller,
                   const AP_SpdHgtControl_Heli& helispdhgtctrl,
                   AP_Mission& mission) :
    AC_WPNav(inav,ahrs,pos_control,attitude_control),
    _L1_controller(L1_controller),
    _helispdhgtctrl(helispdhgtctrl),
    _mission(mission)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav_Heli::wp_and_spline_init()
{

    AC_WPNav::wp_and_spline_init();

    // if waypoint controller is not active, set origin to current position
    _ahrs.get_position(_prev_WP_loc);

    _helispdhgtctrl.init_controller();
    _helispdhgtctrl.set_desired_speed(_wp_speed_cms);
    _helispdhgtctrl.set_max_accel(_wp_accel_cmss);

    //
}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav_Heli::set_wp_destination(const Location_Class& destination)
{

    // set next wp location for L1 Navigation
    set_L1_wp_origin_and_destination(destination);

    return AC_WPNav::set_wp_destination(destination);

}

/// set_wp_destination waypoint using position vector (distance from home in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
bool AC_WPNav_Heli::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{

    return AC_WPNav::set_wp_destination(destination, terrain_alt);

}

/// set the L1 navigation controller origin and destination
void AC_WPNav_Heli::set_L1_wp_origin_and_destination(const Location_Class& destination)
{

    // if waypoint controller is active use the existing position target as the origin
    if ((AP_HAL::millis() - _wp_last_l1_update) < 1000) {
        _prev_WP_loc = _next_WP_loc;
    } else {
        // if waypoint controller is not active, set origin to current position
        _ahrs.get_position(_prev_WP_loc);
    }
    _next_WP_loc = destination;

//    int32_t temp_alt;
//    destination.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_ORIGIN, temp_alt);
//    _pos_control.set_alt_target(temp_alt);

    _reached_l1_destination = false;

}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav_Heli::update_l1_wpnav()
{
    bool ret = true;

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // advance the target if necessary
    if (!advance_l1_wp_target_along_track(dt)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    // wp_speed_update - update _pos_control.set_max_speed_xy if speed change has been requested
    wp_speed_update(dt);

    // run plane waypoint controller
    AP_Mission::Mission_Command cmd;
    cmd = _mission.get_current_nav_cmd();

    _helispdhgtctrl.set_max_accel(_wp_accel_cmss);
    float desired_speed = _wp_speed_cms;
    Location_Class curr_loc = _inav.get_position();
    float speed_forward = _ahrs.groundspeed_vector().x*_ahrs.cos_yaw() + _ahrs.groundspeed_vector().y*_ahrs.sin_yaw();
    float dist = 100.0f * get_distance(curr_loc, _next_WP_loc);
    float stop_distance = 0.6f * sq(speed_forward * 100.0f) / _wp_accel_cmss;

    switch (cmd.id) {

    case MAV_CMD_NAV_LAND:
        _L1_controller.update_waypoint(_prev_WP_loc, _next_WP_loc);
        if (dist < stop_distance || _stopping_at_waypoint) {
            desired_speed = 100.0f;
            _stopping_at_waypoint = true;
        }
        break;

    case MAV_CMD_NAV_WAYPOINT:
        _L1_controller.update_waypoint(_prev_WP_loc, _next_WP_loc);
        AP_Mission::Mission_Command dummy_cmd;
        if (_mission.get_next_nav_cmd(cmd.index+1,dummy_cmd)) {
            _stopping_at_waypoint = false;
        } else {
            if (dist < stop_distance || _stopping_at_waypoint) {
                desired_speed = 100.0f;
                _stopping_at_waypoint = true;
            }
        }
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
        _L1_controller.update_loiter(_next_WP_loc, _loiter_radius, 1);
        _stopping_at_waypoint = false;
        break;
    }

    // if last nav command is waypoint then stop at waypoint
    

    _helispdhgtctrl.set_desired_speed(desired_speed);
    _helispdhgtctrl.update_speed_controller();
    _wp_last_l1_update = AP_HAL::millis();

    return ret;
}


bool AC_WPNav_Heli::advance_l1_wp_target_along_track(float dt)
{

    // set up for altitude calculation
    int32_t temp_alt;
    int32_t wp_alt;
    int32_t prev_wp_alt;
    _next_WP_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_ORIGIN, wp_alt);
    _prev_WP_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_ORIGIN, prev_wp_alt);
    Location_Class curr_loc = _inav.get_position();
    float dist_to_curr_loc = _prev_WP_loc.get_distance(curr_loc);
    float dist_btwn_wp = _prev_WP_loc.get_distance(_next_WP_loc);

    // get current waypoint nav command
    AP_Mission::Mission_Command cmd;
    cmd = _mission.get_current_nav_cmd();

    switch (cmd.id) {

    case MAV_CMD_NAV_LOITER_TURNS:
        if (_L1_controller.reached_loiter_target()) {
            _L1_controller.loiter_angle_update();
        } else {
            _L1_controller.loiter_angle_reset();
        }
        // fallthrough
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_UNLIM:
        if (_L1_controller.reached_loiter_target()) {
            temp_alt = wp_alt;
            _reached_l1_destination = true;
        } else {
            float radius = _L1_controller.loiter_radius(_loiter_radius);
            temp_alt = (wp_alt - prev_wp_alt) * dist_to_curr_loc / (dist_btwn_wp - radius) + prev_wp_alt;
        }
        break;

    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_WAYPOINT:
    default:

        Location flex_next_WP_loc = _next_WP_loc;

        _L1_controller.update_waypoint(_prev_WP_loc, flex_next_WP_loc);
        float acceptance_distance_m = 5.0f; // default to: if overflown - let it fly up to the point

        if (!_stopping_at_waypoint) {
            int32_t next_ground_course_cd = _mission.get_next_ground_course_cd(-1);
            float next_turn_angle;
            if (next_ground_course_cd == -1) {
                // the mission library can't determine a turn angle, assume 90 degrees
                next_turn_angle = 90.0f;
            } else {
                // get the heading of the current leg
                int32_t ground_course_cd = get_bearing_cd(_prev_WP_loc, _next_WP_loc);

                // work out the angle we need to turn through
                next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
            }
            acceptance_distance_m = _L1_controller.turn_distance(_wp_radius_cm * 0.01f, next_turn_angle);
        }
        if (get_distance(curr_loc, _next_WP_loc) <= acceptance_distance_m) {
            _reached_l1_destination = true;
	}

        // have we flown past the waypoint?
        if (location_passed_point(curr_loc, _prev_WP_loc, flex_next_WP_loc)) {
            _reached_l1_destination = true;
        }

        if (wp_alt == prev_wp_alt) {
            temp_alt = wp_alt;
        } else {
            temp_alt = (wp_alt - prev_wp_alt) * dist_to_curr_loc / dist_btwn_wp + prev_wp_alt;
        }
        break;

    }

    // don't allow temp_alt to beyond wp altitudes
    if (wp_alt > prev_wp_alt) {
        temp_alt = constrain_float(temp_alt, prev_wp_alt, wp_alt);
    } else if (prev_wp_alt > wp_alt) {
        temp_alt = constrain_float(temp_alt, wp_alt, prev_wp_alt);
    }
    _pos_control.set_alt_target(temp_alt);

    return true;
}

/// using_l1_navigation - true when using L1 navigation controller
bool AC_WPNav_Heli::use_l1_navigation()
{
    if (_l1_nav_use == 1) {
        return true;
    } else {
        return false;
    }
}
