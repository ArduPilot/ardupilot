#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode AutoLand parameters
 */
const AP_Param::GroupInfo ModeAutoLand::var_info[] = {
    // @Param: WP_ALT
    // @DisplayName: Final approach WP altitude
    // @Description: This is the target altitude for final approach waypoint
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_ALT", 1, ModeAutoLand, final_wp_alt, 55),

    // @Param: DIST
    // @DisplayName: Final approach WP distance
    // @Description: This is the distance from Home that the final approach waypoint is set. The waypoint point will be in the opposite direction of takeoff (the direction the plane is facing when the plane sets its takeoff heading)
    // @Range: 0 700
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_DIST", 2, ModeAutoLand, final_wp_dist, 400),

    AP_GROUPEND
};

ModeAutoLand::ModeAutoLand() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAutoLand::_enter()
{
    //is flying check here and DO_LAND_START check here
    plane.prev_WP_loc = plane.current_loc;
    const Location &home = ahrs.get_home();
    plane.set_target_altitude_current();
    plane.next_WP_loc = home;
    uint16_t bearing_cd = wrap_360((plane.takeoff_state.takeoff_initial_direction + 180));
    plane.next_WP_loc.offset_bearing(bearing_cd, final_wp_dist);
    plane.next_WP_loc.alt = home.alt + final_wp_alt*100;
    return true;
}

void ModeAutoLand::update()
{
    plane.set_target_altitude_location(plane.next_WP_loc);
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeAutoLand::navigate()
{    
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
}
