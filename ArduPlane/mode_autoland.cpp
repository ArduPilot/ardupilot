#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode AutoLand parameters
 */
const AP_Param::GroupInfo ModeAutoLand::var_info[] = {
    // @Param: WP_ALT
    // @DisplayName: Final approach WP altitude
    // @Description: This is the target altitude above HOME for final approach waypoint
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_ALT", 1, ModeAutoLand, final_wp_alt, 55),

    // @Param: WP_DIST
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
    //must be flying to enter
    if (!plane.is_flying()) { 
        gcs().send_text(MAV_SEVERITY_WARNING, "Must already be flying!"); 
        return false;
    }
    
    //takeoff direction must be set and must not be a quadplane, otherwise since flying switch to RTL so this can be used as FS action
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoLand is fixed wing only mode"); 
        return false; 
    }
#endif
    if (!plane.takeoff_state.takeoff_direction_initialized) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff initial direction not set,must use autotakeoff");
        return false;
    }


    //setup final approach waypoint
    plane.prev_WP_loc = plane.current_loc;
    const Location &home = ahrs.get_home();
    plane.set_target_altitude_current();
    plane.next_WP_loc = home;
    uint16_t bearing_cd = wrap_360_cd((plane.takeoff_state.takeoff_initial_direction + 180)*100);
    plane.next_WP_loc.offset_bearing(bearing_cd * 0.01f, final_wp_dist);
    plane.next_WP_loc.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_HOME);
    
    // create a command to fly to final approach waypoint and start it
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location = plane.next_WP_loc;
    plane.start_command(cmd);
    land_started = false;
    return true;
}

void ModeAutoLand::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.set_offset_altitude_location(plane.prev_WP_loc, plane.next_WP_loc);
    if (plane.landing.is_throttle_suppressed()) {
       // if landing is considered complete throttle is never allowed, regardless of landing type
       SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    } else {
       plane.calc_throttle();
    }
}

void ModeAutoLand::navigate()
{    
    // check to see if if we have reached final approach waypoint, switch to NAV_LAND command and start it once if so
    if (!land_started){
        if (plane.verify_nav_wp(cmd)){
            const Location &home_loc = ahrs.get_home();
            cmd.id = MAV_CMD_NAV_LAND;
            cmd.content.location = home_loc;
            land_started = true;
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = home_loc;
            plane.start_command(cmd);
        }
        return;
    //otherwise keep flying the current command   
    } else {
       plane.verify_command(cmd);
    }
}
