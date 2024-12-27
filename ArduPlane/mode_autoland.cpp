#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

#if MODE_AUTOLAND_ENABLED

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

    // @Param: DIR_OFF
    // @DisplayName: Landing direction offset from takeoff
    // @Description: The captured takeoff direction (at arming,if TKOFF_OPTION bit1 is set, or after ground course is established in autotakeoffs)is offset by this amount to create a different landing direction and approach.
    // @Range: -360 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("DIR_OFF", 3, ModeAutoLand, landing_dir_off, 0),

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
    if (quadplane.available() && !quadplane.option_is_set(QuadPlane::OPTION::ALLOW_FW_LAND)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Option not set to allow fixed wing autoland"); 
        return false; 
    }
#endif
    if (!plane.takeoff_state.initial_direction.initialized) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff initial direction not set,must use autotakeoff");
        return false;
    }

    plane.prev_WP_loc = plane.current_loc;
    plane.set_target_altitude_current();

    /*
      landing is in 3 steps:
        1) a base leg waypoint
        2) a land start WP, with crosstrack
        3) the landing WP, with crosstrack

      the base leg point is 90 degrees off from the landing leg
     */
    const Location &home = ahrs.get_home();

    /*
      first calculate the starting waypoint we will use when doing the
      NAV_LAND. This is offset by final_wp_dist from home, in a
      direction 180 degrees from takeoff direction
     */
    Location land_start_WP = home;
    land_start_WP.offset_bearing(wrap_360(plane.takeoff_state.initial_direction.heading + 180), final_wp_dist);
    land_start_WP.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_HOME);
    land_start_WP.change_alt_frame(Location::AltFrame::ABSOLUTE);

    /*
      now create the initial target waypoint for the base leg. We
      choose if we will do a right or left turn onto the landing based
      on where we are when we enter the landing mode
     */
    const float bearing_to_current_deg = wrap_180(degrees(land_start_WP.get_bearing(plane.current_loc)));
    const float bearing_err_deg = wrap_180(wrap_180(plane.takeoff_state.initial_direction.heading) - bearing_to_current_deg);
    const float bearing_offset_deg = bearing_err_deg > 0? -90 : 90;
    const float base_leg_length = final_wp_dist * 0.333;
    cmd[0].id = MAV_CMD_NAV_WAYPOINT;
    cmd[0].content.location = land_start_WP;
    cmd[0].content.location.offset_bearing(plane.takeoff_state.initial_direction.heading + bearing_offset_deg, base_leg_length);
    // set a 1m acceptance radius, we want to fly all the way to this waypoint
    cmd[0].p1 = 1;

    /*
      create the WP for the start of the landing
     */
    cmd[1].content.location = land_start_WP;
    cmd[1].id = MAV_CMD_NAV_WAYPOINT;

    // and the land WP
    cmd[2].id = MAV_CMD_NAV_LAND;
    cmd[2].content.location = home;

    // start first leg
    stage = 0;
    plane.start_command(cmd[0]);

    // don't crosstrack initially
    plane.auto_state.crosstrack = false;
    plane.auto_state.next_wp_crosstrack = true;

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
    if (stage == 2) {
        // we are on final landing leg
        plane.set_flight_stage(AP_FixedWing::FlightStage::LAND);
        plane.verify_command(cmd[stage]);
        return;
    }

    // see if we have completed the leg
    if (plane.verify_nav_wp(cmd[stage])) {
        stage++;
        plane.prev_WP_loc = plane.next_WP_loc;
        plane.auto_state.next_turn_angle = 90;
        plane.start_command(cmd[stage]);
        plane.auto_state.crosstrack = true;
        plane.auto_state.next_wp_crosstrack = true;
    }
}
#endif
