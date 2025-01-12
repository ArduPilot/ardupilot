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
    // @Description: The captured takeoff direction after ground course is established in autotakeoffsis offset by this amount to create a different landing direction and approach.However,if TKOFF_OPTION bit1 is set, the takeoff(landing) direction is captured immediately via compass heading upon arming, then this offset is NOT applied.
    // @Range: -360 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("DIR_OFF", 3, ModeAutoLand, landing_dir_off, 0),

    // @Param: OPTIONS
    // @DisplayName: Autoland mode options
    // @Description: Enables optional autoland mode behaviors
    // @Bitmask: 1: When set if there is a healthy compass in use the compass heading will be captured at arming and used for the AUTOLAND mode's initial takeoff direction instead of capturing ground course in NAV_TAKEOFF or Mode TAKEOFF or other modes.
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 4, ModeAutoLand, options, 0),

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

    // autoland not available for quadplanes as capture of takeoff direction
    // doesn't make sense
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "autoland not available");
        return false;
    }
#endif

    if (!plane.takeoff_state.initial_direction.initialized) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff initial direction not set,must use autotakeoff");
        return false;
    }

    plane.set_target_altitude_current();
    plane.auto_state.next_wp_crosstrack = true; // not really needed since previous/next_WP is not used by loiter cmd navigation

    plane.prev_WP_loc = plane.current_loc;

    /*
      landing is in 3 steps:
        1) a base leg loitering to alt waypoint
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
      now create the initial target waypoint for the loitering to alt base leg. We
      choose if we will do a right or left turn onto the landing based
      on where we are when we enter the landing mode
     */
    const float bearing_to_current_deg = wrap_180(degrees(land_start_WP.get_bearing(plane.current_loc)));
    const float bearing_err_deg = wrap_180(wrap_180(plane.takeoff_state.initial_direction.heading) - bearing_to_current_deg);
    const float bearing_offset_deg = bearing_err_deg > 0? -90 : 90;

    // make the "base leg" the smaller of the loiter radius and 1/3 of the final WP distance
    // this results in a neat landing no matter the loiter radius
    const float base_leg_length = MIN(final_wp_dist * 0.333, fabsf(plane.aparm.loiter_radius));

    cmd[0].id = MAV_CMD_NAV_LOITER_TO_ALT;
    cmd[0].content.location = land_start_WP;
    cmd[0].content.location.offset_bearing(plane.takeoff_state.initial_direction.heading + bearing_offset_deg, base_leg_length);
    cmd[0].content.location.loiter_ccw = bearing_err_deg>0? 1 :0;

    /*
      create the WP for the start of the landing
     */
    cmd[1].content.location = land_start_WP;
    cmd[1].id = MAV_CMD_NAV_WAYPOINT;

    // and the land WP
    cmd[2].id = MAV_CMD_NAV_LAND;
    cmd[2].content.location = home;

    // start first leg
    stage = AutoLandStage::LOITER;
    plane.start_command(cmd[0]);

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
    switch (stage) {
    case AutoLandStage::LOITER:
        if (plane.verify_loiter_to_alt(cmd[0])) {
            stage = AutoLandStage::BASE_LEG;
            plane.start_command(cmd[1]);
            // setup a predictable turn angle for the base leg
            plane.auto_state.next_turn_angle = 90;
            plane.auto_state.crosstrack = false;
        }
        break;

    case AutoLandStage::BASE_LEG:
        if (plane.verify_nav_wp(cmd[1])) {
            stage = AutoLandStage::LANDING;
            plane.start_command(cmd[2]);
        }
        break;

    case AutoLandStage::LANDING:
        plane.set_flight_stage(AP_FixedWing::FlightStage::LAND);
        plane.verify_command(cmd[2]);
        // make sure we line up
        plane.auto_state.crosstrack = true;
        break;
    }
}

/*
  Takeoff direction is initialized after arm when sufficient altitude
  and ground speed is obtained, then captured takeoff direction +
  offset used as landing direction in AUTOLAND
*/
void ModeAutoLand::check_takeoff_direction()
{
#if HAL_QUADPLANE_ENABLED
    // we don't allow fixed wing autoland for quadplanes
    if (quadplane.available()) {
        return;
    }
#endif

    if (plane.takeoff_state.initial_direction.initialized) {
        return;
    }
    //set autoland direction to GPS course over ground
    if (plane.control_mode->allows_autoland_direction_capture() &&
        plane.is_flying() &&
        hal.util->get_soft_armed() &&
        plane.gps.ground_speed() > GPS_GND_CRS_MIN_SPD) {
        set_autoland_direction();
    }
}

// Sets autoland direction using ground course + offest parameter
void ModeAutoLand::set_autoland_direction()
{
    plane.takeoff_state.initial_direction.heading = wrap_360(plane.gps.ground_course() + landing_dir_off);
    plane.takeoff_state.initial_direction.initialized = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland direction= %u",int(plane.takeoff_state.initial_direction.heading));
}

/*
  return true when the LOITER_TO_ALT is lined up ready for the landing
 */
bool ModeAutoLand::landing_lined_up(void)
{
    // use the line between the center of the LOITER_TO_ALT and the
    // start of the landing leg
    return plane.mode_loiter.isHeadingLinedUp_cd(cmd[0].content.location.get_bearing(cmd[1].content.location)*100);
}

// possibly capture heading for autoland mode if option is set and using a compass
void ModeAutoLand::arm_check(void)
{
    if (plane.ahrs.use_compass() && autoland_option_is_set(ModeAutoLand::AutoLandOption::AUTOLAND_DIR_ON_ARM)) {
        plane.takeoff_state.initial_direction.heading = wrap_360(plane.ahrs.yaw_sensor * 0.01f);
        plane.takeoff_state.initial_direction.initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Autoland direction= %u",int(plane.takeoff_state.initial_direction.heading));
    }
}

#endif // MODE_AUTOLAND_ENABLED

